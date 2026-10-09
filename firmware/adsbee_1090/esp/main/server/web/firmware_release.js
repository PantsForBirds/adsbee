// Firmware update check for the ADSBee web UIs, using the adsbee.aero firmware service (API v1).
//
// The page reads AT+DEVICE_INFO? from the receiver and trades its unique ID and OTA keys for a short-lived token
// (POST auth). With that token it lists releases (GET releases?channel=stable|rc) and downloads one file
// (GET files/<tag>/<name>), which the page's own uploader then flashes. The OTA keys are sent only to the service and
// never shown or logged. A receiver without valid keys (DIY builds) gets a link to the GitHub releases page instead.
//
// The logic has no DOM access and is tested in node (test/firmware_release.test.js); mount() is the UI. The 1090 and
// 1421 consoles carry verbatim copies (test/firmware_release_drift.test.js).

(function (root) {
    'use strict';

    const DEFAULT_API_BASE = 'https://adsbee.aero/api/fw/v1/';
    // localStorage key that overrides the API base, for testing against another server.
    const API_BASE_KEY = 'adsbee-fw-api';
    const GITHUB_RELEASES_URL = 'https://github.com/PantsForBirds/adsbee/releases';
    const RC_WARNING = 'Release candidates have no guarantee of stability or of a clean migration to or from other ' +
                       'firmware versions.';
    // Per product: the file the page's uploader takes, and the AT+DEVICE_INFO? lines that identify the device.
    const PRODUCTS = {
        1090: { kind: 'ota', asset: /\.ota$/, idLine: 'RP2040 Flash Unique ID',
                versions: { rp2040: 'RP2040 Firmware Version', esp32: 'ESP32 Firmware Version' } },
        1421: { kind: 'hex', asset: /\.hex$/, idLine: 'CC1314R10 Unique ID',
                versions: { cc1314: 'CC1314R10 Firmware Version' } },
    };
    // AT+DEVICE_INFO? lines the pages capture. Anything else (MAC addresses etc.) is left out.
    const DEVICE_INFO_LINE = /^(Part Code|RP2040 Flash Unique ID|CC1314R10 Unique ID|OTA Key \d+|RP2040 Firmware Version|ESP32 Firmware Version|CC1314R10 Firmware Version):/;

    class ReleaseError extends Error {
        constructor(code, message, retryAt) {
            super(message);
            // offline | no-keys | rate-limit | bad-response | download | checksum | device
            this.code = code;
            this.retryAt = retryAt || null;
        }
    }

    /** "0.9.1-rc6" -> {major, minor, patch, rc}; rc is 0 for a final release. null if it doesn't parse. */
    function parseVersion(s) {
        const m = /^v?(\d+)\.(\d+)\.(\d+)(?:-rc(\d+))?$/.exec(String(s || '').trim());
        return m ? { major: +m[1], minor: +m[2], patch: +m[3], rc: m[4] ? +m[4] : 0 } : null;
    }

    function formatVersion(v) {
        return `${v.major}.${v.minor}.${v.patch}` + (v.rc ? `-rc${v.rc}` : '');
    }

    /** Sorts like numbers; a final release is newer than its release candidates. */
    function compareVersions(a, b) {
        for (const k of ['major', 'minor', 'patch']) if (a[k] !== b[k]) return a[k] - b[k];
        if (a.rc === b.rc) return 0;
        if (!a.rc) return 1;
        if (!b.rc) return -1;
        return a.rc - b.rc;
    }

    /** The POST auth body from AT+DEVICE_INFO? lines. Throws ReleaseError('device') if the ID is missing. */
    function parseDeviceInfo(lines, product) {
        const p = PRODUCTS[product];
        const get = (key) => {
            for (const l of lines) if (l.startsWith(key + ':')) return l.slice(key.length + 1).trim();
            return null;
        };
        const uniqueId = get(p.idLine);
        if (!uniqueId) throw new ReleaseError('device', 'The receiver did not report its unique ID.');
        const otaKeys = [];
        for (let i = 0; get(`OTA Key ${i}`) !== null; i++) otaKeys.push(get(`OTA Key ${i}`));
        const versions = {};
        for (const [k, line] of Object.entries(p.versions)) {
            const v = get(line);
            if (v) versions[k] = v;
        }
        return { product: `adsbee_${product}`, unique_id: uniqueId, part_code: get('Part Code') || '',
                 ota_keys: otaKeys, versions };
    }

    /** Running firmware version from the parsed device info: the RP2040 on the 1090, the CC1314 on the 1421. */
    function runningVersion(info) {
        return info.versions.rp2040 || info.versions.cc1314 || null;
    }

    const defaultFetch = (...args) => root.fetch(...args);

    function apiBase(base) {
        if (base) return base;
        try { return localStorage.getItem(API_BASE_KEY) || DEFAULT_API_BASE; } catch (e) { return DEFAULT_API_BASE; }
    }

    /** Client for the firmware service. One instance per device; it keeps the token. */
    class FirmwareApi {
        constructor({ base, fetchFn = defaultFetch, timeoutMs = 15000, client = 'web_ui' } = {}) {
            this.base = apiBase(base);
            this.fetchFn = fetchFn;
            this.timeoutMs = timeoutMs;
            this.client = client;
            this.info = null;
            this.token = null;
        }

        async _fetch(path, options = {}, timeoutMs = this.timeoutMs) {
            const ctl = typeof AbortController !== 'undefined' ? new AbortController() : null;
            const timer = ctl ? setTimeout(() => ctl.abort(), timeoutMs) : null;
            try {
                return await this.fetchFn(new URL(path, this.base).href,
                                          Object.assign({ cache: 'no-store' }, options, ctl ? { signal: ctl.signal } : {}));
            } catch (e) {
                throw new ReleaseError('offline', `Can't reach the firmware service (${new URL(this.base).host}).`);
            } finally {
                clearTimeout(timer);
            }
        }

        _check(resp) {
            if (resp.status === 429) {
                const s = Number(resp.headers.get('retry-after'));
                const at = s ? Date.now() + s * 1000 : null;
                throw new ReleaseError('rate-limit', 'Too many requests to the firmware service' +
                    (at ? `; try again after ${new Date(at).toLocaleTimeString()}.` : '.'), at);
            }
            if (!resp.ok) throw new ReleaseError('bad-response', `The firmware service answered ${resp.status}.`);
        }

        /** Trades the device identity for a token. info is parseDeviceInfo()'s result. */
        async auth(info) {
            this.info = info;
            this.token = null;
            const resp = await this._fetch('auth', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(Object.assign({}, info, { client: this.client })),
            });
            if (resp.status === 401 || resp.status === 403) {
                throw new ReleaseError('no-keys', 'This receiver has no valid OTA keys, so the firmware service ' +
                    'can\'t offer updates. Download the firmware from GitHub and use Upload Firmware.');
            }
            this._check(resp);
            const json = await resp.json().catch(() => null);
            if (!json || !json.token) throw new ReleaseError('bad-response', 'The firmware service sent no token.');
            this.token = json.token;
        }

        /** GET with the token, re-authenticating once if it expired. */
        async _authed(path, timeoutMs) {
            for (let attempt = 0; ; attempt++) {
                if (!this.token) await this.auth(this.info);
                const resp = await this._fetch(path, { headers: { Authorization: `Bearer ${this.token}` } }, timeoutMs);
                if (resp.status === 401 && attempt === 0) { this.token = null; continue; }
                this._check(resp);
                return resp;
            }
        }

        /** {product, current, recommended, releases} for channel 'stable' or 'rc'. */
        async releases(channel) {
            const resp = await this._authed(`releases?channel=${channel === 'rc' ? 'rc' : 'stable'}`);
            const json = await resp.json().catch(() => null);
            if (!json || !Array.isArray(json.releases)) {
                throw new ReleaseError('bad-response', 'The firmware service sent an unreadable release list.');
            }
            return json;
        }

        /** Downloads an asset and checks its size and SHA-256. Resolves to a Uint8Array. */
        async download(asset, onProgress = () => {}) {
            let bytes;
            try {
                const resp = await this._authed(asset.url, 300000);
                const total = Number(resp.headers.get('content-length')) || asset.size || 0;
                if (resp.body && resp.body.getReader) {
                    const reader = resp.body.getReader();
                    const parts = [];
                    let got = 0;
                    for (;;) {
                        const { done, value } = await reader.read();
                        if (done) break;
                        parts.push(value);
                        got += value.length;
                        if (total) onProgress(Math.min(99, Math.floor((100 * got) / total)));
                    }
                    bytes = new Uint8Array(got);
                    let o = 0;
                    for (const p of parts) { bytes.set(p, o); o += p.length; }
                } else {
                    bytes = new Uint8Array(await resp.arrayBuffer());
                }
            } catch (e) {
                if (e instanceof ReleaseError && e.code !== 'bad-response') throw e;
                throw new ReleaseError('download', `Download of ${asset.name} failed (${e.message}).`);
            }
            if (asset.size && bytes.length !== asset.size) {
                throw new ReleaseError('download', `${asset.name} is ${bytes.length} bytes, expected ${asset.size}.`);
            }
            if (asset.sha256 && (await sha256Hex(bytes)) !== asset.sha256.toLowerCase()) {
                throw new ReleaseError('checksum', `${asset.name} failed its SHA-256 check.`);
            }
            onProgress(100);
            return bytes;
        }
    }

    /**
     * Picks what to offer from a releases listing. channel 'stable' offers only the rollout's recommended version;
     * 'rc' offers the newest release listed. Returns {state, release, asset, current}, state one of:
     *   update (newer than running), current (same), ahead (running is newer; never offered as a downgrade),
     *   unknown (running version unreadable), none (nothing to offer).
     */
    function selectUpdate(listing, { product, current, channel }) {
        const p = PRODUCTS[product];
        const cur = parseVersion(current);
        const cands = [];
        for (const r of listing.releases || []) {
            const v = parseVersion(r.version);
            const asset = (r.assets || []).find((a) => a.kind === p.kind) ||
                          (r.assets || []).find((a) => p.asset.test(a.name || ''));
            if (v && asset && asset.url) cands.push({ release: r, version: v, asset });
        }
        cands.sort((a, b) => compareVersions(b.version, a.version));
        let best;
        if (channel === 'rc') {
            best = cands[0];
        } else {
            const rec = parseVersion(listing.recommended);
            best = rec && cands.find((c) => compareVersions(c.version, rec) === 0);
        }
        if (!best) return { state: 'none', current: cur };
        let state = 'unknown';
        if (cur) {
            const c = compareVersions(best.version, cur);
            state = c > 0 ? 'update' : c === 0 ? 'current' : 'ahead';
        }
        return { state, release: best.release, version: best.version, asset: best.asset, current: cur };
    }

    // SHA-256. crypto.subtle only exists in secure contexts, and the device page is plain http.
    const SHA_K = new Uint32Array(64);
    const SHA_H0 = new Uint32Array(8);
    (() => {
        const frac = (x) => ((x - Math.floor(x)) * 0x100000000) >>> 0;
        for (let n = 2, i = 0; i < 64; n++) {
            let prime = true;
            for (let d = 2; d * d <= n; d++) if (n % d === 0) { prime = false; break; }
            if (!prime) continue;
            if (i < 8) SHA_H0[i] = frac(Math.sqrt(n));
            SHA_K[i++] = frac(Math.cbrt(n));
        }
    })();

    function sha256Js(bytes) {
        const n = bytes.length;
        const total = Math.ceil((n + 9) / 64) * 64;
        const m = new Uint8Array(total);
        m.set(bytes);
        m[n] = 0x80;
        const dv = new DataView(m.buffer);
        dv.setUint32(total - 8, Math.floor(n / 0x20000000));
        dv.setUint32(total - 4, (n * 8) >>> 0);
        const H = SHA_H0.slice();
        const W = new Uint32Array(64);
        const ror = (x, r) => (x >>> r) | (x << (32 - r));
        for (let o = 0; o < total; o += 64) {
            for (let i = 0; i < 16; i++) W[i] = dv.getUint32(o + 4 * i);
            for (let i = 16; i < 64; i++) {
                const a = W[i - 15], b = W[i - 2];
                W[i] = (ror(a, 7) ^ ror(a, 18) ^ (a >>> 3)) + W[i - 7] + (ror(b, 17) ^ ror(b, 19) ^ (b >>> 10)) + W[i - 16];
            }
            let [a, b, c, d, e, f, g, h] = H;
            for (let i = 0; i < 64; i++) {
                const t1 = (h + (ror(e, 6) ^ ror(e, 11) ^ ror(e, 25)) + ((e & f) ^ (~e & g)) + SHA_K[i] + W[i]) | 0;
                const t2 = ((ror(a, 2) ^ ror(a, 13) ^ ror(a, 22)) + ((a & b) ^ (a & c) ^ (b & c))) | 0;
                h = g; g = f; f = e; e = (d + t1) | 0; d = c; c = b; b = a; a = (t1 + t2) | 0;
            }
            H[0] += a; H[1] += b; H[2] += c; H[3] += d; H[4] += e; H[5] += f; H[6] += g; H[7] += h;
        }
        return Array.from(H, (x) => x.toString(16).padStart(8, '0')).join('');
    }

    async function sha256Hex(bytes) {
        const subtle = root.crypto && root.crypto.subtle;
        if (!subtle) return sha256Js(bytes);
        const d = new Uint8Array(await subtle.digest('SHA-256', bytes));
        return Array.from(d, (x) => x.toString(16).padStart(2, '0')).join('');
    }

    /** Hides OTA key values in console text shown to the user. */
    function redactKeys(text) {
        return text.replace(/(OTA Key \d+:\s*)\S+/g, '$1••••');
    }

    /**
     * The update check UI, rendered into el. opts:
     *   product        '1090' | '1421'
     *   readDeviceInfo async () => AT+DEVICE_INFO? lines (DEVICE_INFO_LINE matches); throws if not connected
     *   install        async (Uint8Array, fileName) => runs the page's firmware upload with that file
     *   client         'web_ui' | 'console'
     *   apiBase        optional API base URL
     * Returns {check(), setAutoUpdate({enabled, state, version} | null)}.
     */
    function mount(el, opts) {
        const RC_KEY = 'adsbee-fw-include-rc';
        const h = (tag, props, ...kids) => {
            const e = Object.assign(document.createElement(tag), props || {});
            e.append(...kids.filter((k) => k !== null && k !== undefined && k !== false));
            return e;
        };
        const check = h('button', { className: 'tools-button fw-check-btn', textContent: '↻ Updates',
                                    title: 'Check for new firmware' });
        const rc = h('input', { type: 'checkbox' });
        try { rc.checked = localStorage.getItem(RC_KEY) === '1'; } catch (e) { /* storage blocked */ }
        const status = h('span', { className: 'fw-status' });
        const auto = h('span', { className: 'fw-auto fw-muted' });
        const warn = h('div', { className: 'fw-rc-warning', textContent: '⚠ RC: no stability or migration guarantee',
                                title: RC_WARNING });
        el.classList.add('fw-check');
        el.replaceChildren(h('div', { className: 'fw-row' }, check,
                             h('label', { className: 'fw-rc', title: `Include release candidates. ${RC_WARNING}` },
                               rc, ' RC'), status, auto), warn);

        const api = new FirmwareApi({ base: opts.apiBase, client: opts.client || 'web_ui' });
        let busy = false;
        let current = null;
        const show = (...kids) => status.replaceChildren(...kids.filter(Boolean));
        const github = (title) => h('a', { href: GITHUB_RELEASES_URL, target: '_blank', rel: 'noopener',
                                           textContent: 'GitHub ↗', title });
        const fail = (e) => {
            const short = { offline: 'offline', 'no-keys': 'no keys', 'rate-limit': 'busy', checksum: 'checksum',
                            download: 'download failed', device: 'no device' }[e.code] || 'error';
            show(h('span', { className: 'fw-bad', textContent: `⚠ ${short}`, title: e.message || String(e) }),
                 (e.code === 'offline' || e.code === 'no-keys') &&
                     github('Download the firmware from GitHub, then use Upload Firmware.'));
        };
        const notes = (r) => r.notes_url && h('a', { href: r.notes_url, target: '_blank', rel: 'noopener',
                                                     textContent: 'notes ↗', title: `${r.tag || r.version} release notes` });

        function render(sel) {
            const channel = rc.checked ? 'RC' : 'stable';
            const cur = sel.current ? formatVersion(sel.current) : '?';
            if (sel.state === 'none') {
                show(h('span', { className: 'fw-ok', textContent: `✓ ${cur}`,
                                 title: rc.checked ? 'No release found.' : 'No update assigned to this receiver.' }));
                return;
            }
            const latest = formatVersion(sel.version);
            if (sel.state === 'current' || sel.state === 'ahead') {
                show(h('span', { className: 'fw-ok', textContent: `✓ ${cur}`,
                                 title: sel.state === 'current' ? `Up to date (${channel}).`
                                     : `Running firmware is newer than the ${channel} offer (${latest}).` }),
                     notes(sel.release));
                return;
            }
            show(h('span', { textContent: `${cur} → ${latest}`,
                             title: sel.state === 'unknown' ? 'Running version unknown.' : 'Update available.' }),
                 h('button', { className: 'tools-button fw-update-btn', textContent: '⇣ Update',
                               title: `Download ${sel.asset.name} and install it`, onclick: () => update(sel) }),
                 notes(sel.release));
        }

        async function run() {
            if (busy) return;
            busy = true;
            check.disabled = true;
            show(h('span', { className: 'fw-muted', textContent: '…' }));
            try {
                let info;
                try {
                    info = parseDeviceInfo(await opts.readDeviceInfo(), opts.product);
                } catch (e) {
                    throw e instanceof ReleaseError ? e : new ReleaseError('device', e.message || 'Not connected.');
                }
                current = runningVersion(info);
                await api.auth(info);
                await refresh();
            } catch (e) {
                fail(e);
            } finally {
                busy = false;
                check.disabled = false;
            }
        }

        async function refresh() {
            const channel = rc.checked ? 'rc' : 'stable';
            const listing = await api.releases(channel);
            render(selectUpdate(listing, { product: opts.product, current, channel }));
        }

        async function update(sel) {
            if (busy) return;
            const to = formatVersion(sel.version);
            const from = sel.current ? formatVersion(sel.current) : 'unknown';
            const isRc = sel.release.prerelease || sel.version.rc > 0;
            if (!confirm(`Update ADSBee ${opts.product} firmware ${from} → ${to}?` + (isRc ? `\n\n${RC_WARNING}` : ''))) {
                return;
            }
            busy = true;
            check.disabled = true;
            try {
                const bytes = await api.download(sel.asset, (p) => show(h('span', { className: 'fw-muted',
                                                                                     textContent: `⇣ ${p}%` })));
                show(h('span', { className: 'fw-muted', textContent: `↻ ${to}` }));
                await opts.install(bytes, sel.asset.name);
                show();
            } catch (e) {
                fail(e);
            } finally {
                busy = false;
                check.disabled = false;
            }
        }

        function setAutoUpdate(a) {
            if (!a) { auto.replaceChildren(); return; }
            auto.textContent = `⟳ ${a.enabled ? 'auto' : 'manual'}${a.state && a.state !== 'idle' ? ` · ${a.state}` : ''}`;
            auto.title = `Automatic updates ${a.enabled ? 'on' : 'off'}` + (a.state ? `; ${a.state}` : '') +
                         (a.version ? ` (${a.version})` : '') + '.';
        }

        check.addEventListener('click', run);
        rc.addEventListener('change', () => {
            try { localStorage.setItem(RC_KEY, rc.checked ? '1' : '0'); } catch (e) { /* storage blocked */ }
            warn.style.display = rc.checked ? '' : 'none';
            if (api.token && !busy) {
                busy = true;
                refresh().catch(fail).finally(() => { busy = false; });
            }
        });
        warn.style.display = rc.checked ? '' : 'none';
        return { check: run, setAutoUpdate };
    }

    const api = { DEFAULT_API_BASE, API_BASE_KEY, GITHUB_RELEASES_URL, RC_WARNING, DEVICE_INFO_LINE, ReleaseError,
                  FirmwareApi, parseVersion, formatVersion, compareVersions, parseDeviceInfo, runningVersion,
                  selectUpdate, sha256Js, sha256Hex, redactKeys, mount };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.AdsbeeFirmwareRelease = api;
})(typeof window !== 'undefined' ? window : globalThis);
