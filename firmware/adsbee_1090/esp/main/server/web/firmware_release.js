// Firmware update check for the ADSBee web UIs, using the adsbee.aero firmware service (API v1).
//
// The page reads AT+DEVICE_INFO? from the receiver and trades its unique ID and OTA keys for a short-lived token
// (POST auth). With that token it lists releases (GET releases?channel=stable|rc) and downloads one file
// (GET files/<tag>/<name>), which the page's own uploader then flashes. A receiver without valid keys (DIY builds) gets
// a link to the GitHub releases page instead.
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
        // Units provisioned before 2026 have keys that sign the ESP32 base MAC; the service tries both.
        1090: { kind: 'ota', asset: /\.ota$/, idLine: 'RP2040 Flash Unique ID', macLine: 'ESP32 Base MAC Address',
                versions: { rp2040: 'RP2040 Firmware Version', esp32: 'ESP32 Firmware Version' } },
        1421: { kind: 'hex', asset: /\.hex$/, idLine: 'CC1314R10 Unique ID',
                versions: { cc1314: 'CC1314R10 Firmware Version' } },
    };
    // AT+DEVICE_INFO? lines the pages capture. Anything else (MAC addresses etc.) is left out.
    const DEVICE_INFO_LINE = /^(Part Code|RP2040 Flash Unique ID|CC1314R10 Unique ID|OTA Key \d+|RP2040 Firmware Version|ESP32 Firmware Version|ESP32 Base MAC Address|CC1314R10 Firmware Version):/;

    class ReleaseError extends Error {
        constructor(code, message, retryAt) {
            super(message);
            // offline | no-keys | wrong-product | rate-limit | bad-response | download | checksum | device | install
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
        const body = { product: `adsbee_${product}`, unique_id: uniqueId, part_code: get('Part Code') || '',
                       ota_keys: otaKeys, versions };
        if (p.macLine && get(p.macLine)) body.mac_address = get(p.macLine);
        return body;
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
            if (resp.status === 403) {
                const json = await resp.json().catch(() => null);
                if (json && json.error === 'wrong_product') {
                    throw new ReleaseError('wrong-product', `This receiver's part code doesn't belong to an ADSBee ` +
                        `${info.product.slice(7)}.`);
                }
            }
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
     * Picks what to offer from a releases listing. channel 'stable' offers the rollout's recommended version; 'rc'
     * offers the newest release listed. An older version is offered only when the service flags an admin rollback
     * (listing.rollback), on either channel. Returns {state, release, version, asset, current}, state one of:
     *   update (newer than running), rollback, current (same), ahead (running is newer than the offer),
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
        const rec = parseVersion(listing.recommended);
        const recommended = rec && cands.find((c) => compareVersions(c.version, rec) === 0);
        const rollback = listing.rollback === true && recommended && cur && compareVersions(rec, cur) < 0;
        const best = rollback ? recommended : channel === 'rc' ? cands[0] : recommended;
        if (!best) return { state: 'none', current: cur };
        let state = 'unknown';
        if (cur) {
            const c = compareVersions(best.version, cur);
            state = c > 0 ? 'update' : c === 0 ? 'current' : rollback ? 'rollback' : 'ahead';
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

    // How long a release listing is reused before the page asks the service again.
    const CHECK_INTERVAL_MS = 6 * 3600 * 1000;
    const CACHE_KEY = 'adsbee-fw-cache';
    const FAILED = ['checksum', 'download', 'bad-response', 'wrong-product', 'install'];

    /** Identifies what a cached listing belongs to: service, receiver and the firmware it runs. */
    function cacheKey(base, info) {
        return [base, info.product, info.unique_id, runningVersion(info) || '?'].join('|');
    }

    /** The cached {key, at, stable, rc} if it is for key and younger than CHECK_INTERVAL_MS, else null. */
    function readCache(store, key, now) {
        try {
            const c = JSON.parse(store.getItem(CACHE_KEY));
            return c && c.key === key && c.stable && now - c.at >= 0 && now - c.at < CHECK_INTERVAL_MS ? c : null;
        } catch (e) {
            return null;
        }
    }

    /**
     * The nav bar dot. s = {phase: '' | 'check' | 'install', err, stable, rc}; stable and rc are selectUpdate()
     * results or null (rc only when the RC box is ticked). Returns {dot, tip}, dot one of:
     *   off (gray: not checked, offline, no keys), busy (spinner), ok (green), new (amber), bad (red).
     */
    function chipState(s) {
        if (s.phase) return { dot: 'busy', tip: s.phase === 'install' ? 'Installing…' : 'Checking for updates…' };
        if (s.err) return { dot: FAILED.includes(s.err.code) ? 'bad' : 'off', tip: s.err.message || String(s.err) };
        if (!s.stable) return { dot: 'off', tip: 'Firmware updates: not checked.' };
        const offer = [s.stable, s.rc].find((x) => x && ['update', 'rollback', 'unknown'].includes(x.state));
        if (!offer) return { dot: 'ok', tip: 'Firmware is up to date.' };
        const v = formatVersion(offer.version);
        return { dot: 'new', tip: offer.state === 'rollback' ? `Rollback to ${v} requested.` : `Update available: ${v}.` };
    }

    const CSS = `
.fw-chip{display:inline-flex;align-items:center;gap:6px;background:none;border:1px solid var(--border-subtle);border-radius:999px;padding:4px 10px;font:12px ui-monospace,Menlo,Consolas,monospace;color:var(--text-secondary);cursor:pointer;white-space:nowrap}
.fw-chip:hover{color:var(--text-primary)}
.fw-dot{width:9px;height:9px;border-radius:50%;background:#9ca3af;flex:none;box-sizing:border-box}
.fw-dot.ok{background:#16a34a}
.fw-dot.bad{background:#dc2626}
.fw-dot.new{background:#f59e0b;animation:fw-pulse 1.4s ease-in-out infinite}
.fw-dot.busy{background:none;border:2px solid #9ca3af;border-top-color:transparent;animation:fw-spin .8s linear infinite}
@keyframes fw-pulse{50%{box-shadow:0 0 0 4px rgba(245,158,11,.3)}}
@keyframes fw-spin{to{transform:rotate(360deg)}}
.fw-menu{position:fixed;top:52px;right:8px;z-index:900;box-sizing:border-box;min-width:250px;max-width:calc(100vw - 16px);padding:10px 12px;background:var(--bg-card);color:var(--text-primary);border:1px solid var(--border-subtle);border-radius:8px;box-shadow:0 6px 20px rgba(0,0,0,.25);font:13px Arial,sans-serif;line-height:1.3;user-select:none}
.fw-menu[hidden]{display:none}
.fw-line{display:flex;flex-wrap:wrap;align-items:center;gap:8px;min-height:28px}
.fw-line>:first-child{min-width:52px;color:var(--text-secondary)}
.fw-line label{cursor:pointer;display:inline-flex;align-items:center;gap:5px}
.fw-line input{margin:0}
.fw-grow{flex:1}
.fw-menu a{color:inherit}
.fw-menu button{font:inherit;border:0;border-radius:5px;padding:4px 9px;cursor:pointer;background:#ffcb00;color:#18181b}
.fw-menu button.fw-plain{background:none;color:var(--text-secondary);padding:2px 4px;font-size:15px}
.fw-menu button:disabled{opacity:.5;cursor:default}
.fw-menu .fw-ok{color:#16a34a}
.fw-menu .fw-bad{color:#dc2626}
.fw-menu .fw-warn{color:#d97706}
.fw-menu div.fw-warn{font-size:12px}
.fw-muted{color:var(--text-secondary)}
@media(max-width:760px){.fw-chip{padding:6px}.fw-chip span+span{display:none}}
@media(max-width:520px){.theme-toggle-label{display:none}.tab-btn,.tab-logo{padding:0 6px}}
`;

    /**
     * The update UI: a version chip with a status dot, rendered into el (in the nav bar), and its dropdown. opts:
     *   product        '1090' | '1421'
     *   readDeviceInfo async () => AT+DEVICE_INFO? lines (DEVICE_INFO_LINE matches); throws if not connected
     *   install        async (Uint8Array, fileName) => runs the page's firmware upload with that file
     *   client         'web_ui' | 'console'
     *   apiBase        optional API base URL
     * The page calls check() when the receiver connects, and again when it is back after an update; listings are
     * cached for CHECK_INTERVAL_MS and refreshed at that interval while the page stays open.
     * Returns {check(force), reset(), setAutoUpdate({enabled, state, version} | null)}.
     */
    function mount(el, opts) {
        const RC_KEY = 'adsbee-fw-include-rc';
        const h = (tag, props, ...kids) => {
            const e = Object.assign(document.createElement(tag), props || {});
            e.append(...kids.filter((k) => k !== null && k !== undefined && k !== false));
            return e;
        };
        const store = { getItem: (k) => { try { return localStorage.getItem(k); } catch (e) { return null; } },
                        setItem: (k, v) => { try { localStorage.setItem(k, v); } catch (e) { /* storage blocked */ } } };
        if (!document.getElementById('fw-css')) document.head.append(h('style', { id: 'fw-css', textContent: CSS }));

        const api = new FirmwareApi({ base: opts.apiBase, client: opts.client || 'web_ui' });
        const dot = h('span', { className: 'fw-dot' });
        const ver = h('span', { textContent: '—' });
        const chip = h('button', { className: 'fw-chip' }, dot, ver);
        const menu = h('div', { className: 'fw-menu', hidden: true });
        const rc = h('input', { type: 'checkbox', checked: store.getItem(RC_KEY) === '1' });
        el.replaceChildren(chip, menu);

        let phase = '';         // '' | 'check' | 'install'
        let progress = '';
        let err = null;
        let info = null;
        let lists = null;       // {key, at, stable, rc}
        let retryAt = 0;        // The service's Retry-After.
        let auto = null;
        let again = false;

        const pick = (channel) => (lists && lists[channel] && info
            ? selectUpdate(lists[channel], { product: opts.product, current: runningVersion(info), channel }) : null);
        const link = (href, text, title) => h('a', { href, target: '_blank', rel: 'noopener', textContent: text, title });

        // One dropdown line for a channel: the offer with its button, or "✓ up to date".
        function offer(label, sel, channel) {
            const kids = [label];
            if (sel && !sel.state) {
                kids.push(h('span', { className: 'fw-grow' }));
            } else if (!sel) {
                kids.push(h('span', { className: 'fw-muted fw-grow', textContent: '—', title: 'Not checked.' }));
            } else if (['update', 'rollback', 'unknown'].includes(sel.state)) {
                const back = sel.state === 'rollback';
                kids.push(h('span', { className: 'fw-grow', textContent: formatVersion(sel.version),
                                      title: back ? 'The rollout moved this receiver back to this version.'
                                          : sel.state === 'unknown' ? 'Running version unknown.' : 'Update available.' }),
                          h('button', { textContent: back ? '↓ Roll back' : '⇣ Update', disabled: !!phase,
                                        title: `Download ${sel.asset.name} and install it`, onclick: () => update(sel) }));
            } else {
                kids.push(h('span', { className: 'fw-ok fw-grow', textContent: '✓ up to date',
                                      title: sel.state === 'none' ? (channel === 'rc' ? 'No release found.'
                                                                     : 'No update assigned to this receiver.')
                                          : sel.state === 'ahead' ? `Running firmware is newer than the ${channel} offer ` +
                                                                    `(${formatVersion(sel.version)}).`
                                          : `Running the newest ${channel} release.` }));
            }
            const r = sel && sel.release;
            if (r && r.notes_url) kids.push(link(r.notes_url, 'notes ↗', `${r.tag || r.version} release notes`));
            return h('div', { className: 'fw-line' }, ...kids);
        }

        function render() {
            const s = chipState({ phase, err, stable: pick('stable'), rc: rc.checked ? pick('rc') : null });
            const cur = info && runningVersion(info);
            dot.className = `fw-dot ${s.dot}`;
            ver.textContent = cur || '—';
            chip.title = s.tip;
            const short = err && ({ offline: 'offline', 'no-keys': 'no keys', 'wrong-product': 'wrong product',
                                    'rate-limit': 'busy', checksum: 'checksum', download: 'download failed',
                                    device: 'no device' }[err.code] || 'error');
            menu.replaceChildren(...[
                h('div', { className: 'fw-line' },
                  h('span', { textContent: 'installed' }), h('span', { className: 'fw-grow', textContent: cur || '—' }),
                  phase && h('span', { className: 'fw-muted', textContent: progress || '…' }),
                  h('button', { className: 'fw-plain', textContent: '↻', disabled: !!phase, onclick: () => check(true),
                                title: 'Check now' + (lists ? ` (last: ${new Date(lists.at).toLocaleString()})` : '') })),
                offer(h('span', { textContent: 'stable', title: 'The release recommended for this receiver.' }),
                      pick('stable'), 'stable'),
                offer(h('label', { title: `Show release candidates. ${RC_WARNING}` }, rc, ' RC'),
                      rc.checked ? pick('rc') : { state: '' }, 'rc'),
                rc.checked && h('div', { className: 'fw-warn', textContent: '⚠ no stability or migration guarantee',
                                         title: RC_WARNING }),
                auto && h('div', { className: 'fw-line fw-muted', title: `Automatic updates ${auto.enabled ? 'on' : 'off'}` +
                                   (auto.state ? `; ${auto.state}` : '') + (auto.version ? ` (${auto.version})` : '') + '.',
                                   textContent: `⟳ ${auto.enabled ? 'auto' : 'manual'}` +
                                                (auto.state && auto.state !== 'idle' ? ` · ${auto.state}` : '') }),
                err && h('div', { className: 'fw-line' },
                         h('span', { className: s.dot === 'bad' ? 'fw-bad' : 'fw-warn', textContent: `⚠ ${short}`,
                                     title: err.message || String(err) }),
                         ['offline', 'no-keys', 'rate-limit'].includes(err.code) &&
                             link(GITHUB_RELEASES_URL, 'GitHub ↗',
                                  'Download the firmware from GitHub, then use Upload Firmware.')),
            ].filter(Boolean));
        }

        // Listings from the cache, or from the service when forced, stale or missing.
        async function load(force) {
            const key = cacheKey(api.base, info);
            const now = Date.now();
            const cached = !force && readCache(store, key, now);
            if (cached && (cached.rc || !rc.checked)) { lists = cached; return; }
            if (now < retryAt) {
                throw new ReleaseError('rate-limit', 'Too many requests to the firmware service; try again after ' +
                                       `${new Date(retryAt).toLocaleTimeString()}.`, retryAt);
            }
            try {
                if (api.info !== info) await api.auth(info);
                const stable = cached ? cached.stable : await api.releases('stable');
                lists = { key, at: cached ? cached.at : now, stable, rc: rc.checked ? await api.releases('rc') : null };
                store.setItem(CACHE_KEY, JSON.stringify(lists));
            } catch (e) {
                if (e.retryAt) retryAt = e.retryAt;
                throw e;
            }
        }

        /** Reads the receiver and updates the dot. force skips the cache. Resolves to true if the receiver answered. */
        async function check(force) {
            again = phase === 'install';    // Asked while installing: check when that is done.
            if (phase) return false;
            phase = 'check';
            err = null;
            render();
            let answered = false;
            try {
                let next;
                try {
                    next = parseDeviceInfo(await opts.readDeviceInfo(), opts.product);
                } catch (e) {
                    info = lists = null;
                    throw e instanceof ReleaseError ? e : new ReleaseError('device', e.message || 'Not connected.');
                }
                answered = true;
                if (!info || cacheKey(api.base, info) !== cacheKey(api.base, next)) { info = next; lists = null; }
                await load(force);
            } catch (e) {
                err = e;
            }
            phase = '';
            render();
            return answered;
        }

        async function update(sel) {
            if (phase) return;
            const to = formatVersion(sel.version);
            const from = sel.current ? formatVersion(sel.current) : 'unknown';
            const isRc = sel.release.prerelease || sel.version.rc > 0;
            const verb = sel.state === 'rollback' ? 'Roll back' : 'Update';
            if (!confirm(`${verb} ADSBee ${opts.product} firmware ${from} → ${to}?` + (isRc ? `\n\n${RC_WARNING}` : ''))) {
                return;
            }
            phase = 'install';
            err = null;
            menu.hidden = true;
            const say = (text) => { progress = text; render(); };
            try {
                say('⇣ 0%');
                if (api.info !== info) await api.auth(info);
                const bytes = await api.download(sel.asset, (p) => say(`⇣ ${p}%`));
                say(`↻ ${to}`);
                try {
                    await opts.install(bytes, sel.asset.name);
                } catch (e) {
                    throw e instanceof ReleaseError ? e : new ReleaseError('install', e.message || 'Install failed.');
                }
                // The receiver reboots; the page calls check() when it answers again.
                info = lists = null;
            } catch (e) {
                err = e;
            }
            phase = progress = '';
            render();
            if (again) check();
        }

        chip.addEventListener('click', () => { menu.hidden = !menu.hidden; });
        document.addEventListener('click', (e) => { if (!el.contains(e.target)) menu.hidden = true; });
        document.addEventListener('keydown', (e) => { if (e.key === 'Escape') menu.hidden = true; });
        rc.addEventListener('change', async () => {
            store.setItem(RC_KEY, rc.checked ? '1' : '0');
            if (rc.checked && info && !phase) {
                phase = 'check';
                render();
                await load(false).then(() => { err = null; }, (e) => { err = e; });
                phase = '';
            }
            render();
        });
        // Refresh the listing when it has aged out; the receiver isn't asked again.
        setInterval(async () => {
            if (!info || phase || document.hidden || (lists && Date.now() - lists.at < CHECK_INTERVAL_MS)) return;
            phase = 'check';
            render();
            await load(false).then(() => { err = null; }, (e) => { err = e; });
            phase = '';
            render();
        }, 10 * 60 * 1000);
        render();
        return {
            check,
            reset() { info = lists = err = null; render(); },
            setAutoUpdate(a) { auto = a || null; render(); },
        };
    }

    const api = { DEFAULT_API_BASE, API_BASE_KEY, GITHUB_RELEASES_URL, RC_WARNING, DEVICE_INFO_LINE, CHECK_INTERVAL_MS,
                  CACHE_KEY, ReleaseError, FirmwareApi, parseVersion, formatVersion, compareVersions, parseDeviceInfo,
                  runningVersion, selectUpdate, cacheKey, readCache, chipState, sha256Js, sha256Hex, mount };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.AdsbeeFirmwareRelease = api;
})(typeof window !== 'undefined' ? window : globalThis);
