// Tests for firmware_release.js against test/fw_api_mock.js. Run: node --test firmware/adsbee_1090/esp/main/server/web/test/
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const crypto = require('node:crypto');
const fr = require('../firmware_release.js');
const { startMock } = require('./fw_api_mock.js');

const v = fr.parseVersion;
const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
const bytes = (n, seed) => Uint8Array.from({ length: n }, (_, i) => (i * 31 + seed) & 0xFF);

// AT+DEVICE_INFO? as the 1090 prints it (the pages keep only DEVICE_INFO_LINE matches).
const INFO_1090 = [
    'Part Code: 0010-0021-1090U',
    'RP2040 Flash Unique ID: E46478B14B4C1F23',
    'RP2040 Firmware Version: 0.9.1-rc5',
    'OTA Key 0: valid-E46478B14B4C1F23',
    'OTA Key 1: 3045022100abcdef',
    'ESP32 Firmware Version: 0.9.1-rc5',
    'ESP32 Base MAC Address: DC:54:75:D2:6C:24',
];
const INFO_1421 = [
    'Part Code: 0020-0001-1421',
    'CC1314R10 Unique ID: 00124B0029A1B2C3',
    'CC1314R10 Firmware Version: 0.3.10',
    'OTA Key 0: valid-00124B0029A1B2C3',
    'OTA Key 1: k1',
];

const REL = (version, assets, extra = {}) => Object.assign({
    tag: `adsbee_1090-${version}`, version, notes_url: `https://github.com/n/${version}`, assets,
}, extra);
const OTA = [{ name: 'adsbee_1090.ota', kind: 'ota' }, { name: 'combined.uf2', kind: 'uf2' }];

async function mock1090(opts = {}) {
    const files = {};
    const releases = ['0.9.0', '0.9.1-rc5', '0.9.1-rc6', '0.9.0-rc19'].map((ver, i) => {
        files[`adsbee_1090-${ver}/adsbee_1090.ota`] = Buffer.from(bytes(3000 + i, i));
        files[`adsbee_1090-${ver}/combined.uf2`] = Buffer.from(bytes(10, i));
        return REL(ver, OTA);
    });
    return startMock(Object.assign({ releases, files, recommended: '0.9.0' }, opts));
}

test('parses and orders versions, release candidates before the final release', () => {
    assert.deepStrictEqual(v('0.9.1-rc6'), { major: 0, minor: 9, patch: 1, rc: 6 });
    assert.deepStrictEqual(v(' 0.9.0 '), { major: 0, minor: 9, patch: 0, rc: 0 });
    assert.strictEqual(v('0.9'), null);
    assert.strictEqual(v(undefined), null);
    const order = ['0.8.2', '0.9.0-rc2', '0.9.0-rc10', '0.9.0-rc19', '0.9.0', '0.9.1-rc1', '0.9.1-rc6', '0.9.1',
                   '0.10.0-rc1', '1.0.0'];
    const shuffled = [...order].reverse().sort((a, b) => fr.compareVersions(v(a), v(b)));
    assert.deepStrictEqual(shuffled, order);
    assert.strictEqual(fr.formatVersion(v('0.9.1-rc6')), '0.9.1-rc6');
});

test('parseDeviceInfo builds the auth body for each product', () => {
    assert.deepStrictEqual(fr.parseDeviceInfo(INFO_1090, '1090'), {
        product: 'adsbee_1090', unique_id: 'E46478B14B4C1F23', part_code: '0010-0021-1090U',
        ota_keys: ['valid-E46478B14B4C1F23', '3045022100abcdef'],
        versions: { rp2040: '0.9.1-rc5', esp32: '0.9.1-rc5' }, mac_address: 'DC:54:75:D2:6C:24',
    });
    assert.ok(!('mac_address' in fr.parseDeviceInfo(INFO_1090.slice(0, -1), '1090')), 'MAC only when printed');
    const i1421 = fr.parseDeviceInfo(INFO_1421, '1421');
    assert.strictEqual(i1421.product, 'adsbee_1421');
    assert.strictEqual(i1421.unique_id, '00124B0029A1B2C3');
    assert.deepStrictEqual(i1421.versions, { cc1314: '0.3.10' });
    assert.strictEqual(fr.runningVersion(i1421), '0.3.10');
    assert.throws(() => fr.parseDeviceInfo(['Part Code: x'], '1090'), (e) => e.code === 'device');
    for (const l of INFO_1090) assert.match(l, fr.DEVICE_INFO_LINE);
    assert.doesNotMatch('ESP32 WiFi AP MAC Address: 00:11:22:33:44:55', fr.DEVICE_INFO_LINE);
});

const listing = (recommended, releases) => ({ recommended, releases: releases.map((r) => ({
    version: r.version, tag: r.tag, prerelease: /-rc/.test(r.version),
    assets: r.assets.map((a) => Object.assign({ url: `files/${r.tag}/${a.name}`, size: 1, sha256: 'a'.repeat(64) }, a)),
})) });

test('stable offers only the rollout recommendation; null offers nothing', () => {
    const l = listing('0.9.0', [REL('0.9.0', OTA), REL('0.8.2', OTA)]);
    const s = fr.selectUpdate(l, { product: 1090, current: '0.8.2', channel: 'stable' });
    assert.strictEqual(s.state, 'update');
    assert.strictEqual(s.release.version, '0.9.0');
    assert.strictEqual(s.asset.name, 'adsbee_1090.ota', 'kind ota, not the uf2');
    assert.strictEqual(fr.selectUpdate(listing(null, [REL('0.9.0', OTA)]),
                                       { product: 1090, current: '0.8.2', channel: 'stable' }).state, 'none');
    // A staged RC recommendation is offered on stable too.
    assert.strictEqual(fr.selectUpdate(listing('0.9.1-rc6', [REL('0.9.1-rc6', OTA), REL('0.9.0', OTA)]),
                                       { product: 1090, current: '0.9.0', channel: 'stable' }).release.version,
                       '0.9.1-rc6');
});

test('RC channel offers the newest listed release (rc10 > rc6)', () => {
    const l = listing('0.9.0', [REL('0.9.1-rc6', OTA), REL('0.9.1-rc10', OTA), REL('0.9.0', OTA)]);
    const s = fr.selectUpdate(l, { product: '1090', current: '0.9.0', channel: 'rc' });
    assert.strictEqual(s.state, 'update');
    assert.strictEqual(s.release.version, '0.9.1-rc10');
});

test('downgrades only on an admin rollback, on either channel; equal is "current"', () => {
    const l = listing('0.9.0', [REL('0.9.0', OTA), REL('0.9.1-rc6', OTA)]);
    // An RC tester ahead of stable is not offered 0.9.0.
    assert.strictEqual(fr.selectUpdate(l, { product: 1090, current: '0.9.1-rc6', channel: 'stable' }).state, 'ahead');
    assert.strictEqual(fr.selectUpdate(l, { product: 1090, current: '0.9.1-rc7', channel: 'rc' }).state, 'ahead');
    const rolled = Object.assign({}, l, { rollback: true });
    for (const channel of ['stable', 'rc']) {
        const s = fr.selectUpdate(rolled, { product: 1090, current: '0.9.1-rc6', channel });
        assert.strictEqual(s.state, 'rollback', channel);
        assert.strictEqual(s.release.version, '0.9.0');
    }
    assert.strictEqual(fr.selectUpdate(l, { product: 1090, current: '0.9.1-rc6', channel: 'rc' }).state, 'current');
    assert.strictEqual(fr.selectUpdate(l, { product: 1090, current: 'garbage', channel: 'rc' }).state, 'unknown');
});

test('asset pick per product; releases without the file are skipped', () => {
    const hex = [{ name: 'adsbee_1421-0.3.11-rc5.elf', kind: 'elf' }, { name: 'adsbee_1421-0.3.11-rc5.hex', kind: 'hex' },
                 { name: 'adsbee_1421_programmer-fw0.3.11-rc5.uf2', kind: 'programmer_uf2' }];
    const l = listing('0.3.11-rc5', [REL('0.3.11-rc5', hex, { tag: 'adsbee_1421-0.3.11-rc5' })]);
    assert.strictEqual(fr.selectUpdate(l, { product: 1421, current: '0.3.10', channel: 'stable' }).asset.name,
                       'adsbee_1421-0.3.11-rc5.hex');
    const noOta = listing(null, [REL('0.9.5', [{ name: 'combined.uf2', kind: 'uf2' }]), REL('0.9.4', OTA)]);
    assert.strictEqual(fr.selectUpdate(noOta, { product: 1090, current: '0.9.0', channel: 'rc' }).release.version,
                       '0.9.4');
});

test('auth, list and download against the mock service', async () => {
    const mock = await mock1090();
    try {
        const api = new fr.FirmwareApi({ base: mock.base });
        const info = fr.parseDeviceInfo(INFO_1090, '1090');
        await api.auth(info);
        assert.strictEqual(mock.authBodies[0].client, 'web_ui');
        assert.deepStrictEqual(mock.authBodies[0].ota_keys, info.ota_keys);

        const stable = await api.releases('stable');
        assert.deepStrictEqual(stable.releases.map((r) => r.version), ['0.9.0']);
        const rc = await api.releases('rc');
        assert.deepStrictEqual(rc.releases.map((r) => r.version), ['0.9.1-rc6', '0.9.1-rc5', '0.9.0', '0.9.0-rc19']);

        const sel = fr.selectUpdate(rc, { product: 1090, current: '0.9.1-rc5', channel: 'rc' });
        assert.strictEqual(sel.state, 'update');
        const progress = [];
        const got = await api.download(sel.asset, (p) => progress.push(p));
        assert.strictEqual(sha(got), sha(mock.files['adsbee_1090-0.9.1-rc6/adsbee_1090.ota']));
        assert.strictEqual(progress.at(-1), 100);
        assert.ok(mock.requests.includes('GET /api/fw/v1/files/adsbee_1090-0.9.1-rc6/adsbee_1090.ota'));
    } finally {
        await mock.close();
    }
});

test('a device without valid keys gets no-keys', async () => {
    const mock = await mock1090();
    try {
        const api = new fr.FirmwareApi({ base: mock.base });
        const info = fr.parseDeviceInfo(INFO_1090.map((l) => l.replace(/^OTA Key 0: .*/, 'OTA Key 0: bogus')), '1090');
        await assert.rejects(api.auth(info), (e) => e.code === 'no-keys');
        await assert.rejects(api.auth(Object.assign({}, fr.parseDeviceInfo(INFO_1090, '1090'), { part_code: 'WRONG-1421' })),
                             (e) => e.code === 'wrong-product');
    } finally {
        await mock.close();
    }
});

test('an expired token is renewed once', async () => {
    const mock = await mock1090();
    try {
        const api = new fr.FirmwareApi({ base: mock.base });
        await api.auth(fr.parseDeviceInfo(INFO_1090, '1090'));
        // The server forgets the token: the first try gets 401, auth runs again, the retry succeeds.
        mock.tokens.clear();
        const l = await api.releases('stable');
        assert.strictEqual(l.recommended, '0.9.0');
        assert.strictEqual(mock.authBodies.length, 2);
    } finally {
        await mock.close();
    }
});

test('rate limit, offline and bad responses', async () => {
    const mock = await mock1090();
    try {
        const api = new fr.FirmwareApi({ base: mock.base });
        mock.rateLimited = true;
        await assert.rejects(api.auth(fr.parseDeviceInfo(INFO_1090, '1090')),
                             (e) => e.code === 'rate-limit' && e.retryAt > Date.now());
    } finally {
        await mock.close();
    }
    const dead = new fr.FirmwareApi({ base: 'http://127.0.0.1:9/api/fw/v1/' });
    await assert.rejects(dead.auth(fr.parseDeviceInfo(INFO_1090, '1090')), (e) => e.code === 'offline');
    const hang = new fr.FirmwareApi({ base: 'http://x/api/', timeoutMs: 50, fetchFn: (url, o) =>
        new Promise((_, reject) => o.signal.addEventListener('abort', () => reject(new Error('aborted')))) });
    await assert.rejects(hang.auth(fr.parseDeviceInfo(INFO_1090, '1090')), (e) => e.code === 'offline');
    const garbage = new fr.FirmwareApi({ base: 'http://x/api/', fetchFn: async () => ({
        ok: true, status: 200, headers: { get: () => null }, json: async () => ({ nope: 1 }) }) });
    await assert.rejects(garbage.auth(fr.parseDeviceInfo(INFO_1090, '1090')), (e) => e.code === 'bad-response');
});

test('download checks size and SHA-256', async () => {
    const mock = await mock1090();
    try {
        const api = new fr.FirmwareApi({ base: mock.base });
        await api.auth(fr.parseDeviceInfo(INFO_1090, '1090'));
        const asset = (await api.releases('rc')).releases[0].assets.find((a) => a.kind === 'ota');
        await assert.rejects(api.download({ ...asset, sha256: '0'.repeat(64) }), (e) => e.code === 'checksum');
        await assert.rejects(api.download({ ...asset, size: asset.size + 1 }), (e) => e.code === 'download');
        await assert.rejects(api.download({ ...asset, url: 'files/nope/x.ota' }), (e) => e.code === 'download');
        assert.strictEqual((await api.download({ ...asset, sha256: null })).length, asset.size);
    } finally {
        await mock.close();
    }
});

test('SHA-256 in JS matches node crypto, including padding edge lengths', () => {
    for (const n of [0, 1, 55, 56, 63, 64, 65, 119, 120, 1000, 70001]) {
        const b = bytes(n, 7);
        assert.strictEqual(fr.sha256Js(b), sha(b), `length ${n}`);
    }
    assert.strictEqual(fr.sha256Js(new TextEncoder().encode('abc')),
                       'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
});

// ── Nav bar chip ──────────────────────────────────────────────────────────────────────────────────────────────────────

test('chipState: dot color per state', () => {
    const { chipState, ReleaseError, parseVersion } = fr;
    const sel = (state, v = '0.9.1') => ({ state, version: parseVersion(v) });
    assert.strictEqual(chipState({ phase: '', err: null, stable: null, rc: null }).dot, 'off');
    assert.strictEqual(chipState({ phase: 'check', err: null, stable: sel('update'), rc: null }).dot, 'busy');
    assert.match(chipState({ phase: 'install', err: null, stable: null, rc: null }).tip, /Installing/);
    for (const s of ['current', 'ahead', 'none']) {
        assert.strictEqual(chipState({ phase: '', err: null, stable: sel(s), rc: null }).dot, 'ok', s);
    }
    for (const s of ['update', 'rollback', 'unknown']) {
        assert.strictEqual(chipState({ phase: '', err: null, stable: sel(s), rc: null }).dot, 'new', s);
    }
    assert.match(chipState({ phase: '', err: null, stable: sel('update'), rc: null }).tip, /0\.9\.1/);
    assert.match(chipState({ phase: '', err: null, stable: sel('rollback', '0.9.0'), rc: null }).tip, /Rollback to 0\.9\.0/);
    // An RC offer counts only when the page passes it (RC box ticked).
    const rc = chipState({ phase: '', err: null, stable: sel('current'), rc: sel('update', '0.9.2-rc1') });
    assert.deepStrictEqual([rc.dot, /0\.9\.2-rc1/.test(rc.tip)], ['new', true]);
    // Gray when the service can't be asked; red when a check or an install went wrong.
    for (const code of ['offline', 'no-keys', 'rate-limit', 'device']) {
        assert.strictEqual(chipState({ phase: '', err: new ReleaseError(code, 'm'), stable: sel('update'), rc: null }).dot, 'off', code);
    }
    for (const code of ['checksum', 'download', 'bad-response', 'wrong-product', 'install']) {
        const s = chipState({ phase: '', err: new ReleaseError(code, 'why'), stable: sel('current'), rc: null });
        assert.deepStrictEqual([s.dot, s.tip], ['bad', 'why'], code);
    }
});

test('listing cache: per service, receiver and running version; expires', () => {
    const { cacheKey, readCache, CACHE_KEY, CHECK_INTERVAL_MS } = fr;
    const info = { product: 'adsbee_1090', unique_id: 'E46478B14B4C1F23', versions: { rp2040: '0.9.1-rc5' } };
    const key = cacheKey('https://adsbee.aero/api/fw/v1/', info);
    const data = {};
    const store = { getItem: (k) => (k in data ? data[k] : null) };
    assert.strictEqual(readCache(store, key, 1000), null);
    data[CACHE_KEY] = JSON.stringify({ key, at: 1000, stable: { releases: [] }, rc: null });
    assert.strictEqual(readCache(store, key, 1000 + CHECK_INTERVAL_MS - 1).at, 1000);
    assert.strictEqual(readCache(store, key, 1000 + CHECK_INTERVAL_MS), null, 'expired');
    assert.strictEqual(readCache(store, key, 500), null, 'clock went back');
    // Another firmware version, receiver or service: not this cache.
    assert.notStrictEqual(cacheKey('https://adsbee.aero/api/fw/v1/', { ...info, versions: { rp2040: '0.9.1-rc6' } }), key);
    assert.notStrictEqual(cacheKey('https://adsbee.aero/api/fw/v1/', { ...info, unique_id: 'X' }), key);
    assert.notStrictEqual(cacheKey('http://localhost:8093/', info), key);
    assert.strictEqual(readCache(store, key + 'x', 1001), null);
    data[CACHE_KEY] = '{not json';
    assert.strictEqual(readCache(store, key, 1001), null);
    assert.strictEqual(readCache({ getItem: () => { throw new Error('blocked'); } }, key, 1001), null);
});

test('a held receiver is offered nothing, and says so', () => {
    const listing = { recommended: null, hold: true, reason: 'hold', releases: [
        { version: '0.9.1', assets: [{ kind: 'ota', name: 'adsbee_1090.ota', url: 'files/v0.9.1/adsbee_1090.ota' }] }] };
    const sel = fr.selectUpdate(listing, { product: '1090', current: '0.9.0', channel: 'stable' });
    assert.deepStrictEqual([sel.state, sel.hold], ['none', true]);
    assert.strictEqual(fr.chipState({ phase: '', err: null, stable: sel, rc: null }).dot, 'ok');
    assert.strictEqual(fr.selectUpdate({ recommended: null, releases: [] }, { product: '1090', current: '0.9.0', channel: 'stable' }).hold, false);
});

test('a pinned RC is offered on the stable channel', () => {
    const listing = { recommended: '0.9.2-rc1', pinned: true, reason: 'pin', releases: [
        { version: '0.9.2-rc1', prerelease: true, assets: [{ kind: 'ota', name: 'adsbee_1090.ota', url: 'files/v0.9.2-rc1/adsbee_1090.ota' }] }] };
    const sel = fr.selectUpdate(listing, { product: '1090', current: '0.9.1', channel: 'stable' });
    assert.deepStrictEqual([sel.state, fr.formatVersion(sel.version)], ['update', '0.9.2-rc1']);
});
