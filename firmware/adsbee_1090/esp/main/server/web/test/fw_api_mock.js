// Mock of the adsbee.aero firmware service (API v1, adsbee-aero docs/firmware_api.md) for the web UI tests and bench
// runs. In-memory; accepts a device when ota_keys[0] === `valid-<unique_id>`.
//
//   const mock = await startMock({ releases, recommended, files });   // mock.base = 'http://127.0.0.1:<port>/api/fw/v1/'
//   node test/fw_api_mock.js <port> <release dir> [recommended]       // serves <dir>/<tag>/<file>; logs requests
//
// releases: [{tag, version, prerelease, notes_url, assets: [{name, kind}]}]; sizes and SHA-256 come from files.
'use strict';
const http = require('node:http');
const crypto = require('node:crypto');
const fs = require('node:fs');
const path = require('node:path');

const PREFIX = '/api/fw/v1/';
const CORS = {
    'Access-Control-Allow-Origin': '*',
    'Access-Control-Allow-Methods': 'GET, POST, HEAD, OPTIONS',
    'Access-Control-Allow-Headers': 'Authorization, Content-Type, Range',
    'Access-Control-Expose-Headers': 'Retry-After, Content-Length, Content-Range, ETag, Accept-Ranges, X-Request-ID',
};

function startMock({ releases = [], recommended = null, files = {}, port = 0, tokenTtlMs = 3600000,
                     log = () => {} } = {}) {
    const state = { releases, recommended, files, tokens: new Map(), requests: [], authBodies: [],
                    rateLimited: false };
    const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
    const isPre = (r) => /-(rc|alpha|beta)\d*$/.test(r.version);
    const listing = (channel) => {
        const sorted = [...state.releases].sort((a, b) => cmp(b.version, a.version));
        const list = channel === 'rc' ? sorted
            : sorted.filter((r) => !isPre(r) || r.version === state.recommended);
        return list.map((r) => ({
            version: r.version, tag: r.tag, prerelease: isPre(r), published_at: r.published_at || null,
            notes_url: r.notes_url || null,
            assets: r.assets.filter((a) => state.files[`${r.tag}/${a.name}`]).map((a) => {
                const b = state.files[`${r.tag}/${a.name}`];
                return { name: a.name, kind: a.kind, size: b.length, sha256: a.sha256 || sha(b),
                         url: `files/${r.tag}/${a.name}` };
            }),
        }));
    };

    const server = http.createServer(async (req, res) => {
        const url = new URL(req.url, 'http://x');
        const send = (status, body, headers = {}) => {
            const isJson = body !== undefined && !(body instanceof Buffer);
            res.writeHead(status, Object.assign({}, CORS, isJson ? { 'Content-Type': 'application/json' } : {}, headers));
            res.end(isJson ? JSON.stringify(body) : body);
        };
        state.requests.push(`${req.method} ${url.pathname}${url.search}`);
        log(`${req.method} ${url.pathname}${url.search}`);
        if (req.method === 'OPTIONS') return send(204, undefined);
        // Test control: /__set?recommended=<version|null>&rate_limited=0|1
        if (url.pathname === '/__set') {
            if (url.searchParams.has('recommended')) {
                const r = url.searchParams.get('recommended');
                state.recommended = r === 'null' ? null : r;
            }
            if (url.searchParams.has('rate_limited')) state.rateLimited = url.searchParams.get('rate_limited') === '1';
            return send(200, { recommended: state.recommended, rate_limited: state.rateLimited });
        }
        if (!url.pathname.startsWith(PREFIX)) return send(404, { error: 'not_found' });
        if (state.rateLimited) return send(429, { error: 'rate_limited' }, { 'Retry-After': '120' });
        const route = url.pathname.slice(PREFIX.length);

        if (route === 'auth' && req.method === 'POST') {
            let body;
            try {
                let raw = '';
                for await (const c of req) raw += c;
                body = JSON.parse(raw);
            } catch (e) {
                return send(400, { error: 'bad_request', detail: 'invalid JSON' });
            }
            state.authBodies.push(body);
            if (!body.unique_id || !Array.isArray(body.ota_keys) || !/^adsbee_(1090|1421)$/.test(body.product)) {
                return send(400, { error: 'bad_request', detail: 'missing fields' });
            }
            if (body.ota_keys[0] !== `valid-${body.unique_id}`) return send(401, { error: 'bad_key' });
            const token = crypto.randomBytes(16).toString('hex');
            state.tokens.set(token, Date.now() + tokenTtlMs);
            return send(200, { token, expires_in: Math.round(tokenTtlMs / 1000) });
        }

        const m = /^Bearer (\w+)$/.exec(req.headers.authorization || '');
        const exp = m && state.tokens.get(m[1]);
        if (!exp) return send(401, { error: 'bad_token' });
        if (exp < Date.now()) {
            state.tokens.delete(m[1]);
            return send(401, { error: 'token_expired' });
        }

        if (route === 'releases' && req.method === 'GET') {
            return send(200, { product: 'adsbee_1090', current: null, recommended: state.recommended,
                               releases: listing(url.searchParams.get('channel')) });
        }
        const f = /^files\/([^/]+)\/([^/]+)$/.exec(route);
        if (f && (req.method === 'GET' || req.method === 'HEAD')) {
            const b = state.files[`${f[1]}/${f[2]}`];
            if (!b) return send(404, { error: 'not_found' });
            const headers = { 'Content-Type': 'application/octet-stream', 'Content-Length': b.length,
                              'Accept-Ranges': 'bytes', ETag: `"${sha(b)}"` };
            return send(200, req.method === 'HEAD' ? Buffer.alloc(0) : b, headers);
        }
        if (route === 'checkin' && req.method === 'POST') return send(204, undefined);
        return send(404, { error: 'not_found' });
    });

    return new Promise((resolve) => server.listen(port, '127.0.0.1', () => {
        const base = `http://127.0.0.1:${server.address().port}${PREFIX}`;
        resolve(Object.assign(state, { server, base, close: () => new Promise((r) => server.close(r)) }));
    }));
}

function cmp(a, b) {
    const p = (s) => { const m = /^(\d+)\.(\d+)\.(\d+)(?:-rc(\d+))?/.exec(s); return [+m[1], +m[2], +m[3], m[4] ? +m[4] : 1e9]; };
    const x = p(a), y = p(b);
    for (let i = 0; i < 4; i++) if (x[i] !== y[i]) return x[i] - y[i];
    return 0;
}

/** Releases from a directory of <tag>/<file> (tags adsbee_<product>-<version>). */
function loadDir(dir) {
    const releases = [];
    const files = {};
    for (const tag of fs.readdirSync(dir)) {
        const m = /^adsbee_(1090|1421)-(.+)$/.exec(tag);
        if (!m) continue;
        const assets = [];
        for (const name of fs.readdirSync(path.join(dir, tag))) {
            files[`${tag}/${name}`] = fs.readFileSync(path.join(dir, tag, name));
            assets.push({ name, kind: name.endsWith('.ota') ? 'ota' : name.endsWith('.hex') ? 'hex' : 'other' });
        }
        releases.push({ tag, version: m[2], assets,
                        notes_url: `https://github.com/PantsForBirds/adsbee/releases/tag/${tag}` });
    }
    return { releases, files };
}

module.exports = { startMock, loadDir };

if (require.main === module) {
    const [port, dir, recommended] = process.argv.slice(2);
    const { releases, files } = loadDir(dir);
    startMock({ releases, files, recommended: recommended || null, port: +port,
                log: (l) => console.log(new Date().toISOString(), l) })
        .then((m) => console.log(`mock firmware service at ${m.base}: ${releases.map((r) => r.tag).join(' ')}`));
}
