// Tests for the 1090 console's "Open device page" link: the URL built from the host box and the browsers known to
// block ws:// from the https console. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/device_page.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const REPO = path.join(__dirname, '../../../../../../..');
const CONSOLE = fs.readFileSync(path.join(REPO, 'software/adsbee_1090_console/adsbee_1090_console.html'), 'utf8');

function load() {
    const ctx = vm.createContext({});
    for (const name of ['LABEL_HOSTNAME', 'DEVICE_PAGE']) {
        const m = new RegExp(`// BEGIN ${name}[\\s\\S]*?// END ${name}`).exec(CONSOLE);
        assert.ok(m, `${name} block missing from the 1090 console`);
        vm.runInContext(m[0], ctx);
    }
    vm.runInContext('this.fns = { devicePageUrl, networkBlockedByPage, normalizeHost };', ctx);
    return ctx.fns;
}
const { devicePageUrl, networkBlockedByPage, normalizeHost } = load();

test('hostnames and IPs', () => {
    assert.strictEqual(devicePageUrl('adsbee.local'), 'http://adsbee.local/');
    assert.strictEqual(devicePageUrl('attic-bee'), 'http://attic-bee/');
    assert.strictEqual(devicePageUrl('adsbee1090-0250101000001.local'), 'http://adsbee1090-0250101000001.local/');
    assert.strictEqual(devicePageUrl('ADSBee.Local.'), 'http://ADSBee.Local./');
    assert.strictEqual(devicePageUrl('192.168.1.20'), 'http://192.168.1.20/');
    assert.strictEqual(devicePageUrl('fe80::1'), 'http://[fe80::1]/');
    assert.strictEqual(devicePageUrl('[fe80::1]:8080'), 'http://[fe80::1]:8080/');
});

test('keeps the port, drops scheme, path, query and whitespace', () => {
    assert.strictEqual(devicePageUrl('192.168.1.20:8080'), 'http://192.168.1.20:8080/');
    assert.strictEqual(devicePageUrl('adsbee.local:80'), 'http://adsbee.local:80/');
    assert.strictEqual(devicePageUrl('  adsbee.local \n'), 'http://adsbee.local/');
    assert.strictEqual(devicePageUrl('\t192.168.1.20:8080 '), 'http://192.168.1.20:8080/');
    assert.strictEqual(devicePageUrl('http://adsbee.local/'), 'http://adsbee.local/');
    assert.strictEqual(devicePageUrl('ws://adsbee.local:81/console'), 'http://adsbee.local:81/');
    assert.strictEqual(devicePageUrl('https://192.168.1.20/x?y=1#z'), 'http://192.168.1.20/');
});

test('a serial label gives its default hostname', () => {
    assert.strictEqual(devicePageUrl('000000000A-20250101-000001'), 'http://adsbee1090-0250101000001.local/');
    assert.strictEqual(devicePageUrl(' 010250002F-20251103-A70042\r\n'), 'http://adsbee1090-0251103a70042.local/');
});

test('nothing usable gives null', () => {
    for (const bad of ['', '   ', null, undefined, 'http://', 'ads bee.local', 'adsbee.local:port', 'adsbee.local:123456',
        'user@adsbee.local', 'javascript:alert(1)', '<b>x</b>', 'adsbee..local', '-', 'adsbee-.local', '-adsbee.local']) {
        assert.strictEqual(devicePageUrl(bad), null, `"${bad}"`);
    }
});

test('the connect path uses the same host normalization', () => {
    assert.strictEqual(normalizeHost(' http://adsbee.local:80/x '), 'adsbee.local:80');
    assert.match(CONSOLE, /_normalizeHost\(raw\) \{\s*return normalizeHost\(raw\);/);
});

const UA = {
    iphoneSafari: 'Mozilla/5.0 (iPhone; CPU iPhone OS 18_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/18.5 Mobile/15E148 Safari/604.1',
    iphoneChrome: 'Mozilla/5.0 (iPhone; CPU iPhone OS 18_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) CriOS/138.0.7204.156 Mobile/15E148 Safari/604.1',
    iphoneFirefox: 'Mozilla/5.0 (iPhone; CPU iPhone OS 18_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) FxiOS/141.0 Mobile/15E148 Safari/605.1.15',
    macSafari: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15 (KHTML, like Gecko) Version/26.5 Safari/605.1.15',
    macChrome: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/153.0.0.0 Safari/537.36',
    macEdge: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/153.0.0.0 Safari/537.36 Edg/153.0.0.0',
    macFirefox: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10.15; rv:144.0) Gecko/20100101 Firefox/144.0',
    androidChrome: 'Mozilla/5.0 (Linux; Android 15; Pixel 9) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/153.0.0.0 Mobile Safari/537.36',
    androidWebView: 'Mozilla/5.0 (Linux; Android 15; Pixel 9; wv) AppleWebKit/537.36 (KHTML, like Gecko) Version/4.0 Chrome/153.0.0.0 Mobile Safari/537.36',
    androidSamsung: 'Mozilla/5.0 (Linux; Android 15; SM-S921U) AppleWebKit/537.36 (KHTML, like Gecko) SamsungBrowser/28.0 Chrome/130.0.0.0 Mobile Safari/537.36',
};

test('WebKit browsers on https are known to block the sockets', () => {
    for (const k of ['iphoneSafari', 'iphoneChrome', 'iphoneFirefox', 'macSafari']) {
        assert.strictEqual(networkBlockedByPage('https:', UA[k], 5), true, k);
    }
    // iPadOS asks for desktop pages with a Mac user agent; touch points give it away.
    assert.strictEqual(networkBlockedByPage('https:', UA.macSafari.replace('Version/26.5 ', ''), 5), true, 'iPad');
});

test('other browsers, and any http:// or file:// page, are not', () => {
    for (const k of ['macChrome', 'macEdge', 'macFirefox', 'androidChrome', 'androidWebView', 'androidSamsung']) {
        assert.strictEqual(networkBlockedByPage('https:', UA[k], 0), false, k);
    }
    for (const k of Object.keys(UA)) {
        assert.strictEqual(networkBlockedByPage('http:', UA[k], 5), false, `http: ${k}`);
        assert.strictEqual(networkBlockedByPage('file:', UA[k], 5), false, `file: ${k}`);
    }
    assert.strictEqual(networkBlockedByPage('https:', '', 0), false);
});

test('the console wires the link and the hint', () => {
    for (const id of ['device-page-link', 'device-page-hint-btn']) {
        const m = new RegExp(`<a id="${id}"[^>]*>`).exec(CONSOLE);
        assert.ok(m, `${id} missing`);
        assert.match(m[0], /target="_blank"/, `${id} opens a new tab`);
        assert.match(m[0], /rel="noopener"/, `${id} rel=noopener`);
        assert.match(m[0], /title="Opens the device's own web page \(same network\)"/, `${id} tooltip`);
    }
    assert.match(CONSOLE, /connectionManager\.connectToHost\(host, \{ fromLabel: true \}\)/, 'hint after a scan');
    assert.match(CONSOLE, /this\._trackNetworkHealth\(open, state === 'CLOSED'\)/, 'hint after a failed connect');
    assert.match(CONSOLE, /if \(this\.blockedByPage\) this\._showHint\('blocked'\);/, 'hint where the page is blocked');
});
