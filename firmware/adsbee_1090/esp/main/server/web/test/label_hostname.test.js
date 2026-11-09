// Tests for the 1090 console's serial label to hostname mapping (Scan button, typed or pasted serials) and its
// agreement with the firmware's default hostname. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/label_hostname.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const REPO = path.join(__dirname, '../../../../../../..');
const CONSOLE = fs.readFileSync(path.join(REPO, 'software/adsbee_1090_console/adsbee_1090_console.html'), 'utf8');

function loadLabelToHostname() {
    const m = /\/\/ BEGIN LABEL_HOSTNAME[\s\S]*?\/\/ END LABEL_HOSTNAME/.exec(CONSOLE);
    assert.ok(m, 'LABEL_HOSTNAME block missing from the 1090 console');
    const ctx = vm.createContext({});
    vm.runInContext(`${m[0]}\nthis.labelToHostname = labelToHostname;`, ctx);
    return ctx.labelToHostname;
}
const labelToHostname = loadLabelToHostname();

test('matches the firmware default hostname (host_test vector)', () => {
    // settings.hh copies the default SSID into the hostname; test_settings.cc pins the SSID for one part code.
    const src = fs.readFileSync(path.join(REPO, 'firmware/adsbee_1090/pico/host_test/test_settings.cc'), 'utf8');
    const t = /DeviceInfoGetDefaultSSIDAndUID[\s\S]*?part_code, \(const char\*\)"([^"]+)"[\s\S]*?EXPECT_STREQ\(ssid_buf, \(const char\*\)"([^"]+)"\)/
        .exec(src);
    assert.ok(t, 'firmware test vector not found');
    assert.strictEqual(labelToHostname(t[1]), `${t[2].toLowerCase()}.local`);
    const settings = fs.readFileSync(path.join(REPO, 'firmware/common/settings/settings.hh'), 'utf8');
    assert.match(settings, /strncpy\(core_network_settings\.hostname, core_network_settings\.wifi_ap_ssid/,
        'firmware no longer uses the default SSID as the hostname: update labelToHostname');
});

test('converts valid labels', () => {
    assert.strictEqual(labelToHostname('000000000A-20250101-000001'), 'adsbee1090-0250101000001.local');
    assert.strictEqual(labelToHostname('010250002F-20251103-000123'), 'adsbee1090-0251103000123.local');
    // Base-36 vendor code: the firmware copies the characters as they are.
    assert.strictEqual(labelToHostname('010250002F-20251103-A70042'), 'adsbee1090-0251103a70042.local');
});

test('accepts lowercase, whitespace and scanner line endings', () => {
    const want = 'adsbee1090-0250101000001.local';
    assert.strictEqual(labelToHostname('000000000a-20250101-000001'), want);
    assert.strictEqual(labelToHostname('  000000000A-20250101-000001 '), want);
    assert.strictEqual(labelToHostname('000000000A-20250101-000001\r\n'), want);
    assert.strictEqual(labelToHostname('\t000000000A-20250101-000001\n'), want);
});

test('finds a label wrapped in other text', () => {
    const want = 'adsbee1090-0250101000001.local';
    assert.strictEqual(labelToHostname('https://example.com/device?serial=000000000A-20250101-000001'), want);
    assert.strictEqual(labelToHostname('SN: 000000000A-20250101-000001'), want);
});

test('rejects anything that is not a label', () => {
    for (const bad of [
        '', null, undefined, 'adsbee.local', '192.168.1.20', 'adsbee1090-0250101000001.local',
        'http://adsbee1090-0250101000001.local/',
        '00000000A-20250101-000001',      // 8-digit part number
        '0000000000A-20250101-000001',    // 10-digit part number
        '0000000001-20250101-000001',     // no revision letter
        '000000000A-2025011-000001',      // 7-digit date
        '000000000A-20250101-00001',      // 5-character tail
        '000000000A-20250101-0000012',    // 7-character tail
        '000000000A-20250101-00000A',     // letter in the counter
        '000000000A20250101000001',       // no dashes
    ]) {
        assert.strictEqual(labelToHostname(bad), null, `"${bad}"`);
    }
});

test('the console wires the scanner and the vendored decoder', () => {
    assert.match(CONSOLE, /<button id="scan-btn"/);
    assert.match(CONSOLE, /BEGIN VENDORED jsQR 1\.4\.0[\s\S]*Apache-2\.0[\s\S]*END VENDORED jsQR 1\.4\.0/);
    assert.match(CONSOLE, /const host = fromLabel \|\| this\._normalizeHost\(raw\);/, 'typed serials are converted on connect');
    assert.match(CONSOLE, /getUserMedia\(\s*\{ video: \{ facingMode: \{ ideal: 'environment' \} \}/, 'rear camera on phones');
    assert.match(CONSOLE, /close\(\) \{\s*this\._stopCamera\(\);/, 'closing the dialog stops the camera');
    assert.match(CONSOLE, /this\.close\(\);\s*appendToTerminal\(`Serial label/, 'a read label stops the camera before connecting');
});
