// Checks that the console terminal shows the receiver's welcome banner on first load while the firmware update check
// reads AT+DEVICE_INFO?, and keeps only that reply out of the terminal. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/console_first_load.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const FR = require('../firmware_release.js');

const REPO = path.join(__dirname, '../../../../../../..');
const BANNER = '\r\n █████  ██████ \r\n\r\nFirmware Version: 0.9.1-rc11\r\nAP SSID: ADSBee1090\r\n';
const REPLY = 'Part Code: 010250002D-20250613-020079\r\nRP2040 Flash Unique ID: E46478B14B4C1F23\r\n' +
    'RP2040 Firmware Version: 0.9.1-rc11\r\nOTA Key 0: 8f2b\r\n1090MHz RF Frontend Version: 1\r\n' +
    'ESP32 Firmware Version: 0.9.1-rc11\r\nESP32 Heap: 106 KB free, 43 KB largest block\r\n';
const LOG = 'CoProcessor: \x1b[31mESP32 >> [CommsManager::IPWANTask] Timed out.\x1b[0m\r\n';

test('deviceInfoFilter hides only the AT+DEVICE_INFO? reply', () => {
    let t = 0;
    const f = FR.deviceInfoFilter(() => t);
    assert.strictEqual(f.filter(REPLY), REPLY, 'shown when not hiding');
    f.hide(10000);
    assert.strictEqual(f.filter(BANNER), BANNER);
    assert.strictEqual(f.filter(REPLY + LOG), LOG);
    // Lines split across messages.
    assert.strictEqual(f.filter('Part Co'), '');
    assert.strictEqual(f.filter('de: X\r\nhello\r\nESP32 Hea'), 'hello\r\n');
    assert.strictEqual(f.flush(), 'ESP32 Hea');
    // A partial line that can't become a reply line shows at once.
    assert.strictEqual(f.filter('> '), '> ');
    t = 10001;
    assert.ok(!f.hiding());
    assert.strictEqual(f.filter(REPLY), REPLY, 'shown again after the window');
});

test('the terminal shows the banner sent on connect during the first update check', async () => {
    const shown = [];
    let socket = null;
    class FakeWebSocket {
        constructor(url) { this.url = url; this.readyState = 0; this.listeners = {}; socket = this; }
        addEventListener(type, fn) { (this.listeners[type] ||= []).push(fn); }
        emit(type, ev) { for (const fn of this.listeners[type] || []) fn(ev); }
        send() {}
        close() {}
    }
    FakeWebSocket.OPEN = 1;
    FakeWebSocket.CLOSED = 3;
    Object.assign(globalThis, {
        WebSocket: FakeWebSocket, AdsbeeFirmwareRelease: FR, WS_CONFIG: { reconnectDelayMs: 5000 },
        appendToTerminal: (text) => shown.push(text),
    });
    const { ConsoleWebSocket } = require('../adsbee.js');
    // Page load: the terminal socket opens and the update check starts before the device answers.
    const ws = new ConsoleWebSocket('ws://testbee.local/console');
    ws.hideDeviceInfoReply(10000);
    socket.emit('open', {});
    socket.emit('message', { data: BANNER });
    socket.emit('message', { data: REPLY });
    socket.emit('message', { data: LOG });
    clearTimeout(ws._flushTimer);
    const text = shown.join('');
    assert.ok(text.includes('Firmware Version: 0.9.1-rc11'), 'banner shown');
    assert.ok(text.includes('IPWANTask'), 'log line shown');
    assert.ok(!text.includes('Part Code'), 'AT+DEVICE_INFO? reply hidden');
});

for (const page of ['firmware/adsbee_1090/esp/main/server/web/index.html',
    'software/adsbee_1090_console/adsbee_1090_console.html']) {
    test(`${page} hides the update check's reply instead of muting the terminal`, () => {
        const html = fs.readFileSync(path.join(REPO, page), 'utf8');
        assert.ok(!html.includes('muteUntil'), 'time-based terminal mute is back');
        assert.ok(html.includes('hideDeviceInfoReply('), 'update check does not hide its reply');
    });
}

test('the 1090 console terminal socket filters like the device page', () => {
    const html = fs.readFileSync(path.join(REPO, 'software/adsbee_1090_console/adsbee_1090_console.html'), 'utf8');
    const js = fs.readFileSync(path.join(__dirname, '../adsbee.js'), 'utf8');
    const method = (src) => /\n {4}hideDeviceInfoReply\(ms\) \{[\s\S]*?\n {4}\}\n/.exec(src)?.[0];
    assert.ok(method(js), 'hideDeviceInfoReply not found in adsbee.js');
    assert.strictEqual(method(html), method(js));
    assert.ok(html.includes('const shown = this.replyFilter.filter(text);'));
});
