// Checks that the console terminal shows the receiver's welcome banner and the firmware update check's
// AT+DEVICE_INFO? reply on first load, and that the check still reads the version. Runs in CI (web_ui_test job).
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

test('the terminal shows the banner and the AT+DEVICE_INFO? reply during the first update check', () => {
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
    new ConsoleWebSocket('ws://testbee.local/console');
    socket.emit('open', {});
    socket.emit('message', { data: BANNER });
    socket.emit('message', { data: REPLY });
    socket.emit('message', { data: LOG });
    assert.strictEqual(shown.join(''), BANNER + REPLY + LOG);
});

test('the update check reads the running version from the reply', () => {
    const lines = REPLY.split('\r\n').filter((l) => FR.DEVICE_INFO_LINE.test(l));
    const info = FR.parseDeviceInfo(lines, '1090');
    assert.strictEqual(FR.runningVersion(info), '0.9.1-rc11');
    assert.strictEqual(info.unique_id, 'E46478B14B4C1F23');
});

for (const page of ['firmware/adsbee_1090/esp/main/server/web/index.html',
    'firmware/adsbee_1090/esp/main/server/web/adsbee.js',
    'software/adsbee_1090_console/adsbee_1090_console.html',
    'software/adsbee_1421_console/adsbee_1421_console.html']) {
    test(`${page} does not hide console output during the update check`, () => {
        const src = fs.readFileSync(path.join(REPO, page), 'utf8');
        assert.ok(!/muteUntil|hideDeviceInfoReply|deviceInfoFilter/.test(src), 'terminal mute or filter is back');
    });
}
