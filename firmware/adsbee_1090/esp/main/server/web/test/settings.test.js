// Tests for the settings GUI engine in settings.js against a scripted AT transport. These are run
// locally when changing settings.js; CI only runs settings_drift.test.js.
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/settings.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');

// ── Minimal DOM: just what SettingsEngine touches ──
class FakeElement {
    constructor(doc, tag) {
        this.doc = doc; this.tagName = tag.toUpperCase(); this.children = []; this.listeners = {};
        this.dataset = {}; this.style = {}; this.textContent = ''; this.value = ''; this.checked = false;
        this.disabled = false; this._id = ''; this._classes = new Set();
        const self = this;
        this.classList = {
            add: (c) => self._classes.add(c),
            toggle: (c, on) => { if (on === undefined ? !self._classes.has(c) : on) self._classes.add(c); else self._classes.delete(c); },
            contains: (c) => self._classes.has(c),
        };
    }
    get id() { return this._id; }
    set id(v) { this._id = v; this.doc.byId.set(v, this); }
    get className() { return [...this._classes].join(' '); }
    set className(v) { this._classes = new Set(v.split(/\s+/).filter(Boolean)); }
    appendChild(c) { this.children.push(c); return c; }
    addEventListener(t, f) { (this.listeners[t] ||= []).push(f); }
    querySelectorAll(sel) {
        const out = [];
        const walk = (el) => { for (const c of el.children) { if (c.tagName === 'INPUT' && (sel === 'input' || c.type === 'checkbox')) out.push(c); walk(c); } };
        walk(this);
        return out;
    }
}
class FakeDocument {
    constructor() { this.byId = new Map(); }
    createElement(tag) { return new FakeElement(this, tag); }
    createTextNode(t) { return new FakeElement(this, '#text'); }
    getElementById(id) { return this.byId.get(id) || null; }
}

global.document = new FakeDocument();
global.confirm = () => true;
const { SettingsEngine, SETTINGS_SCHEMA_1090 } = require('../settings.js');

// The AT+SETTINGS?JSON line from an ADSBee 1090 running 0.9.1-rc9.
const DUMP = 'SETTINGS={"ASTERIX_SAC_SIC":[0,0],"AUTO_UPDATE":[0],"BAUD_RATE":[115207,9600],"BIAS_TEE_ENABLE":[0,0],"ETHERNET":[1],"ESP32_ENABLE":[1],' +
    '"FEED_ENABLE":[1],"FILTER_ALTITUDE_FT":["OFF",0,10000],"FILTER_DISTANCE_NM":[0,100.0],"SWARM":[0,"https://adsbee.aero",""],"GNSS":[0,"NONE",0],"HOSTNAME":["test-bee"],"IP_MODE":["DUAL"],"LED_ENABLE":[1],"LOG_LEVEL":["WARNINGS"],' +
    '"MAVLINK_ID":[1,156],"NTP_SERVER":["pool.ntp.org"],"PROTOCOL_OUT":["NONE","NONE"],"REMOTE_ID":[0,7,"0x00"],' +
    '"REMOTE_ID_TX":[0,7,"",1,2,"","0x0000"],"RX_ENABLE":[1,1],' +
    '"RX_POSITION":["LOWEST","OK",37.000000,-122.000000,-50,125,317.7,74,"000000"],"SUBG_ENABLE":["1"],"SUBG_MODE":["ADSL_RX"],' +
    '"TL_OFFSET":[600,"-104 dBm"],"WATCHDOG":[10],"WIFI_AP":[0,"test-bee","yummyflowers",1],"WIFI_STA":[0,""],' +
    '"WIFI_STA_NETWORK":["office",""]}';

// Scripted transport: handler(cmd, opts, n) returns body lines or throws; records every command.
function makeTransport(handler) {
    const t = { sent: [], counts: {} };
    t.sendCommand = async (cmd, opts) => {
        t.sent.push(cmd);
        t.counts[cmd] = (t.counts[cmd] || 0) + 1;
        return handler(cmd, opts, t.counts[cmd]);
    };
    return t;
}

function makeEngine(transport, bulk = true) {
    global.document = new FakeDocument();
    const mk = (tag) => document.createElement(tag);
    const engine = new SettingsEngine({
        schema: SETTINGS_SCHEMA_1090,
        transport,
        bulkQuery: bulk ? { command: 'AT+SETTINGS?JSON', expect: /^SETTINGS=/, timeoutMs: 50 } : undefined,
        formEl: mk('div'), statusEl: mk('span'), saveBtn: mk('button'), refreshBtn: mk('button'),
    });
    engine.render();
    return engine;
}

const input = (cmd, field) => document.getElementById(`settings-input-${cmd}-${field}`);
const timeout = (cmd) => { throw new Error(`Timeout waiting for response to ${cmd}.`); };

test('every GUI setting is in the dump and loads from one command', async () => {
    const json = JSON.parse(DUMP.slice('SETTINGS='.length));
    for (const entry of SETTINGS_SCHEMA_1090) assert.ok(entry.cmd in json, `${entry.cmd} missing from AT+SETTINGS?JSON`);

    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [DUMP];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON']);
    assert.strictEqual(engine.statusEl.textContent, 'Settings loaded.');
    for (const entry of SETTINGS_SCHEMA_1090) assert.strictEqual(engine.readOk[entry.cmd], true, entry.cmd);

    assert.strictEqual(input('HOSTNAME', 'hostname').value, 'test-bee');
    assert.strictEqual(input('BAUD_RATE', 'COMMS_UART').value, '115207');
    assert.strictEqual(input('WIFI_AP', 'pwd').value, 'yummyflowers');
    assert.strictEqual(input('WIFI_AP', 'channel').value, '1');
    assert.strictEqual(input('WIFI_STA', 'pwd').value, '');  // write-only, never shown
    assert.strictEqual(input('WIFI_STA_NETWORK', 'ssid2').value, 'office');
    assert.strictEqual(input('WIFI_STA_NETWORK', 'ssid3').value, '');
    assert.strictEqual(input('SUBG_ENABLE', 'state').value, '1');
    assert.strictEqual(input('SUBG_MODE', 'mode').value, 'ADSL_RX');
    assert.strictEqual(input('LOG_LEVEL', 'level').value, 'WARNINGS');
    assert.strictEqual(input('RX_POSITION', 'source').value, 'LOWEST');
    assert.strictEqual(input('RX_POSITION', 'heading').value, '317.7');
    assert.strictEqual(input('REMOTE_ID_TX', 'ua_type').value, '2');
    assert.strictEqual(input('TL_OFFSET', 'dbm').textContent, '-104 dBm');
    assert.strictEqual(input('RX_ENABLE', 'subg').checked, true);
    assert.strictEqual(engine.dirtyEntries().length, 0);
});

test('a dump cut short is retried instead of falling back to per-setting queries', async () => {
    const t = makeTransport((cmd, opts, n) => {
        if (cmd !== 'AT+SETTINGS?JSON') throw new Error(`unexpected ${cmd}`);
        if (n === 1) return [DUMP.slice(0, DUMP.indexOf('"WIFI_STA"'))];  // tail lost on the console
        if (n === 2) timeout(cmd);                                       // OK lost
        return [DUMP];
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON', 'AT+SETTINGS?JSON', 'AT+SETTINGS?JSON']);
    assert.strictEqual(engine.statusEl.textContent, 'Settings loaded.');
});

test('a dump broken up by an interleaved log line is retried', async () => {
    const cut = DUMP.indexOf('"RX_ENABLE"');
    const t = makeTransport((cmd, opts, n) => n === 1
        ? [DUMP.slice(0, cut) + 'SPICoprocessor::SPIWaitForAck: [ESP32] Timed out while waiting for ack.']
        : [DUMP]);
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(t.sent.length, 2);
    assert.strictEqual(engine.statusEl.textContent, 'Settings loaded.');
});

test('when every dump attempt fails, reports it without querying settings one by one', async () => {
    const t = makeTransport((cmd) => timeout(cmd));
    const engine = makeEngine(t);
    await engine.refresh();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON', 'AT+SETTINGS?JSON', 'AT+SETTINGS?JSON']);
    assert.match(engine.statusEl.textContent, /^Could not read settings: Timeout/);
    for (const entry of SETTINGS_SCHEMA_1090) assert.strictEqual(engine.readOk[entry.cmd], false, entry.cmd);
    assert.strictEqual(input('HOSTNAME', 'hostname').disabled, true);
});

test('firmware that answers the dump with ERROR falls back to per-setting queries', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') throw new Error('ERROR Invalid argument JSON.');
        if (cmd === 'AT+HOSTNAME?') return ['HOSTNAME=old-bee'];
        timeout(cmd);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(t.counts['AT+SETTINGS?JSON'], 1);
    assert.strictEqual(t.sent.length, 1 + SETTINGS_SCHEMA_1090.length);
    assert.strictEqual(input('HOSTNAME', 'hostname').value, 'old-bee');
    assert.strictEqual(engine.readOk.HOSTNAME, true);
    assert.strictEqual(engine.readOk.WIFI_AP, false);
});

test('entries missing from an older dump are queried individually', async () => {
    const json = JSON.parse(DUMP.slice('SETTINGS='.length));
    delete json.FEED_ENABLE;
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return ['SETTINGS=' + JSON.stringify(json)];
        if (cmd === 'AT+FEED_ENABLE?') return ['FEED_ENABLE=0'];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON', 'AT+FEED_ENABLE?']);
    assert.strictEqual(input('FEED_ENABLE', 'en').checked, false);
    assert.strictEqual(engine.statusEl.textContent, 'Settings loaded.');
});

test('a save writes the changed setting, persists, and re-reads through the dump', async () => {
    let dump = DUMP;
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [dump];
        if (cmd === 'AT+HOSTNAME=new-bee') { dump = dump.replace('["test-bee"]', '["new-bee"]'); return []; }
        if (cmd === 'AT+SETTINGS=SAVE') return [];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    input('HOSTNAME', 'hostname').value = 'new-bee';
    assert.strictEqual(engine.dirtyEntries().length, 1);
    await engine.save();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON', 'AT+HOSTNAME=new-bee', 'AT+SETTINGS=SAVE', 'AT+SETTINGS?JSON']);
    assert.strictEqual(engine.deviceValues.HOSTNAME.hostname, 'new-bee');
    assert.strictEqual(engine.dirtyEntries().length, 0);
});

test('the distance filter loads from the dump and saves enable and distance in nm', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [DUMP];
        if (cmd === 'AT+FILTER_DISTANCE_NM=1,35.5' || cmd === 'AT+SETTINGS=SAVE') return [];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('FILTER_DISTANCE_NM', 'en').checked, false);
    assert.strictEqual(input('FILTER_DISTANCE_NM', 'nm').value, '100');
    assert.strictEqual(input('FILTER_DISTANCE_NM', 'nm').step, 0.1);

    input('FILTER_DISTANCE_NM', 'nm').value = '0';  // Below 0.1 nm: caught before anything is written.
    input('FILTER_DISTANCE_NM', 'en').checked = true;
    await engine.save();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON']);

    input('FILTER_DISTANCE_NM', 'nm').value = '35.5';
    await engine.save();
    assert.deepStrictEqual(t.sent.slice(1, 3), ['AT+FILTER_DISTANCE_NM=1,35.5', 'AT+SETTINGS=SAVE']);
});

test('the altitude filter loads from the dump and saves mode and altitudes in ft', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [DUMP];
        if (cmd === 'AT+FILTER_ALTITUDE_FT=BETWEEN,1000,18000' || cmd === 'AT+SETTINGS=SAVE') return [];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('FILTER_ALTITUDE_FT', 'mode').value, 'OFF');
    assert.strictEqual(input('FILTER_ALTITUDE_FT', 'a').value, '0');
    assert.strictEqual(input('FILTER_ALTITUDE_FT', 'b').value, '10000');

    input('FILTER_ALTITUDE_FT', 'mode').value = 'BETWEEN';
    input('FILTER_ALTITUDE_FT', 'b').value = '100001';  // Above 100000 ft: caught before anything is written.
    await engine.save();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON']);

    input('FILTER_ALTITUDE_FT', 'a').value = '1000';
    input('FILTER_ALTITUDE_FT', 'b').value = '18000';
    await engine.save();
    assert.deepStrictEqual(t.sent.slice(1, 3), ['AT+FILTER_ALTITUDE_FT=BETWEEN,1000,18000', 'AT+SETTINGS=SAVE']);
});

test('swarm reports load from the dump and save without resending the token', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [DUMP];
        if (cmd === 'AT+SWARM=1,https://adsbee.aero,0123456789abcdef0123456789abcdef' ||
            cmd === 'AT+SWARM=0,http://10.0.0.2:8000' || cmd === 'AT+SETTINGS=SAVE') return [];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('SWARM', 'en').checked, false);
    assert.strictEqual(input('SWARM', 'server').value, 'https://adsbee.aero');
    assert.strictEqual(input('SWARM', 'token').value, '');

    input('SWARM', 'en').checked = true;
    input('SWARM', 'token').value = '0123456789abcdef0123456789abcdef';
    await engine.save();
    assert.deepStrictEqual(t.sent.slice(1, 3), ['AT+SWARM=1,https://adsbee.aero,0123456789abcdef0123456789abcdef', 'AT+SETTINGS=SAVE']);

    // An untouched token is left out, so the device keeps it.
    input('SWARM', 'server').value = 'ftp://x';  // Rejected before anything is written.
    const num_sent = t.sent.length;
    await engine.save();
    assert.strictEqual(t.sent.length, num_sent);
    input('SWARM', 'server').value = 'http://10.0.0.2:8000';
    input('SWARM', 'en').checked = false;
    await engine.save();
    assert.ok(t.sent.includes('AT+SWARM=0,http://10.0.0.2:8000'), t.sent.join(' | '));
});

test('a save whose re-read fails still reports the save', async () => {
    let saved = false;
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') { if (saved) timeout(cmd); return [DUMP]; }
        if (cmd === 'AT+WATCHDOG=11') return [];
        if (cmd === 'AT+SETTINGS=SAVE') { saved = true; return []; }
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    input('WATCHDOG', 'timeout').value = '11';
    await engine.save();
    assert.match(engine.statusEl.textContent, /^Saved 1 setting\. Could not read settings: Timeout/);
});

test('WiFi passwords the device would reject are caught before anything is written', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [DUMP];
        if (cmd === 'AT+WIFI_STA=0,,12345' || cmd === 'AT+SETTINGS=SAVE') return [];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    const err = (cmd) => document.getElementById(`settings-error-${cmd}-pwd`).textContent;
    input('WIFI_AP', 'pwd').value = 'short';
    input('WIFI_STA', 'pwd').value = 'sixsix';
    await engine.save();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON']);
    assert.strictEqual(err('WIFI_AP'), 'Minimum 8 characters.');
    assert.match(err('WIFI_STA'), /8-63 characters/);

    input('WIFI_AP', 'pwd').value = 'yummyflowers';
    input('WIFI_STA', 'pwd').value = '12345';  // 5 character WEP key
    await engine.save();
    assert.deepStrictEqual(t.sent.slice(1, 3), ['AT+WIFI_STA=0,,12345', 'AT+SETTINGS=SAVE']);
});

test('extra WiFi networks are set, cleared and validated per slot', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return [DUMP];
        if (cmd.startsWith('AT+WIFI_STA_NETWORK=') || cmd === 'AT+SETTINGS=SAVE') return [];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    input('WIFI_STA_NETWORK', 'ssid3').value = 'van-hotspot';
    input('WIFI_STA_NETWORK', 'pwd3').value = 'sixsix';
    await engine.save();
    assert.deepStrictEqual(t.sent, ['AT+SETTINGS?JSON']);

    input('WIFI_STA_NETWORK', 'pwd3').value = 'hotspotpass';
    input('WIFI_STA_NETWORK', 'ssid2').value = '';
    await engine.save();
    assert.deepStrictEqual(t.sent.slice(1, 4),
        ['AT+WIFI_STA_NETWORK=2', 'AT+WIFI_STA_NETWORK=3,van-hotspot,hotspotpass', 'AT+SETTINGS=SAVE']);
});

test('extra WiFi networks read from AT+WIFI_STA_NETWORK? when the dump lacks them', async () => {
    const json = JSON.parse(DUMP.slice('SETTINGS='.length));
    delete json.WIFI_STA_NETWORK;
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') return ['SETTINGS=' + JSON.stringify(json)];
        if (cmd === 'AT+WIFI_STA_NETWORK?') return ['WIFI_STA_NETWORK=2,office,********', 'WIFI_STA_NETWORK=3,lab,'];
        throw new Error(`unexpected ${cmd}`);
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('WIFI_STA_NETWORK', 'ssid2').value, 'office');
    assert.strictEqual(input('WIFI_STA_NETWORK', 'ssid3').value, 'lab');
    assert.strictEqual(input('WIFI_STA_NETWORK', 'pwd2').value, '');
});

// AT+SETTINGS?JSON Remote ID entries from a unit without PSRAM, as [REMOTE_ID, REMOTE_ID_TX].
function ridDump(rx, tx) {
    return DUMP.replace('"REMOTE_ID":[0,7,"0x00"]', `"REMOTE_ID":${rx}`)
        .replace('"REMOTE_ID_TX":[0,7,"",1,2,"","0x0000"]', `"REMOTE_ID_TX":${tx}`);
}
const ridBits = (cmd) => Object.fromEntries(
    input(cmd, 'transports').querySelectorAll('input').map(cb => [cb.dataset.bit, { checked: cb.checked, disabled: cb.disabled }]));
const notice = (cmd) => document.getElementById(`settings-notice-${cmd}`).textContent;

test('Remote ID on Ethernet without PSRAM: WiFi Beacon can be turned off but not on', async () => {
    // Saved before the guard: all transports on, ESP32 runs Bluetooth, the WiFi Beacon receiver is paused.
    const t = makeTransport(() => [ridDump('[1,7,"0x0B",3]', '[1,4,"",1,2,"","0x1008",3]')]);
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('REMOTE_ID', 'en').checked, true);
    assert.strictEqual(input('REMOTE_ID', 'en').disabled, false);
    assert.deepStrictEqual(ridBits('REMOTE_ID'), {
        1: { checked: true, disabled: false }, 2: { checked: true, disabled: false }, 4: { checked: true, disabled: false },
    });
    assert.match(input('REMOTE_ID', 'status').textContent, /Bluetooth scanning, Long Range, paused for WiFi\/Ethernet/);
    assert.match(notice('REMOTE_ID'), /only the Bluetooth transports/);
    assert.strictEqual(input('REMOTE_ID_TX', 'status').textContent, 'paused for WiFi/Ethernet (no PSRAM) (0x1008)');
    // Transmit: the Bluetooth options are free, WiFi Beacon is on and can still be turned off.
    assert.deepStrictEqual(ridBits('REMOTE_ID_TX'), {
        1: { checked: false, disabled: false }, 2: { checked: false, disabled: false }, 4: { checked: true, disabled: false },
    });

    // Clearing the WiFi Beacon option locks it.
    const entry = SETTINGS_SCHEMA_1090.find(e => e.cmd === 'REMOTE_ID');
    input('REMOTE_ID', 'transports').querySelectorAll('input').find(cb => String(cb.dataset.bit) === '4').checked = false;
    engine._onFieldChanged(entry);
    assert.deepStrictEqual(ridBits('REMOTE_ID')[4], { checked: false, disabled: true });
    assert.deepStrictEqual(engine.entryValues(entry), { en: true, transports: 3 });
});

test('Remote ID with WiFi on and no PSRAM: the toggle stays off', async () => {
    const t = makeTransport(() => [ridDump('[0,7,"0x00",0]', '[0,7,"",1,2,"","0x0000",0]')]);
    const engine = makeEngine(t);
    await engine.refresh();
    for (const cmd of ['REMOTE_ID', 'REMOTE_ID_TX']) {
        assert.strictEqual(input(cmd, 'en').checked, false);
        assert.strictEqual(input(cmd, 'en').disabled, true, cmd);
        assert.match(notice(cmd), /can’t run while WiFi is enabled/);
    }
    assert.strictEqual(input('REMOTE_ID', 'status').textContent, 'idle (0x00)');
    assert.strictEqual(input('REMOTE_ID_TX', 'status').textContent, 'idle (0x0000)');
});

test('Remote ID transmit status shows the shared blocked bits only when transmit is blocked', () => {
    const { SETTINGS_RID } = require('../settings.js');
    assert.strictEqual(SETTINGS_RID.statusText('0x1048', true), 'paused: low ESP32 memory, paused for WiFi/Ethernet (no PSRAM) (0x1048)');
    assert.strictEqual(SETTINGS_RID.statusText('0x0404', true), 'WiFi Beacon transmitting (0x0404)');
    assert.strictEqual(SETTINGS_RID.statusText('0x48', false), 'paused: low ESP32 memory, paused for WiFi/Ethernet (no PSRAM) (0x48)');
});

test('Remote ID from firmware without ALLOWED_TRANSPORTS is not locked', async () => {
    const t = makeTransport(() => [DUMP]);
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('REMOTE_ID', 'en').disabled, false);
    assert.ok(Object.values(ridBits('REMOTE_ID')).every(b => !b.disabled));
    assert.strictEqual(notice('REMOTE_ID'), '');
});

test('Remote ID query lines from older firmware (hex) carry the allowed transports', async () => {
    const t = makeTransport((cmd) => {
        if (cmd === 'AT+SETTINGS?JSON') throw new Error('ERROR');
        if (cmd === 'AT+REMOTE_ID?') return ['REMOTE_ID=1,0x04(TRANSPORTS),0x04(ESP32_STATUS),0x04(ALLOWED_TRANSPORTS)'];
        if (cmd === 'AT+REMOTE_ID_TX?') {
            return ['REMOTE_ID_TX=0,0x07(TRANSPORTS),(device serial)(UAS_ID),1(ID_TYPE),2(UA_TYPE),(none)(OPERATOR_ID),0x0000(ESP32_STATUS),0x04(ALLOWED_TRANSPORTS)'];
        }
        throw new Error('ERROR');
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.strictEqual(input('REMOTE_ID', 'status').textContent, 'WiFi Beacon receiving (0x04)');
    assert.deepStrictEqual(ridBits('REMOTE_ID')[2], { checked: false, disabled: true });
    assert.deepStrictEqual(ridBits('REMOTE_ID_TX')[1], { checked: true, disabled: false });
    assert.strictEqual(input('REMOTE_ID_TX', 'uas_id').value, '');
});

test('Remote ID query lines with names', async () => {
    const sent = [];
    const t = makeTransport((cmd) => {
        sent.push(cmd);
        if (cmd === 'AT+SETTINGS?JSON') throw new Error('ERROR');
        if (cmd === 'AT+REMOTE_ID?') {
            return ['REMOTE_ID=1,BT4+BT5_LR+WIFI_BEACON(TRANSPORTS),BT_SCANNING+BT5_LR_SCANNING+BLOCKED_NETWORK(ESP32_STATUS),BT4+BT5_LR(ALLOWED_TRANSPORTS)'];
        }
        if (cmd === 'AT+REMOTE_ID_TX?') {
            return ['REMOTE_ID_TX=0,NONE(TRANSPORTS),(device serial)(UAS_ID),1(ID_TYPE),2(UA_TYPE),(none)(OPERATOR_ID),TX_BLOCKED+BLOCKED_RAM(ESP32_STATUS),BT4+BT5_LR(ALLOWED_TRANSPORTS)'];
        }
        return ['OK'];
    });
    const engine = makeEngine(t);
    await engine.refresh();
    assert.deepStrictEqual(ridBits('REMOTE_ID'), {
        1: { checked: true, disabled: false }, 2: { checked: true, disabled: false }, 4: { checked: true, disabled: false },
    });
    assert.strictEqual(input('REMOTE_ID', 'status').textContent,
        'Bluetooth scanning, Long Range, paused for WiFi/Ethernet (no PSRAM) (0x0B)');
    assert.match(notice('REMOTE_ID'), /only the Bluetooth transports/);
    assert.deepStrictEqual(ridBits('REMOTE_ID_TX'), {
        1: { checked: false, disabled: false }, 2: { checked: false, disabled: false }, 4: { checked: false, disabled: true },
    });
    assert.strictEqual(input('REMOTE_ID_TX', 'status').textContent, 'not enough ESP32 memory (0x1010)');
});

test('Remote ID transport and status names match RemoteIDGuard', () => {
    const fs = require('node:fs');
    const path = require('node:path');
    const { SETTINGS_RID_TRANSPORTS, SETTINGS_RID_STATUS_NAMES, SETTINGS_RID } = require('../settings.js');
    const hh = fs.readFileSync(path.join(__dirname, '../../../../../../common/coprocessor/remote_id_guard.hh'), 'utf8');
    const bits = Object.fromEntries([...hh.matchAll(/(kStatus\w+) = 1 << (\d+)/g)].map(m => [m[1], 1 << Number(m[2])]));
    const statusNames = Object.fromEntries(
        [...hh.matchAll(/\{(kStatus\w+), "(\w+)"\}/g)].map(m => [m[2], bits[m[1]]]));
    assert.deepStrictEqual(SETTINGS_RID_STATUS_NAMES, statusNames);
    const transportNames = [...hh.matchAll(/\{1 << (\d), "(\w+)"\}/g)].map(m => ({ bit: 1 << Number(m[1]), name: m[2] }));
    assert.deepStrictEqual(SETTINGS_RID_TRANSPORTS.map(t => ({ bit: t.bit, name: t.name })), transportNames);
    assert.strictEqual(SETTINGS_RID.transportBits('ALL'), 7);
    assert.strictEqual(SETTINGS_RID.transportBits('NONE'), 0);
    assert.ok(Number.isNaN(SETTINGS_RID.transportBits('BT6')));
    assert.strictEqual(SETTINGS_RID.transportNames(3), 'BT4+BT5_LR');
    assert.strictEqual(SETTINGS_RID.transportNames(7), 'BT4+BT5_LR+WIFI_BEACON');
    assert.strictEqual(SETTINGS_RID.transportNames(0), 'NONE');
});

test('Remote ID saves send transport names', async () => {
    const sent = [];
    const t = makeTransport((cmd) => {
        sent.push(cmd);
        if (cmd === 'AT+SETTINGS?JSON') return [ridDump('[1,7,"0x07",7]', '[0,7,"",1,2,"","0x0000",7]')];
        return ['OK'];
    });
    const engine = makeEngine(t);
    await engine.refresh();
    const entry = SETTINGS_SCHEMA_1090.find(e => e.cmd === 'REMOTE_ID');
    input('REMOTE_ID', 'transports').querySelectorAll('input').find(cb => String(cb.dataset.bit) === '4').checked = false;
    engine._onFieldChanged(entry);
    const tx = SETTINGS_SCHEMA_1090.find(e => e.cmd === 'REMOTE_ID_TX');
    assert.deepStrictEqual(entry.build(engine.entryValues(entry)), ['AT+REMOTE_ID=1,BT4+BT5_LR']);
    assert.deepStrictEqual(tx.build({ en: false, transports: 0, uas_id: '', id_type: '1', ua_type: '2', operator_id: '' }),
        ['AT+REMOTE_ID_TX=0,NONE,-,1,2,-']);
});
