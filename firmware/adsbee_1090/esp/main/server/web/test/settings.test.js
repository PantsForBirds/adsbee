// Tests for the settings GUI engine in settings.js against a scripted AT transport.
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');

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

// The AT+SETTINGS?JSON line from an ADSBee 1090 running 0.9.1-rc5.
const DUMP = 'SETTINGS={"BAUD_RATE":[115207,9600],"BIAS_TEE_ENABLE":[0,0],"ETHERNET":[1],"ESP32_ENABLE":[1],' +
    '"FEED_ENABLE":[1],"GNSS":[0,"NONE",0],"HOSTNAME":["attic-bee"],"LED_ENABLE":[1],"LOG_LEVEL":["WARNINGS"],' +
    '"MAVLINK_ID":[1,156],"PROTOCOL_OUT":["NONE","NONE"],"REMOTE_ID":[0,7,"0x00"],' +
    '"REMOTE_ID_TX":[0,7,"",1,2,"","0x0000"],"RX_ENABLE":[1,1],' +
    '"RX_POSITION":["LOWEST","OK",37.508141,-122.245346,-50,125,317.7,74,"000000"],"SUBG_ENABLE":["1"],' +
    '"TL_OFFSET":[600,"-104 dBm"],"WATCHDOG":[10],"WIFI_AP":[0,"attic-bee","yummyflowers",1],"WIFI_STA":[0,""]}';

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

    assert.strictEqual(input('HOSTNAME', 'hostname').value, 'attic-bee');
    assert.strictEqual(input('BAUD_RATE', 'COMMS_UART').value, '115207');
    assert.strictEqual(input('WIFI_AP', 'pwd').value, 'yummyflowers');
    assert.strictEqual(input('WIFI_AP', 'channel').value, '1');
    assert.strictEqual(input('WIFI_STA', 'pwd').value, '');  // write-only, never shown
    assert.strictEqual(input('SUBG_ENABLE', 'state').value, '1');
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
        if (cmd === 'AT+HOSTNAME=new-bee') { dump = dump.replace('["attic-bee"]', '["new-bee"]'); return []; }
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

// The standalone consoles vendor parts of settings.js verbatim (see their READMEs).
const REPO = path.join(__dirname, '../../../../../../..');
const SETTINGS_SRC = fs.readFileSync(path.join(__dirname, '../settings.js'), 'utf8');
const consoleHtml = (name) => fs.readFileSync(path.join(REPO, `software/${name}/${name}.html`), 'utf8');

// Lines strictly between the first line containing `begin` and the next containing `end`.
function between(text, begin, end) {
    const lines = text.split('\n');
    const b = lines.findIndex(l => l.includes(begin));
    const e = lines.findIndex((l, i) => i > b && l.includes(end));
    assert.ok(b >= 0 && e > b, `markers ${begin} / ${end} not found`);
    return lines.slice(b + 1, e).join('\n').trim();
}

function vendoredEngine(html) {
    // The vendored block opens with two comment lines on where it comes from.
    const body = between(html, '══ BEGIN VENDORED ADSBee settings engine', '══ END VENDORED ADSBee settings engine');
    return body.split('\n').slice(2).join('\n').trim();
}

const sharedEngine = between(SETTINGS_SRC, '══ BEGIN SHARED SETTINGS ENGINE', '══ END SHARED SETTINGS ENGINE');
const recopy = (name) => `Re-copy the SHARED SETTINGS ENGINE block from settings.js into software/${name}/${name}.html`;

for (const name of ['adsbee_1421_console', 'adsbee_1090_console']) {
    test(`${name} carries an exact copy of the shared engine`, () => {
        assert.strictEqual(vendoredEngine(consoleHtml(name)), sharedEngine, recopy(name));
    });
}

test('adsbee_1090_console carries the same 1090 schema and transport', () => {
    const html = consoleHtml('adsbee_1090_console');
    for (const [begin, end] of [['const SETTINGS_SCHEMA_1090 = [', '\n];'], ['class Settings1090Transport {', '\n}\n']]) {
        const grab = (text) => text.slice(text.indexOf(begin), text.indexOf(end, text.indexOf(begin)));
        assert.ok(html.includes(begin), `${begin} not found in adsbee_1090_console.html`);
        assert.strictEqual(grab(html), grab(SETTINGS_SRC),
            `Re-copy ${begin} from settings.js into software/adsbee_1090_console/adsbee_1090_console.html`);
    }
});
