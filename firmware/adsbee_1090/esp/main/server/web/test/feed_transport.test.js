// Tests for transports, the shared MQTT broker, presets and the RAM bar of the feed editor. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/feed_transport.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const ADSBEE_SRC = fs.readFileSync(path.join(__dirname, '../adsbee.js'), 'utf8');

// adsbee.js against a fake DOM, /api/feed and console.
function load(feed) {
    const els = {};
    const sent = [];
    const get = (id) => (els[id] ||= {
        id, value: '', checked: false, style: {}, dataset: {}, textContent: '', innerHTML: '', className: '',
        children: [], appendChild(child) { this.children.push(child); }, replaceChildren() {},
    });
    const ctx = vm.createContext({
        HOST_URI: 'adsbee.local',
        document: { getElementById: get, createElement: () => ({ style: {}, classList: { add() {}, remove() {} }, appendChild() {}, replaceChildren() {} }) },
        fetch: async () => ({ ok: true, json: async () => feed }),
        setTimeout: () => {},
        clearTimeout: () => {},
        WebSocket: function () {},
    });
    vm.runInContext(`${ADSBEE_SRC}\nthis.FeedEditor = FeedEditor; this.feedLabel = feedLabel;
        this.ConsoleWebSocket = ConsoleWebSocket; this.MetricsWebSocket = MetricsWebSocket;`, ctx);
    // The editor's receiver ID query (feed_receiver_id.test.js) is left out of `sent`.
    ctx.consoleWebSocket = { sendAndCapture: async (cmd) => { if (!cmd.startsWith('AT+FEED_RECEIVER_ID?')) sent.push(cmd.trim()); } };
    vm.runInContext('consoleWebSocket = this.consoleWebSocket;', ctx);
    // Fake <select>s take their value from the selected option written into innerHTML.
    for (const id of ['feed-protocol', 'feed-transport']) {
        Object.defineProperty(get(id), 'value', {
            get() { return (/value="([^"]+)" selected/.exec(this.innerHTML) || [])[1] || ''; },
            set(v) { this.innerHTML = this.innerHTML.replace(' selected', '').replace(`value="${v}"`, `value="${v}" selected`); },
        });
    }
    return { ctx, FeedEditor: ctx.FeedEditor, get, sent };
}

const BROKER = {
    host: 'broker.lan', port: 1883, username: 'bee', has_password: true, tls: false, qos: 1, topic_prefix: 'home',
    transport: 'TCP', path: '',
};
const MQTT_FEED = {
    index: 3, uri: '', port: 0, active: 1, protocol: 'AIRCRAFT_JSON', receiver_id: 'BEE0000123456789',
    transport: 'MQTT', mqtt: BROKER,
};

test('MQTT feed shows the shared broker instead of host and port, and saves the broker first', async () => {
    const { FeedEditor, get, sent } = load(MQTT_FEED);
    await FeedEditor.open(3);
    assert.strictEqual(get('feed-mqtt').style.display, '');
    assert.strictEqual(get('feed-endpoint').style.display, 'none');
    assert.strictEqual(get('feed-mqtt-host').value, 'broker.lan');
    assert.strictEqual(get('feed-mqtt-password').placeholder, '(unchanged)');
    assert.strictEqual(get('feed-mqtt-path').style.display, 'none');
    assert.match(get('feed-mqtt-topics').textContent, /home\/bee0000123456789\/aircraft_json/);

    await FeedEditor.save();
    assert.deepStrictEqual(sent, [
        'AT+MQTT=broker.lan,1883,bee,,0,1,home,TCP,-',  // Empty password keeps the stored one.
        'AT+FEED=3,-,0,1,AIRCRAFT_JSON,MQTT',
        'AT+SETTINGS=SAVE',
    ]);
});

test('WebSocket and TLS go to the broker, and clearing the username clears the password', async () => {
    const { FeedEditor, get, sent } = load(MQTT_FEED);
    await FeedEditor.open(3);
    get('feed-mqtt-username').value = '';
    get('feed-mqtt-prefix').value = '';
    get('feed-mqtt-tls').checked = true;
    get('feed-mqtt-ws').checked = true;
    get('feed-mqtt-port').value = '443';
    FeedEditor.updateMapLink();
    assert.strictEqual(get('feed-mqtt-path').style.display, '');
    await FeedEditor.save();
    assert.strictEqual(sent[0], 'AT+MQTT=broker.lan,443,-,-,1,1,-,WS,-');
});

test('TCP and UDP feeds hide the broker and send no AT+MQTT', async () => {
    for (const transport of [undefined, 'UDP']) {
        const { FeedEditor, get, sent } = load(
            { index: 1, uri: '192.168.1.255', port: 4000, active: 1, protocol: 'GDL90', receiver_id: '', transport });
        await FeedEditor.open(1);
        assert.strictEqual(get('feed-mqtt').style.display, 'none');
        assert.strictEqual(get('feed-endpoint').style.display, '');
        await FeedEditor.save();
        assert.deepStrictEqual(sent, [`AT+FEED=1,192.168.1.255,4000,1,GDL90,${transport || 'TCP'}`, 'AT+SETTINGS=SAVE']);
    }
});

test('changing the protocol selects its default transport', async () => {
    const { FeedEditor, get } = load({ index: 0, uri: '', port: 0, active: 0, protocol: 'NONE', receiver_id: '' });
    await FeedEditor.open(0);
    const protocol = get('feed-protocol');
    for (const [p, t] of [['GDL90', 'UDP'], ['MAVLINK2', 'UDP'], ['AIRCRAFT_JSON', 'MQTT'], ['BEAST', 'TCP'],
        ['CSBEE', 'TCP'], ['RAW', 'TCP']]) {
        protocol.value = p;
        protocol.onchange();
        assert.strictEqual(get('feed-transport').value, t, p);
    }
});

test('presets fill protocol, transport, host and port', async () => {
    const { FeedEditor, get, sent } = load(
        { index: 5, uri: '', port: 0, active: 0, protocol: 'NONE', receiver_id: 'BEE0000123456789' });
    await FeedEditor.open(5);
    const names = Array.from(FeedEditor.PRESETS, p => p.name);
    assert.deepStrictEqual(Array.from(get('feed-presets').children, c => c.textContent), names);
    for (const name of ['adsb.fi', 'adsb.lol', 'airplanes.live', 'whereplane', 'EFB tablets', 'Ground station',
        'Home Assistant']) {
        assert.ok(names.includes(name), name);
    }

    FeedEditor.applyPreset(names.indexOf('EFB tablets'));
    await FeedEditor.save();
    assert.deepStrictEqual(sent, ['AT+FEED=5,192.168.4.255,4000,1,GDL90,UDP', 'AT+SETTINGS=SAVE']);

    FeedEditor.applyPreset(names.indexOf('Ground station'));
    assert.strictEqual(get('feed-port').value, 14550);
    assert.strictEqual(get('feed-transport').value, 'UDP');

    FeedEditor.applyPreset(names.indexOf('whereplane'));
    assert.strictEqual(get('feed-uri').value, 'feed.whereplane.xyz');
    assert.strictEqual(get('feed-map-row').style.display, '');

    FeedEditor.applyPreset(names.indexOf('Home Assistant'));
    assert.strictEqual(get('feed-mqtt-host').value, 'homeassistant.local');
    assert.strictEqual(get('feed-mqtt-port').value, 1883);
    assert.strictEqual(get('feed-endpoint').style.display, 'none');
    sent.length = 0;
    await FeedEditor.save();
    assert.match(sent[0], /^AT\+MQTT=homeassistant\.local,1883,/);
    assert.strictEqual(sent[1], 'AT+FEED=5,-,0,1,AIRCRAFT_JSON,MQTT');
});

test('slots: an MQTT feed without a URI is in use', () => {
    const { ctx } = load({});
    assert.strictEqual(ctx.feedLabel('feed.adsb.fi', 'TCP', 9), 'feed.adsb.fi');
    assert.strictEqual(ctx.feedLabel('', 'MQTT', 2), 'mqtt#2');
    assert.strictEqual(ctx.feedLabel('', 'TCP', 2), null);
    assert.strictEqual(ctx.feedLabel('', undefined, 2), null);
});

test('RAM bar shows the share of the budget', () => {
    const { ctx, get } = load({});
    const updateFeedRAM = ctx.MetricsWebSocket.prototype.updateFeedRAM;
    updateFeedRAM.call({}, 24 * 1024, 48 * 1024);
    assert.strictEqual(get('feed-ram-fill').style.width, '50%');
    assert.strictEqual(get('feed-ram-fill').className, '');
    assert.match(get('feed-ram').dataset.tip, /24 of 48 kB/);
    updateFeedRAM.call({}, 40 * 1024, 48 * 1024);
    assert.strictEqual(get('feed-ram-fill').className, 'high');
    updateFeedRAM.call({}, 60 * 1024, 48 * 1024);
    assert.strictEqual(get('feed-ram-fill').style.width, '100%');
    assert.strictEqual(get('feed-ram-fill').className, 'full');
});

test('a refused AT command rejects with its reason, log lines do not', async () => {
    const { ctx } = load({});
    const sendAndCapture = ctx.ConsoleWebSocket.prototype.sendAndCapture;
    const ws = { send() {} };
    const refused = sendAndCapture.call(ws, 'AT+FEED=1,,,1\r\n', 'OK');
    ws.captureCallback('ERROR: CommsManager::IPWANTask: some log line\r\n');
    ws.captureCallback('ERROR Not enough ESP32 RAM: the active feeds would need 52 kB of the 48 kB\r\n');
    await assert.rejects(refused, { message: /^Not enough ESP32 RAM/ });
    const ok = sendAndCapture.call(ws, 'AT+FEED=1,,,0\r\n', 'OK');
    ws.captureCallback('OK\r\n');
    await ok;
});
