// Tests for the whereplane.xyz receiver map link in the feed editor (#153). Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/whereplane_link.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { whereplaneMapUrl } = require('../adsbee.js');

const REPO = path.join(__dirname, '../../../../../../..');
const ADSBEE_SRC = fs.readFileSync(path.join(__dirname, '../adsbee.js'), 'utf8');
const ID = '0bee00038172d18c';
const MAP = 'https://globe.whereplane.xyz/?feed=0bee0003-8172-d18c-0bee-00038172d18c';

test('whereplaneMapUrl builds the uuid the feed sends', () => {
    // Example uuid documented next to BeastReporter::BuildFeedStartFrame.
    const beast = fs.readFileSync(path.join(REPO, 'firmware/common/comms/beast/beast_utils.cpp'), 'utf8');
    assert.match(beast, /Example ADSBee UUID: 0bee0003-8172-d18c-0bee-00038172d18c/);
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz', 'BEAST', ID), MAP);
    assert.strictEqual(whereplaneMapUrl(' Feed.WherePlane.xyz ', 'BEAST_NO_UAT', ID.toUpperCase()), MAP);
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz', 'BEAST_NO_UAT_UPLINK', ID), MAP);
});

test('whereplaneMapUrl is null for other feeds', () => {
    assert.strictEqual(whereplaneMapUrl('feed.adsb.fi', 'BEAST', ID), null);
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz.example.com', 'BEAST', ID), null);
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz', 'RAW', ID), null);  // Only Beast feeds send the uuid.
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz', 'BEAST', ''), null);
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz', 'BEAST', undefined), null);
    assert.strictEqual(whereplaneMapUrl('feed.whereplane.xyz', 'BEAST', '0bee0003"><x'), null);
});

// FeedEditor.open() from adsbee.js against a fake DOM and /api/feed.
function loadFeedEditor(feed) {
    const els = {};
    const get = (id) => (els[id] ||= { id, value: '', checked: false, style: {}, textContent: '', innerHTML: '', href: '' });
    const ctx = vm.createContext({
        HOST_URI: 'adsbee.local',
        document: { getElementById: get },
        fetch: async () => ({ ok: true, json: async () => feed }),
    });
    vm.runInContext(`${ADSBEE_SRC}\nthis.FeedEditor = FeedEditor;`, ctx);
    // The fake <select> takes its value from the selected option written into innerHTML.
    Object.defineProperty(get('feed-protocol'), 'value', {
        get() { return (/value="([^"]+)" selected/.exec(this.innerHTML) || [])[1] || ''; },
        set(v) { this.innerHTML = this.innerHTML.replace(' selected', '').replace(`value="${v}"`, `value="${v}" selected`); },
    });
    return { FeedEditor: ctx.FeedEditor, get };
}

test('feed editor shows the map link only for a whereplane Beast feed', async () => {
    const { FeedEditor, get } = loadFeedEditor(
        { index: 2, uri: 'feed.whereplane.xyz', port: 30004, active: 1, protocol: 'BEAST', receiver_id: ID });
    await FeedEditor.open(2);
    assert.strictEqual(get('feed-map-row').style.display, '');
    assert.strictEqual(get('feed-map-link').href, MAP);

    get('feed-uri').value = 'feed.adsb.lol';
    get('feed-uri').oninput();
    assert.strictEqual(get('feed-map-row').style.display, 'none');

    get('feed-uri').value = 'feed.whereplane.xyz';
    get('feed-uri').oninput();
    assert.strictEqual(get('feed-map-row').style.display, '');
    get('feed-protocol').value = 'MAVLINK2';
    get('feed-protocol').onchange();
    assert.strictEqual(get('feed-map-row').style.display, 'none');
});

test('feed editor hides the map link for other feeds and older firmware', async () => {
    for (const feed of [
        { index: 0, uri: 'feed.adsb.fi', port: 30004, active: 1, protocol: 'BEAST_NO_UAT', receiver_id: ID },
        { index: 3, uri: 'feed.whereplane.xyz', port: 30004, active: 1, protocol: 'BEAST' },  // No receiver_id.
    ]) {
        const { FeedEditor, get } = loadFeedEditor(feed);
        await FeedEditor.open(feed.index);
        assert.strictEqual(get('feed-map-row').style.display, 'none', feed.uri);
    }
});

test('adsbee_1090_console carries the same whereplaneMapUrl and map link markup', () => {
    const html = fs.readFileSync(path.join(REPO, 'software/adsbee_1090_console/adsbee_1090_console.html'), 'utf8');
    const grab = (text) => {
        const start = text.indexOf('const WHEREPLANE_FEED_HOST');
        const end = text.indexOf('\n}\n', text.indexOf('function whereplaneMapUrl', start));
        assert.ok(start >= 0 && end > start, 'whereplaneMapUrl not found');
        return text.slice(start, end);
    };
    const recopy = 'Re-copy whereplaneMapUrl from adsbee.js into software/adsbee_1090_console/adsbee_1090_console.html';
    assert.strictEqual(grab(html), grab(ADSBEE_SRC), recopy);
    assert.ok(html.includes('id="feed-map-link"') && html.includes('receiver_id: args[5]'), recopy);
});
