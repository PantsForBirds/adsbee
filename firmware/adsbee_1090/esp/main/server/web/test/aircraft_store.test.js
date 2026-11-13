// Tests for the live map's aircraft store. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/aircraft_store.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const { AircraftStore } = require('../adsbee.js');

const REPO = path.join(__dirname, '../../../../../../..');
const CONSOLE = fs.readFileSync(path.join(REPO, 'software/adsbee_1090_console/adsbee_1090_console.html'), 'utf8');
const ADSBEE_SRC = fs.readFileSync(path.join(__dirname, '../adsbee.js'), 'utf8');

const report = (extra) => ({ hex: 'acbe3e', type: 'adsb_icao', link: '1090', messages: 1, ...extra });

test('on_ground from the first report clears once a report leaves it out', () => {
    // The first report of a new aircraft can say on_ground before any airborne message (0.9.1-rc9 did).
    const store = new AircraftStore();
    store.ingest(report({ on_ground: 1 }));
    assert.strictEqual(store.aircraft.get('acbe3e').on_ground, 1);
    store.ingest(report({ alt_baro: 31375, gs: 500, lat: 37.5, lon: -122.3 }));
    const ac = store.aircraft.get('acbe3e');
    assert.ok(!('on_ground' in ac));
    assert.strictEqual(ac.alt_baro, 31375);
});

test('alert, spi and emergency clear the same way; other fields carry over', () => {
    const store = new AircraftStore();
    store.ingest(report({ alert: 1, spi: 1, emergency: 'general', squawk: '7700' }));
    store.ingest(report({ gs: 300 }));
    const ac = store.aircraft.get('acbe3e');
    for (const f of ['alert', 'spi', 'emergency']) assert.ok(!(f in ac), f);
    assert.strictEqual(ac.squawk, '7700');
    assert.strictEqual(ac.gs, 300);
});

test('adsbee_1090_console has the same store', () => {
    const fields = (src) => /const kTrueOnlyFields = .*;/.exec(src)[0];
    const merge = (src) => src.slice(src.indexOf('        let prev = this.aircraft.get(key)'), src.indexOf('        this.lastSeen.set(key'));
    assert.strictEqual(fields(CONSOLE), fields(ADSBEE_SRC), 'Re-copy kTrueOnlyFields from adsbee.js');
    assert.ok(merge(ADSBEE_SRC).includes('kTrueOnlyFields'));
    assert.strictEqual(merge(CONSOLE), merge(ADSBEE_SRC), 'Re-copy AircraftStore.ingest from adsbee.js');
});

test('ADS-L targets get their own source and key', () => {
    const store = new AircraftStore();
    store.ingest(report({ lat: 47.25, lon: 8.5 }));
    store.ingest({ hex: 'acbe3e', type: 'adsl', link: 'adsl', lat: 47.26, lon: 8.51, alt_geom: 4924 });
    assert.strictEqual(store.aircraft.get('acbe3e').src, 'adsb_1090');
    const adsl = store.aircraft.get('adsl-acbe3e');
    assert.strictEqual(adsl.src, 'adsl');
    assert.strictEqual(adsl.srcLabel, 'ADS-L');
    assert.strictEqual(adsl.alt_geom, 4924);
});
