// Checks that the standalone consoles carry exact copies of the shared settings code in settings.js.
// Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/settings_drift.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');

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
