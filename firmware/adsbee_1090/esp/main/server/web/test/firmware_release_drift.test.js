// Checks that the standalone consoles carry exact copies of firmware_release.js.
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/firmware_release_drift.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');

const REPO = path.join(__dirname, '../../../../../../..');
const SRC = fs.readFileSync(path.join(__dirname, '../firmware_release.js'), 'utf8');

for (const name of ['adsbee_1090_console', 'adsbee_1421_console']) {
    test(`${name} carries an exact copy of firmware_release.js`, () => {
        const html = fs.readFileSync(path.join(REPO, `software/${name}/${name}.html`), 'utf8');
        const lines = html.split('\n');
        const b = lines.findIndex((l) => l.includes('══ BEGIN VENDORED firmware_release.js'));
        const e = lines.findIndex((l, i) => i > b && l.includes('══ END VENDORED firmware_release.js'));
        assert.ok(b >= 0 && e > b, 'VENDORED firmware_release.js markers not found');
        // The block is the file wrapped in <script> and </script> lines.
        const copy = lines.slice(b + 2, e - 1).join('\n') + '\n';
        assert.strictEqual(copy, SRC, `Re-copy firmware_release.js into software/${name}/${name}.html`);
    });
}
