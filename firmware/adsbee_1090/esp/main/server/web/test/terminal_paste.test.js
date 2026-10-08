// Checks that the console terminal's document-wide copy/paste handlers leave other text boxes alone (#273), in the
// device page and both standalone consoles. Runs in CI (web_ui_test job).
// Run: node --test firmware/adsbee_1090/esp/main/server/web/test/terminal_paste.test.js
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const { isOtherTextEntry } = require('../adsbee.js');

const REPO = path.join(__dirname, '../../../../../../..');
const PAGES = [
    'firmware/adsbee_1090/esp/main/server/web/index.html',
    'software/adsbee_1090_console/adsbee_1090_console.html',
    'software/adsbee_1421_console/adsbee_1421_console.html',
];

const el = (tagName, props = {}) => Object.assign({ tagName, type: '', isContentEditable: false }, props);

test('isOtherTextEntry classifies event targets', () => {
    const terminal = el('INPUT', { type: 'text' });
    assert.strictEqual(isOtherTextEntry(terminal, terminal), false);
    assert.strictEqual(isOtherTextEntry(null, terminal), false);
    assert.strictEqual(isOtherTextEntry(el('BODY'), terminal), false);
    assert.strictEqual(isOtherTextEntry(el('DIV'), terminal), false);
    for (const type of ['text', 'number', 'password', 'url', 'TEXT', '']) {
        assert.strictEqual(isOtherTextEntry(el('INPUT', { type }), terminal), true, `input type="${type}"`);
    }
    for (const type of ['checkbox', 'radio', 'file', 'button']) {
        assert.strictEqual(isOtherTextEntry(el('INPUT', { type }), terminal), false, `input type="${type}"`);
    }
    assert.strictEqual(isOtherTextEntry(el('TEXTAREA'), terminal), true);
    assert.strictEqual(isOtherTextEntry(el('SELECT'), terminal), true);
    assert.strictEqual(isOtherTextEntry(el('DIV', { isContentEditable: true }), terminal), true);
});

// Source of the statement `document.addEventListener('<type>', ...);` in a page.
function listenerSource(html, type) {
    const start = html.indexOf(`document.addEventListener('${type}', (e) => {`);
    assert.ok(start >= 0, `no document ${type} listener`);
    const end = html.indexOf('\n        });', start);
    return html.slice(start, end + '\n        });'.length);
}

// Runs a page's copy/paste listeners against a minimal terminal state.
function loadPage(file) {
    const html = fs.readFileSync(path.join(REPO, file), 'utf8');
    const listeners = {};
    const ctx = vm.createContext({
        isOtherTextEntry,
        document: { addEventListener: (type, fn) => { listeners[type] = fn; } },
        window: { getSelection: () => ({ toString: () => ctx.selectedText }) },
        hiddenInput: el('INPUT', { type: 'text', value: '' }),
        selectedText: '',
    });
    vm.runInContext(`
        var currentCommand = 'AT+', cursorPosition = 3;
        function updateDisplay() {}
        ${listenerSource(html, 'copy')}
        ${listenerSource(html, 'paste')}
    `, ctx);
    const fire = (type, target, text = '') => {
        const data = { text };
        const e = {
            target, defaultPrevented: false,
            preventDefault() { this.defaultPrevented = true; },
            clipboardData: { getData: () => data.text, setData: (_, v) => { data.text = v; } },
        };
        listeners[type](e);
        return { prevented: e.defaultPrevented, data: data.text };
    };
    return { ctx, fire, command: () => vm.runInContext('currentCommand', ctx) };
}

for (const file of PAGES) {
    test(`${path.basename(file)}: paste into a focused text box stays there`, () => {
        const page = loadPage(file);
        for (const target of [el('INPUT', { type: 'text' }), el('INPUT', { type: 'number' }), el('TEXTAREA')]) {
            const r = page.fire('paste', target, 'feed.example.com');
            assert.strictEqual(r.prevented, false, `${target.tagName} paste was taken over`);
            assert.strictEqual(page.command(), 'AT+');
        }
    });

    test(`${path.basename(file)}: paste into the terminal still reaches the console`, () => {
        const page = loadPage(file);
        let r = page.fire('paste', page.ctx.hiddenInput, 'FEED?');
        assert.strictEqual(r.prevented, true);
        assert.strictEqual(page.command(), 'AT+FEED?');
        // A paste with nothing text-like focused (e.g. after clicking a checkbox) goes to the terminal as before.
        r = page.fire('paste', el('INPUT', { type: 'checkbox' }), 'X');
        assert.strictEqual(r.prevented, true);
        assert.strictEqual(page.command(), 'AT+FEED?X');
    });

    test(`${path.basename(file)}: copy from a text box keeps the browser default`, () => {
        const page = loadPage(file);
        page.ctx.selectedText = 'terminal output';
        assert.strictEqual(page.fire('copy', el('INPUT', { type: 'text' })).prevented, false);
        const r = page.fire('copy', el('DIV'));
        assert.strictEqual(r.prevented, true);
        assert.strictEqual(r.data, 'terminal output');
    });
}

test('standalone consoles carry the same isOtherTextEntry as adsbee.js', () => {
    const src = fs.readFileSync(path.join(__dirname, '../adsbee.js'), 'utf8');
    const grab = (text) => {
        const start = text.indexOf('const NON_TEXT_INPUT_TYPES');
        const fn = text.indexOf('function isOtherTextEntry', start);
        const end = fn + text.slice(fn).search(/\n *\}\n/);
        assert.ok(start >= 0 && end > fn, 'isOtherTextEntry not found');
        return text.slice(start, end).split('\n').map(l => l.trim()).join('\n');
    };
    for (const file of PAGES.slice(1)) {
        assert.strictEqual(grab(fs.readFileSync(path.join(REPO, file), 'utf8')), grab(src),
            `Re-copy isOtherTextEntry from adsbee.js into ${file}`);
    }
});
