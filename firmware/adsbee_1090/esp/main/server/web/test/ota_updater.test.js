// Tests for ota_updater.js against a simulated ADSBee. Run: node --test firmware/adsbee_1090/esp/main/server/web/test/
'use strict';
const test = require('node:test');
const assert = require('node:assert');
const ota = require('../ota_updater.js');

const SECTOR = 4096;

// Fast timeouts so failure paths finish quickly.
const FAST = {
    connectRetryDelayMs: 10,
    cmdTimeoutMs: 300, readyTimeoutMs: 300, writeTimeoutMs: 400, eraseTimeoutMs: 500, verifyTimeoutMs: 500,
    queryTimeoutMs: 150, drainQuietMs: 60, drainMaxMs: 1000, reconnectDelayMs: 10, interWriteDelayMs: 0,
};

/** Builds a two-partition .ota whose partitions hold appLen bytes of pseudo-random data. */
function makeOta(appLen) {
    const parts = [];
    for (let p = 0; p < 2; p++) {
        const app = new Uint8Array(appLen);
        let x = 12345 + p;
        for (let i = 0; i < appLen; i++) { x = (x * 1103515245 + 12345) >>> 0; app[i] = x >>> 24; }
        app[100] = 0x0A;  // Newlines and text, like a real image.
        const hdr = new DataView(new ArrayBuffer(20));
        hdr.setUint32(0, ota.OTA_MAGIC, true);
        hdr.setUint32(8, appLen, true);
        hdr.setUint32(12, ota.crc32(app), true);
        hdr.setUint32(16, 0xFFFFFFFF, true);
        const img = new Uint8Array(20 + appLen);
        img.set(new Uint8Array(hdr.buffer), 0);
        img.set(app, 20);
        parts.push(img);
    }
    const file = new Uint8Array(12 + parts[0].length + parts[1].length);
    const v = new DataView(file.buffer);
    v.setUint32(0, 2, true);
    v.setUint32(4, 12, true);
    v.setUint32(8, 12 + parts[0].length, true);
    file.set(parts[0], 12);
    file.set(parts[1], 12 + parts[0].length);
    return { file, parts };
}

/**
 * A simulated ADSBee console. faults:
 *   latePayloadWrites: Set of WRITE command numbers (1-based) whose payload arrives after the device's timeout; the
 *                      device answers ERROR and then echoes the payload as parse errors, one of which contains "OK".
 *   closeOnWrite:      WRITE command number on which the connection drops right after READY.
 *   dropAllWrites:     every WRITE times out (the update must give up and restore).
 */
class FakeAdsbee {
    constructor(faults = {}) {
        this.faults = Object.assign({ latePayloadWrites: new Set() }, faults);
        this.flash = [new Uint8Array(0x40000).fill(0), new Uint8Array(0x40000).fill(0)];  // Not erased.
        this.running = 0;
        this.logLevel = 'WARNINGS';
        this.rx = [1, 0];
        this.consoleProtocol = 'CSBEE';
        this.writeCount = 0;
        this.pending = null;
        this.conn = null;
        this.commands = [];
        this.dataWrites = [];
        this.booted = false;
        this.verified = false;
    }

    makeTransport() {
        const dev = this;
        const c = { open: null, onText: null, onClose: null, openState: false };
        return {
            open(onText, onClose) {
                if (dev.faults.refuseConnections > 0) {
                    dev.faults.refuseConnections--;
                    return Promise.reject(new ota.OtaError('cannot connect'));
                }
                c.onText = onText; c.onClose = onClose; c.openState = true;
                dev.conn = c;
                dev.pending = null;
                return Promise.resolve();
            },
            send(bytes) { if (c.openState) setImmediate(() => dev.receive(c, bytes)); },
            close() { c.openState = false; },
            isOpen() { return c.openState; },
        };
    }

    out(c, text) {
        setImmediate(() => { if (c.openState) c.onText(text); });
    }

    drop(c) {
        c.openState = false;
        setImmediate(() => c.onClose());
    }

    get target() { return this.flash[1 - this.running]; }

    receive(c, bytes) {
        if (this.pending) {
            const p = this.pending;
            this.pending = null;
            if (p.late) {
                // Arrives after the timeout: echoed as console parse errors (including one that contains "OK").
                setTimeout(() => {
                    this.out(c, 'CppAT::ParseMessage: Unable to find AT prefix in string \x01OK\x02.\r\n');
                    this.out(c, 'CppAT::ParseMessage: Unable to find AT prefix in string READY?.\r\n');
                }, 5);
                return;
            }
            if (bytes.length !== p.len || ota.crc32(bytes) !== p.crc) {
                this.out(c, 'ERROR Calculated CRC did not match.\r\n');
                return;
            }
            for (let i = 0; i < bytes.length; i++) this.target[p.off + i] &= bytes[i];  // NOR: program clears bits.
            this.dataWrites.push(p.off);
            this.out(c, `Writing ${p.len} Bytes.\r\nOK\r\n`);
            return;
        }
        const text = new TextDecoder().decode(bytes).trim();
        this.commands.push(text);
        const m = text.match(/^AT\+(\w+)([?=]?)(.*)$/);
        if (!m) { this.out(c, `CppAT::ParseMessage: Unable to find AT prefix in string ${text}.\r\n`); return; }
        const [, name, op, arg] = m;
        const ok = () => this.out(c, 'OK\r\n');
        if (name === 'LOG_LEVEL' && op === '?') this.out(c, `LOG_LEVEL=${this.logLevel}\r\n`);
        else if (name === 'LOG_LEVEL') { this.logLevel = arg; ok(); }
        else if (name === 'RX_ENABLE' && op === '?') {
            this.out(c, `1090 Receiver: ${this.rx[0] ? 'ENABLED' : 'DISABLED'}\r\nSubG Receiver: ${this.rx[1] ? 'ENABLED' : 'DISABLED'}\r\n`);
        } else if (name === 'RX_ENABLE') {
            const a = arg.split(',');
            this.rx = a[0] !== '' ? [+a[0], +a[0]] : [+a[1], +a[2]];
            ok();
        } else if (name === 'PROTOCOL_OUT' && op === '?') {
            this.out(c, `PROTOCOL_OUT=CONSOLE,${this.consoleProtocol}\r\nPROTOCOL_OUT=COMMS_UART,NONE\r\n`);
        } else if (name === 'PROTOCOL_OUT') { this.consoleProtocol = arg.split(',')[1]; ok(); }
        else if (name === 'OTA') {
            const a = arg.split(',');
            if (a[0] === 'GET_PARTITION') { this.out(c, `Partition: ${1 - this.running}\r\n`); ok(); }
            else if (a[0] === 'ERASE') {
                const off = parseInt(a[1], 16), len = +a[2];
                const start = Math.floor(off / SECTOR) * SECTOR, end = Math.ceil((off + len) / SECTOR) * SECTOR;
                this.target.fill(0xFF, start, end);
                ok();
            } else if (a[0] === 'WRITE') {
                this.writeCount++;
                const late = this.faults.dropAllWrites || this.faults.latePayloadWrites.has(this.writeCount);
                this.pending = { off: parseInt(a[1], 16), len: +a[2], crc: parseInt(a[3], 16), late };
                this.out(c, 'READY\r\n');
                if (this.faults.closeOnWrite === this.writeCount) { this.pending = null; this.drop(c); return; }
                if (late) setTimeout(() => this.out(c, 'ERROR Timed out after 5000 ms. Received 0 Bytes.\r\n'), 20);
            } else if (a[0] === 'VERIFY') {
                const v = new DataView(this.target.buffer);
                const len = v.getUint32(8, true), crc = v.getUint32(12, true);
                const good = v.getUint32(0, true) === ota.OTA_MAGIC && ota.crc32(this.target.subarray(0x1000, 0x1000 + len)) === crc;
                this.verified = good;
                this.out(c, good ? 'OK\r\n' : 'ERROR Partition failed verification.\r\n');
            } else if (a[0] === 'BOOT') { this.booted = true; }
        } else this.out(c, `CppAT::ParseMessage: Unable to match AT command ${name}.\r\n`);
    }
}

function updater(dev, extra = {}) {
    const progress = [];
    const u = new ota.OtaUpdater(Object.assign({}, FAST, {
        makeTransport: () => dev.makeTransport(),
        onProgress: (p) => progress.push(p),
    }, extra));
    return { u, progress };
}

function assertFlashed(dev, image) {
    const got = dev.target.subarray(0, 20);
    assert.deepStrictEqual(Array.from(got), Array.from(image.subarray(0, 20)), 'header');
    const app = image.subarray(20);
    assert.ok(Buffer.from(dev.target.subarray(0x1000, 0x1000 + app.length)).equals(Buffer.from(app)), 'application');
}

test('clean update writes the image, final chunk first, then verifies and boots', async () => {
    const { file, parts } = makeOta(3 * SECTOR + 100);
    const dev = new FakeAdsbee();
    const { u, progress } = updater(dev);
    const res = await u.run(file);
    assert.strictEqual(res.partition, 1);
    assert.strictEqual(res.retries, 0);
    assertFlashed(dev, parts[1]);
    assert.ok(dev.verified && dev.booted);
    // Warm-up writes of 0xFF at 0x1000, then the header, then the final chunk before the others.
    assert.deepStrictEqual(dev.dataWrites.slice(0, 4), [0x1000, 0x1000, 0, 0x1000 + 3 * SECTOR]);
    assert.strictEqual(progress.filter((p) => p.phase === 'write').pop().percent, 100);
});

test('first writes after the erase time out: the warm-up absorbs them', async () => {
    const { file, parts } = makeOta(2 * SECTOR);
    const dev = new FakeAdsbee({ latePayloadWrites: new Set([1, 2]) });
    const { u } = updater(dev);
    const res = await u.run(file);
    assertFlashed(dev, parts[1]);
    assert.strictEqual(res.retries, 0, 'no retries of real data');
    assert.ok(dev.booted);
});

test('a late payload whose echo contains "OK" counts as a failed write and is retried', async () => {
    const { file, parts } = makeOta(4 * SECTOR);
    // Writes 1-2 warm-up, 3 header, 4 final chunk, 5 = first chunk: make it (and its first retry) late.
    const dev = new FakeAdsbee({ latePayloadWrites: new Set([5, 6]) });
    const { u } = updater(dev);
    const res = await u.run(file);
    assert.strictEqual(res.retries, 2);
    assertFlashed(dev, parts[1]);
    assert.ok(dev.verified && dev.booted);
    // Each retry erases its sector first.
    assert.ok(dev.commands.filter((c) => c === 'AT+OTA=ERASE,1000,4096').length >= 2);
});

test('header write failures are retried', async () => {
    const { file, parts } = makeOta(SECTOR + 10);
    const dev = new FakeAdsbee({ latePayloadWrites: new Set([3]) });  // 3 = header.
    const { u } = updater(dev);
    const res = await u.run(file);
    assert.strictEqual(res.retries, 1);
    assertFlashed(dev, parts[1]);
});

test('a dropped connection is re-opened and the write retried', async () => {
    const { file, parts } = makeOta(3 * SECTOR);
    const dev = new FakeAdsbee({ closeOnWrite: 5 });
    const { u } = updater(dev);
    await u.run(file);
    assertFlashed(dev, parts[1]);
    assert.ok(dev.booted);
});

test('an update that cannot write gives up, restores the settings and never boots', async () => {
    const { file } = makeOta(2 * SECTOR);
    const dev = new FakeAdsbee({ dropAllWrites: true });
    const { u, progress } = updater(dev, { warmupMaxAttempts: 3, maxAttemptsPerWrite: 2 });
    await assert.rejects(u.run(file), (e) => {
        assert.ok(e instanceof ota.OtaError);
        assert.match(e.message, /still running its current firmware/);
        return true;
    });
    assert.ok(!dev.booted && !dev.verified);
    assert.strictEqual(dev.logLevel, 'WARNINGS');
    assert.deepStrictEqual(dev.rx, [1, 0]);
    assert.strictEqual(dev.consoleProtocol, 'CSBEE');
    assert.ok(progress.some((p) => p.phase === 'restore'));
});

test('a refused first connection is retried', async () => {
    const { file, parts } = makeOta(SECTOR);
    const dev = new FakeAdsbee({ refuseConnections: 2 });
    const { u, progress } = updater(dev);
    await u.run(file);
    assertFlashed(dev, parts[1]);
    assert.ok(progress.some((p) => /refused the connection/.test(p.message || '')));
});

test('parseOtaImage rejects damaged files', () => {
    const { file } = makeOta(1000);
    assert.strictEqual(ota.parseOtaImage(file, 1).appLen, 1000);
    const bad = file.slice(); bad[12] ^= 1;  // Partition 0 magic.
    assert.throws(() => ota.parseOtaImage(bad, 0), /not a valid ADSBee 1090 firmware file/);
    const corrupt = file.slice(); corrupt[12 + 20 + 500] ^= 0xFF;
    assert.throws(() => ota.parseOtaImage(corrupt, 0), /checksum mismatch/);
    assert.throws(() => ota.parseOtaImage(file.subarray(0, 500), 1), /length out of range|offset out of range/);
    assert.throws(() => ota.parseOtaImage(new Uint8Array(10), 0), /too short/);
});
