// Firmware update (OTA) over the /console websocket for the ADSBee 1090 web UI.
//
// No DOM access, so it runs in node for the tests in test/ota_updater.test.js. The page (adsbee.js,
// FirmwareUploader) drives it with a websocket transport and shows its progress.
//
// Why it does more than send the chunks in order: the receiving RP2040 reads each OTA=WRITE payload with a 5 s
// timeout, and on a busy network the payload often arrives later (always for the first write after the 12 s erase on
// 0.9.0-rc19). The write then fails, and the late bytes show up as console noise; on firmware up to 0.9.1-rc4 they are
// even parsed as AT commands. So:
//   - Warm-up writes of 0xFF (a no-op on erased flash, and no newline for an old parser to act on) absorb the first
//     failures before any real data is sent.
//   - Every write, including the header, is retried. Before a retry the console is drained until it has been quiet
//     for a while, on the same connection; reconnecting at once makes the ESP32 drop the next command while its queue
//     still holds the late payload. From the third retry a fresh connection is used, in case the session broke.
//   - The final, short chunk is sent first, padded with 0xFF (bytes past the image length are never checked).
//   - OK / READY / ERROR are matched as whole lines: late payload echoed in parse errors can contain "OK".
//   - If the update fails, the log level, receivers and console protocol are put back as they were, and the ADSBee
//     keeps running its current firmware (the new partition is only marked bootable by OTA=VERIFY).
//
// "AT" + "+" is never written contiguously in this file: it is stored in the firmware image, and firmware up to
// 0.9.1-rc4 can run AT commands found in an image it is receiving (see firmware/common/comms/at_text.hh).

(function (root) {
    'use strict';

    const AT = 'AT' + '+';

    const OTA_HEADER_SIZE_BYTES = 5 * 4;
    const OTA_APP_OFFSET_BYTES = 4 * 1024;
    const OTA_MAGIC = 0x0AD5BEEE;

    const DEFAULTS = {
        // One flash sector per write: each chunk is one websocket frame, and the ESP32 needs a contiguous buffer for
        // it (with Bluetooth Remote ID the largest free block can be ~10 KB). Must stay a multiple of 4096 so retry
        // erases stay sector-aligned.
        chunkBytes: 0x1000,
        warmupSuccesses: 2,    // 0xFF writes that must succeed in a row before the image.
        warmupMaxAttempts: 12,
        maxAttemptsPerWrite: 8,
        freshConnectionFromAttempt: 3,
        cmdTimeoutMs: 6000,
        readyTimeoutMs: 6000,
        writeTimeoutMs: 8000,   // Longer than the device's 5 s payload timeout, so its ERROR arrives first.
        eraseTimeoutMs: 30000,
        verifyTimeoutMs: 30000,
        queryTimeoutMs: 3000,
        drainQuietMs: 3000,
        drainMaxMs: 60000,
        reconnectDelayMs: 2000,
        // The ESP32 allows 4 /console clients, and sessions of a page that was just closed or reloaded can hold a
        // slot for a while, so the first connection may be refused.
        connectAttempts: 6,
        connectRetryDelayMs: 5000,
        interWriteDelayMs: 50,
        logLevelDuringUpdate: 'ERRORS',
    };

    class OtaError extends Error {}

    const CRC_TABLE = (() => {
        const table = new Uint32Array(256);
        for (let i = 0; i < 256; i++) {
            let c = i;
            for (let j = 0; j < 8; j++) {
                c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
            }
            table[i] = c >>> 0;
        }
        return table;
    })();

    function crc32(data) {
        let crc = 0xFFFFFFFF;
        for (let i = 0; i < data.length; i++) {
            crc = (crc >>> 8) ^ CRC_TABLE[(crc ^ data[i]) & 0xFF];
        }
        return (crc ^ 0xFFFFFFFF) >>> 0;
    }

    /**
     * Checks an .ota file and returns the image for one partition: {numPartitions, image (header + application),
     * appLen}. Throws OtaError with a message for the user if the file isn't a valid ADSBee 1090 .ota.
     */
    function parseOtaImage(fileBytes, partition) {
        const fail = (why) => { throw new OtaError(`This is not a valid ADSBee 1090 firmware file (.ota): ${why}.`); };
        if (!(fileBytes instanceof Uint8Array) || fileBytes.length < 12 + OTA_HEADER_SIZE_BYTES) fail('file too short');
        const view = new DataView(fileBytes.buffer, fileBytes.byteOffset, fileBytes.byteLength);
        const numPartitions = view.getUint32(0, true);
        if (numPartitions < 1 || numPartitions > 8) fail(`bad partition count ${numPartitions}`);
        if (partition >= numPartitions) fail(`it has no image for partition ${partition}`);
        const off = view.getUint32(4 + 4 * partition, true);
        if (off + OTA_HEADER_SIZE_BYTES > fileBytes.length) fail('partition offset out of range');
        const magic = view.getUint32(off, true);
        const appLen = view.getUint32(off + 8, true);
        const appCrc = view.getUint32(off + 12, true);
        if (magic !== OTA_MAGIC) fail('bad image header');
        if (appLen === 0 || off + OTA_HEADER_SIZE_BYTES + appLen > fileBytes.length) fail('image length out of range');
        const image = fileBytes.subarray(off, off + OTA_HEADER_SIZE_BYTES + appLen);
        if (crc32(image.subarray(OTA_HEADER_SIZE_BYTES)) !== appCrc) fail('checksum mismatch (file damaged?)');
        return { numPartitions, image, appLen };
    }

    /**
     * Wraps a transport and splits what arrives into lines. A transport is
     *   { open(onText, onClose): Promise, send(Uint8Array), close(), isOpen(): bool }.
     */
    class LineChannel {
        constructor(makeTransport, clock) {
            this.makeTransport = makeTransport;
            this.clock = clock;
            this.transport = null;
            this.lines = [];
            this.partial = '';
            this.closed = true;
            this.waiter = null;
            this.lastRxMs = 0;
        }

        async open() {
            this.lines = [];
            this.partial = '';
            this.transport = this.makeTransport();
            await this.transport.open((text) => this._onText(text), () => { this.closed = true; this._wake(); });
            this.closed = false;
        }

        close() {
            if (this.transport) {
                try { this.transport.close(); } catch (e) { /* already closed */ }
            }
            this.transport = null;
            this.closed = true;
            this._wake();
        }

        isOpen() { return !this.closed && this.transport && this.transport.isOpen(); }

        sendText(text) { this.transport.send(new TextEncoder().encode(text)); }

        sendBytes(bytes) { this.transport.send(bytes); }

        discard() { this.lines = []; this.partial = ''; }

        _onText(text) {
            this.lastRxMs = this.clock.now();
            const parts = (this.partial + text).split('\n');
            this.partial = parts.pop();
            for (const p of parts) this.lines.push(p.replace(/\r$/, ''));
            this._wake();
        }

        _wake() {
            if (this.waiter) { const w = this.waiter; this.waiter = null; w(); }
        }

        async _nextLine(deadlineMs) {
            while (this.lines.length === 0) {
                if (this.closed) throw new OtaError('connection closed');
                const left = deadlineMs - this.clock.now();
                if (left <= 0) return null;
                await new Promise((resolve) => {
                    this.waiter = resolve;
                    this.clock.setTimeout(() => { if (this.waiter === resolve) { this.waiter = null; resolve(); } }, left);
                });
            }
            return this.lines.shift();
        }

        /**
         * Waits for a line that matches `want` (a string compared with the trimmed line, or a RegExp). A line that
         * starts with "ERROR" throws. Returns the matching line; throws OtaError on timeout.
         */
        async waitLine(want, timeoutMs, what) {
            const deadline = this.clock.now() + timeoutMs;
            for (;;) {
                const line = await this._nextLine(deadline);
                if (line === null) throw new OtaError(`no ${what || want} within ${Math.round(timeoutMs / 1000)} s`);
                const t = line.trim();
                if (typeof want === 'string' ? t === want : want.test(t)) return t;
                if (t.startsWith('ERROR')) throw new OtaError(t);
            }
        }

        /** Reads and drops input until nothing has arrived for quietMs (or maxMs has passed). Returns lines dropped. */
        async drainUntilQuiet(quietMs, maxMs) {
            const end = this.clock.now() + maxMs;
            let n = 0;
            for (;;) {
                const now = this.clock.now();
                if (now >= end) break;
                let line;
                try {
                    line = await this._nextLine(Math.min(end, now + quietMs));
                } catch (e) {
                    break;  // Connection closed: nothing more to drain.
                }
                if (line === null) break;
                n++;
            }
            this.partial = '';
            return n;
        }
    }

    const realClock = {
        now: () => Date.now(),
        setTimeout: (fn, ms) => setTimeout(fn, ms),
        sleep: (ms) => new Promise((r) => setTimeout(r, ms)),
    };

    /**
     * Updates the ADSBee with an .ota file.
     *   options.makeTransport  () => transport (see LineChannel)
     *   options.onProgress     ({phase, percent, retries, message}) => void; phase is one of
     *                          connect, prepare, erase, warmup, write, verify, boot, restore
     *   options.log            (text) => void, for the detailed log
     */
    class OtaUpdater {
        constructor(options) {
            this.opt = Object.assign({}, DEFAULTS, options);
            this.clock = this.opt.clock || realClock;
            this.ch = new LineChannel(this.opt.makeTransport, this.clock);
            this.retries = 0;
            this.saved = {};
            this.log = this.opt.log || (() => {});
            this.onProgress = this.opt.onProgress || (() => {});
        }

        progress(phase, percent, message) {
            this.onProgress({ phase, percent, retries: this.retries, message });
        }

        async cmd(text, want = 'OK', timeoutMs = this.opt.cmdTimeoutMs) {
            this.log(`> ${text}`);
            this.ch.sendText(text + '\r\n');
            return this.ch.waitLine(want, timeoutMs, want === 'OK' ? `OK for ${text}` : undefined);
        }

        /** Remembers the settings the update changes, so they can be restored if it fails. Best effort. */
        async saveState() {
            const q = async (text, re) => {
                try {
                    this.ch.sendText(text + '\r\n');
                    return (await this.ch.waitLine(re, this.opt.queryTimeoutMs)).match(re);
                } catch (e) {
                    return null;
                }
            };
            const lvl = await q(AT + 'LOG_LEVEL?', /^\+?LOG_LEVEL=(\w+)$/);
            if (lvl) this.saved.logLevel = lvl[1];
            const r1090 = await q(AT + 'RX_ENABLE?', /^1090 Receiver: (ENABLED|DISABLED)$/);
            const subg = r1090 ? await this.ch.waitLine(/^SubG Receiver: (ENABLED|DISABLED)$/, this.opt.queryTimeoutMs).catch(() => null) : null;
            if (r1090 && subg) {
                this.saved.rx = [r1090[1] === 'ENABLED' ? 1 : 0, subg.endsWith('ENABLED') ? 1 : 0];
            }
            const proto = await q(AT + 'PROTOCOL_OUT?', /^\+?PROTOCOL_OUT=CONSOLE,(\w+)/);
            if (proto) this.saved.consoleProtocol = proto[1];
            await this.ch.drainUntilQuiet(300, 2000);
            this.log(`saved state: ${JSON.stringify(this.saved)}`);
        }

        async erase(offset, len) {
            await this.cmd(`${AT}OTA=ERASE,${offset.toString(16)},${len}`, 'OK', this.opt.eraseTimeoutMs);
        }

        async writeOnce(offset, data) {
            const text = `${AT}OTA=WRITE,${offset.toString(16)},${data.length},${crc32(data).toString(16)}`;
            this.log(`> ${text}`);
            this.ch.sendText(text + '\r\n');
            await this.ch.waitLine('READY', this.opt.readyTimeoutMs);
            this.ch.sendBytes(data);
            await this.ch.waitLine('OK', this.opt.writeTimeoutMs, `OK for the write at 0x${offset.toString(16)}`);
        }

        /** Opens the console connection, retrying while the ADSBee refuses it. */
        async openWithRetry() {
            for (let attempt = 1; ; attempt++) {
                try {
                    await this.ch.open();
                    return;
                } catch (e) {
                    this.ch.close();
                    if (attempt >= this.opt.connectAttempts) throw e;
                    this.log(`connection attempt ${attempt} failed: ${e.message}`);
                    this.progress('connect', null, `The ADSBee refused the connection; retrying (${attempt}/${this.opt.connectAttempts - 1})...`);
                    await this.clock.sleep(this.opt.connectRetryDelayMs);
                }
            }
        }

        async reconnect() {
            this.ch.close();
            await this.clock.sleep(this.opt.reconnectDelayMs);
            await this.openWithRetry();
            await this.ch.drainUntilQuiet(300, 2000);
        }

        /** Writes data at offset, retrying as described at the top of this file. eraseFirst erases before attempt 1. */
        async writeWithRetry(offset, data, eraseFirst) {
            for (let attempt = 0; ; attempt++) {
                try {
                    if (attempt > 0) {
                        this.retries++;
                        this.progress('write', null, `Retrying the write at 0x${offset.toString(16)} (attempt ${attempt + 1})`);
                        if (this.ch.isOpen()) {
                            const n = await this.ch.drainUntilQuiet(this.opt.drainQuietMs, this.opt.drainMaxMs);
                            this.log(`drained ${n} console lines`);
                        }
                        if (attempt >= this.opt.freshConnectionFromAttempt || !this.ch.isOpen()) {
                            await this.reconnect();
                        }
                    }
                    if (eraseFirst || attempt > 0) await this.erase(offset, data.length);
                    await this.writeOnce(offset, data);
                    await this.clock.sleep(this.opt.interWriteDelayMs);
                    return;
                } catch (e) {
                    this.log(`write at 0x${offset.toString(16)} failed: ${e.message}`);
                    if (attempt + 1 >= this.opt.maxAttemptsPerWrite) {
                        throw new OtaError(`the write at 0x${offset.toString(16)} failed ${attempt + 1} times (${e.message})`);
                    }
                }
            }
        }

        async warmUp() {
            const ff = new Uint8Array(Math.min(this.opt.chunkBytes, 0x1000)).fill(0xFF);
            let inRow = 0;
            for (let attempt = 1; inRow < this.opt.warmupSuccesses; attempt++) {
                if (attempt > this.opt.warmupMaxAttempts) throw new OtaError('the ADSBee is not accepting writes');
                this.progress('warmup', 0, 'Preparing flash...');
                try {
                    await this.writeOnce(OTA_APP_OFFSET_BYTES, ff);
                    inRow++;
                } catch (e) {
                    inRow = 0;
                    this.log(`warm-up write failed: ${e.message}`);
                    if (this.ch.isOpen()) await this.ch.drainUntilQuiet(this.opt.drainQuietMs, this.opt.drainMaxMs);
                    if (attempt >= this.opt.freshConnectionFromAttempt || !this.ch.isOpen()) await this.reconnect();
                }
                await this.clock.sleep(this.opt.interWriteDelayMs);
            }
        }

        async restoreState() {
            this.progress('restore', null, 'Update failed; restoring the receiver settings...');
            try {
                await this.reconnect();
                if (this.saved.logLevel) await this.cmd(`${AT}LOG_LEVEL=${this.saved.logLevel}`).catch(() => {});
                if (this.saved.consoleProtocol) {
                    await this.cmd(`${AT}PROTOCOL_OUT=CONSOLE,${this.saved.consoleProtocol}`).catch(() => {});
                }
                // Per-receiver form (empty first argument), as the settings page uses.
                const rx = this.saved.rx || [1, 1];
                await this.cmd(`${AT}RX_ENABLE=,${rx[0]},${rx[1]}`).catch(() => {});
            } catch (e) {
                this.log(`restore failed: ${e.message}`);
            }
        }

        /** Runs the whole update. Resolves when the ADSBee has been told to boot the new firmware. */
        async run(fileBytes) {
            let stage = 'connecting';
            let booting = false;
            try {
                this.progress('connect', 0, 'Connecting...');
                await this.openWithRetry();
                await this.ch.drainUntilQuiet(300, 2000);

                stage = 'preparing';
                this.progress('prepare', 0, 'Preparing the ADSBee...');
                await this.saveState();
                await this.cmd(`${AT}LOG_LEVEL=${this.opt.logLevelDuringUpdate}`);
                await this.cmd(`${AT}PROTOCOL_OUT=CONSOLE,NONE`);
                await this.cmd(`${AT}RX_ENABLE=0`);
                const partLine = await this.cmd(`${AT}OTA=GET_PARTITION`, /^Partition: (\d+)$/, this.opt.cmdTimeoutMs);
                await this.ch.waitLine('OK', this.opt.cmdTimeoutMs).catch(() => {});
                const partition = parseInt(partLine.match(/(\d+)/)[1], 10);
                const { image, appLen } = parseOtaImage(fileBytes, partition);
                this.log(`partition ${partition}, application ${appLen} bytes`);

                const chunk = this.opt.chunkBytes;
                const starts = [];
                for (let i = OTA_HEADER_SIZE_BYTES; i < OTA_HEADER_SIZE_BYTES + appLen; i += chunk) starts.push(i);
                starts.unshift(starts.pop());  // Final (short) chunk first.

                stage = 'erasing';
                this.progress('erase', 0, 'Erasing flash (about 15 s)...');
                await this.erase(0, OTA_APP_OFFSET_BYTES + appLen + chunk);

                stage = 'writing';
                await this.warmUp();
                await this.writeWithRetry(0, image.subarray(0, OTA_HEADER_SIZE_BYTES), false);
                let done = 0;
                for (const i of starts) {
                    const data = new Uint8Array(chunk).fill(0xFF);
                    data.set(image.subarray(i, Math.min(i + chunk, OTA_HEADER_SIZE_BYTES + appLen)));
                    await this.writeWithRetry(i - OTA_HEADER_SIZE_BYTES + OTA_APP_OFFSET_BYTES, data, false);
                    done += Math.min(chunk, OTA_HEADER_SIZE_BYTES + appLen - i);
                    const pct = Math.floor((100 * done) / appLen);
                    this.progress('write', pct, `Writing firmware... ${pct}%` + (this.retries ? ` (${this.retries} retries)` : ''));
                }

                stage = 'verifying';
                this.progress('verify', 100, 'Verifying...');
                await this.cmd(`${AT}OTA=VERIFY`, 'OK', this.opt.verifyTimeoutMs);

                stage = 'rebooting';
                booting = true;
                this.progress('boot', 100, 'Rebooting into the new firmware...');
                this.log(`> ${AT}OTA=BOOT`);
                this.ch.sendText(`${AT}OTA=BOOT\r\n`);
                await this.clock.sleep(500);
                return { retries: this.retries, partition };
            } catch (e) {
                if (booting) throw e;
                await this.restoreState();
                const why = (e instanceof OtaError ? e.message : String((e && e.message) || e)).replace(/\.+$/, '');
                throw new OtaError(`Update failed while ${stage}: ${why}. The ADSBee is still running its current ` +
                                   'firmware and can be updated again.');
            } finally {
                this.ch.close();
            }
        }
    }

    /** Transport over a browser WebSocket. */
    function webSocketTransport(url) {
        let ws = null;
        const decoder = new TextDecoder('utf-8');
        return {
            open(onText, onClose) {
                return new Promise((resolve, reject) => {
                    ws = new WebSocket(url);
                    ws.binaryType = 'arraybuffer';
                    let opened = false;
                    ws.onopen = () => { opened = true; resolve(); };
                    ws.onerror = () => { if (!opened) reject(new OtaError(`cannot connect to ${url}`)); };
                    ws.onclose = () => { if (!opened) reject(new OtaError(`cannot connect to ${url}`)); onClose(); };
                    ws.onmessage = (ev) => onText(typeof ev.data === 'string' ? ev.data : decoder.decode(ev.data));
                });
            },
            send(bytes) { ws.send(bytes); },
            close() { if (ws) ws.close(); ws = null; },
            isOpen() { return !!ws && ws.readyState === 1; },
        };
    }

    const api = { OtaUpdater, OtaError, LineChannel, parseOtaImage, crc32, webSocketTransport, DEFAULTS,
                  OTA_HEADER_SIZE_BYTES, OTA_APP_OFFSET_BYTES, OTA_MAGIC };
    if (typeof module !== 'undefined' && module.exports) module.exports = api;
    else root.AdsbeeOta = api;
})(typeof window !== 'undefined' ? window : this);
