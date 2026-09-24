// Settings GUI for the ADSBee web pages.
//
// The engine between the BEGIN/END SHARED SETTINGS ENGINE markers is shared with the
// ADSBee 1421 console (software/adsbee_1421_console/adsbee_1421_console.html), where it
// is vendored verbatim between matching markers. Edit it here, then re-copy the block.
//
// Everything below the shared block is 1090-specific: the settings schema, the console
// websocket transport, and the tab glue.

// ══════════ BEGIN SHARED SETTINGS ENGINE ══════════

// Schema-driven settings form. Each schema entry describes one AT command:
//
//   {
//     cmd: 'WIFI_STA',              // AT+<cmd> (no '+' appears in responses: "WIFI_STA=...")
//     label: 'WiFi Station',
//     group: 'Network',             // form section; sections appear in first-use order
//     help: 'Join an existing WiFi network.',
//     disconnects: true,            // warn before saving; saved last; SAVE timeout tolerated
//     experimental: true,           // badge the entry: the feature is not yet stable
//     query: { expect: /^WIFI_STA=/, quietMs: 350, timeoutMs: 3000 },  // all optional
//     fields: [ { id, label, type, ... } ],
//     parse(lines) { ... },         // optional; default: positional split of first matching line
//     parseJson(raw) { ... },       // optional; maps this entry's value from the bulk
//                                   // AT+SETTINGS?JSON dump; default: positional array
//                                   // mapped onto fields (writeOnly fields excluded)
//     build(values, dirtyIds) {...},// optional; default: 'AT+<cmd>=' + all args joined by ','
//     async afterWrite(values) {...}// optional; awaited right after this entry's commands
//                                   // succeed, before AT+SETTINGS=SAVE (e.g. the 1421
//                                   // reopens its serial port after a baud change)
//   }
//
// Field types:
//   bool     checkbox, canonical true/false, written as 1/0
//   int      number input; optional min/max
//   float    number input; optional min/max
//   string   text input; optional maxLen
//   password password input; writeOnly: true means the device never returns the real
//            value, so it is only dirty (and only written) when the user types one
//   enum     select; options: ['A','B'] or [{value, label}]
//   bitmask  checkbox group; options: [{bit, label}]; canonical number
//   hex      text input for hex values (no 0x prefix); optional maxVal
//   display  read-only informational value populated by parse(); never written
// Fields may declare visibleIf(values) to show/hide based on sibling values, and
// note: '...' for a hint line under the input.
//
// To expose a new AT command in the GUI, add one entry to the product's schema array.

// Strips a single trailing "(ANNOTATION)" from a query response arg, e.g.
// "115200(COMMS_UART)" -> "115200". Values that are themselves parenthesized
// ("(device serial)") survive one strip only if an annotation follows them.
function atStripAnnotation(arg) {
    return arg.replace(/\([^()]*\)\s*$/, '').trim();
}

// Splits the text after the first '=' on commas. AT args cannot contain commas
// (there is no quoting), so a plain split is exact.
function atSplitArgs(line) {
    const eq = line.indexOf('=');
    return (eq < 0 ? line : line.slice(eq + 1)).split(',').map(s => s.trim());
}

class SettingsEngine {
    // config: { schema, transport, formEl, statusEl, saveBtn, refreshBtn, bulkQuery }
    // transport contract: sendCommand(cmd, {expect, terminator: 'ok'|'quiet', quietMs,
    // timeoutMs}) -> Promise<string[] of trimmed body lines>; rejects on a whole-line
    // 'ERROR ...' response or timeout.
    // bulkQuery (optional): { command: 'AT+SETTINGS?JSON', expect: /^SETTINGS=/ } — a
    // one-round-trip read of every setting as a JSON object keyed by AT command name.
    // Entries the dump doesn't cover (or firmware without the command) fall back to
    // per-command queries automatically.
    constructor(config) {
        this.schema = config.schema;
        this.transport = config.transport;
        this.bulkQuery = config.bulkQuery || null;
        this.formEl = config.formEl;
        this.statusEl = config.statusEl;
        this.saveBtn = config.saveBtn;
        this.refreshBtn = config.refreshBtn;
        this.deviceValues = {};   // cmd -> {fieldId: canonical value} from the last read
        this.readOk = {};         // cmd -> bool; entries that failed to read stay disabled
        this.busy = false;
        this.rendered = false;
        this.gen = 0;             // bumped by abort(); in-flight refresh/save loops bail when stale
        this.saveBtn.addEventListener('click', () => this.save());
        this.refreshBtn.addEventListener('click', () => {
            if (this.dirtyEntries().length > 0 &&
                !confirm('Discard unsaved changes and re-read settings from the device?')) {
                return;
            }
            this.refresh();
        });
    }

    // ── Rendering ──

    render() {
        if (this.rendered) return;
        this.rendered = true;
        const groups = new Map();
        for (const entry of this.schema) {
            if (!groups.has(entry.group)) {
                const groupEl = document.createElement('div');
                groupEl.className = 'settings-group';
                const title = document.createElement('div');
                title.className = 'settings-group-title';
                title.textContent = entry.group;
                groupEl.appendChild(title);
                this.formEl.appendChild(groupEl);
                groups.set(entry.group, groupEl);
            }
            groups.get(entry.group).appendChild(this._renderEntry(entry));
        }
        this._updateDirtyUI();
    }

    _renderEntry(entry) {
        const el = document.createElement('div');
        el.className = 'settings-entry';
        el.id = `settings-entry-${entry.cmd}`;

        const header = document.createElement('div');
        header.className = 'settings-entry-header';
        const label = document.createElement('span');
        label.className = 'settings-entry-label';
        label.textContent = entry.label;
        header.appendChild(label);
        if (entry.disconnects) {
            const badge = document.createElement('span');
            badge.className = 'settings-danger';
            badge.textContent = 'may drop connection';
            header.appendChild(badge);
        }
        if (entry.experimental) {
            const badge = document.createElement('span');
            badge.className = 'settings-experimental';
            badge.textContent = 'experimental';
            header.appendChild(badge);
        }
        const status = document.createElement('span');
        status.className = 'settings-entry-status';
        status.id = `settings-status-${entry.cmd}`;
        header.appendChild(status);
        el.appendChild(header);

        if (entry.help) {
            const help = document.createElement('div');
            help.className = 'settings-help';
            help.textContent = entry.help;
            el.appendChild(help);
        }

        for (const field of entry.fields) {
            el.appendChild(this._renderField(entry, field));
        }
        return el;
    }

    _renderField(entry, field) {
        const row = document.createElement('div');
        row.className = 'settings-field';
        row.id = `settings-field-${entry.cmd}-${field.id}`;
        const label = document.createElement('label');
        label.textContent = field.label;
        row.appendChild(label);

        const inputId = SettingsEngine.inputId(entry, field);
        let input;
        switch (field.type) {
            case 'bool':
                input = document.createElement('input');
                input.type = 'checkbox';
                row.classList.add('settings-field-inline');
                break;
            case 'int':
            case 'float':
                input = document.createElement('input');
                input.type = 'number';
                if (field.min !== undefined) input.min = field.min;
                if (field.max !== undefined) input.max = field.max;
                input.step = field.type === 'float' ? 'any' : '1';
                break;
            case 'enum': {
                input = document.createElement('select');
                for (const opt of field.options) {
                    const o = document.createElement('option');
                    o.value = typeof opt === 'object' ? opt.value : opt;
                    o.textContent = typeof opt === 'object' ? opt.label : opt;
                    input.appendChild(o);
                }
                break;
            }
            case 'bitmask': {
                input = document.createElement('div');
                input.className = 'settings-bitmask';
                for (const opt of field.options) {
                    const optLabel = document.createElement('label');
                    const cb = document.createElement('input');
                    cb.type = 'checkbox';
                    cb.dataset.bit = opt.bit;
                    optLabel.appendChild(cb);
                    optLabel.appendChild(document.createTextNode(' ' + opt.label));
                    input.appendChild(optLabel);
                }
                break;
            }
            case 'display':
                input = document.createElement('span');
                input.className = 'settings-display';
                input.textContent = '—';
                break;
            case 'password':
                input = document.createElement('input');
                input.type = 'password';
                if (field.placeholder) input.placeholder = field.placeholder;
                if (field.maxLen) input.maxLength = field.maxLen;
                break;
            default:  // string, hex
                input = document.createElement('input');
                input.type = 'text';
                if (field.placeholder) input.placeholder = field.placeholder;
                if (field.maxLen) input.maxLength = field.maxLen;
                break;
        }
        input.id = inputId;
        if (field.type !== 'display') {
            input.addEventListener('input', () => this._onFieldChanged(entry));
            input.addEventListener('change', () => this._onFieldChanged(entry));
        }
        row.appendChild(input);

        if (field.note) {
            const note = document.createElement('div');
            note.className = 'settings-note';
            note.textContent = field.note;
            row.appendChild(note);
        }
        const err = document.createElement('div');
        err.className = 'settings-error';
        err.id = `settings-error-${entry.cmd}-${field.id}`;
        row.appendChild(err);
        return row;
    }

    static inputId(entry, field) { return `settings-input-${entry.cmd}-${field.id}`; }

    _input(entry, field) { return document.getElementById(SettingsEngine.inputId(entry, field)); }

    _onFieldChanged(entry) {
        this._updateVisibility(entry);
        this._updateDirtyUI();
    }

    // ── Values ──

    // Reads the canonical value of one field from the DOM. Returns undefined for
    // display fields and for blank numeric inputs.
    _readField(entry, field) {
        const input = this._input(entry, field);
        switch (field.type) {
            case 'bool': return input.checked;
            case 'int': return input.value === '' ? undefined : parseInt(input.value, 10);
            case 'float': return input.value === '' ? undefined : parseFloat(input.value);
            case 'bitmask': {
                let mask = 0;
                for (const cb of input.querySelectorAll('input[type="checkbox"]')) {
                    if (cb.checked) mask |= parseInt(cb.dataset.bit, 10);
                }
                return mask;
            }
            case 'display': return undefined;
            case 'hex': return input.value.trim().replace(/^0x/i, '').toUpperCase();
            default: return input.value;
        }
    }

    _writeField(entry, field, value) {
        const input = this._input(entry, field);
        switch (field.type) {
            case 'bool': input.checked = !!value; break;
            case 'bitmask':
                for (const cb of input.querySelectorAll('input[type="checkbox"]')) {
                    cb.checked = (value & parseInt(cb.dataset.bit, 10)) !== 0;
                }
                break;
            case 'display': input.textContent = value === undefined ? '—' : String(value); break;
            default: input.value = value === undefined ? '' : String(value); break;
        }
    }

    entryValues(entry) {
        const values = {};
        for (const field of entry.fields) {
            if (field.type === 'display') continue;
            values[field.id] = this._readField(entry, field);
        }
        return values;
    }

    _fieldIsDirty(entry, field) {
        if (field.type === 'display') return false;
        const current = this._readField(entry, field);
        if (field.writeOnly) return current !== '' && current !== undefined;
        const device = (this.deviceValues[entry.cmd] || {})[field.id];
        if (device === undefined) return false;  // never read; nothing to compare against
        return current !== device;
    }

    entryDirtyFields(entry) {
        const dirty = new Set();
        for (const field of entry.fields) {
            if (this._fieldIsDirty(entry, field)) dirty.add(field.id);
        }
        return dirty;
    }

    dirtyEntries() {
        return this.schema.filter(e => this.readOk[e.cmd] && this.entryDirtyFields(e).size > 0);
    }

    _updateVisibility(entry) {
        if (!entry.fields.some(f => f.visibleIf)) return;
        const values = this.entryValues(entry);
        for (const field of entry.fields) {
            if (!field.visibleIf) continue;
            const row = document.getElementById(`settings-field-${entry.cmd}-${field.id}`);
            row.style.display = field.visibleIf(values) ? '' : 'none';
        }
    }

    _updateDirtyUI() {
        let dirtyCount = 0;
        for (const entry of this.schema) {
            const dirty = this.readOk[entry.cmd] ? this.entryDirtyFields(entry) : new Set();
            dirtyCount += dirty.size > 0 ? 1 : 0;
            for (const field of entry.fields) {
                const row = document.getElementById(`settings-field-${entry.cmd}-${field.id}`);
                if (row) row.classList.toggle('dirty', dirty.has(field.id));
            }
        }
        this.saveBtn.disabled = this.busy || dirtyCount === 0;
        this.saveBtn.textContent = dirtyCount > 0 ? `Save (${dirtyCount})` : 'Save';
        this.refreshBtn.disabled = this.busy;
    }

    _setEntryEnabled(entry, enabled) {
        for (const field of entry.fields) {
            const input = this._input(entry, field);
            if (!input || field.type === 'display') continue;
            if (field.type === 'bitmask') {
                for (const cb of input.querySelectorAll('input')) cb.disabled = !enabled;
            } else {
                input.disabled = !enabled;
            }
        }
    }

    _entryStatus(entry, text, isError) {
        const el = document.getElementById(`settings-status-${entry.cmd}`);
        el.textContent = text;
        el.classList.toggle('error', !!isError);
    }

    _status(text) { this.statusEl.textContent = text; }

    // ── Default parse/build ──

    // Positional parse: first line matching the query regex, args mapped onto fields in
    // declared order. writeOnly fields consume a position but their value is discarded.
    _defaultParse(entry, lines) {
        if (lines.length === 0) return null;
        const args = atSplitArgs(lines[0]).map(atStripAnnotation);
        const values = {};
        entry.fields.forEach((field, i) => {
            if (i >= args.length) return;
            if (field.writeOnly) return;
            values[field.id] = this._coerce(field, args[i]);
        });
        return values;
    }

    _coerce(field, str) {
        switch (field.type) {
            case 'bool': return str === '1';
            case 'int': return parseInt(str, 10);
            case 'float': return parseFloat(str);
            case 'bitmask': return parseInt(str, /^0x/i.test(str) ? 16 : 10);
            case 'hex': return str.replace(/^0x/i, '').toUpperCase();
            default: return str;
        }
    }

    // Maps one command's value from the bulk JSON dump (an array in field order,
    // writeOnly fields excluded; a scalar counts as a one-element array) onto canonical
    // field values.
    _defaultParseJson(entry, raw) {
        const args = Array.isArray(raw) ? raw : [raw];
        const fields = entry.fields.filter(f => !f.writeOnly);
        const values = {};
        fields.forEach((field, i) => {
            if (i < args.length) values[field.id] = this._coerceJson(field, args[i]);
        });
        return values;
    }

    // JSON values arrive typed (numbers, booleans) unlike line-parse strings.
    _coerceJson(field, v) {
        switch (field.type) {
            case 'bool': return v === true || v === 1 || v === '1';
            case 'int':
            case 'float': return typeof v === 'number' ? v : parseFloat(v);
            case 'bitmask': return typeof v === 'number' ? v : parseInt(v, /^0x/i.test(v) ? 16 : 10);
            case 'hex': return String(v).replace(/^0x/i, '').toUpperCase();
            case 'display': return v;
            default: return String(v);
        }
    }

    _formatArg(field, value) {
        if (field.type === 'bool') return value ? '1' : '0';
        if (value === undefined) return '';
        return String(value);
    }

    _defaultBuild(entry, values) {
        const parts = entry.fields
            .filter(f => f.type !== 'display')
            .map(f => this._formatArg(f, values[f.id]));
        // Trailing writeOnly fields left blank (e.g. an untouched password) are omitted so
        // the device keeps its stored value instead of receiving an empty arg.
        const fields = entry.fields.filter(f => f.type !== 'display');
        while (parts.length > 0 && fields[parts.length - 1].writeOnly && parts[parts.length - 1] === '') {
            parts.pop();
        }
        return [`AT+${entry.cmd}=${parts.join(',')}`];
    }

    // ── Validation ──

    // Returns true if all dirty fields of the entry hold sendable values; marks
    // per-field inline errors otherwise.
    _validateEntry(entry, dirty) {
        let ok = true;
        for (const field of entry.fields) {
            const errEl = document.getElementById(`settings-error-${entry.cmd}-${field.id}`);
            errEl.textContent = '';
            if (!dirty.has(field.id)) continue;
            const value = this._readField(entry, field);
            let msg = '';
            if ((field.type === 'int' || field.type === 'float')) {
                if (value === undefined || Number.isNaN(value)) msg = 'Enter a number.';
                else if (field.min !== undefined && value < field.min) msg = `Minimum is ${field.min}.`;
                else if (field.max !== undefined && value > field.max) msg = `Maximum is ${field.max}.`;
            } else if (typeof value === 'string') {
                if (/[,\r\n]/.test(value)) msg = 'Commas and line breaks are not allowed.';
                else if (field.maxLen && value.length > field.maxLen) msg = `Maximum ${field.maxLen} characters.`;
                else if (field.type === 'hex' && value !== '' && !/^[0-9A-F]+$/.test(value)) msg = 'Enter a hex value.';
            }
            if (msg) { errEl.textContent = msg; ok = false; }
        }
        return ok;
    }

    // ── Refresh ──

    // Stops the in-flight refresh/save loop after its current command completes, e.g.
    // when the settings tab is left mid-read.
    abort() { this.gen++; }

    async refresh() {
        if (this.busy) return;
        this.busy = true;
        const gen = ++this.gen;
        this._updateDirtyUI();
        try {
            // One-round-trip bulk read first; anything it doesn't cover (older firmware,
            // or a schema entry the dump doesn't know) is queried individually.
            const covered = this.bulkQuery ? await this._refreshFromBulk() : null;
            if (gen !== this.gen) return;
            const remaining = this.schema.filter(e => !covered || !covered.has(e.cmd));
            let index = 0;
            for (const entry of remaining) {
                index++;
                this._status(`Reading ${index}/${remaining.length}…`);
                await this._refreshEntry(entry);
                if (gen !== this.gen) return;
            }
            const failed = this.schema.filter(e => !this.readOk[e.cmd]).length;
            this._status(failed === 0 ? 'Settings loaded.' : `Settings loaded (${failed} failed to read).`);
        } finally {
            this.busy = false;
            this._updateDirtyUI();
        }
    }

    // Reads every setting in one round trip via the bulk JSON dump. Returns the Set of
    // cmds successfully populated, or null when the bulk query is unavailable (e.g.
    // firmware predating AT+SETTINGS?JSON responds ERROR) so refresh() falls back to
    // querying each command individually.
    async _refreshFromBulk() {
        this._status('Reading settings…');
        let json;
        try {
            const lines = await this.transport.sendCommand(this.bulkQuery.command, {
                expect: this.bulkQuery.expect,
                terminator: 'ok',
                timeoutMs: this.bulkQuery.timeoutMs ?? 5000,
            });
            const line = lines.find(l => this.bulkQuery.expect.test(l));
            if (!line) return null;
            json = JSON.parse(line.slice(line.indexOf('=') + 1));
        } catch (e) {
            return null;
        }
        const covered = new Set();
        for (const entry of this.schema) {
            const raw = json[entry.cmd];
            if (raw === undefined) continue;
            const values = entry.parseJson ? entry.parseJson(raw) : this._defaultParseJson(entry, raw);
            if (!values) continue;
            this._applyEntryValues(entry, values);
            covered.add(entry.cmd);
        }
        return covered;
    }

    async _refreshEntry(entry) {
        this._entryStatus(entry, '…');
        const query = entry.query || {};
        try {
            const lines = await this.transport.sendCommand(`AT+${entry.cmd}?`, {
                expect: query.expect || new RegExp(`^${entry.cmd}=`),
                terminator: 'quiet',
                quietMs: query.quietMs ?? 350,
                timeoutMs: query.timeoutMs ?? 3000,
            });
            const values = entry.parse ? entry.parse(lines) : this._defaultParse(entry, lines);
            if (!values) throw new Error('No response.');
            this._applyEntryValues(entry, values);
        } catch (e) {
            this.readOk[entry.cmd] = false;
            this._setEntryEnabled(entry, false);
            this._entryStatus(entry, `read failed: ${e.message}`, true);
        }
    }

    _applyEntryValues(entry, values) {
        this.deviceValues[entry.cmd] = values;
        for (const field of entry.fields) {
            if (field.writeOnly) { this._writeField(entry, field, ''); continue; }
            this._writeField(entry, field, values[field.id]);
        }
        this.readOk[entry.cmd] = true;
        this._setEntryEnabled(entry, true);
        this._updateVisibility(entry);
        this._entryStatus(entry, '');
        this._validateEntry(entry, new Set());  // clear stale inline errors
    }

    // ── Save ──

    async save() {
        if (this.busy) return;
        const dirty = this.dirtyEntries();
        if (dirty.length === 0) return;

        // Validate everything up front so a typo doesn't leave a half-applied batch.
        let valid = true;
        for (const entry of dirty) {
            if (!this._validateEntry(entry, this.entryDirtyFields(entry))) valid = false;
        }
        if (!valid) { this._status('Fix the highlighted fields before saving.'); return; }

        const disconnecting = dirty.filter(e => e.disconnects);
        if (disconnecting.length > 0) {
            const names = disconnecting.map(e => e.label).join(', ');
            if (!confirm(`Saving these settings may drop this connection: ${names}.\n\nContinue?`)) return;
        }

        this.busy = true;
        const gen = ++this.gen;
        this._updateDirtyUI();
        let succeeded = 0, failed = 0;
        try {
            // Connection-dropping entries go last so everything else lands first.
            const ordered = [...dirty.filter(e => !e.disconnects), ...disconnecting];
            for (const entry of ordered) {
                if (gen !== this.gen) return;
                this._entryStatus(entry, 'saving…');
                const values = this.entryValues(entry);
                const dirtyIds = this.entryDirtyFields(entry);
                const commands = entry.build ? entry.build(values, dirtyIds)
                                             : this._defaultBuild(entry, values);
                try {
                    for (const cmd of commands) {
                        await this.transport.sendCommand(cmd, { terminator: 'ok', timeoutMs: 5000 });
                    }
                    if (entry.afterWrite) await entry.afterWrite(values);
                    this._entryStatus(entry, 'saved');
                    succeeded++;
                } catch (e) {
                    this._entryStatus(entry, e.message, true);
                    failed++;
                }
            }

            if (succeeded > 0) {
                this._status('Persisting to flash…');
                try {
                    await this.transport.sendCommand('AT+SETTINGS=SAVE', { terminator: 'ok', timeoutMs: 15000 });
                    this._status(failed === 0 ? `Saved ${succeeded} setting${succeeded === 1 ? '' : 's'}.`
                                              : `${succeeded} saved, ${failed} failed — see errors above.`);
                } catch (e) {
                    if (disconnecting.length > 0) {
                        this._status('Settings sent; the connection may have dropped while saving. ' +
                                     'Reconnect and Refresh to verify.');
                    } else {
                        this._status(`Settings applied but AT+SETTINGS=SAVE failed: ${e.message}`);
                    }
                }
            } else {
                this._status('No settings were saved — see errors above.');
            }
        } finally {
            this.busy = false;
        }
        if (gen === this.gen) await this.refresh();
    }
}

// Constants and schema-entry factories for AT commands shared by both products.

const SETTINGS_PROTOCOLS = ['NONE', 'RAW', 'BEAST', 'BEAST_NO_UAT', 'BEAST_NO_UAT_UPLINK', 'CSBEE',
    'MAVLINK1', 'MAVLINK2', 'GDL90', 'AIRCRAFT_JSON', 'GDL90_NO_UAT_UPLINK'];
const SETTINGS_LOG_LEVELS = ['SILENT', 'ERRORS', 'WARNINGS', 'INFO'];

// RX_POSITION is identical on both products; the entry is built by a factory so each
// product's schema can reuse it verbatim.
function makeRxPositionEntry() {
    return {
        cmd: 'RX_POSITION', label: 'Receiver Position', group: 'Position',
        help: 'Where the receiver is located, used for range calculations and position decoding.',
        query: { expect: /^(Receiver Position:|Source:|Latitude:|Longitude:|GNSS Altitude:|Barometric Altitude:|Heading:|Speed:|ICAO:)/ },
        fields: [
            {
                id: 'source', label: 'Source', type: 'enum',
                options: [
                    { value: 'NONE', label: 'None' },
                    { value: 'FIXED', label: 'Fixed (enter below)' },
                    { value: 'GNSS', label: 'GNSS receiver' },
                    { value: 'LOWEST', label: 'Lowest tracked aircraft' },
                    { value: 'ICAO', label: 'Aircraft with ICAO address' },
                ],
            },
            { id: 'avail', label: 'Position Status', type: 'display' },
            { id: 'lat', label: 'Latitude (deg)', type: 'float', min: -90, max: 90, visibleIf: v => v.source === 'FIXED' },
            { id: 'lon', label: 'Longitude (deg)', type: 'float', min: -180, max: 180, visibleIf: v => v.source === 'FIXED' },
            { id: 'gnss_alt', label: 'GNSS Altitude (ft)', type: 'int', visibleIf: v => v.source === 'FIXED' },
            { id: 'baro_alt', label: 'Barometric Altitude (ft)', type: 'int', visibleIf: v => v.source === 'FIXED' },
            { id: 'heading', label: 'Heading (deg)', type: 'float', min: 0, max: 359.9, visibleIf: v => v.source === 'FIXED' },
            { id: 'speed', label: 'Speed (kts)', type: 'int', min: 0, visibleIf: v => v.source === 'FIXED' },
            { id: 'icao', label: 'ICAO Address (hex)', type: 'hex', maxLen: 6, visibleIf: v => v.source === 'ICAO' },
        ],
        parse(lines) {
            const values = {};
            for (const l of lines) {
                let m;
                if ((m = l.match(/^Source:\s*(\w+)\s*\[([^\]]*)\]/))) {
                    values.source = m[1];
                    values.avail = m[2];
                } else if ((m = l.match(/^Latitude:\s*(-?[\d.]+)/))) values.lat = parseFloat(m[1]);
                else if ((m = l.match(/^Longitude:\s*(-?[\d.]+)/))) values.lon = parseFloat(m[1]);
                else if ((m = l.match(/^GNSS Altitude:\s*(-?\d+)/))) values.gnss_alt = parseInt(m[1], 10);
                else if ((m = l.match(/^Barometric Altitude:\s*(-?\d+)/))) values.baro_alt = parseInt(m[1], 10);
                else if ((m = l.match(/^Heading:\s*(-?[\d.]+)/))) values.heading = parseFloat(m[1]);
                else if ((m = l.match(/^Speed:\s*(-?\d+)/))) values.speed = parseInt(m[1], 10);
                else if ((m = l.match(/^ICAO:\s*0x([0-9A-Fa-f]+)/))) values.icao = m[1].toUpperCase();
            }
            return values.source === undefined ? null : values;
        },
        build(v) {
            if (v.source === 'FIXED') {
                return [`AT+RX_POSITION=FIXED,${v.lat},${v.lon},${v.gnss_alt},${v.baro_alt},${v.heading},${v.speed}`];
            }
            if (v.source === 'ICAO') return [`AT+RX_POSITION=ICAO,${v.icao}`];
            return [`AT+RX_POSITION=${v.source}`];
        },
    };
}

// MAVLINK_ID is identical on both products.
function makeMavlinkIdEntry() {
    return {
        cmd: 'MAVLINK_ID', label: 'MAVLink IDs', group: 'Reporting',
        help: 'System and component IDs used when reporting via MAVLink protocols.',
        query: { expect: /^(System|Component) ID:/ },
        fields: [
            { id: 'sys', label: 'System ID', type: 'int', min: 1, max: 255 },
            { id: 'comp', label: 'Component ID', type: 'int', min: 1, max: 255 },
        ],
        parse(lines) {
            const values = {};
            for (const l of lines) {
                let m;
                if ((m = l.match(/^System ID:\s*(\d+)/))) values.sys = parseInt(m[1], 10);
                else if ((m = l.match(/^Component ID:\s*(\d+)/))) values.comp = parseInt(m[1], 10);
            }
            return values.sys === undefined ? null : values;
        },
        build(v) { return [`AT+MAVLINK_ID=${v.sys},${v.comp}`]; },
    };
}

// RX_ENABLE is identical on both products.
function makeRxEnableEntry() {
    return {
        cmd: 'RX_ENABLE', label: 'Receivers', group: 'Radio',
        query: { expect: /^(1090|SubG) Receiver:/ },
        fields: [
            { id: 'r1090', label: '1090 MHz receiver enabled', type: 'bool' },
            { id: 'subg', label: 'Sub-GHz receiver enabled', type: 'bool' },
        ],
        parse(lines) {
            const values = {};
            for (const l of lines) {
                let m;
                if ((m = l.match(/^1090 Receiver:\s*(\w+)/))) values.r1090 = m[1] === 'ENABLED';
                else if ((m = l.match(/^SubG Receiver:\s*(\w+)/))) values.subg = m[1] === 'ENABLED';
            }
            return values.r1090 === undefined ? null : values;
        },
        // Leading empty arg skips the all-receivers override argument.
        build(v) { return [`AT+RX_ENABLE=,${v.r1090 ? 1 : 0},${v.subg ? 1 : 0}`]; },
    };
}

// ══════════ END SHARED SETTINGS ENGINE ══════════

// ─── ADSBee 1090 settings schema ─────────────────────────────────────────────
// Query/write formats mirror the AT callbacks in
// firmware/adsbee_1090/pico/application/comms/at/comms_at.cc — parse() functions are
// pinned to those printf strings.

const SETTINGS_RID_TRANSPORTS = [
    { bit: 1, label: 'Bluetooth 4 (legacy)' },
    { bit: 2, label: 'Bluetooth 5 Long Range' },
    { bit: 4, label: 'WiFi Beacon' },
];

const SETTINGS_SCHEMA_1090 = [
    // ── Network ──
    {
        cmd: 'HOSTNAME', label: 'Hostname', group: 'Network',
        help: 'mDNS hostname used to reach this page (http://<hostname>.local).',
        fields: [{ id: 'hostname', label: 'Hostname', type: 'string', maxLen: 32 }],
    },
    {
        cmd: 'WIFI_AP', label: 'WiFi Access Point', group: 'Network', disconnects: true,
        help: 'WiFi network hosted by the ADSBee.',
        fields: [
            { id: 'en', label: 'Enabled', type: 'bool' },
            { id: 'ssid', label: 'SSID', type: 'string', maxLen: 31 },
            { id: 'pwd', label: 'Password', type: 'password', maxLen: 63 },
            { id: 'channel', label: 'Channel', type: 'int', min: 1, max: 11 },
        ],
    },
    {
        cmd: 'WIFI_STA', label: 'WiFi Station', group: 'Network', disconnects: true,
        help: 'Existing WiFi network the ADSBee joins.',
        fields: [
            { id: 'en', label: 'Enabled', type: 'bool' },
            { id: 'ssid', label: 'SSID', type: 'string', maxLen: 31 },
            {
                id: 'pwd', label: 'Password', type: 'password', writeOnly: true, maxLen: 63,
                placeholder: '(unchanged)', note: 'Leave blank to keep the stored password.',
            },
        ],
    },
    {
        cmd: 'ETHERNET', label: 'Ethernet', group: 'Network', disconnects: true,
        fields: [{ id: 'en', label: 'Enabled', type: 'bool' }],
    },
    // ── Radio ──
    makeRxEnableEntry(),
    {
        cmd: 'BIAS_TEE_ENABLE', label: 'Bias Tees', group: 'Radio',
        help: 'DC power on the antenna ports for active antennas.',
        fields: [
            { id: 'bt_1090', label: '1090 MHz bias tee enabled', type: 'bool' },
            { id: 'bt_subg', label: 'Sub-GHz bias tee enabled', type: 'bool' },
        ],
    },
    {
        cmd: 'SUBG_ENABLE', label: 'Sub-GHz Radio', group: 'Radio',
        fields: [{
            id: 'state', label: 'State', type: 'enum',
            options: [
                { value: '0', label: 'Disabled' },
                { value: '1', label: 'Enabled' },
                { value: 'EXTERNAL', label: 'External' },
            ],
        }],
    },
    {
        cmd: 'TL_OFFSET', label: 'Trigger Level Offset', group: 'Radio',
        help: 'Offset above the noise floor for 1090 MHz demodulation, in millivolts.',
        fields: [
            { id: 'offset_mv', label: 'Offset (mV)', type: 'int', min: 0 },
            { id: 'dbm', label: 'Equivalent Power', type: 'display' },
        ],
        parse(lines) {
            const m = lines[0] && lines[0].match(/^TL_OFFSET=(\d+)mV\s*\((-?\d+) dBm\)/);
            return m ? { offset_mv: parseInt(m[1], 10), dbm: `${m[2]} dBm` } : null;
        },
        build(v) { return [`AT+TL_OFFSET=${v.offset_mv}`]; },
    },
    // ── Reporting ──
    {
        cmd: 'PROTOCOL_OUT', label: 'Output Protocols', group: 'Reporting',
        help: 'Reporting protocol emitted on each serial interface.',
        fields: [
            { id: 'CONSOLE', label: 'Console', type: 'enum', options: SETTINGS_PROTOCOLS },
            { id: 'COMMS_UART', label: 'Comms UART', type: 'enum', options: SETTINGS_PROTOCOLS },
        ],
        parse(lines) {
            const values = {};
            for (const l of lines) {
                const m = l.match(/^PROTOCOL_OUT=(\w+),(\S+)/);
                if (m) values[m[1]] = m[2];
            }
            return values.CONSOLE === undefined ? null : values;
        },
        build(v, dirty) { return [...dirty].map(iface => `AT+PROTOCOL_OUT=${iface},${v[iface]}`); },
    },
    makeMavlinkIdEntry(),
    // ── Position ──
    makeRxPositionEntry(),
    {
        cmd: 'GNSS', label: 'GNSS Receiver', group: 'Position',
        fields: [
            { id: 'en', label: 'Enabled', type: 'bool' },
            { id: 'type', label: 'Receiver Type', type: 'enum', options: ['NONE', 'GENERIC', 'UBX_MIA'] },
            { id: 'notify', label: 'Print fix notifications', type: 'bool' },
        ],
    },
    // ── Remote ID ──
    {
        cmd: 'REMOTE_ID', label: 'Remote ID Receive', group: 'Remote ID', experimental: true,
        help: 'Receive Broadcast Remote ID (drone ID) messages. Not yet a stable part of the system \u2014 these settings and their behavior may change in future firmware.',
        fields: [
            { id: 'en', label: 'Enabled', type: 'bool' },
            { id: 'transports', label: 'Transports', type: 'bitmask', options: SETTINGS_RID_TRANSPORTS },
            { id: 'status', label: 'ESP32 Status', type: 'display' },
        ],
        build(v) { return [`AT+REMOTE_ID=${v.en ? 1 : 0},${v.transports}`]; },
    },
    {
        cmd: 'REMOTE_ID_TX', label: 'Remote ID Transmit', group: 'Remote ID', experimental: true,
        help: 'Transmit Broadcast Remote ID from this device. Only the Fixed and GNSS receiver position sources are transmitted, since they describe this device; the aircraft-derived sources carry a received aircraft\u2019s position, so an unknown position is broadcast instead. Not yet a stable part of the system \u2014 these settings and their behavior may change in future firmware.',
        fields: [
            { id: 'en', label: 'Enabled', type: 'bool' },
            { id: 'transports', label: 'Transports', type: 'bitmask', options: SETTINGS_RID_TRANSPORTS },
            {
                id: 'uas_id', label: 'UAS ID', type: 'string', maxLen: 20,
                placeholder: '(device serial)', note: 'Leave blank to use this device\'s serial number.',
            },
            {
                id: 'id_type', label: 'ID Type', type: 'enum',
                options: [
                    { value: '1', label: 'Serial Number' },
                    { value: '2', label: 'CAA Registration' },
                    { value: '3', label: 'UTM UUID' },
                    { value: '4', label: 'Session ID' },
                ],
            },
            {
                id: 'ua_type', label: 'UA Type', type: 'enum',
                options: [
                    { value: '1', label: 'Aeroplane' },
                    { value: '2', label: 'Helicopter / Multirotor' },
                    { value: '3', label: 'Gyroplane' },
                    { value: '4', label: 'Hybrid Lift' },
                    { value: '5', label: 'Ornithopter' },
                    { value: '6', label: 'Glider' },
                    { value: '7', label: 'Kite' },
                    { value: '8', label: 'Free Balloon' },
                    { value: '9', label: 'Captive Balloon' },
                    { value: '10', label: 'Airship' },
                    { value: '11', label: 'Parachute' },
                    { value: '12', label: 'Rocket' },
                    { value: '13', label: 'Tethered Powered Aircraft' },
                    { value: '14', label: 'Ground Obstacle' },
                    { value: '15', label: 'Other' },
                ],
            },
            { id: 'operator_id', label: 'Operator ID', type: 'string', maxLen: 20, placeholder: '(none)' },
            { id: 'status', label: 'ESP32 Status', type: 'display' },
        ],
        parse(lines) {
            // REMOTE_ID_TX=<en>,0xXX(TRANSPORTS),<uas>(UAS_ID),<n>(ID_TYPE),<n>(UA_TYPE),<op>(OPERATOR_ID),0xXXXX(ESP32_STATUS)
            if (!lines[0]) return null;
            const args = atSplitArgs(lines[0]).map(atStripAnnotation);
            if (args.length < 7) return null;
            const clearSentinel = (s, sentinel) => (s === sentinel ? '' : s);
            return {
                en: args[0] === '1',
                transports: parseInt(args[1], 16),
                uas_id: clearSentinel(args[2], '(device serial)'),
                id_type: args[3],
                ua_type: args[4],
                operator_id: clearSentinel(args[5], '(none)'),
                status: args[6],
            };
        },
        // '-' clears a blank ID so the device falls back to its default.
        build(v) {
            return [`AT+REMOTE_ID_TX=${v.en ? 1 : 0},${v.transports},${v.uas_id || '-'},${v.id_type},${v.ua_type},${v.operator_id || '-'}`];
        },
    },
    // ── Serial ──
    {
        cmd: 'BAUD_RATE', label: 'UART Baud Rates', group: 'Serial',
        fields: [
            { id: 'COMMS_UART', label: 'Comms UART (baud)', type: 'int', min: 1200, max: 1000000 },
            { id: 'GNSS_UART', label: 'GNSS UART (baud)', type: 'int', min: 1200, max: 1000000 },
        ],
        build(v, dirty) { return [...dirty].map(iface => `AT+BAUD_RATE=${iface},${v[iface]}`); },
    },
    // ── System ──
    {
        cmd: 'LOG_LEVEL', label: 'Console Log Level', group: 'System',
        fields: [{ id: 'level', label: 'Log Level', type: 'enum', options: SETTINGS_LOG_LEVELS }],
    },
    {
        cmd: 'LED_ENABLE', label: 'Status LEDs', group: 'System',
        fields: [{ id: 'en', label: 'Enabled', type: 'bool' }],
    },
    {
        cmd: 'WATCHDOG', label: 'Watchdog', group: 'System',
        help: 'Reboot automatically if the firmware locks up. 0 disables the watchdog.',
        fields: [{ id: 'timeout', label: 'Timeout (seconds)', type: 'int', min: 0, max: 65535 }],
    },
    {
        cmd: 'ESP32_ENABLE', label: 'ESP32 (Network Processor)', group: 'System', disconnects: true,
        help: 'Disabling the ESP32 takes down WiFi, Ethernet, and this web page.',
        fields: [{ id: 'en', label: 'Enabled', type: 'bool' }],
    },
];

// ─── 1090 transport: dedicated console websocket ─────────────────────────────
// Queries are silent-success (body lines, no OK), so completion is detected with a
// quiet timer, mirroring the 1421 console's AtQueue. Uses its own /console connection
// (like ADSBeeAT) so capture state never collides with the terminal's socket; the
// terminal socket is paused while the Settings tab is active, so the broadcast echo of
// settings traffic never reaches the visible terminal.
class Settings1090Transport {
    constructor(url) {
        this.url = url;
        this.ws = null;
        this.chain = Promise.resolve();
        this.current = null;
        this.lineBuf = '';
    }

    connect() {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) return Promise.resolve();
        return new Promise((resolve, reject) => {
            const ws = new WebSocket(this.url);
            this.ws = ws;
            ws.addEventListener('open', () => resolve());
            ws.addEventListener('message', (event) => {
                if (typeof event.data === 'string') {
                    this._onText(event.data);
                } else if (event.data instanceof Blob) {
                    const reader = new FileReader();
                    reader.onload = () => this._onText(reader.result);
                    reader.readAsText(event.data);
                } else if (event.data instanceof ArrayBuffer) {
                    this._onText(new TextDecoder().decode(event.data));
                }
            });
            ws.addEventListener('close', () => {
                if (this.ws === ws) this.ws = null;
                reject(new Error('Console connection closed.'));
                if (this.current) this.current.finish(new Error('Console connection closed.'));
            });
            ws.addEventListener('error', () => { /* close fires afterwards */ });
        });
    }

    disconnect() {
        if (this.current) this.current.finish(new Error('Settings tab closed.'));
        if (this.ws) {
            this.ws.close(1000, 'Settings tab closed');
            this.ws = null;
        }
        this.lineBuf = '';
    }

    // Same contract as AtQueue.send() on the 1421: serialized, one command in flight.
    sendCommand(cmd, opts = {}) {
        const job = {
            cmd,
            expect: opts.expect || null,
            terminator: opts.terminator || 'ok',
            quietMs: opts.quietMs ?? 350,
            timeoutMs: opts.timeoutMs ?? 5000,
        };
        const p = this.chain.then(() => this._run(job));
        this.chain = p.then(() => { }, () => { });
        return p;
    }

    async _run(job) {
        await this.connect();
        return new Promise((resolve, reject) => {
            const state = { job, lines: [], quietTimer: null, timeoutTimer: null };
            state.finish = (err) => {
                if (this.current !== state) return;
                clearTimeout(state.timeoutTimer);
                clearTimeout(state.quietTimer);
                this.current = null;
                if (err) reject(err); else resolve(state.lines);
            };
            state.armQuiet = () => {
                clearTimeout(state.quietTimer);
                state.quietTimer = setTimeout(() => state.finish(), job.quietMs);
            };
            this.current = state;
            state.timeoutTimer = setTimeout(
                () => state.finish(new Error(`Timeout waiting for response to ${job.cmd}.`)), job.timeoutMs);
            if (job.terminator === 'quiet') state.armQuiet();
            this.ws.send(job.cmd + '\r\n');
        });
    }

    _onText(text) {
        this.lineBuf += text;
        let nl;
        while ((nl = this.lineBuf.search(/[\r\n]/)) >= 0) {
            const line = this.lineBuf.slice(0, nl).trim();
            this.lineBuf = this.lineBuf.slice(nl + 1);
            if (line !== '') this._onLine(line);
        }
    }

    _onLine(line) {
        const state = this.current;
        if (!state) return;
        const job = state.job;
        if (line === 'OK') { state.finish(); return; }
        if (line.startsWith('ERROR')) { state.finish(new Error(line)); return; }
        if (job.expect && job.expect.test(line)) {
            state.lines.push(line);
            if (job.terminator === 'quiet') state.armQuiet();
        }
        // Anything else is broadcast chatter (log lines, other clients) — ignored.
    }
}

// ─── Settings tab glue ───────────────────────────────────────────────────────
const settingsTab = {
    engine: null,
    transport: null,

    enter() {
        if (!this.engine) {
            this.transport = new Settings1090Transport(WS_CONFIG.console_ws_url);
            this.engine = new SettingsEngine({
                schema: SETTINGS_SCHEMA_1090,
                transport: this.transport,
                bulkQuery: { command: 'AT+SETTINGS?JSON', expect: /^SETTINGS=/ },
                formEl: document.getElementById('settings-form'),
                statusEl: document.getElementById('settings-status'),
                saveBtn: document.getElementById('settings-save-btn'),
                refreshBtn: document.getElementById('settings-refresh-btn'),
            });
            this.engine.render();
        }
        this.engine.refresh().catch(e => console.error('Settings refresh error:', e));
    },

    leave() {
        if (this.engine) this.engine.abort();
        if (this.transport) this.transport.disconnect();
    },
};
