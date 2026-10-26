const METRIC_UNITS = {
    'num_mode_s_aircraft': 'aircraft',
    'num_uat_aircraft': 'aircraft',
    'num_remote_id_aircraft': 'drones',
};

// Input types that take no typed or pasted text.
const NON_TEXT_INPUT_TYPES = new Set(['button', 'checkbox', 'color', 'file', 'hidden', 'image', 'radio', 'range', 'reset', 'submit']);

/**
 * True when target is a text box other than the console terminal's hidden input. The terminal's document-wide
 * copy/paste handlers leave these alone so pasting into settings and feed fields works.
 */
function isOtherTextEntry(target, terminalInput) {
    if (!target || target === terminalInput) return false;
    if (target.isContentEditable) return true;
    if (target.tagName === 'TEXTAREA' || target.tagName === 'SELECT') return true;
    return target.tagName === 'INPUT' && !NON_TEXT_INPUT_TYPES.has((target.type || 'text').toLowerCase());
}

class ConsoleWebSocket {
    constructor(url) {
        this.url = url;
        this.paused = false;
        this.captureCallback = null;
        this._reconnectTimer = null;
        // The firmware update check keeps its AT+DEVICE_INFO? reply out of the terminal.
        this.replyFilter = AdsbeeFirmwareRelease.deviceInfoFilter();
        this._flushTimer = null;
        this.init();
    }

    // Hides the AT+DEVICE_INFO? reply for ms; other console text still shows.
    hideDeviceInfoReply(ms) {
        this.replyFilter.hide(ms);
        clearTimeout(this._flushTimer);
        this._flushTimer = setTimeout(() => {
            const held = this.replyFilter.flush();
            if (held) appendToTerminal(held);
        }, ms + 50);
    }

    init() {
        this.ws = new WebSocket(this.url)
        this.attachEventHandlers();
    }

    attachEventHandlers() {
        // Connection opened
        this.ws.addEventListener('open', () => {
            console.log(`Connected to ${this.url}`);
        });

        // Listen for messages
        this.ws.addEventListener('message', (event) => {
            const dispatch = (text) => {
                const shown = this.replyFilter.filter(text);
                if (shown) appendToTerminal(shown);
                if (this.captureCallback) this.captureCallback(text);
            };
            if (typeof event.data === 'string') {
                dispatch(event.data);
            } else if (event.data instanceof Blob) {
                const reader = new FileReader();
                reader.onload = () => dispatch(reader.result);
                reader.readAsText(event.data);
            } else if (event.data instanceof ArrayBuffer) {
                dispatch(new TextDecoder().decode(event.data));
            }
        });

        // Connection closed (both clean and unclean closes)
        this.ws.addEventListener('close', (event) => {
            console.log(`Connection closed. Code: ${event.code}, Reason: ${event.reason}`);
            appendToTerminal(`Connection closed. Code: ${event.code}\n`);
            this.attemptReconnect();
        });

        // Connection error
        this.ws.addEventListener('error', (error) => {
            console.error('WebSocket error:', error);
            // Don't call attemptReconnect here as the close event will be fired after error
        });
    }

    send(message) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(message);
            console.log(`Sent message: ${message}`);
            return true;
        } else {
            console.warn('Cannot send message - connection is not open');
            return false;
        }
    }

    // Send a command and collect incoming messages until one contains `terminator`.
    // 'OK' and 'ERROR' are matched as whole trimmed lines; anything else as a substring.
    sendAndCapture(command, terminator, timeoutMs = 5000) {
        return new Promise((resolve, reject) => {
            const captured = [];
            const isLineTerminator = (terminator === 'OK' || terminator === 'ERROR');
            const matches = (text) => isLineTerminator
                ? text.split(/[\r\n]+/).some(l => l.trim() === terminator)
                : text.includes(terminator);

            const timer = setTimeout(() => {
                this.captureCallback = null;
                reject(new Error(`Response timeout after ${timeoutMs} ms.`));
            }, timeoutMs);

            this.captureCallback = (text) => {
                captured.push(text);
                // A failed command prints "ERROR <reason>"; log lines start with "ERROR:".
                const errorLine = text.split(/[\r\n]+/).map(l => l.trim()).find(l => /^ERROR(?!:)\b/.test(l));
                if (errorLine !== undefined) {
                    clearTimeout(timer);
                    this.captureCallback = null;
                    reject(new Error(errorLine.replace(/^ERROR\s*/, '') || 'ERROR response'));
                } else if (matches(isLineTerminator ? text : captured.join(''))) {
                    clearTimeout(timer);
                    this.captureCallback = null;
                    resolve(captured);
                }
            };

            this.send(command);
        });
    }

    attemptReconnect() {
        if (this._reconnectTimer) return;
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            if (!this.paused && (!this.ws || this.ws.readyState === WebSocket.CLOSED)) {
                this.init();
            }
        }, WS_CONFIG.reconnectDelayMs);
    }

    pause() { this.paused = true; if (this.ws) this.ws.close(); }
    resume() { if (this.paused) { this.paused = false; this.init(); } }

    close(code, reason) {
        if (this.ws) {
            this.ws.close(code, reason);
        }
    }

    getState() {
        const states = {
            0: 'CONNECTING',
            1: 'OPEN',
            2: 'CLOSING',
            3: 'CLOSED'
        };
        return states[this.ws ? this.ws.readyState : 3];
    }

    // Method to manually trigger reconnection
    reconnect() {
        if (this.ws) {
            this.ws.close();
        } else {
            this.attemptReconnect();
        }
    }

}

class SparklineChart {
    // options.zeroBaseline (default true): scale from 0 to the max value, for rates and counts.
    // options.zeroBaseline = false: autoscale between the min and max visible values (for levels such as the noise
    // floor, which never approach 0), padded so a flat trace sits mid-chart. options.minSpan is the smallest
    // full-scale range in that mode, so sub-LSB jitter doesn't fill the chart.
    constructor(id, windowSeconds = 20, options = {}) {
        this.svg = document.getElementById(id);
        this.windowSeconds = windowSeconds;
        this.zeroBaseline = options.zeroBaseline !== false;
        this.minSpan = options.minSpan || 0;
        this.buffer = [];
        this.valueEl = this.svg.parentNode.querySelector('.metric-value');
        this._createPaths();
        this._startRAF();
    }

    _createPaths() {
        const area = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        area.classList.add('area');
        this.svg.appendChild(area);
        this.svg.appendChild(line);
        this.linePath = line;
        this.areaPath = area;
    }

    _startRAF() {
        const tick = () => {
            this._render();
            this._rafId = requestAnimationFrame(tick);
        };
        this._rafId = requestAnimationFrame(tick);
    }

    push(value, unit = 'msg/s', displayText = null) {
        this.buffer.push({ value, ts: performance.now() });
        this.valueEl.textContent = displayText ?? `${value} ${unit}`;
        const cutoff = performance.now() - (this.windowSeconds + 5) * 1000;
        while (this.buffer.length > 1 && this.buffer[0].ts < cutoff) this.buffer.shift();
    }

    _render() {
        const now = performance.now();
        const windowMs = this.windowSeconds * 1000;
        const startMs = now - windowMs;

        // Collect visible points; prepend one point before the window so the
        // line reaches the left edge instead of starting mid-chart.
        const pts = [];
        for (let i = 0; i < this.buffer.length; i++) {
            if (this.buffer[i].ts >= startMs) {
                if (i > 0 && pts.length === 0) pts.push(this.buffer[i - 1]);
                pts.push(this.buffer[i]);
            }
        }
        if (pts.length < 2) return;

        const values = pts.map(p => p.value);
        let min = 0;
        let max = Math.max(...values, 1);
        if (!this.zeroBaseline) {
            min = Math.min(...values);
            max = Math.max(...values);
            const pad = Math.max((max - min) * 0.25, this.minSpan / 2);
            min -= pad;
            max += pad;
        }
        const span = max - min || 1;
        const toXY = p => [
            ((p.ts - startMs) / windowMs) * 100,
            30 - ((p.value - min) / span) * 28,
        ];
        const coords = pts.map(toXY);
        const lineCmds = coords.map((p, i) => `${i === 0 ? 'M' : 'L'}${p[0].toFixed(2)},${p[1].toFixed(2)}`).join(' ');
        const areaCmds = `${lineCmds} L${coords[coords.length - 1][0].toFixed(2)},30 L${coords[0][0].toFixed(2)},30 Z`;

        this.linePath.setAttribute('d', lineCmds);
        this.areaPath.setAttribute('d', areaCmds);
    }

    destroy() {
        if (this._rafId) { cancelAnimationFrame(this._rafId); this._rafId = null; }
    }
}

// BEGIN RECEIVER_METRICS
// Error correction kinds that drop a frame instead of fixing it.
const UNCORRECTED_KINDS = new Set(['rejected', 'skipped']);

/**
 * Maps aircraft_dictionary_metrics to receiver cards, {label: {value, tip}}. corrected_frames ({kind: count}) becomes
 * one card with the corrected total and every count in its tooltip. Other non-numeric values are skipped.
 */
function receiverMetricCards(metrics) {
    const cards = {};
    for (const [label, value] of Object.entries(metrics || {})) {
        if (typeof value === 'number') {
            cards[label] = { value, tip: null };
        } else if (label === 'corrected_frames' && value && typeof value === 'object' && !Array.isArray(value)) {
            const counts = Object.entries(value).filter(([, n]) => typeof n === 'number');
            const total = counts.filter(([kind]) => !UNCORRECTED_KINDS.has(kind)).reduce((sum, [, n]) => sum + n, 0);
            cards[label] = { value: total, tip: counts.map(([kind, n]) => `${kind}: ${n}`).join('\n') };
        }
    }
    return cards;
}
// END RECEIVER_METRICS

class MetricCard {
    constructor(container, label, unit = 'msg/s', feedSlot = null, chartOptions = {}, title = label) {
        this.container = container;
        this.label = label;
        this.unit = unit;
        this.feedSlot = feedSlot;
        this.id = `chart-${this.labelToID(label)}`;

        const card = document.createElement('div');
        card.className = 'metrics-card';
        if (feedSlot !== null) {
            card.classList.add('editable');
            card.onclick = () => FeedEditor.open(feedSlot);
        }
        const slotBadge = feedSlot !== null
            ? `<span class="feed-slot-badge">#${feedSlot}</span>`
            : '';
        card.innerHTML = `
  <div class="metric-title" title="${title}">${title}${slotBadge}</div>
  <div class="metric-value">0 ${unit}</div>
  <svg class="sparkline" id="${this.id}" viewBox="0 0 100 30"></svg>
`;
        this.card = card;
        this.container.appendChild(card);
        this.chart = new SparklineChart(this.id, 20, chartOptions);
    }

    destroy() {
        this.chart.destroy();
        this.container.removeChild(this.card);
    }

    // Receiver ID chip under the title of a feed card.
    setReceiverId(id) {
        if (!id || id === this.receiverId) return;
        this.receiverId = id;
        const chip = receiverIdChip(id);
        if (this.ridChip) this.card.replaceChild(chip, this.ridChip);
        else this.card.insertBefore(chip, this.card.children[1] || null);
        this.ridChip = chip;
    }

    labelToID(label) {
        return label.replace(/[^a-zA-Z0-9]/g, '-');
    }

    // displayText overrides the "<value> <unit>" readout while value still drives the sparkline.
    update(value, displayText = null) {
        this.chart.push(value, this.unit, displayText);
    }

    // Hover text for the whole card.
    setTip(tip) {
        if (tip === this.tip) return;
        this.tip = tip;
        this.card.title = tip || '';
        this.card.querySelector('.metric-title').title = tip || this.label;
    }
}

class MetricsWebSocket {
    constructor(url) {
        this.url = url;
        this.paused = false;
        this.cards = {
            'feed': {},
            'receiver': {}
        };
        this.noiseFloorCard = null;
        this.feedSlotMap = {};
        this.onGNSSStatus = null;
        this.onAutoUpdate = null;
        this.connect();
        this.feedsCountEl = document.getElementById('feed-count');
        this.container = document.getElementById('metrics-container');
    }

    updateReceiverCards(data) {
        const parentLabel = 'receiver';
        const container = document.getElementById('receiver-metrics-container');
        Object.keys(this.cards[parentLabel]).forEach(label => {
            if (!data.hasOwnProperty(label)) this.cards[parentLabel][label].destroy();
        });
        Object.entries(data).forEach(([label, { value, tip }]) => {
            if (!this.cards[parentLabel][label]) {
                const unit = METRIC_UNITS[label] || 'msg/s';
                this.cards[parentLabel][label] = new MetricCard(container, label, unit);
            }
            this.cards[parentLabel][label].update(value);
            this.cards[parentLabel][label].setTip(tip);
        });
    }

    // Live 1090 noise floor from the RP2040, shown as a sparkline card alongside the receiver metrics. The sparkline
    // plots the RSSI voltage (finer than the integer dBm reading and in the same units as the TL offset), while the
    // readout shows both.
    updateNoiseFloorCard(rp2040Status) {
        if (!rp2040Status || rp2040Status.noise_floor_mv === undefined) return;
        if (!this.noiseFloorCard) {
            const container = document.getElementById('receiver-metrics-container');
            this.noiseFloorCard = new MetricCard(container, 'noise_floor', 'dBm', null,
                { zeroBaseline: false, minSpan: 50 });
        }
        this.noiseFloorCard.update(rp2040Status.noise_floor_mv,
            `${rp2040Status.noise_floor_dbm} dBm (${rp2040Status.noise_floor_mv} mV)`);
    }

    updateFeedCards(data) {
        const parentLabel = 'feed';
        const container = document.getElementById('feed-metrics-container');
        Object.keys(this.cards[parentLabel]).forEach(label => {
            if (!data.hasOwnProperty(label)) this.cards[parentLabel][label].destroy();
        });
        Object.entries(data).forEach(([label, value]) => {
            if (!this.cards[parentLabel][label]) {
                const slot = this.feedSlotMap[label] ?? null;
                // MQTT feeds have no URI: they publish to the shared broker.
                const title = label.startsWith('mqtt#') ? 'MQTT' : label;
                this.cards[parentLabel][label] = new MetricCard(container, label, 'msg/s', slot, {}, title);
            }
            this.cards[parentLabel][label].update(value);
        });
        this.updateFeedReceiverIds();
    }

    updateFeedReceiverIds() {
        const ids = this._feedReceiverIds || [];
        Object.values(this.cards['feed']).forEach(card => {
            if (card.feedSlot !== null && ids[card.feedSlot]) card.setReceiverId(ids[card.feedSlot]);
        });
    }

    // Bar of the ESP32 RAM the active feeds take out of this unit's budget.
    updateFeedRAM(usedBytes, budgetBytes) {
        const bar = document.getElementById('feed-ram');
        if (!bar || !budgetBytes) return;
        const percent = Math.min(100, Math.round(100 * usedBytes / budgetBytes));
        const fill = document.getElementById('feed-ram-fill');
        fill.style.width = `${percent}%`;
        fill.className = percent >= 100 ? 'full' : percent >= 75 ? 'high' : '';
        bar.dataset.tip = `ESP32 RAM for feeds: ${Math.round(usedBytes / 1024)} of ${Math.round(budgetBytes / 1024)} kB. `
            + 'TCP feeds take the most, UDP the least; all MQTT feeds share one connection (TLS adds to it).';
        bar.style.display = '';
    }

    updateDeviceStatus(deviceStatus) {
        const container = document.getElementById('device-status-cards-container');

        // Clear existing content
        container.innerHTML = '';

        // Create cards for each device
        Object.entries(deviceStatus).forEach(([deviceName, status]) => {
            const card = document.createElement('div');
            card.className = 'device-status-card';

            let statusHtml = `<div class="device-card">
                                <div class="device-status-title">${deviceName.toUpperCase()}</div>`;

            // Add status information
            Object.entries(status).forEach(([key, value]) => {
                if (key === 'noise_floor_mv') return;  // Folded into the noise_floor_dbm row.
                let displayKey = key.replace(/_/g, ' ').replace(/\b\w/g, l => l.toUpperCase());
                let displayValue = value;

                // Format specific values
                if (key === 'noise_floor_dbm') {
                    displayKey = 'Noise Floor';
                    displayValue = status.noise_floor_mv !== undefined
                        ? `${value} dBm (${status.noise_floor_mv} mV)`
                        : `${value} dBm`;
                } else if (key.includes('uptime_ms')) {
                    displayKey = 'Uptime';
                    displayValue = `${Math.floor(value / 1000)}s`;
                } else if (key.includes('temperature')) {
                    displayKey = 'Temperature';
                    displayValue = `${value}\u00B0C`;
                } else if (key.includes('usage_percent')) {
                    displayKey = displayKey.replace(' Percent', '');
                    displayValue = `${value}%`;
                } else if (key.includes('heap_free_bytes')) {
                    displayKey = 'Free Heap';
                    displayValue = `${value} B`;
                } else if (key.includes('heap_largest_free_block_bytes')) {
                    displayKey = 'Largest Free Block';
                    displayValue = `${value} B`;
                } else if (key === 'enabled' || key === 'fix_valid') {
                    displayValue = value ? 'Yes' : 'No';
                } else if (key === 'latitude_deg' || key === 'longitude_deg') {
                    displayValue = Number(value).toFixed(6);
                } else if (key === 'altitude_ft') {
                    displayKey = 'Altitude';
                    displayValue = `${Math.round(value).toLocaleString()} ft`;
                } else if (key === 'heading_deg') {
                    displayKey = 'Heading';
                    displayValue = `${Number(value).toFixed(1)}°`;
                } else if (key === 'speed_kts') {
                    displayKey = 'Speed';
                    displayValue = `${value} kts`;
                } else if (key === 'num_satellites') {
                    displayKey = 'Satellites';
                    displayValue = `${value}`;
                }

                statusHtml += `<div class="status-item">
                                    <span class="status-key">${displayKey}:</span>
                                    <span class="status-value">${displayValue}</span>
                                </div>`;
            });
            statusHtml += `</div>`;

            card.innerHTML = statusHtml;
            container.appendChild(card);
        });
    }

    connect() {
        this.ws = new WebSocket(this.url);

        this.pingInterval = setInterval(() => {
            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                this.ws.send('ping');
            }
        }, 60000);

        this.ws.onopen = () => {
            console.log('Connected to WebSocket');
        };

        this.ws.onclose = () => {
            clearInterval(this.pingInterval);
            console.log('Disconnected from WebSocket');
            setTimeout(() => { if (!this.paused) this.connect(); }, 3000);
        };

        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);

                if (data.hasOwnProperty('aircraft_dictionary_metrics')) {
                    this.updateReceiverCards(receiverMetricCards(data['aircraft_dictionary_metrics']));
                }
                if (data.hasOwnProperty('server_metrics')) {
                    const serverMetrics = data['server_metrics'];
                    this._feedUris = serverMetrics['feed_uri'];
                    this._feedTransports = serverMetrics['feed_transport'] || [];
                    const feedMetrics = {};
                    for (let i = 0; i < serverMetrics['feed_uri'].length; i++) {
                        const label = feedLabel(serverMetrics['feed_uri'][i], this._feedTransports[i], i);
                        if (label === null) continue;
                        feedMetrics[label] = serverMetrics['feed_mps'][i];
                        this.feedSlotMap[label] = i;
                    }
                    this.updateFeedRAM(serverMetrics['feed_ram_bytes'], serverMetrics['feed_ram_budget_bytes']);
                    this.updateFeedCards(feedMetrics);
                    this.feedsCountEl.textContent = `Feeds: ${Object.keys(feedMetrics).length}`;
                    const addBtn = document.getElementById('add-feed-btn');
                    if (addBtn) addBtn.disabled = (this.getFirstEmptySlot() === -1);
                }
                if (data.hasOwnProperty('feed_receiver_id')) {
                    this._feedReceiverIds = data['feed_receiver_id'];
                    this.updateFeedReceiverIds();
                }
                if (data.hasOwnProperty('device_status')) {
                    const deviceStatus = data['device_status'];
                    // console.log('Device Status:', deviceStatus);
                    this.updateDeviceStatus(deviceStatus);
                    this.updateNoiseFloorCard(deviceStatus.rp2040);
                    if (deviceStatus.gnss && this.onGNSSStatus) {
                        this.onGNSSStatus(deviceStatus.gnss);
                    }
                    // ESP32 automatic updates, {enabled, state, version, progress}, from firmware that reports it.
                    if (this.onAutoUpdate) this.onAutoUpdate(deviceStatus.auto_update || null);
                }
            } catch (error) {
                console.error('Error parsing WebSocket message:', error);
            }
        };
    }

    getFirstEmptySlot() {
        if (!this._feedUris) return 0;
        for (let i = 0; i < this._feedUris.length; i++) {
            if (feedLabel(this._feedUris[i], (this._feedTransports || [])[i], i) === null) return i;
        }
        return -1;
    }

    pause() { this.paused = true; if (this.ws) this.ws.close(); }
    resume() { if (this.paused) { this.paused = false; this.connect(); } }

    close(code, reason) {
        if (this.ws) {
            this.ws.close(code, reason);
        }
    }
}

// whereplane.xyz shows one receiver's traffic at globe.whereplane.xyz/?feed=<uuid> (tar1090). The uuid is the one the
// device sends when a Beast feed connects (BeastReporter::BuildFeedStartFrame): MMMMMMMM-MMMM-MMMM-NNNN-NNNNNNNNNNNN,
// where M and N are both the 16 hex digit receiver ID.
const WHEREPLANE_FEED_HOST = 'feed.whereplane.xyz';
const BEAST_FEED_PROTOCOLS = new Set(['BEAST', 'BEAST_NO_UAT', 'BEAST_NO_UAT_UPLINK']);

/** Map URL for this receiver on whereplane.xyz, or null unless the feed is a Beast feed to feed.whereplane.xyz. */
function whereplaneMapUrl(uri, protocol, receiverId) {
    if (String(uri).trim().toLowerCase() !== WHEREPLANE_FEED_HOST || !BEAST_FEED_PROTOCOLS.has(protocol)) return null;
    const id = String(receiverId).toLowerCase();
    if (!/^[0-9a-f]{16}$/.test(id)) return null;
    const uuid = `${id.slice(0, 8)}-${id.slice(8, 12)}-${id.slice(12, 16)}-${id.slice(0, 4)}-${id.slice(4)}`;
    return `https://globe.whereplane.xyz/?feed=${uuid}`;
}

/** Card label of a feed slot, or null if the slot is empty. MQTT feeds have no URI. */
function feedLabel(uri, transport, slot) {
    if (uri) return uri;
    return transport === 'MQTT' ? `mqtt#${slot}` : null;
}

// BEGIN FEED_RECEIVER_ID (same copy in software/adsbee_1090_console; feed_receiver_id.test.js checks it)
const RECEIVER_ID_RE = /^[0-9a-f]{16}$/i;
const RECEIVER_ID_TIP = 'Receiver ID this feed sends. Aggregators (adsb.fi, adsb.lol, airplanes.live...) tell '
    + 'receivers apart by it. Feed 0\'s also names the MQTT topics.';
const RECEIVER_UUID_TIP = 'The same ID as a UUID: whereplane.xyz, readsb and tar1090 show this form.';

/** UUID form of a 16 hex digit receiver ID (BeastReporter::BuildFeedStartFrame), or null. */
function receiverIdUuid(id) {
    const h = String(id || '').toLowerCase();
    if (!RECEIVER_ID_RE.test(h)) return null;
    return `${h.slice(0, 8)}-${h.slice(8, 12)}-${h.slice(12, 16)}-${h.slice(0, 4)}-${h.slice(4)}`;
}

/** {index, id, defaultId} from an AT+FEED_RECEIVER_ID? line, or null. defaultId is '' if the device can't tell. */
function parseFeedReceiverIdLine(text) {
    const m = /FEED_RECEIVER_ID=(\d+)\(INDEX\),([0-9a-fA-F]*)\(RECEIVER_ID\),([0-9a-fA-F]*)\(DEFAULT\)/.exec(text);
    return m ? { index: Number(m[1]), id: m[2].toLowerCase(), defaultId: m[3].toLowerCase() } : null;
}

/** Copies text. The device page is plain HTTP, where navigator.clipboard is missing. */
async function copyText(text) {
    try {
        if (navigator.clipboard && window.isSecureContext) {
            await navigator.clipboard.writeText(text);
            return true;
        }
    } catch (e) { /* fall back */ }
    const ta = document.createElement('textarea');
    ta.value = text;
    ta.style.position = 'fixed';
    ta.style.opacity = '0';
    document.body.appendChild(ta);
    ta.select();
    let ok = false;
    try { ok = document.execCommand('copy'); } catch (e) { ok = false; }
    document.body.removeChild(ta);
    return ok;
}

function ridEl(tag, className, text, title) {
    const el = document.createElement(tag);
    if (className) el.className = className;
    if (text !== undefined) el.textContent = text;
    if (title) el.title = title;
    return el;
}

const RID_COPY_ICON = '<svg viewBox="0 0 16 16" width="13" height="13" fill="none" stroke="currentColor" '
    + 'stroke-width="1.5" aria-hidden="true"><rect x="5" y="5" width="9" height="9" rx="1.5"/>'
    + '<path d="M11 5V3.5A1.5 1.5 0 0 0 9.5 2h-6A1.5 1.5 0 0 0 2 3.5v6A1.5 1.5 0 0 0 3.5 11H5"/></svg>';

/** A copy button that copies text() and shows ✓ for a second. */
function ridCopyButton(text, title) {
    const b = ridEl('button', 'rid-btn rid-copy', undefined, title);
    b.type = 'button';
    b.innerHTML = RID_COPY_ICON;
    b.onclick = async (ev) => {
        if (ev && ev.stopPropagation) ev.stopPropagation();
        b.textContent = (await copyText(text())) ? '✓' : '✗';
        setTimeout(() => { b.innerHTML = RID_COPY_ICON; }, 1000);
    };
    return b;
}

/** Receiver ID chip for a feed card: the 16 hex digits; a click copies them (and doesn't open the editor). */
function receiverIdChip(id) {
    const chip = ridEl('span', 'rid-chip', String(id).toLowerCase(),
        `${RECEIVER_ID_TIP}\n${receiverIdUuid(id) || ''}\nClick to copy.`);
    chip.onclick = async (ev) => {
        if (ev && ev.stopPropagation) ev.stopPropagation();
        if (await copyText(String(id).toLowerCase())) {
            chip.classList.add('copied');
            setTimeout(() => chip.classList.remove('copied'), 1000);
        }
    };
    return chip;
}

/**
 * Receiver ID in the feed editor: hex and UUID with copy buttons, ✎ inline edit, ↺ back to the device default.
 * onSet(value) gets 16 hex digits or 'DEFAULT', and resolves to {id, defaultId} as the device now reports them.
 */
class ReceiverIdField {
    constructor(el, { id, defaultId = null, onSet }) {
        this.el = el;
        this.id = String(id || '').toLowerCase();
        this.defaultId = defaultId ? String(defaultId).toLowerCase() : null;
        this.onSet = onSet;
        this.editing = false;
        this.message = '';
        this.render();
    }

    setDefault(defaultId) {
        this.defaultId = defaultId ? String(defaultId).toLowerCase() : null;
        if (!this.editing) this.render();
    }

    async apply(value) {
        this.message = '…';
        this.render();
        try {
            const now = await this.onSet(value);
            this.id = now.id;
            if (now.defaultId) this.defaultId = now.defaultId;
            this.editing = false;
            this.message = '';
        } catch (e) {
            this.message = `✗ ${e.message}`;
        }
        this.render();
    }

    render() {
        this.uuidCode = null;
        const rows = [];
        const isDefault = this.defaultId !== null && this.id === this.defaultId;
        const hexRow = ridEl('div', 'rid-row');
        hexRow.appendChild(ridEl('span', 'rid-label', 'ID', RECEIVER_ID_TIP));
        if (this.editing) {
            const input = ridEl('input', 'rid-input');
            input.type = 'text';
            input.maxLength = 16;
            input.spellcheck = false;
            input.value = this.draft;
            input.placeholder = this.defaultId || '16 hex digits';
            input.title = '16 hex digits (0-9, a-f)';
            const ok = ridEl('button', 'rid-btn', '✓', 'Set this ID (saved right away)');
            ok.type = 'button';
            const cancel = ridEl('button', 'rid-btn', '✕', 'Cancel');
            cancel.type = 'button';
            const check = () => {
                this.draft = input.value.trim();
                const valid = RECEIVER_ID_RE.test(this.draft);
                ok.disabled = !valid;
                input.className = valid || this.draft === '' ? 'rid-input' : 'rid-input bad';
                if (this.uuidCode) this.uuidCode.textContent = receiverIdUuid(this.draft) || '—';
            };
            input.oninput = check;
            input.onkeydown = (ev) => {
                if (ev.key === 'Enter' && !ok.disabled) { ev.preventDefault(); ok.onclick(); }
                if (ev.key === 'Escape') { ev.preventDefault(); cancel.onclick(); }
            };
            ok.onclick = () => this.apply(this.draft.toLowerCase());
            cancel.onclick = () => { this.editing = false; this.message = ''; this.render(); };
            check();
            hexRow.appendChild(input);
            hexRow.appendChild(ok);
            hexRow.appendChild(cancel);
            this.input = input;
        } else {
            hexRow.appendChild(ridEl('code', 'rid-hex', this.id || '—', RECEIVER_ID_TIP));
            hexRow.appendChild(ridCopyButton(() => this.id, 'Copy the ID'));
            const edit = ridEl('button', 'rid-btn', '✎', 'Edit, e.g. to keep the ID an aggregator already knows');
            edit.type = 'button';
            edit.onclick = () => { this.editing = true; this.draft = this.id; this.render(); if (this.input && this.input.focus) this.input.focus(); };
            hexRow.appendChild(edit);
            if (this.defaultId !== null) {
                const reset = ridEl('button', 'rid-btn', '↺',
                    isDefault ? 'Device default' : `Back to this device's default: ${this.defaultId}`);
                reset.type = 'button';
                reset.disabled = isDefault;
                reset.onclick = () => this.apply('DEFAULT');
                hexRow.appendChild(reset);
                if (!isDefault) hexRow.appendChild(ridEl('span', 'rid-custom', '✱', `Custom. Device default: ${this.defaultId}`));
            }
        }
        rows.push(hexRow);
        const uuid = receiverIdUuid(this.editing ? this.draft : this.id);
        const uuidRow = ridEl('div', 'rid-row');
        uuidRow.appendChild(ridEl('span', 'rid-label', 'UUID', RECEIVER_UUID_TIP));
        this.uuidCode = ridEl('code', 'rid-uuid', uuid || '—', RECEIVER_UUID_TIP);
        uuidRow.appendChild(this.uuidCode);
        if (uuid && !this.editing) uuidRow.appendChild(ridCopyButton(() => receiverIdUuid(this.id), 'Copy the UUID'));
        rows.push(uuidRow);
        if (this.message) rows.push(ridEl('div', 'rid-msg', this.message));
        this.el.replaceChildren(...rows);
    }
}
// END FEED_RECEIVER_ID

class FeedEditor {
    static PROTOCOLS = ['NONE', 'RAW', 'BEAST', 'BEAST_NO_UAT', 'BEAST_NO_UAT_UPLINK',
        'CSBEE', 'MAVLINK1', 'MAVLINK2', 'GDL90', 'AIRCRAFT_JSON', 'AIRCRAFT_JSON_UAT_UPLINK', 'ASTERIX'];
    static TRANSPORTS = ['TCP', 'UDP', 'MQTT'];
    // Transport a protocol's feeds usually take (the firmware's DefaultFeedTransport()).
    static DEFAULT_TRANSPORTS = {
        GDL90: 'UDP', GDL90_NO_UAT_UPLINK: 'UDP', MAVLINK1: 'UDP', MAVLINK2: 'UDP', ASTERIX: 'UDP',
        AIRCRAFT_JSON: 'MQTT', AIRCRAFT_JSON_UAT_UPLINK: 'MQTT',
    };
    // 192.168.4.255 reaches every device on the ADSBee's WiFi access point.
    static PRESETS = [
        { name: 'adsb.fi', protocol: 'BEAST_NO_UAT', transport: 'TCP', host: 'feed.adsb.fi', port: 30004 },
        { name: 'adsb.lol', protocol: 'BEAST_NO_UAT', transport: 'TCP', host: 'feed.adsb.lol', port: 30004 },
        { name: 'airplanes.live', protocol: 'BEAST_NO_UAT', transport: 'TCP', host: 'feed.airplanes.live', port: 30004 },
        { name: 'whereplane', protocol: 'BEAST', transport: 'TCP', host: 'feed.whereplane.xyz', port: 30004 },
        {
            name: 'EFB tablets', protocol: 'GDL90', transport: 'UDP', host: '192.168.4.255', port: 4000,
            tip: 'GDL90 broadcast on port 4000 to tablets on the ADSBee WiFi (ForeFlight, Garmin Pilot, SkyDemon...)',
        },
        {
            name: 'Ground station', protocol: 'MAVLINK2', transport: 'UDP', host: '192.168.4.255', port: 14550,
            tip: 'MAVLink on UDP 14550 for QGroundControl or Mission Planner on the ADSBee WiFi. Use the ground station\'s address on other networks.',
        },
        {
            name: 'Home Assistant', protocol: 'AIRCRAFT_JSON', transport: 'MQTT', host: 'homeassistant.local', port: 1883,
            tip: 'Aircraft JSON to the Home Assistant MQTT broker (Mosquitto add-on). Set its username and password.',
        },
    ];

    static async open(feedSlot) {
        const modal = document.getElementById('feed-modal');
        const statusEl = document.getElementById('feed-modal-status');
        statusEl.textContent = 'Loading...';
        document.getElementById('feed-form').style.display = 'none';
        modal.style.display = 'flex';

        let current = null;
        try {
            const resp = await fetch(`http://${HOST_URI}/api/feed?index=${feedSlot}`);
            if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
            current = await resp.json();
        } catch (e) {
            statusEl.textContent = `Error: ${e.message}`;
            return;
        }

        document.getElementById('feed-slot').value = current.index;
        document.getElementById('feed-uri').value = current.uri;
        document.getElementById('feed-port').value = current.port;
        document.getElementById('feed-active').checked = !!current.active;
        const sel = document.getElementById('feed-protocol');
        sel.innerHTML = FeedEditor.PROTOCOLS
            .map(p => `<option value="${p}"${p === current.protocol ? ' selected' : ''}>${p}</option>`)
            .join('');
        FeedEditor.showReceiverId(current.index, current.receiver_id);
        const transportSel = document.getElementById('feed-transport');
        transportSel.innerHTML = FeedEditor.TRANSPORTS.map(t => `<option value="${t}">${t}</option>`).join('');
        transportSel.value = current.transport || 'TCP';
        const mqtt = current.mqtt || {};
        document.getElementById('feed-mqtt-host').value = mqtt.host || '';
        document.getElementById('feed-mqtt-port').value = mqtt.port || 1883;
        document.getElementById('feed-mqtt-prefix').value = mqtt.topic_prefix || '';
        document.getElementById('feed-mqtt-username').value = mqtt.username || '';
        const password = document.getElementById('feed-mqtt-password');
        password.value = '';
        password.placeholder = mqtt.has_password ? '(unchanged)' : 'none';
        document.getElementById('feed-mqtt-tls').checked = !!mqtt.tls;
        document.getElementById('feed-mqtt-ws').checked = mqtt.transport === 'WS';
        document.getElementById('feed-mqtt-path').value = mqtt.path || '';
        document.getElementById('feed-mqtt-qos').value = String(mqtt.qos || 0);
        FeedEditor.renderPresets();
        document.getElementById('feed-uri').oninput = FeedEditor.updateMapLink;
        sel.onchange = () => {
            transportSel.value = FeedEditor.DEFAULT_TRANSPORTS[sel.value] || 'TCP';
            FeedEditor.updateMapLink();
        };
        transportSel.onchange = FeedEditor.updateMapLink;
        document.getElementById('feed-mqtt-prefix').oninput = FeedEditor.updateMapLink;
        document.getElementById('feed-mqtt-ws').onchange = FeedEditor.updateMapLink;
        FeedEditor.updateMapLink();
        statusEl.textContent = '';
        document.getElementById('feed-form').style.display = '';
    }

    // Receiver ID field; the device default comes from AT+FEED_RECEIVER_ID? (firmware without it: no ↺).
    static showReceiverId(slot, id) {
        FeedEditor.receiverId = id;
        FeedEditor.ridField = new ReceiverIdField(document.getElementById('feed-receiver-id'), {
            id, onSet: (value) => FeedEditor.setReceiverId(slot, value),
        });
        FeedEditor.queryReceiverId(slot).then((r) => {
            if (r && FeedEditor.ridField && String(document.getElementById('feed-slot').value) === String(slot)) {
                FeedEditor.ridField.setDefault(r.defaultId);
            }
        }).catch(() => {});
    }

    static async queryReceiverId(slot) {
        const out = await consoleWebSocket.sendAndCapture(`AT+FEED_RECEIVER_ID?${slot}\r\n`, '(DEFAULT)', 3000);
        return parseFeedReceiverIdLine(out.join(''));
    }

    // value: 16 hex digits or DEFAULT. Saves right away; resolves to what the device now reports.
    static async setReceiverId(slot, value) {
        await consoleWebSocket.sendAndCapture(`AT+FEED_RECEIVER_ID=${slot},${value}\r\n`, 'OK', 5000);
        await consoleWebSocket.sendAndCapture(`AT+SETTINGS=SAVE\r\n`, 'OK', 5000);
        const now = await FeedEditor.queryReceiverId(slot);
        if (!now) throw new Error('no reply');
        FeedEditor.receiverId = now.id;
        FeedEditor.updateMapLink();
        return now;
    }

    static renderPresets() {
        const row = document.getElementById('feed-presets');
        row.innerHTML = '';
        FeedEditor.PRESETS.forEach((preset, i) => {
            const chip = document.createElement('button');
            chip.type = 'button';
            chip.className = 'feed-preset';
            chip.textContent = preset.name;
            chip.title = preset.tip || `${preset.protocol} over ${preset.transport} to ${preset.host}:${preset.port}`;
            chip.onclick = () => FeedEditor.applyPreset(i);
            row.appendChild(chip);
        });
    }

    // Fills protocol, transport, host and port. MQTT presets fill the shared broker.
    static applyPreset(i) {
        const preset = FeedEditor.PRESETS[i];
        document.getElementById('feed-protocol').value = preset.protocol;
        document.getElementById('feed-transport').value = preset.transport;
        const isMqtt = preset.transport === 'MQTT';
        document.getElementById(isMqtt ? 'feed-mqtt-host' : 'feed-uri').value = preset.host;
        document.getElementById(isMqtt ? 'feed-mqtt-port' : 'feed-port').value = preset.port;
        document.getElementById('feed-active').checked = true;
        FeedEditor.updateMapLink();
    }

    // Shows the whereplane.xyz map link while the form describes a Beast feed to whereplane, and the broker options
    // and topics instead of host and port while the transport is MQTT.
    static updateMapLink() {
        const isMqtt = document.getElementById('feed-transport').value === 'MQTT';
        document.getElementById('feed-mqtt').style.display = isMqtt ? '' : 'none';
        document.getElementById('feed-endpoint').style.display = isMqtt ? 'none' : '';
        document.getElementById('feed-mqtt-path').style.display =
            document.getElementById('feed-mqtt-ws').checked ? '' : 'none';
        if (isMqtt) {
            const prefix = document.getElementById('feed-mqtt-prefix').value.trim() || 'adsbee';
            const base = `${prefix}/${String(FeedEditor.receiverId).toLowerCase()}`;
            const protocol = document.getElementById('feed-protocol').value.toLowerCase();
            document.getElementById('feed-mqtt-topics').textContent = `${base}/${protocol}, /status, /metrics`;
        }
        const url = isMqtt ? null : whereplaneMapUrl(document.getElementById('feed-uri').value,
            document.getElementById('feed-protocol').value, FeedEditor.receiverId);
        const link = document.getElementById('feed-map-link');
        if (url) link.href = url;
        document.getElementById('feed-map-row').style.display = url ? '' : 'none';
    }

    static close() {
        document.getElementById('feed-modal').style.display = 'none';
    }

    static openFirstEmpty() {
        const slot = metricsWebSocket ? metricsWebSocket.getFirstEmptySlot() : 0;
        if (slot === -1) return;
        FeedEditor.open(slot);
    }

    static async remove() {
        const slot = document.getElementById('feed-slot').value;
        const statusEl = document.getElementById('feed-modal-status');
        statusEl.textContent = 'Removing...';
        try {
            await consoleWebSocket.sendAndCapture(`AT+FEED=${slot},-,0,0,NONE,TCP\r\n`, 'OK', 5000);
            await consoleWebSocket.sendAndCapture(`AT+SETTINGS=SAVE\r\n`, 'OK', 5000);
            statusEl.textContent = 'Removed.';
            setTimeout(() => FeedEditor.close(), 800);
        } catch (e) {
            statusEl.textContent = `Error: ${e.message}`;
        }
    }

    static async save() {
        const slot = document.getElementById('feed-slot').value;
        const active = document.getElementById('feed-active').checked ? 1 : 0;
        const protocol = document.getElementById('feed-protocol').value;
        const transport = document.getElementById('feed-transport').value || 'TCP';
        const statusEl = document.getElementById('feed-modal-status');
        // MQTT feeds publish to the shared broker and have no host and port of their own.
        const uri = transport === 'MQTT' ? '-' : document.getElementById('feed-uri').value.trim();
        const port = transport === 'MQTT' ? 0 : document.getElementById('feed-port').value;

        statusEl.textContent = 'Saving...';
        try {
            if (transport === 'MQTT') {
                // "-" clears a string; an empty password keeps the stored one unless the username is cleared.
                const host = document.getElementById('feed-mqtt-host').value.trim() || '-';
                const brokerPort = document.getElementById('feed-mqtt-port').value || 1883;
                const prefix = document.getElementById('feed-mqtt-prefix').value.trim() || '-';
                const username = document.getElementById('feed-mqtt-username').value.trim() || '-';
                let password = document.getElementById('feed-mqtt-password').value;
                if (username === '-') password = '-';
                const tls = document.getElementById('feed-mqtt-tls').checked ? 1 : 0;
                const qos = document.getElementById('feed-mqtt-qos').value;
                const ws = document.getElementById('feed-mqtt-ws').checked ? 'WS' : 'TCP';
                const path = document.getElementById('feed-mqtt-path').value.trim() || '-';
                await consoleWebSocket.sendAndCapture(
                    `AT+MQTT=${host},${brokerPort},${username},${password},${tls},${qos},${prefix},${ws},${path}\r\n`,
                    'OK', 5000);
            }
            await consoleWebSocket.sendAndCapture(
                `AT+FEED=${slot},${uri},${port},${active},${protocol},${transport}\r\n`, 'OK', 5000);
            await consoleWebSocket.sendAndCapture(`AT+SETTINGS=SAVE\r\n`, 'OK', 5000);
            statusEl.textContent = 'Saved.';
            setTimeout(() => FeedEditor.close(), 800);
        } catch (e) {
            statusEl.textContent = `Error: ${e.message}`;
        }
    }
}

class ADSBeeAT {
    static MS_PER_SEC = 1000;
    static ERROR_BUFFER_FLUSH_INTERVAL_MS = 1000; // How long to wait after a parse error to let the error message print.

    messageQueue = [];

    constructor(url = HOST_URI) {
        if (!url) {
            throw new Error('URL is required');
        }
        this.url = url;
        this.ws = null;
        this.lastCmdTimestamp = Date.now();

        this.decoder = new TextDecoder('utf-8');
    }

    async connect() {
        return new Promise((resolve, reject) => {
            this.connectTimestamp = Date.now();
            this.ws = new WebSocket(`ws://${this.url}/console`);
            this.ws.binaryType = 'arraybuffer';

            this.ws.onmessage = (event) => {
                const decoded_message = this.decoder.decode(event.data);
                console.log("ADSBeeAT received message:", decoded_message);
                this.messageQueue.push(decoded_message);
            };

            this.ws.onopen = () => {
                resolve();
            };

            this.ws.onerror = (error) => reject(error);

            this.ws.onclose = (event) => {
                reject(new Error(`WebSocket closed (code ${event.code})`));
            };
        });
    }

    async disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
    }

    getMsTimestamp() {
        return Math.floor((Date.now() - this.connectTimestamp));
    }

    async flushInputBuffer(flushTimeoutMs = 10) {
        await new Promise(resolve => setTimeout(resolve, flushTimeoutMs));
        this.messageQueue = [];
    }

    async receiveResponse(printResponse = false, waitForResponse = false, responseToWaitFor = 'OK', maxCumulativeTimeoutMs = 6000) {
        const CPP_AT_PARSE_ERROR_FLUSH_INTERVAL_MS = 1000; // How long to wait after a parse error to let the error message print.
        const RECEIVE_LOOP_PERIOD_MS = 10; // How often to check for new messages.
        const decodedResponse = [];

        const cumulativeTimeout = Date.now() + maxCumulativeTimeoutMs;

        while (waitForResponse) {
            if (Date.now() > cumulativeTimeout) {
                throw new Error(`Response timeout after ${maxCumulativeTimeoutMs} ms.`);
            }

            if (waitForResponse && this.messageQueue.length > 0) {
                const decoded_message = this.messageQueue.shift();
                // Match on individual lines so ambient log messages like "[ERROR] ..."
                // or strings containing "OK" don't cause false positives.
                // Use substring match for the expected terminator (handles both 'OK'
                // and custom prefixes like '+FEED='), but match ERROR strictly on a
                // whole line so ambient log lines like "[ERROR] ..." don't false-trigger.
                if (decoded_message.includes(responseToWaitFor)) {
                    waitForResponse = false;
                } else if (decoded_message.split(/[\r\n]+/).some(l => l.trim() === 'ERROR')) {
                    // Catch AT command errors (standalone ERROR response line).
                    this.flushInputBuffer(ADSBeeAT.ERROR_BUFFER_FLUSH_INTERVAL_MS);
                    throw new Error(`ERROR response: ${decoded_message}`);
                } else if (decoded_message.includes('CppAT::')) {
                    // Catch CppAT parse errors.
                    await new Promise(resolve => setTimeout(resolve, CPP_AT_PARSE_ERROR_FLUSH_INTERVAL_MS));
                    this.flushInputBuffer(ADSBeeAT.ERROR_BUFFER_FLUSH_INTERVAL_MS);
                    throw new Error(`CppAT parse error: ${decoded_message}`);
                }

                // Received a decoded message without errors.
                let logStr = '';
                try {
                    decodedResponse.push(decoded_message);
                    logStr = `Response (+${this.getMsTimestamp() - this.lastCmdTimestamp} ms): \t\t${decoded_message}`;
                } catch {
                    logStr = `Non-ASCII response (+${this.getMsTimestamp() - this.lastCmdTimestamp} ms): \t${raw}`;
                }

                if (printResponse && logStr) {
                    console.log(logStr);
                }
            }

            // Wait for a timeout so that we don't block the event loop.
            if (this.messageQueue.length > 0) {
                continue; // If we have messages, don't wait.
            }
            await new Promise(resolve => setTimeout(resolve, RECEIVE_LOOP_PERIOD_MS));

        }

        return decodedResponse;
    }

    /**
     * Send an AT command to the ADSBee device.
     * @param[in] cmdStr The AT command string to send.
     * @param[in] delay_ms Delay in milliseconds after sending the command.
     * @param[in] printResponse Whether to print the response to the console.
     * @param[in] waitForOkOrError Whether to wait for an OK or ERROR response before returning.
     * @param[in] maxCumulativeTimeoutMs Maximum cumulative timeout in milliseconds.
     * @return A promise that resolves with the decoded_message response lines.
     */
    async sendCmd(cmdStr, delay_ms = 10, printResponse = false, waitForResponse = false, responseToWaitFor = 'OK', maxCumulativeTimeoutMs = 6000) {
        this.lastCmdTimestamp = this.getMsTimestamp();

        if (printResponse) {
            console.log(`Command (${this.getMsTimestamp()} ms): \t${cmdStr.trim()}`);
        }

        if (this.ws.readyState !== WebSocket.OPEN) {
            await this.disconnect();
            await this.connect();
        }
        this.ws.send(new TextEncoder().encode(cmdStr));

        let attempts = 0;
        const maxAttempts = 3;

        while (attempts < maxAttempts) {
            attempts++;
            try {
                return await this.receiveResponse(printResponse, waitForResponse, responseToWaitFor, maxCumulativeTimeoutMs);
            } catch (error) {
                if (error.message.includes('Response timeout')) {
                    console.log(`Response timeout detected (attempt ${attempts}/${maxAttempts}), reconnecting...`);
                    await this.disconnect();
                    await this.connect();

                    if (attempts >= maxAttempts) {
                        throw error;
                    }
                    // Retry the send operation
                    this.ws.send(new TextEncoder().encode(cmdStr));
                } else {
                    throw error;
                }
            }
        }
    }
}

class FirmwareUploader {
    constructor(adsbeeUrl) {
        this.adsbeeUrl = adsbeeUrl;
        this.uploadButton = document.getElementById('uploadButton');
        this.fileInput = document.getElementById('fileInput');

        this.setupEventListeners();
    }

    setupEventListeners() {
        this.uploadButton.addEventListener('click', () => {
            this.fileInput.click();
        });

        this.fileInput.addEventListener('change', async (event) => {
            const file = event.target.files[0];
            if (file) {
                await this.uploadFirmware(file);
            }
            this.fileInput.value = '';
        });
    }

    setUploadingState(isUploading) {
        this.uploadButton.disabled = isUploading;
        if (isUploading) {
            this.uploadButton.innerHTML = '<span class="spinner"></span> Uploading...';
        } else {
            this.uploadButton.innerHTML = 'Upload Firmware';
        }
    }

    showModal() {
        this.setModalTitle('Firmware Update In Progress');
        const log = document.getElementById('firmware-modal-log');
        if (log) log.innerHTML = '';
        document.getElementById('firmware-modal').style.display = 'flex';
    }

    appendToModalLog(text) {
        const log = document.getElementById('firmware-modal-log');
        if (!log) return;
        const trimmed = text.trim();
        if (!trimmed) return;
        const line = document.createElement('div');
        line.textContent = trimmed;
        log.appendChild(line);
        while (log.children.length > 100) {
            log.removeChild(log.firstChild);
        }
        log.scrollTop = log.scrollHeight;
    }

    hideModal() {
        document.getElementById('firmware-modal').style.display = 'none';
    }

    setModalStatus(text) {
        document.getElementById('firmware-modal-status').textContent = text;
    }

    updateProgress(percent) {
        document.getElementById('firmware-modal-fill').style.width = `${percent}%`;
        document.getElementById('firmware-modal-text').textContent = `${percent}%`;
    }

    setModalTitle(text) {
        const title = document.querySelector('#firmware-modal-box h2');
        if (title) title.textContent = text;
    }

    showSuccess(message) {
        this.uploadButton.innerHTML = 'Upload Complete';
        this.setModalTitle('Firmware Update Complete');
        this.setModalStatus(message);
    }

    showError(message) {
        this.uploadButton.innerHTML = 'Upload Failed';
        this.setModalTitle('Firmware Update Failed');
        this.setModalStatus(message);
        const close = document.getElementById('firmware-modal-close');
        if (close) close.style.display = '';
    }

    /** After the reboot, waits for the page to be served again (new firmware), then reloads it. */
    async waitForReboot() {
        const start = Date.now();
        await new Promise((r) => setTimeout(r, 20000));
        while (Date.now() - start < 300000) {
            try {
                const resp = await fetch('/', { cache: 'no-store' });
                if (resp.ok) {
                    this.showSuccess('Update complete. The ADSBee is back online; reloading the page...');
                    setTimeout(() => location.reload(), 3000);
                    return;
                }
            } catch (e) { /* still rebooting */ }
            const s = Math.round((Date.now() - start) / 1000);
            this.setModalStatus(`Update complete. The ADSBee is rebooting and updating its network and sub-GHz ` +
                                `coprocessors (about 1 minute)... ${s} s`);
            await new Promise((r) => setTimeout(r, 3000));
        }
        this.showError('Update complete, but the ADSBee has not come back after 5 minutes. Check its power and network.');
    }

    async uploadFirmware(file) {
        this.setUploadingState(true);
        this.updateProgress(0);
        this.showModal();
        const close = document.getElementById('firmware-modal-close');
        if (close) close.style.display = 'none';
        // Pause the page's own websockets so the device's httpd isn't busy with them during the update.
        consoleWebSocket.pause();
        metricsWebSocket.pause();
        if (mapWeather) mapWeather.setPaused(true);
        let ok = false;
        try {
            // Give the ESP32 time to release the page's /console session, or it resets the updater's connection.
            await new Promise((r) => setTimeout(r, 1500));
            const bytes = new Uint8Array(await file.arrayBuffer());
            const updater = new AdsbeeOta.OtaUpdater({
                makeTransport: () => AdsbeeOta.webSocketTransport(`ws://${this.adsbeeUrl}/console`),
                log: (text) => { console.log(text); this.appendToModalLog(text); },
                onProgress: (p) => {
                    if (p.percent !== null && p.percent !== undefined) this.updateProgress(p.percent);
                    if (p.message) this.setModalStatus(p.message);
                },
            });
            const res = await updater.run(bytes);
            ok = true;
            this.updateProgress(100);
            this.appendToModalLog(`Upload finished with ${res.retries} retried writes.`);
            this.showSuccess('Update complete. The ADSBee is rebooting...');
        } catch (error) {
            console.error('Firmware upload failed:', error);
            this.showError(error.message || 'Failed to upload firmware. Please try again.');
        } finally {
            this.setUploadingState(false);
            if (!ok) {
                consoleWebSocket.resume();
                metricsWebSocket.resume();
                if (mapWeather) mapWeather.setPaused(false);
            }
        }
        if (ok) await this.waitForReboot();
    }
}

class SettingsManager {
    constructor(adsbeeUrl) {
        this.adsbeeUrl = adsbeeUrl;
        this.downloadButton = document.getElementById('downloadSettings');
        this.restoreButton = document.getElementById('restoreSettings');
        this.settingsFileInput = document.getElementById('settingsFile');

        this.setupEventListeners();
    }

    setupEventListeners() {
        this.downloadButton.addEventListener('click', () => {
            this.downloadSettings();
        });

        this.restoreButton.addEventListener('click', () => {
            this.settingsFileInput.click();
        });

        this.settingsFileInput.addEventListener('change', async (event) => {
            const file = event.target.files[0];
            if (file) {
                await this.restoreSettings(file);
            }
            this.settingsFileInput.value = '';
        });
    }

    async downloadSettings() {
        const adsbee = new ADSBeeAT(this.adsbeeUrl);
        await adsbee.connect();
        const response = await adsbee.sendCmd('AT+SETTINGS?DUMP\r\n', 0, true, true);

        // Only keep lines starting with AT+
        const settingsLines = response.filter(line => line.startsWith('AT+'));

        const blob = new Blob([settingsLines.join('\n')], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `adsbee_${new Date().toISOString().replace(/[:.]/g, '-')}.settings`;
        a.click();
        URL.revokeObjectURL(url);
    }

    async restoreSettings(file) {
        const adsbee = new ADSBeeAT(this.adsbeeUrl);
        await adsbee.connect();

        const text = await file.text();
        const commands = text.split('\n').filter(line => line.trim());

        for (const command of commands) {
            try {
                await adsbee.sendCmd(command + '\r\n', 0, true, true);
            } catch (error) {
                throw new Error(`Failed executing command: ${command} - ${error.message}`);
            }
        }
    }
}

// ─── Altitude color (tar1090 continuous HSL gradient) ────────────────────────
function acAltColor(altFt) {
    if (altFt == null) return '#888888';
    const hue = 30 + (Math.max(0, Math.min(altFt, 40000)) / 40000) * 255;
    return `hsl(${hue.toFixed(1)}, 90%, 55%)`;
}

// Preferred pointing/steering direction: ground track when available, else
// true/magnetic heading — the emitter sends exactly one of the three keys
// (surface aircraft typically report a heading rather than a track).
function acDirection(ac) {
    return ac.track ?? ac.true_heading ?? ac.mag_heading ?? null;
}

// Velocity vectors are drawn in screen space (zoom-invariant): pixel length
// proportional to the reported speed — about one icon width (32 px) at
// 250 kt, clamped so taxiing traffic stays visible and jets don't dominate.
const kVectorPxPerKt = 0.128;
const kVectorMinPx = 8;
const kVectorMaxPx = 80;

// Trails coalesce consecutive same-color points into a single polyline, but the
// altitude gradient above is continuous, so adjacent points almost never match
// exactly. Quantizing to 500 ft bins first (about 3 hue degrees — imperceptible)
// is what lets level flight collapse to one polyline instead of hundreds.
const kTrailAltBucketFt = 500;
function trailColor(altFt) {
    return acAltColor(altFt == null ? null : Math.round(altFt / kTrailAltBucketFt) * kTrailAltBucketFt);
}

// With traces enabled every aircraft gets a trail, so unselected ones are capped
// well short of kMaxTrailPoints to bound the work on a busy receiver. The selected
// aircraft still draws its full history.
const kTrailAllMaxPoints = 100;

// Map display options persist across reloads. Storage can throw (private mode,
// disabled cookies), so both sides are defensive.
function readMapFlag(key, dflt) {
    try {
        const v = localStorage.getItem(key);
        return v == null ? dflt : v === 'true';
    } catch (_) {
        return dflt;
    }
}

function writeMapFlag(key, on) {
    try { localStorage.setItem(key, on ? 'true' : 'false'); } catch (_) { }
}

// True if a WebSocket was closed by a receiver that already serves its limit of clients on that endpoint: close code
// 1013, or from older firmware a close without a code before any message.
function webSocketRefused(closeEvent, gotMessage) {
    return closeEvent.code === 1013 || (closeEvent.code === 1005 && !gotMessage);
}

// ─── AircraftWebSocket ────────────────────────────────────────────────────────
class AircraftWebSocket {
    constructor(url) {
        this.url = url;
        this.paused = false;
        this.onMessage = null;
        this.onDisconnect = null;
        this.onRefused = null;  // (refused) => void, see webSocketRefused().
        this.ws = null;
        this._timer = null;
        this._connect();
    }

    _connect() {
        this.pingInterval = setInterval(() => {
            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                this.ws.send('ping');
            }
        }, 60000);
        if (this.paused) return;
        this.ws = new WebSocket(this.url);
        let gotMessage = false;
        this.ws.onopen = () => console.log('[AircraftWS] connected');
        this.ws.onmessage = (ev) => {
            if (!gotMessage && this.onRefused) this.onRefused(false);
            gotMessage = true;
            if (!this.onMessage) return;
            // Firmware batches aircraft into JSON arrays; older firmware sent one object per message.
            let data;
            try { data = JSON.parse(ev.data); } catch (_) { return; }
            for (const ac of (Array.isArray(data) ? data : [data])) {
                try { this.onMessage(ac); } catch (_) { }
            }
        };
        this.ws.onclose = (ev) => {
            clearInterval(this.pingInterval);
            if (this.onRefused && !this.paused) this.onRefused(webSocketRefused(ev, gotMessage));
            if (this.onDisconnect) this.onDisconnect();
            clearTimeout(this._timer);
            this._timer = setTimeout(() => { if (!this.paused) this._connect(); }, 3000);
        };
        this.ws.onerror = (e) => console.error('[AircraftWS] error', e);
    }

    pause() { this.paused = true; clearTimeout(this._timer); if (this.ws) this.ws.close(); }
    resume() { if (!this.paused) return; this.paused = false; this._connect(); }
    close(code, reason) { this.paused = true; clearTimeout(this._timer); if (this.ws) this.ws.close(code, reason); }
}

// ─── Weather (FIS-B) ──────────────────────────────────────────────────────────
// Reads UAT uplinks from the /uplink WebSocket. A source for MapWeather.
class UplinkWebSocket {
    constructor(url) {
        this.url = url;
        this.onUplink = null;
        this.onRefused = null;  // (refused) => void, see webSocketRefused().
        this.ws = null;
        this._timer = null;
    }

    start(onUplink) {
        this.onUplink = onUplink;
        this._connect();
    }

    stop() {
        this.onUplink = null;
        if (this.onRefused) this.onRefused(false);
        clearTimeout(this._timer);
        const ws = this.ws;
        this.ws = null;
        if (ws) ws.close();
    }

    _connect() {
        const ws = this.ws = new WebSocket(this.url);
        ws.binaryType = 'arraybuffer';
        let gotMessage = false;
        ws.onmessage = (ev) => {
            if (!gotMessage && this.onRefused) this.onRefused(false);
            gotMessage = true;
            const uplink = typeof ev.data === 'string' ? null : Fisb.parseUplinkMessage(ev.data);
            if (uplink && this.onUplink) this.onUplink(uplink);
        };
        ws.onclose = (ev) => {
            if (this.ws !== ws) return;
            if (this.onRefused) this.onRefused(webSocketRefused(ev, gotMessage));
            this._timer = setTimeout(() => this._connect(), 5000);
        };
    }
}

// FIS-B weather layer of the Live Map. While switched off it loads nothing and its source stays stopped.
class MapWeather {
    // source: {start(onUplink), stop()}. scriptUrl: fisb.js, for pages that don't embed it.
    constructor({ source, scriptUrl = null, stationsUrl = null, position = 'topleft' }) {
        this.source = source;
        this.scriptUrl = scriptUrl;
        this.stationsUrl = stationsUrl;
        this.position = position;
        this.enabled = readMapFlag('showWeather', false);
        this.map = null;
        this.decoder = null;
        this.layer = null;
        this.mapShown = false;
        this.paused = false;
        this.running = false;
        document.addEventListener('visibilitychange', () => this._apply());
    }

    attach(map) {
        this.map = map;
        this._apply();
    }

    setEnabled(on) {
        this.enabled = on;
        writeMapFlag('showWeather', on);
        this._apply();
    }

    // Call when the map becomes visible or hidden within the page.
    setMapShown(on) {
        this.mapShown = on;
        this._apply();
    }

    // Stops the source during a firmware update.
    setPaused(on) {
        this.paused = on;
        this._apply();
    }

    ingest(uplink) {
        if (this.decoder) this.decoder.ingestUplink(uplink.payload, uplink.meta);
    }

    // Forgets everything received, for a change of receiver.
    clear() {
        if (!this.decoder) return;
        this.decoder.reset();
        this.layer.refresh();
    }

    async _apply() {
        const wanted = () => this.enabled && !!this.map;
        if (wanted() && !this.layer) {
            if (!window.Fisb) {
                if (!this._loading) this._loading = this._loadScript().finally(() => { this._loading = null; });
                try { await this._loading; } catch (_) { return; }
            }
            if (wanted() && !this.layer) {
                this.decoder = new Fisb.Decoder();
                this.layer = Fisb.createLayer(L, this.decoder, { stationsUrl: this.stationsUrl, position: this.position });
            }
        }
        if (!this.layer) return;
        if (wanted() !== this.map.hasLayer(this.layer)) {
            if (wanted()) this.layer.addTo(this.map); else this.layer.remove();
        }
        const run = wanted() && this.mapShown && !this.paused && !document.hidden;
        if (run === this.running) return;
        this.running = run;
        if (run) this.source.start((uplink) => this.ingest(uplink)); else this.source.stop();
    }

    _loadScript() {
        return new Promise((resolve, reject) => {
            const script = document.createElement('script');
            script.src = this.scriptUrl;
            script.onload = resolve;
            script.onerror = reject;
            document.head.appendChild(script);
        });
    }
}

// ─── AircraftStore ────────────────────────────────────────────────────────────
const kMaxTrailPoints = 500;

// Open Drone ID (Broadcast Remote ID) enum labels, indexed by the raw value in the rid_id_type / rid_ua_type fields.
const RID_ID_TYPE_STRINGS = ['None', 'Serial Number', 'CAA Registration', 'UTM (USS) Assigned', 'Specific Session'];
const RID_UA_TYPE_STRINGS = [
    'None', 'Aeroplane', 'Helicopter/Multirotor', 'Gyroplane', 'Hybrid Lift', 'Ornithopter', 'Glider', 'Kite',
    'Free Balloon', 'Captive Balloon', 'Airship', 'Free Fall/Parachute', 'Rocket', 'Tethered Powered Aircraft',
    'Ground Obstacle', 'Other'
];

// Traffic sources. The firmware sends each aircraft from its preferred source (direct ADS-B, else ADS-R or TIS-B
// rebroadcast); the map outlines each target by source.
const TRAFFIC_SOURCES = {
    adsb_1090: { label: '1090 ADS-B', stroke: 'rgba(0,0,0,0.5)', dash: '' },
    adsb_uat: { label: 'UAT ADS-B', stroke: '#0057b8', dash: '' },
    adsr: { label: 'ADS-R', stroke: '#e07000', dash: '4,2' },
    tisb: { label: 'TIS-B', stroke: '#7b2fbe', dash: '2,2' },
    rid: { label: 'Remote ID', stroke: '#c026d3', dash: '' },
};
// Positions older than this (seconds, from "seen_pos") are drawn faded.
const kStalePositionS = 15;

function acSource(ac) {
    const type = ac.type ?? '';
    if (type === 'remote_id') return 'rid';
    if (type.startsWith('tisb')) return 'tisb';
    if (type.startsWith('adsr')) return 'adsr';
    return ac.link === 'uat' ? 'adsb_uat' : 'adsb_1090';  // Firmware without "link" only sent 1090 as direct ADS-B.
}

const kTrueOnlyFields = ['on_ground', 'alert', 'spi', 'emergency'];

class AircraftStore {
    constructor() {
        this.aircraft = new Map();  // key (hex) → latest merged data
        this.trails = new Map();  // key → [{lat, lon, alt}]
        this.trailRev = new Map();  // key → bump count, so the map can skip redrawing unchanged trails
        this.lastSeen = new Map();  // key → Date.now() timestamp
    }

    ingest(ac) {
        if (!ac.hex) return;
        const src = acSource(ac);
        const key = ac.hex;
        let prev = this.aircraft.get(key) || {};
        if (prev.src && prev.src !== src) prev = {};  // Don't mix fields from two sources; the trail carries over.
        const merged = { ...prev, ...ac, key, src, srcLabel: TRAFFIC_SOURCES[src].label };
        // The firmware sends these only while true, so a report without one clears it.
        for (const f of kTrueOnlyFields) if (!(f in ac)) delete merged[f];
        this.aircraft.set(key, merged);
        this.lastSeen.set(key, Date.now());
        if (ac.lat != null && ac.lon != null) {
            const t = this.trails.get(key) || [];
            t.push({ lat: ac.lat, lon: ac.lon, alt: ac.alt_baro ?? null });
            if (t.length > kMaxTrailPoints) t.shift();
            this.trails.set(key, t);
            this.trailRev.set(key, (this.trailRev.get(key) ?? 0) + 1);
        }
    }

    // Remove aircraft not heard from in the last 5 seconds.
    sweep() {
        const cutoff = Date.now() - 5000;
        for (const key of this.aircraft.keys()) {
            if ((this.lastSeen.get(key) ?? 0) < cutoff) {
                this.aircraft.delete(key);
                this.trails.delete(key);
                this.trailRev.delete(key);
                this.lastSeen.delete(key);
            }
        }
    }

    clearAll() {
        this.aircraft.clear();
        this.trails.clear();
        this.trailRev.clear();
        this.lastSeen.clear();
    }

    all() { return [...this.aircraft.values()]; }
}

// ─── LiveMap ─────────────────────────────────────────────────────────────────
// OSM standard tiles through adsbee.aero's caching proxy. tile.openstreetmap.org blocks requests without a Referer,
// which pages opened from file:// don't send.
const MAP_TILE_URL = 'https://adsbee.aero/tiles/{z}/{x}/{y}.png';

class LiveMap {
    constructor(containerId, store) {
        this.containerId = containerId;
        this.store = store;
        this.map = null;
        this.markers = new Map();   // hex → L.Marker
        this.trailLines = new Map();   // hex → L.LayerGroup
        this.trailDrawnRev = new Map();      // hex → store.trailRev value the drawn trail was built from
        this.trailDrawnOpacity = new Map();  // hex → opacity it was drawn at (selection changes it)
        this.receiverMarker = null;
        this.receiverStatus = null;
        this.selectedHex = null;
        this.onSelectionChanged = null;        // (hex | null) → void
        this._ready = false;
        this._offline = false;
        this._autoCenter = true;
        // Velocity vectors default on; traces default off, since a trail per aircraft is
        // a busy view. Both persist across reloads.
        this.showVectors = readMapFlag('showVelocityVectors', true);
        this.showAllTrails = readMapFlag('showAllTrails', false);
    }

    setShowVectors(on) {
        this.showVectors = on;
        writeMapFlag('showVelocityVectors', on);
        this.update();  // apply now rather than waiting for the next 1 Hz tick
    }

    setShowAllTrails(on) {
        this.showAllTrails = on;
        writeMapFlag('showAllTrails', on);
        if (!on) {
            // Drop every trail but the selected aircraft's, which is drawn either way.
            for (const hex of [...this.trailLines.keys()]) {
                if (hex !== this.selectedHex) this._clearTrail(hex);
            }
        }
        this.update();
    }

    async init() {
        if (this._ready || this._offline) return;
        try {
            await this._loadLeaflet();
            this._initMap();
            this._ready = true;
        } catch (_) {
            this._offline = true;
            const el = document.getElementById(this.containerId);
            if (el) el.innerHTML = '<div class="map-offline-banner">Map tiles require internet access — aircraft data is still live in the table below.</div>';
        }
    }

    _loadLeaflet() {
        return new Promise((resolve, reject) => {
            if (window.L) { resolve(); return; }
            const tid = setTimeout(reject, 8000);
            const link = document.createElement('link');
            link.rel = 'stylesheet';
            link.href = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
            document.head.appendChild(link);
            const script = document.createElement('script');
            script.src = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
            script.onload = () => { clearTimeout(tid); resolve(); };
            script.onerror = () => { clearTimeout(tid); reject(); };
            document.head.appendChild(script);
        });
    }

    _initMap() {
        this.map = L.map(this.containerId, { preferCanvas: true }).setView([20, 0], 2);
        L.tileLayer(MAP_TILE_URL, {
            attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
            maxZoom: 18,
        }).addTo(this.map);
        this._addSourceLegend();
    }

    _addSourceLegend() {
        const legend = L.control({ position: 'bottomleft' });
        legend.onAdd = () => {
            const div = L.DomUtil.create('div', 'map-source-legend');
            div.style.cssText = 'background:rgba(255,255,255,0.9);color:#222;padding:4px 8px;border-radius:4px;' +
                'font:12px sans-serif;line-height:18px';
            const swatch = (s, opacity = 1) =>
                `<svg width="14" height="14" viewBox="0 0 14 14" style="vertical-align:middle;margin-right:4px">` +
                `<polygon points="7,1 12,13 7,10 2,13" fill="#9aa0a6" opacity="${opacity}" stroke="${s.stroke}" ` +
                `stroke-width="${s === TRAFFIC_SOURCES.adsb_1090 ? 1 : 2}" stroke-dasharray="${s.dash ? '2,1' : ''}"/></svg>`;
            div.innerHTML = Object.entries(TRAFFIC_SOURCES)
                .filter(([k]) => k !== 'rid')
                .map(([, s]) => `<div>${swatch(s)}${s.label}</div>`).join('') +
                `<div>${swatch(TRAFFIC_SOURCES.adsb_1090, 0.35)}Position held &gt;${kStalePositionS} s</div>`;
            return div;
        };
        legend.addTo(this.map);
    }

    invalidate() {
        if (this.map) setTimeout(() => this.map.invalidateSize(), 0);
    }

    setReceiverStatus(status) {
        this.receiverStatus = status;
        if (!this._ready) return;

        const lat = Number(status?.latitude_deg);
        const lon = Number(status?.longitude_deg);
        const positionValid = status?.enabled === true && status?.fix_valid === true &&
            Number.isFinite(lat) && Number.isFinite(lon) &&
            lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180;

        if (!positionValid) {
            if (this.receiverMarker) {
                this.receiverMarker.remove();
                this.receiverMarker = null;
            }
            return;
        }

        if (this.receiverMarker) {
            this.receiverMarker.setLatLng([lat, lon]);
        } else {
            this.receiverMarker = L.circleMarker([lat, lon], {
                radius: 7,
                color: '#1a1a1a',
                weight: 2,
                fillColor: '#ffcb00',
                fillOpacity: 1,
            }).addTo(this.map).bindTooltip('Receiver position', { direction: 'top' });
        }
    }

    _makeIcon(color, track, isGround, isSelected, isDrone, gs, src = 'adsb_1090', isStale = false) {
        if (isDrone) {
            // Remote ID drones get a distinct quadcopter marker (four rotors) in a fixed accent color so they stand out
            // from ADS-B / UAT aircraft. Not rotated (Remote ID track is often unavailable or noisy at low speed), and
            // for the same reason they get no velocity vector — an unreliable heading would point it the wrong way.
            const ring = isSelected ? '<circle cx="16" cy="16" r="15" fill="none" stroke="white" stroke-width="2" opacity="0.7"/>' : '';
            const dfill = '#c026d3';  // magenta accent.
            const rotor = (cx, cy) => `<circle cx="${cx}" cy="${cy}" r="4" fill="${dfill}" stroke="rgba(0,0,0,0.5)" stroke-width="1"/>`;
            const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="32" height="32" viewBox="0 0 32 32">` +
                ring +
                `<line x1="9" y1="9" x2="23" y2="23" stroke="${dfill}" stroke-width="2"/>` +
                `<line x1="23" y1="9" x2="9" y2="23" stroke="${dfill}" stroke-width="2"/>` +
                rotor(9, 9) + rotor(23, 9) + rotor(9, 23) + rotor(23, 23) +
                `</svg>`;
            return L.divIcon({ html: svg, className: '', iconSize: [32, 32], iconAnchor: [16, 16] });
        }
        // The velocity vector is part of the marker SVG itself: SVG pixels are
        // screen pixels, so it is zoom-invariant by construction, always moves
        // with the icon, and needs no separate map layer. The 192x192 SVG
        // overflows the 32x32 icon box (centers aligned); pointer events stay
        // limited to the icon box + aircraft shape so hitboxes don't balloon.
        const ring = isSelected ? '<circle cx="96" cy="96" r="15" fill="none" stroke="white" stroke-width="2" opacity="0.7"/>' : '';
        const fill = isGround ? '#888888' : color;
        const source = TRAFFIC_SOURCES[src] ?? TRAFFIC_SOURCES.adsb_1090;
        const strokeWidth = src === 'adsb_1090' ? 1.2 : 2.4;  // Heavier outline marks the other sources.
        const opacity = isStale ? 0.35 : 1;
        const rot = (track != null) ? track : 0;
        const lenPx = (this.showVectors && gs > 0)
            ? Math.min(kVectorMaxPx, Math.max(kVectorMinPx, gs * kVectorPxPerKt)) : 0;
        const vec = lenPx
            ? `<line x1="96" y1="82" x2="96" y2="${(82 - lenPx).toFixed(1)}" stroke="${fill}" stroke-width="2" opacity="0.8"/>`
            : '';
        const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="192" height="192" viewBox="0 0 192 192" style="position:absolute;left:-80px;top:-80px;pointer-events:none">` +
            ring +
            `<g transform="rotate(${rot},96,96)">` +
            vec +
            `<polygon points="96,82 103,107 96,102 89,107" style="pointer-events:auto" fill="${fill}" opacity="${opacity}" ` +
            `stroke="${source.stroke}" stroke-width="${strokeWidth}" stroke-dasharray="${source.dash}"/>` +
            `</g></svg>`;
        return L.divIcon({ html: svg, className: '', iconSize: [32, 32], iconAnchor: [16, 16] });
    }

    selectAircraft(hex) {
        if (this.selectedHex === hex) {
            this._clearTrail(this.selectedHex);
            this.selectedHex = null;
            this._hideInfoPanel();
            if (this.onSelectionChanged) this.onSelectionChanged(null);
        } else {
            if (this.selectedHex) this._clearTrail(this.selectedHex);
            this.selectedHex = hex;
            this._drawTrail(hex);
            const ac = this.store.aircraft.get(hex);
            if (ac) this._showInfoPanel(ac);
            if (this.onSelectionChanged) this.onSelectionChanged(hex);
        }
    }

    deselect() {
        if (this.selectedHex) this._clearTrail(this.selectedHex);
        this.selectedHex = null;
        this._hideInfoPanel();
        if (this.onSelectionChanged) this.onSelectionChanged(null);
    }

    _clearTrail(hex) {
        if (this.trailLines.has(hex)) {
            this.trailLines.get(hex).remove();
            this.trailLines.delete(hex);
        }
        // Forget the cache keys too, or the next _drawTrail would skip the rebuild.
        this.trailDrawnRev.delete(hex);
        this.trailDrawnOpacity.delete(hex);
    }

    update() {
        if (!this._ready) return;
        const live = new Set();

        for (const ac of this.store.all()) {
            if (ac.lat == null || ac.lon == null) continue;
            live.add(ac.key);
            const isDrone = ac.type === 'remote_id';
            const color = acAltColor(ac.alt_baro ?? ac.alt_geom);
            const isGround = !!ac.on_ground;
            const isSel = ac.key === this.selectedHex;
            const isStale = ac.seen_pos != null && ac.seen_pos > kStalePositionS;
            const icon = this._makeIcon(color, acDirection(ac), isGround, isSel, isDrone, ac.gs, ac.src, isStale);

            if (this.markers.has(ac.key)) {
                const m = this.markers.get(ac.key);
                m.setLatLng([ac.lat, ac.lon]);
                m.setIcon(icon);
            } else {
                const key = ac.key;
                const m = L.marker([ac.lat, ac.lon], { icon })
                    .addTo(this.map)
                    .on('click', () => this.selectAircraft(key));
                this.markers.set(key, m);
            }
        }

        // `live` is exactly the aircraft with a position, which is exactly what has
        // a trail. _drawTrail no-ops for any whose trail is already current.
        if (this.showAllTrails) {
            for (const hex of live) this._drawTrail(hex);
        }

        // Refresh trail and info panel for selected aircraft each tick
        if (this.selectedHex && this.store.aircraft.has(this.selectedHex)) {
            this._drawTrail(this.selectedHex);
            const ac = this.store.aircraft.get(this.selectedHex);
            if (ac) this._showInfoPanel(ac);
        }

        // Auto-fit view on first aircraft
        if (this._autoCenter && (this.markers.size > 0 || this.receiverMarker)) {
            try {
                const mapLayers = [...this.markers.values()];
                if (this.receiverMarker) mapLayers.push(this.receiverMarker);
                const bounds = L.featureGroup(mapLayers).getBounds();
                if (bounds.isValid()) {
                    this.map.fitBounds(bounds, { padding: [50, 50], maxZoom: 9 });
                    this._autoCenter = false;
                }
            } catch (_) { }
        }

        // Remove markers for gone aircraft
        for (const hex of [...this.markers.keys()]) {
            if (!live.has(hex)) {
                this.markers.get(hex).remove();
                this.markers.delete(hex);
                this._clearTrail(hex);
                if (hex === this.selectedHex) this.deselect();
            }
        }
    }

    // Rebuilding a trail is the expensive part of a tick, so this does two things
    // to stay affordable once every aircraft has one: it skips outright when
    // nothing about the trail changed, and it merges consecutive points of the
    // same (quantized) altitude color into one multi-point polyline instead of
    // emitting one polyline per segment.
    _drawTrail(hex) {
        if (!this._ready) return;
        const isSel = (hex === this.selectedHex);
        const rev = this.store.trailRev.get(hex) ?? 0;
        // Selection changes the opacity, so it belongs in the cache key alongside rev.
        const opacity = isSel ? 0.75 : 0.35;
        if (this.trailDrawnRev.get(hex) === rev && this.trailDrawnOpacity.get(hex) === opacity) return;

        this._clearTrail(hex);
        let pts = this.store.trails.get(hex);
        if (!pts || pts.length < 2) return;
        if (!isSel && pts.length > kTrailAllMaxPoints) pts = pts.slice(-kTrailAllMaxPoints);

        const group = L.layerGroup().addTo(this.map);
        const emit = (run, color) => {
            if (run.length >= 2) {
                L.polyline(run.map(p => [p.lat, p.lon]), { color, weight: 2, opacity }).addTo(group);
            }
        };
        let run = [pts[0]];
        let runColor = trailColor(pts[0].alt);
        for (let i = 1; i < pts.length; i++) {
            const c = trailColor(pts[i].alt);
            run.push(pts[i]);  // the boundary point joins both runs, so the trail stays unbroken
            if (c !== runColor) {
                emit(run, runColor);
                run = [pts[i]];
                runColor = c;
            }
        }
        emit(run, runColor);

        this.trailLines.set(hex, group);
        this.trailDrawnRev.set(hex, rev);
        this.trailDrawnOpacity.set(hex, opacity);
    }

    _showInfoPanel(ac) {
        const panel = document.getElementById('aircraft-sidebar');
        const title = document.getElementById('sidebar-title');
        const fields = document.getElementById('sidebar-fields');
        if (!panel) return;
        if (title) {
            title.textContent = ((ac.flight || '').trim() || ac.hex) + (ac.src && ac.src !== 'adsb_1090' ? ` · ${ac.srcLabel}` : '');
        }

        const defs = [
            ['hex', 'ICAO', v => v],
            ['srcLabel', 'Source', v => v],
            ['flight', 'Callsign', v => v.trim() || null],
            ['squawk', 'Squawk', v => v],
            ['type', 'Type', v => v],
            ['category', 'Category', v => v],
            ['on_ground', 'On ground', v => v ? 'Yes' : null],
            ['alt_baro', 'Alt baro', v => v.toLocaleString() + ' ft'],
            ['alt_geom', 'Alt geom', v => v.toLocaleString() + ' ft'],
            ['gs', 'Speed', v => Math.round(v) + ' kt'],
            ['track', 'Track', v => v.toFixed(1) + '°'],
            ['mag_heading', 'Mag heading', v => v.toFixed(1) + '°'],
            ['true_heading', 'True heading', v => v.toFixed(1) + '°'],
            ['baro_rate', 'Vert rate', v => v.toLocaleString() + ' fpm'],
            ['geom_rate', 'Geom rate', v => v.toLocaleString() + ' fpm'],
            ['lat', 'Latitude', v => v.toFixed(5)],
            ['lon', 'Longitude', v => v.toFixed(5)],
            ['seen_pos', 'Position age', v => v.toFixed(1) + ' s'],
            ['r_dst', 'Distance', v => v.toFixed(1) + ' nm'],
            ['r_dir', 'Bearing', v => Math.round(v) % 360 + '°'],
            ['rssi', 'RSSI', v => v.toFixed(1) + ' dBm'],
            ['messages', 'Messages', v => v.toLocaleString()],
            ['nic', 'NIC', v => String(v)],
            ['nic_baro', 'NIC baro', v => String(v)],
            ['nac_p', 'NAC p', v => String(v)],
            ['nac_v', 'NAC v', v => String(v)],
            ['sil', 'SIL', v => String(v)],
            ['gva', 'GVA', v => String(v)],
            ['sda', 'SDA', v => String(v)],
            ['version', 'ADS-B version', v => String(v)],
            ['alert', 'Alert', v => v ? 'Yes' : null],
            ['spi', 'SPI (Ident)', v => v ? 'Yes' : null],
            ['emergency', 'Emergency', v => v],
            // Broadcast Remote ID (drone) fields.
            ['rid_uas_id', 'UAS ID', v => v.trim() || null],
            ['rid_id_type', 'UAS ID type', v => RID_ID_TYPE_STRINGS[v] || String(v)],
            ['rid_ua_type', 'UA type', v => RID_UA_TYPE_STRINGS[v] || String(v)],
            ['rid_operator_id', 'Operator ID', v => v.trim() || null],
            ['rid_operator_lat', 'Operator lat', v => v.toFixed(5)],
            ['rid_operator_lon', 'Operator lon', v => v.toFixed(5)],
            ['rid_height', 'Height AGL/ATO', v => v.toLocaleString() + ' ft'],
        ];

        const rows = [];
        for (const [key, label, fmt] of defs) {
            const raw = ac[key];
            if (raw == null || raw === '') continue;
            let val;
            try { val = fmt(raw); } catch (_) { val = String(raw); }
            if (val == null || val === '') continue;
            rows.push(`<span class="info-label">${label}</span><span class="info-value">${val}</span>`);
        }

        if (fields) fields.innerHTML = rows.join('');
        panel.style.display = 'flex';
        this.invalidate();
    }

    _hideInfoPanel() {
        const panel = document.getElementById('aircraft-sidebar');
        if (panel) panel.style.display = 'none';
        this.invalidate();
    }
}

// ─── AircraftTable ────────────────────────────────────────────────────────────
const kNumericSortKeys = new Set(['alt_baro', 'gs', 'track', 'lat', 'lon', 'r_dst', 'r_dir', 'rssi']);

class AircraftTable {
    constructor(tbodyId, store, onSelect) {
        this.tbody = document.getElementById(tbodyId);
        this.store = store;
        this.onSelect = onSelect;   // (hex) → void
        this.selectedHex = null;
        this.sortKey = 'alt_baro';
        this.sortDir = 'desc';

        // Wire header click handlers
        const thead = this.tbody && this.tbody.closest('table').querySelector('thead');
        if (thead) {
            for (const th of thead.querySelectorAll('th[data-sort-key]')) {
                th.style.cursor = 'pointer';
                th.addEventListener('click', () => {
                    const key = th.dataset.sortKey;
                    if (key === this.sortKey) {
                        this.sortDir = this.sortDir === 'asc' ? 'desc' : 'asc';
                    } else {
                        this.sortKey = key;
                        this.sortDir = kNumericSortKeys.has(key) ? 'desc' : 'asc';
                    }
                    this._updateSortIndicators(thead);
                    this.update();
                });
            }
            this._updateSortIndicators(thead);
        }
    }

    _updateSortIndicators(thead) {
        for (const th of thead.querySelectorAll('th[data-sort-key]')) {
            const active = th.dataset.sortKey === this.sortKey;
            // Strip any existing indicator and re-add if active
            th.textContent = th.textContent.replace(/ [↑↓]$/, '');
            if (active) th.textContent += (this.sortDir === 'asc' ? ' ↑' : ' ↓');
        }
    }

    _sorted() {
        const key = this.sortKey;
        const dir = this.sortDir === 'asc' ? 1 : -1;
        const numeric = kNumericSortKeys.has(key);
        return this.store.all().sort((a, b) => {
            const av = a[key] ?? null;
            const bv = b[key] ?? null;
            if (av === null && bv === null) return 0;
            if (av === null) return 1;   // nulls last
            if (bv === null) return -1;
            if (numeric) return dir * (av - bv);
            return dir * String(av).localeCompare(String(bv));
        });
    }

    update() {
        if (!this.tbody) return;
        const acs = this._sorted();

        const rows = new Map();
        for (const row of this.tbody.querySelectorAll('tr[data-key]')) rows.set(row.dataset.key, row);

        const seen = new Set();
        for (const ac of acs) {
            seen.add(ac.key);
            const cs = (ac.flight || '').trim();
            const source = ac.srcLabel ?? '';
            const sqwk = ac.squawk ?? '';
            const alt = ac.alt_baro != null ? ac.alt_baro.toLocaleString() : '';
            const gs = ac.gs != null ? Math.round(ac.gs) : '';
            const hdg = ac.track != null ? Math.round(ac.track) + '°' : '';
            const lat = ac.lat != null ? ac.lat.toFixed(4) : '';
            const lon = ac.lon != null ? ac.lon.toFixed(4) : '';
            const dst = ac.r_dst != null ? ac.r_dst.toFixed(1) : '';
            const dir = ac.r_dir != null ? Math.round(ac.r_dir) % 360 + '°' : '';
            const rssi = ac.rssi != null ? ac.rssi.toFixed(1) : '';
            const color = acAltColor(ac.alt_baro);

            if (rows.has(ac.key)) {
                const r = rows.get(ac.key);
                const c = r.querySelectorAll('td');
                c[0].style.color = color; c[0].textContent = ac.hex;
                c[1].textContent = cs; c[2].textContent = source;
                c[3].textContent = sqwk; c[4].textContent = alt;
                c[5].textContent = gs; c[6].textContent = hdg;
                c[7].textContent = lat; c[8].textContent = lon;
                c[9].textContent = dst; c[10].textContent = dir;
                c[11].textContent = rssi;
                r.classList.toggle('trail-active', this.selectedHex === ac.key);
            } else {
                const r = document.createElement('tr');
                r.dataset.key = ac.key;
                r.innerHTML = `<td style="color:${color};font-family:monospace">${ac.hex}</td>` +
                    `<td>${cs}</td><td>${source}</td><td>${sqwk}</td><td>${alt}</td><td>${gs}</td>` +
                    `<td>${hdg}</td><td>${lat}</td><td>${lon}</td><td>${dst}</td><td>${dir}</td><td>${rssi}</td>`;
                r.addEventListener('click', () => {
                    if (this.onSelect) this.onSelect(ac.key);
                });
                this.tbody.appendChild(r);
            }
        }

        // Remove rows for aircraft no longer in store
        for (const [key, row] of rows) {
            if (!seen.has(key)) {
                row.remove();
                if (this.selectedHex === key) this.selectedHex = null;
            }
        }

        // Re-order DOM rows to match sort (avoids flicker vs. delete+recreate)
        for (const ac of acs) {
            const row = this.tbody.querySelector(`tr[data-key="${ac.key}"]`);
            if (row) this.tbody.appendChild(row);
        }
    }

    setSelected(hex) {
        this.selectedHex = hex;
        if (!this.tbody) return;
        for (const row of this.tbody.querySelectorAll('tr[data-key]')) {
            row.classList.toggle('trail-active', row.dataset.key === hex);
        }
    }
}

if (typeof module !== 'undefined' && module.exports) {
    module.exports = { isOtherTextEntry, whereplaneMapUrl, webSocketRefused, ConsoleWebSocket, AircraftWebSocket,
        UplinkWebSocket, MapWeather, receiverIdUuid, parseFeedReceiverIdLine, AircraftStore, receiverMetricCards };
}
