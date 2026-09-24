const METRIC_UNITS = {
    'num_mode_s_aircraft': 'aircraft',
    'num_uat_aircraft': 'aircraft',
    'num_remote_id_aircraft': 'drones',
};

class ConsoleWebSocket {
    constructor(url) {
        this.url = url;
        this.paused = false;
        this.captureCallback = null;
        this._reconnectTimer = null;
        this.init();
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
                appendToTerminal(text);
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
                if (text.split(/[\r\n]+/).some(l => l.trim() === 'ERROR')) {
                    clearTimeout(timer);
                    this.captureCallback = null;
                    reject(new Error(`ERROR response: ${text.trim()}`));
                } else if (matches(text)) {
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

class MetricCard {
    constructor(container, label, unit = 'msg/s', feedSlot = null, chartOptions = {}) {
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
  <div class="metric-title" title="${label}">${label}${slotBadge}</div>
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

    labelToID(label) {
        return label.replace(/[^a-zA-Z0-9]/g, '-');
    }

    // displayText overrides the "<value> <unit>" readout while value still drives the sparkline.
    update(value, displayText = null) {
        this.chart.push(value, this.unit, displayText);
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
        Object.entries(data).forEach(([label, value]) => {
            if (!this.cards[parentLabel][label]) {
                const unit = METRIC_UNITS[label] || 'msg/s';
                this.cards[parentLabel][label] = new MetricCard(container, label, unit);
            }
            this.cards[parentLabel][label].update(value);
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
                this.cards[parentLabel][label] = new MetricCard(container, label, 'msg/s', slot);
            }
            this.cards[parentLabel][label].update(value);
        });
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
                    const dictionaryMetrics = Object.fromEntries(
                        Object.entries(data['aircraft_dictionary_metrics']).filter(([key, value]) => !Array.isArray(value))
                    );
                    this.updateReceiverCards(dictionaryMetrics);
                }
                if (data.hasOwnProperty('server_metrics')) {
                    const serverMetrics = data['server_metrics'];
                    this._feedUris = serverMetrics['feed_uri'];
                    const feedMetrics = {};
                    for (let i = 0; i < serverMetrics['feed_uri'].length; i++) {
                        if (serverMetrics['feed_uri'][i] === '') continue;
                        feedMetrics[serverMetrics['feed_uri'][i]] = serverMetrics['feed_mps'][i];
                        this.feedSlotMap[serverMetrics['feed_uri'][i]] = i;
                    }
                    this.updateFeedCards(feedMetrics);
                    this.feedsCountEl.textContent = `Feeds: ${Object.keys(feedMetrics).length}`;
                    const addBtn = document.getElementById('add-feed-btn');
                    if (addBtn) addBtn.disabled = (this.getFirstEmptySlot() === -1);
                }
                if (data.hasOwnProperty('device_status')) {
                    const deviceStatus = data['device_status'];
                    // console.log('Device Status:', deviceStatus);
                    this.updateDeviceStatus(deviceStatus);
                    this.updateNoiseFloorCard(deviceStatus.rp2040);
                    if (deviceStatus.gnss && this.onGNSSStatus) {
                        this.onGNSSStatus(deviceStatus.gnss);
                    }
                }
            } catch (error) {
                console.error('Error parsing WebSocket message:', error);
            }
        };
    }

    getFirstEmptySlot() {
        if (!this._feedUris) return 0;
        for (let i = 0; i < this._feedUris.length; i++) {
            if (this._feedUris[i] === '') return i;
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

class FeedEditor {
    static PROTOCOLS = ['NONE', 'RAW', 'BEAST', 'BEAST_NO_UAT', 'BEAST_NO_UAT_UPLINK',
        'CSBEE', 'MAVLINK1', 'MAVLINK2', 'GDL90', 'AIRCRAFT_JSON'];

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
        statusEl.textContent = '';
        document.getElementById('feed-form').style.display = '';
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
            await consoleWebSocket.sendAndCapture(`AT+FEED=${slot},-,0,0,NONE\r\n`, 'OK', 5000);
            await consoleWebSocket.sendAndCapture(`AT+SETTINGS=SAVE\r\n`, 'OK', 5000);
            statusEl.textContent = 'Removed.';
            setTimeout(() => FeedEditor.close(), 800);
        } catch (e) {
            statusEl.textContent = `Error: ${e.message}`;
        }
    }

    static async save() {
        const slot = document.getElementById('feed-slot').value;
        const uri = document.getElementById('feed-uri').value.trim();
        const port = document.getElementById('feed-port').value;
        const active = document.getElementById('feed-active').checked ? 1 : 0;
        const protocol = document.getElementById('feed-protocol').value;
        const statusEl = document.getElementById('feed-modal-status');

        statusEl.textContent = 'Saving...';
        try {
            await consoleWebSocket.sendAndCapture(`AT+FEED=${slot},${uri},${port},${active},${protocol}\r\n`, 'OK', 5000);
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

    /**
     * Send raw bytes to the ADSBee device.
     * @param {Uint8Array} data The data to send.
     * @param {boolean} printResponse Whether to print the response to the console.
     * @return A promise that resolves with the decoded_message response lines.
     */
    async sendBytes(data, printResponse = false, waitForOkOrError = false) {
        const RESPONSE_TO_WAIT_FOR = 'OK';
        const RESPONSE_TIMEOUT_MS = 3000;
        if (this.ws.readyState !== WebSocket.OPEN) {
            throw new Error(`Response timeout: WebSocket not open (readyState=${this.ws.readyState})`);
        }
        this.ws.send(data);
        return await this.receiveResponse(printResponse, waitForOkOrError, RESPONSE_TO_WAIT_FOR, RESPONSE_TIMEOUT_MS);
    }

    /**
     * Silence the console by silencing logs and turning off any output protocols. Call this before sending data
     * for firmware updates.
     */
    async silenceConsole() {
        await this.sendCmd("AT+LOG_LEVEL=ERRORS\r\n", 0, true, true);
        await this.sendCmd("AT+PROTOCOL_OUT=CONSOLE,NONE\r\n", 0, true, true);
    }

    /**
     * Get the flash partition that can currently be read from / written to (e.g. the flash partition that is
     * not currently being executed from).
     * @return The current OTA flash partition.
     */
    async otaGetFlashPartition() {
        const decodedResponse = await this.sendCmd("AT+OTA=GET_PARTITION\r\n", 0, true, true);

        for (const line of decodedResponse) {
            const match = line.match(/Partition: (\d+)/);
            if (match) {
                return parseInt(match[1]);
            }
        }
        return null;
    }

    /**
     * Write bytes to the current OTA flash partition.
     * @param {number} offset The offset in bytes to write to.
     * @param {Uint8Array} data The data to write.
     */
    async otaWriteBytes(offset, data) {
        const crc32 = this.calculateCRC32(data);
        const cmdResponse = await this.sendCmd(`AT+OTA=WRITE,${offset.toString(16)},${data.length},${crc32.toString(16)}\r\n`, 0, true, true, "READY");
        for (const line of cmdResponse) {
            if (line.includes('ERROR')) {
                // Flush remaining lines then throw the error.
                await this.flushInputBuffer(ADSBeeAT.ERROR_BUFFER_FLUSH_INTERVAL_MS);
                throw new Error(`Command failed with error: ${line}`);
            }
        }
        const dataResponse = await this.sendBytes(data, true, true);
        for (const line of dataResponse) {
            if (line.includes('ERROR')) {
                // Flush remaining lines then throw the error.
                await this.flushInputBuffer(ADSBeeAT.ERROR_BUFFER_FLUSH_INTERVAL_MS);
                throw new Error(`Data transfer failed with error: ${line}`);
            }
        }
    }

    /**
     * Erase bytes from the current OTA flash partition.
     * @param {number} offsetBytes The offset in bytes to erase from.
     * @param {number} lenBytes The length in bytes to erase.
     */
    async otaErase(offsetBytes = NaN, lenBytes = NaN) {
        const ERASE_TIMEOUT_MS = 20000; // Allow 20 seconds for flash erase to complete.
        if (!isNaN(offsetBytes) && !isNaN(lenBytes)) {
            await this.sendCmd(`AT+OTA=ERASE,${offsetBytes.toString(16)},${lenBytes}\r\n`, 0, true, true, "OK", ERASE_TIMEOUT_MS);
        } else {
            // Allow 20 seconds for flash erase to complete.
            await this.sendCmd("AT+OTA=ERASE\r\n", 0, true, true, "OK", ERASE_TIMEOUT_MS);
        }
    }

    /**
     * Write a file to the current OTA flash partition.
     * @param {File} file The file to write.
     */
    async otaWriteFile(file) {
        const HEADER_SIZE_BYTES = 5 * 4;
        const APP_OFFSET_BYTES = 4 * 1024;
        // Each chunk is received on the ESP32 as one WebSocket frame, which callocs a contiguous buffer of the chunk
        // size. With Bluetooth Remote ID enabled the ESP32-S3's internal heap is tight and fragmented (largest free
        // block can be ~10 KB even with ~25 KB free), so a large chunk fails to allocate and aborts the OTA. Keep this
        // small AND a multiple of the 4096-Byte flash sector size (chunk offsets must stay sector-aligned for the
        // retry-erase path below). One sector (4096) allocates reliably under fragmentation; it was previously 3 sectors
        // (12288), which no longer fits.
        const WRITE_CHUNK_BYTES = 0x1000;
        const MAX_ATTEMPTS_PER_CHUNK = 3;

        const partition = await this.otaGetFlashPartition();

        if (partition === null) {
            throw new Error("Failed to get OTA partition from ADSBee.");
        }

        const fileData = await this.readFile(file);
        const dataView = new DataView(fileData.buffer);

        const numPartitions = dataView.getUint32(0, true);
        if (partition > numPartitions) {
            throw new Error(`Partition ${partition} is out of range. Only ${numPartitions} partitions available.`);
        }

        const offset = dataView.getUint32(4 + 4 * partition, true);
        const contents = fileData.slice(offset);
        const headerContents = contents.slice(0, HEADER_SIZE_BYTES);
        const appLenBytes = dataView.getUint32(offset + 8, true);

        await this.otaErase(0, appLenBytes + HEADER_SIZE_BYTES);

        // Write the header
        await this.otaWriteBytes(0, headerContents);

        // Write the application in chunks
        for (let i = HEADER_SIZE_BYTES; i < HEADER_SIZE_BYTES + appLenBytes; i += WRITE_CHUNK_BYTES) {
            const offsetBytes = (i - HEADER_SIZE_BYTES) + APP_OFFSET_BYTES;
            let success = false;
            let attempts = 0;
            while (!success && attempts < MAX_ATTEMPTS_PER_CHUNK) {
                try {
                    if (attempts > 0) {
                        // We are here on a retry, so erase the chunk first.
                        await this.otaErase(offsetBytes, WRITE_CHUNK_BYTES);
                    }
                    const chunk = contents.slice(i, Math.min(i + WRITE_CHUNK_BYTES, HEADER_SIZE_BYTES + appLenBytes));
                    await this.otaWriteBytes(offsetBytes, chunk);
                    success = true;
                } catch (error) {
                    console.error(error);
                    console.error(`Failed to write chunk at offset ${offsetBytes}. Retrying (${attempts + 1}/${MAX_ATTEMPTS_PER_CHUNK})... `);
                    attempts++; // Wait to erase until the next loop so that we can re-use the try-catch and don't erase before final failure if the first few retry attempts also fail.
                }
            }
        }

        // Verify and boot the new partition
        await this.sendCmd("AT+OTA=VERIFY\r\n", 0, true, true);
    }

    /**
     * Boot the current OTA flash partition.
     */
    async otaBoot() {
        await this.sendCmd("AT+OTA=BOOT\r\n", 0, true);
    }

    /**
     * Read a file as an ArrayBuffer.
     * @param {File} file The file to read.
     * @return {Promise<Uint8Array>} A promise that resolves with the file data as an ArrayBuffer.
     */
    async readFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = () => resolve(new Uint8Array(reader.result));
            reader.onerror = reject;
            reader.readAsArrayBuffer(file);
        });
    }

    /**
     * Calculate the CRC32 checksum of a Uint8Array.
     * @param {Uint8Array} data The data to calculate the CRC32 checksum for.
     * @return {number} The CRC32 checksum.
     */
    calculateCRC32(data) {
        let crc = 0xFFFFFFFF;
        for (let i = 0; i < data.length; i++) {
            crc = (crc >>> 8) ^ ADSBeeAT.crcTable[(crc ^ data[i]) & 0xFF];
        }
        return (crc ^ 0xFFFFFFFF) >>> 0;
    }

    static crcTable = (() => {
        const table = new Uint32Array(256);
        for (let i = 0; i < 256; i++) {
            let c = i;
            for (let j = 0; j < 8; j++) {
                c = ((c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1));
            }
            table[i] = c;
        }
        return table;
    })();
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
        if (percent === 0) {
            this.setModalStatus('Erasing flash...');
        } else if (percent < 100) {
            this.setModalStatus('Writing firmware...');
        } else {
            this.setModalStatus('Verifying & rebooting...');
        }
    }

    showSuccess() {
        this.uploadButton.innerHTML = 'Upload Complete';
        this.setModalStatus('Upload complete! Device is rebooting.');
        setTimeout(() => {
            this.uploadButton.innerHTML = 'Upload Firmware';
            this.hideModal();
        }, 5000);
    }

    showError(message) {
        this.uploadButton.innerHTML = 'Upload Failed';
        this.setModalStatus(`Upload failed: ${message}`);
        setTimeout(() => {
            this.uploadButton.innerHTML = 'Upload Firmware';
            this.hideModal();
        }, 5000);
    }

    async uploadFirmware(file) {
        let originalOtaWriteBytes = NaN;
        let adsbee = null;
        const origConsoleLog = console.log;
        try {
            this.setUploadingState(true);
            this.updateProgress(0);
            this.showModal();
            console.log = (...args) => {
                origConsoleLog.apply(console, args);
                this.appendToModalLog(args.join(' '));
            };

            adsbee = new ADSBeeAT(this.adsbeeUrl);
            await adsbee.connect();

            const totalSize = file.size / 2; // Two firmware images in file, we're only flashing one.
            let uploadedSize = 0;

            // Override otaWriteBytes to track progress
            originalOtaWriteBytes = adsbee.otaWriteBytes.bind(adsbee);
            adsbee.otaWriteBytes = async (offset, data) => {
                await originalOtaWriteBytes(offset, data);
                uploadedSize += data.length;
                const progress = Math.min(Math.round((uploadedSize / totalSize) * 100), 100);
                this.updateProgress(progress);
            };

            await adsbee.flushInputBuffer();
            await adsbee.silenceConsole();
            await adsbee.flushInputBuffer();

            // Receiver already gets disabled during data chunk transfers, but this simplifies it a bit.
            await adsbee.sendCmd('AT+RX_ENABLE=0\r\n', 0, true, true);

            // Pause other WebSocket clients to prevent httpd async queue contention during OTA.
            consoleWebSocket.pause();
            metricsWebSocket.pause();

            await adsbee.otaWriteFile(file);
            await adsbee.otaBoot();

            this.updateProgress(100);
            this.showSuccess();

        } catch (error) {
            console.error('Firmware upload failed:', error);
            this.showError(error.message || 'Failed to upload firmware. Please try again.');
        } finally {
            console.log = origConsoleLog;
            this.setUploadingState(false);
            if (adsbee && !isNaN(originalOtaWriteBytes)) {
                adsbee.otaWriteBytes = originalOtaWriteBytes;
            }
            if (adsbee) await adsbee.disconnect();
            consoleWebSocket.resume();
            metricsWebSocket.resume();
        }
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

// ─── Altitude colour (tar1090 continuous HSL gradient) ───────────────────────
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

// Trails coalesce consecutive same-colour points into a single polyline, but the
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

// ─── AircraftWebSocket ────────────────────────────────────────────────────────
class AircraftWebSocket {
    constructor(url) {
        this.url = url;
        this.paused = false;
        this.onMessage = null;
        this.onDisconnect = null;
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
        this.ws.onopen = () => console.log('[AircraftWS] connected');
        this.ws.onmessage = (ev) => {
            if (!this.onMessage) return;
            try { this.onMessage(JSON.parse(ev.data)); } catch (_) { }
        };
        this.ws.onclose = () => {
            clearInterval(this.pingInterval);
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

// ─── AircraftStore ────────────────────────────────────────────────────────────
const kMaxTrailPoints = 500;

// Open Drone ID (Broadcast Remote ID) enum labels, indexed by the raw value in the rid_id_type / rid_ua_type fields.
const RID_ID_TYPE_STRINGS = ['None', 'Serial Number', 'CAA Registration', 'UTM (USS) Assigned', 'Specific Session'];
const RID_UA_TYPE_STRINGS = [
    'None', 'Aeroplane', 'Helicopter/Multirotor', 'Gyroplane', 'Hybrid Lift', 'Ornithopter', 'Glider', 'Kite',
    'Free Balloon', 'Captive Balloon', 'Airship', 'Free Fall/Parachute', 'Rocket', 'Tethered Powered Aircraft',
    'Ground Obstacle', 'Other'
];

class AircraftStore {
    constructor() {
        this.aircraft = new Map();  // hex → latest merged data
        this.trails = new Map();  // hex → [{lat, lon, alt}]
        this.trailRev = new Map();  // hex → bump count, so the map can skip redrawing unchanged trails
        this.lastSeen = new Map();  // hex → Date.now() timestamp
    }

    ingest(ac) {
        if (!ac.hex) return;
        const prev = this.aircraft.get(ac.hex) || {};
        this.aircraft.set(ac.hex, { ...prev, ...ac });
        this.lastSeen.set(ac.hex, Date.now());
        if (ac.lat != null && ac.lon != null) {
            const t = this.trails.get(ac.hex) || [];
            t.push({ lat: ac.lat, lon: ac.lon, alt: ac.alt_baro ?? null });
            if (t.length > kMaxTrailPoints) t.shift();
            this.trails.set(ac.hex, t);
            this.trailRev.set(ac.hex, (this.trailRev.get(ac.hex) ?? 0) + 1);
        }
    }

    // Remove aircraft not heard from in the last 5 seconds.
    sweep() {
        const cutoff = Date.now() - 5000;
        for (const hex of this.aircraft.keys()) {
            if ((this.lastSeen.get(hex) ?? 0) < cutoff) {
                this.aircraft.delete(hex);
                this.trails.delete(hex);
                this.trailRev.delete(hex);
                this.lastSeen.delete(hex);
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

// ─── RadarMap ─────────────────────────────────────────────────────────────────
class RadarMap {
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
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>',
            maxZoom: 18,
        }).addTo(this.map);
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

    _makeIcon(color, track, isGround, isSelected, isDrone, gs) {
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
            `<polygon points="96,82 103,107 96,102 89,107" style="pointer-events:auto" fill="${fill}" stroke="rgba(0,0,0,0.5)" stroke-width="1.2"/>` +
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
            live.add(ac.hex);
            const isDrone = ac.type === 'remote_id';
            const color = acAltColor(ac.alt_baro ?? ac.alt_geom);
            const isGround = !!ac.on_ground;
            const isSel = ac.hex === this.selectedHex;
            const icon = this._makeIcon(color, acDirection(ac), isGround, isSel, isDrone, ac.gs);

            if (this.markers.has(ac.hex)) {
                const m = this.markers.get(ac.hex);
                m.setLatLng([ac.lat, ac.lon]);
                m.setIcon(icon);
            } else {
                const hex = ac.hex;
                const m = L.marker([ac.lat, ac.lon], { icon })
                    .addTo(this.map)
                    .on('click', () => this.selectAircraft(hex));
                this.markers.set(hex, m);
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
    // same (quantized) altitude colour into one multi-point polyline instead of
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
        if (title) title.textContent = (ac.flight || '').trim() || ac.hex;

        const defs = [
            ['hex', 'ICAO', v => v],
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
const kNumericSortKeys = new Set(['alt_baro', 'gs', 'track', 'lat', 'lon', 'rssi']);

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
        for (const row of this.tbody.querySelectorAll('tr[data-hex]')) rows.set(row.dataset.hex, row);

        const seen = new Set();
        for (const ac of acs) {
            seen.add(ac.hex);
            const cs = (ac.flight || '').trim();
            const type = ac.type ?? '';
            const sqwk = ac.squawk ?? '';
            const alt = ac.alt_baro != null ? ac.alt_baro.toLocaleString() : '';
            const gs = ac.gs != null ? Math.round(ac.gs) : '';
            const hdg = ac.track != null ? Math.round(ac.track) + '°' : '';
            const lat = ac.lat != null ? ac.lat.toFixed(4) : '';
            const lon = ac.lon != null ? ac.lon.toFixed(4) : '';
            const rssi = ac.rssi != null ? ac.rssi.toFixed(1) : '';
            const color = acAltColor(ac.alt_baro);

            if (rows.has(ac.hex)) {
                const r = rows.get(ac.hex);
                const c = r.querySelectorAll('td');
                c[0].style.color = color; c[0].textContent = ac.hex;
                c[1].textContent = cs; c[2].textContent = type;
                c[3].textContent = sqwk; c[4].textContent = alt;
                c[5].textContent = gs; c[6].textContent = hdg;
                c[7].textContent = lat; c[8].textContent = lon;
                c[9].textContent = rssi;
                r.classList.toggle('trail-active', this.selectedHex === ac.hex);
            } else {
                const r = document.createElement('tr');
                r.dataset.hex = ac.hex;
                r.innerHTML = `<td style="color:${color};font-family:monospace">${ac.hex}</td>` +
                    `<td>${cs}</td><td>${type}</td><td>${sqwk}</td><td>${alt}</td><td>${gs}</td>` +
                    `<td>${hdg}</td><td>${lat}</td><td>${lon}</td><td>${rssi}</td>`;
                r.addEventListener('click', () => {
                    if (this.onSelect) this.onSelect(ac.hex);
                });
                this.tbody.appendChild(r);
            }
        }

        // Remove rows for aircraft no longer in store
        for (const [hex, row] of rows) {
            if (!seen.has(hex)) {
                row.remove();
                if (this.selectedHex === hex) this.selectedHex = null;
            }
        }

        // Re-order DOM rows to match sort (avoids flicker vs. delete+recreate)
        for (const ac of acs) {
            const row = this.tbody.querySelector(`tr[data-hex="${ac.hex}"]`);
            if (row) this.tbody.appendChild(row);
        }
    }

    setSelected(hex) {
        this.selectedHex = hex;
        if (!this.tbody) return;
        for (const row of this.tbody.querySelectorAll('tr[data-hex]')) {
            row.classList.toggle('trail-active', row.dataset.hex === hex);
        }
    }
}
