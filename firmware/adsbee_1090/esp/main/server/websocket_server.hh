#ifndef WEBSOCKET_MANAGER_HH_
#define WEBSOCKET_MANAGER_HH_

#include <functional>

#include "esp_http_server.h"

class WebSocketServer {
   public:
    static const uint16_t kWebSocketLabelMaxLen =
        100;  // Number of chars available for label used in WebSocketManager error messages.
    static const uint16_t kMaxNumClients =
        8;  // Sizes the per-server client arrays. The three live endpoints each set num_clients_allowed=4, so 8 leaves
            // headroom while saving internal SRAM vs the old 20. See config_.num_clients_allowed for the runtime limit.
    static const uint16_t kURIMaxLen = 200;
    static const uint32_t kInactivityTimeoutMsDefault =
        60 * 60e3;  // 1 hour default inactivity timeout for network console clients.
    static const uint16_t kWebSocketMessageMaxLen = 4000;  // Maximum size of a single websocket message.

    // Heap guards. A browser page opens three websockets (console, metrics, aircraft) and each pins several kB of
    // httpd/lwIP buffers; the aircraft stream additionally sends one frame per aircraft per second per client. On the
    // PSRAM-less ESP32-S3 the heap is tight enough that one extra page can exhaust it, which starves WiFi/lwIP
    // allocations and takes the ESP32 down. Refuse new clients and skip broadcasts while free heap is low so the web UI
    // degrades (stale map, rejected tab) instead of crashing the device.
    static constexpr uint32_t kMinFreeHeapBytesToAcceptClient = 36 * 1024;
    static constexpr uint32_t kMinFreeHeapBytesToBroadcast = 14 * 1024;
    static constexpr uint32_t kLowHeapWarningIntervalMs = 5000;

    struct WebSocketServerConfig {
        char label[kWebSocketLabelMaxLen] = "Untitled";
        httpd_handle_t server;
        char uri[kURIMaxLen] = "/your-websocket-uri";
        uint16_t num_clients_allowed = kMaxNumClients;
        bool send_as_binary = false;  // Set to true if sending anything other than ASCII.
        // Time without a message before a network console client is disconnected.
        uint32_t inactivity_timeout_ms = kInactivityTimeoutMsDefault;
        // Callback function that gets executed when the websocket connects.
        std::function<void(WebSocketServer *ws_server, int client_fd)> post_connect_callback = nullptr;
        // Callback function that gets executed when the websocket disconnects.
        std::function<void(WebSocketServer *ws_server, int client_fd)> pre_disconnect_callback = nullptr;
        // Callback function that gets executed when a message is received via the websocket.
        std::function<void(WebSocketServer *ws_server, int client_fd, httpd_ws_frame_t &ws_pkt)>
            message_received_callback = nullptr;
    };

    /**
     * Constructor.
     */
    WebSocketServer(WebSocketServerConfig config_in) : config_(config_in) {};

    /**
     * Default constructor.
     */
    WebSocketServer() {};

    bool Init();
    bool Update();

    uint16_t GetNumClients() const {
        uint16_t count = 0;
        for (uint16_t i = 0; i < kMaxNumClients; i++) {
            if (clients_[i].in_use) count++;
        }
        return count;
    }
    uint16_t GetNumClientsAllowed() const { return config_.num_clients_allowed; }

    /**
     * Send a message to all clients currently connected to this websocket.
     * @param[in] message Null-terminated string to send.
     * @param[in] len_bytes Optional value for length of message to send, in case the full string in message should not
     * be sent. Does not add a null terminator.
     */
    void BroadcastMessage(const char *message, int16_t len_bytes = -1);

    /**
     * Send a message to a specific client matching the provided client file descriptor.
     * @param[in] client_fd Websocket file descriptor for the client to send the message to.
     * @param[in] message Null-terminated string to send.
     * @param[in] len_bytes Optional value for length of message to send, in case the full string in message should not
     * be sent. Does not add a null terminator.
     * @retval Result of httpd_ws_send_frame_async.
     */
    esp_err_t SendMessage(int client_fd, const char *message, int16_t len_bytes = -1);

    /**
     * Handler function that gets called with all HTTP requests made to the web socket.
     * @param[in] req Incoming HTTP request.
     * @retval ESP_OK if request was handled successfully, ESP_FAIL if connection should be rejected.
     */
    esp_err_t Handler(httpd_req_t *req);

    /**
     * Looks up a currently connected client by its websocket file descriptor and removes it from the pool of connected
     * clients.
     * @param[in] client_fd File descriptor for the websocket to remove from the pool, obtained with
     * httpd_req_to_sockfd(req).
     * @retval True if the client was successfully removed, false if the provided client_fd wasn't a match for any
     * currently connected websocket clients.
     */
    bool RemoveClient(int client_fd);

   private:
    struct WSClientInfo {
        bool in_use = false;
        int client_fd = 0;
        uint32_t last_message_timestamp_ms = 0;
    };

    /**
     * Adds a file descriptor for a websocket client to the currently tracked pool of open websockets.
     * @param[in] client_fd File descriptor for a newly opened websocket, obtained with httpd_req_to_sockfd(req).
     * @retval True if client was successfully added to the pool, false if already tracking the maximum number of
     * clients.
     */
    bool AddClient(int client_fd);

    bool UpdateActivityTimer(int client_fd);

    WebSocketServerConfig config_;
    uint32_t last_low_heap_warning_timestamp_ms_ = 0;

    WSClientInfo clients_[kMaxNumClients] = {0};
};

#endif /* WEBSOCKET_MANAGER_HH_ */