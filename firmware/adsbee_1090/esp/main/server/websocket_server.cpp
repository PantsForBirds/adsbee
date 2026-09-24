#include "websocket_server.hh"

#include "esp_heap_caps.h"

#include "comms.hh"
#include "hal.hh"

/**
 * This helper function digs out the WebSocketServer stored in the user context of an HTTP request in order to allow the
 * Handler() function of the proper WebSocketServer to be called.
 */
esp_err_t ws_handler(httpd_req_t* req) {
    WebSocketServer* instance = static_cast<WebSocketServer*>(req->user_ctx);
    if (instance) {
        return instance->Handler(req);
    }
    return ESP_FAIL;
}

bool WebSocketServer::Init() {
    // if (!config_.message_received_callback) {
    //     CONSOLE_WARNING("WebSocketServer::Init",
    //                     "[%s] Message received callback is nullptr, WebSocketServer will probably do nothing.",
    //                     config_.label);
    // }

    // Network console Websocket handler
    httpd_uri_t console_ws = {.uri = config_.uri,
                              .method = HTTP_GET,
                              // Pass in instance as user context to allow use of member functions in callbacks.
                              .handler = ws_handler,
                              .user_ctx = this,
                              .is_websocket = true,
                              .handle_ws_control_frames = false,
                              .supported_subprotocol = nullptr};
    esp_err_t ret = httpd_register_uri_handler(config_.server, &console_ws);
    if (ret != ESP_OK) {
        CONSOLE_ERROR("WebSocketServer::Init", "[%s] Failed to register websocket URI handler due to error %s.",
                      config_.label, esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool WebSocketServer::Update() {
    // Prune inactive network console clients.
    uint32_t timestamp_ms = get_time_since_boot_ms();  // Refresh timestamp to avoid negative values for time since last
                                                       // message (except for wraps)
    for (uint16_t i = 0; i < config_.num_clients_allowed; i++) {
        uint32_t last_ts = clients_[i].last_message_timestamp_ms;
        uint32_t time_since_last_message_ms = (last_ts <= timestamp_ms) ? (timestamp_ms - last_ts) : 0;
        if (config_.inactivity_timeout_ms > 0 && clients_[i].in_use &&
            time_since_last_message_ms > config_.inactivity_timeout_ms) {
            // Client is in use and has timed out.
            int client_fd = clients_[i].client_fd;
            CONSOLE_WARNING("WebSocketServer::Update",
                            "[%s] Network console client %d with fd %d timed out after %lu ms.", config_.label, i,
                            client_fd, time_since_last_message_ms);
            RemoveClient(client_fd);  // Free slot immediately; don't wait for ws_close_fd which may not fire if session
                                      // is already dead from httpd's perspective.
            httpd_sess_trigger_close(config_.server, client_fd);
        }
    }
    return true;
}

esp_err_t WebSocketServer::Handler(httpd_req_t* req) {
    int client_fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        CONSOLE_INFO("WebSocketServer::Handler", " [%s] Handshake done, new connection was opened.", config_.label);
        uint32_t free_heap_bytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        bool heap_ok = free_heap_bytes >= kMinFreeHeapBytesToAcceptClient;
        if (!heap_ok) {
            CONSOLE_WARNING("WebSocketServer::Handler",
                            "[%s] Rejecting websocket connection: %lu Bytes of heap free, need at least %lu.",
                            config_.label, (unsigned long)free_heap_bytes,
                            (unsigned long)kMinFreeHeapBytesToAcceptClient);
        }
        if (!heap_ok || !AddClient(client_fd)) {
            CONSOLE_ERROR("WebSocketServer::Handler", "[%s] Rejecting websocket connection.", config_.label);
            // Send a close frame
            httpd_ws_frame_t ws_pkt = {
                .final = true, .fragmented = false, .type = HTTPD_WS_TYPE_CLOSE, .payload = NULL, .len = 0};

            httpd_ws_send_frame(req, &ws_pkt);

            // Return error to reject the connection
            return ESP_FAIL;
        }
        if (config_.post_connect_callback) {
            config_.post_connect_callback(this, client_fd);
        }

        return ESP_OK;
    }
    httpd_ws_frame_t ws_pkt;
    uint8_t* buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = config_.send_as_binary ? HTTPD_WS_TYPE_BINARY : HTTPD_WS_TYPE_TEXT;
    /* Set max_len = 0 to get the frame len */
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        CONSOLE_ERROR("WebSocketServer::Handler", "[%s] httpd_ws_recv_frame failed to get frame len with %d.",
                      config_.label, ret);
        return ret;
    }
    CONSOLE_INFO("WebSocketServer::Handler", "[%s] frame len is %d.", config_.label, ws_pkt.len);
    if (ws_pkt.len) {
        /* ws_pkt.len + 1 is for NULL termination as we are expecting a string */
        buf = (uint8_t*)calloc(1, ws_pkt.len + 1);
        if (buf == NULL) {
            CONSOLE_ERROR("WebSocketServer::Handler", "[%s] Failed to calloc memory for buf.", config_.label);
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        /* Set max_len = ws_pkt.len to get the frame payload */
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            CONSOLE_ERROR("WebSocketServer::Handler", "[%s] httpd_ws_recv_frame failed with %d.", config_.label, ret);
            free(buf);
            if (config_.pre_disconnect_callback) {
                config_.pre_disconnect_callback(this, client_fd);
            }
            return ret;  // httpd closes the session on handler error, triggering ws_close_fd.
        }

        UpdateActivityTimer(client_fd);
        // Don't print packet payload, it's not safe if it's binary data that isn't null-terminated.
        // CONSOLE_INFO("WebSocketServer::Handler", "[%s] Got packet with message: %s", config_.label, ws_pkt.payload);
        if (config_.message_received_callback) {
            config_.message_received_callback(this, client_fd, ws_pkt);
        }
    }

    free(buf);
    return ret;
}

bool WebSocketServer::AddClient(int client_fd) {
    for (int i = 0; i < config_.num_clients_allowed; i++) {
        if (!clients_[i].in_use) {
            clients_[i].in_use = true;
            clients_[i].client_fd = client_fd;
            clients_[i].last_message_timestamp_ms = get_time_since_boot_ms();
            CONSOLE_INFO("WebSocketServer::AddClient", "[%s] New client stored at index %d.", config_.label, i);
            return true;
        }
    }
    CONSOLE_ERROR("WebSocketServer::AddClient", "[%s] Can't connect additional clients, already reached maximum of %d.",
                  config_.label, config_.num_clients_allowed);
    return false;
}

bool WebSocketServer::RemoveClient(int client_fd) {
    for (int i = 0; i < config_.num_clients_allowed; i++) {
        if (clients_[i].in_use && clients_[i].client_fd == client_fd) {
            clients_[i].in_use = false;
            CONSOLE_INFO("WebSocketServer::RemoveClient", "[%s] Client %d with fd %d removed.", config_.label, i,
                         client_fd);
            clients_[i].client_fd = -1;
            return true;
        }
    }
    CONSOLE_INFO(
        "WebSocketServer::RemoveClient",
        "[%s] Client with fd %d not found; it belong to a different websocket server or may have already been removed.",
        config_.label, client_fd);
    return false;
}

void WebSocketServer::BroadcastMessage(const char* message, int16_t len_bytes) {
    if (GetNumClients() == 0) {
        return;
    }
    uint32_t free_heap_bytes = heap_caps_get_free_size(MALLOC_CAP_8BIT);
    if (free_heap_bytes < kMinFreeHeapBytesToBroadcast) {
        // Sending queues TCP segments on the heap; when it's this low, dropping the message is the safe choice.
        uint32_t timestamp_ms = get_time_since_boot_ms();
        if (timestamp_ms - last_low_heap_warning_timestamp_ms_ > kLowHeapWarningIntervalMs) {
            last_low_heap_warning_timestamp_ms_ = timestamp_ms;
            CONSOLE_WARNING("WebSocketServer::BroadcastMessage",
                            "[%s] Dropping messages while heap is low (%lu Bytes free, need %lu).", config_.label,
                            (unsigned long)free_heap_bytes, (unsigned long)kMinFreeHeapBytesToBroadcast);
        }
        return;
    }
    for (int i = 0; i < config_.num_clients_allowed; i++) {
        if (clients_[i].in_use) {
            esp_err_t ret = SendMessage(clients_[i].client_fd, message, len_bytes);
            if (ret != ESP_OK) {
                if (ret == ESP_ERR_NO_MEM || ret == ESP_FAIL) {
                    // Transient resource exhaustion — drop this message but keep the session alive.
                    CONSOLE_WARNING("WebSocketServer::BroadcastMessage",
                                    "[%s] Dropping message to client %d (transient error: %s).", config_.label, i,
                                    esp_err_to_name(ret));
                } else {
                    // Permanent / unrecoverable error — close the session.
                    CONSOLE_ERROR("WebSocketServer::BroadcastMessage",
                                  "[%s] Failed to send message to client %d due to error %s.", config_.label, i,
                                  esp_err_to_name(ret));
                    int fd = clients_[i].client_fd;
                    RemoveClient(fd);  // Free slot immediately; don't wait for ws_close_fd which may not fire.
                    httpd_sess_trigger_close(config_.server, fd);
                }
            }
        }
    }
}

esp_err_t WebSocketServer::SendMessage(int client_fd, const char* message, int16_t len_bytes) {
    if (len_bytes > kWebSocketMessageMaxLen) {
        CONSOLE_ERROR("WebSocketServer::SendMessage",
                      "[%s] Message length %d exceeds maximum of %d, truncating message.", config_.label, len_bytes,
                      kWebSocketMessageMaxLen);
        len_bytes = kWebSocketMessageMaxLen;
    }
    httpd_ws_frame_t ws_pkt = {.final = true,
                               .fragmented = false,
                               .type = config_.send_as_binary ? HTTPD_WS_TYPE_BINARY : HTTPD_WS_TYPE_TEXT,
                               .payload = (uint8_t*)message,
                               .len = len_bytes > 0 ? len_bytes : strnlen(message, kWebSocketMessageMaxLen)};

    return httpd_ws_send_frame_async(config_.server, client_fd, &ws_pkt);
}

bool WebSocketServer::UpdateActivityTimer(int client_fd) {
    for (int i = 0; i < config_.num_clients_allowed; i++) {
        if (clients_[i].client_fd == client_fd) {
            clients_[i].last_message_timestamp_ms = get_time_since_boot_ms();
            return true;
        }
    }
    return false;  // Couldn't find client.
}