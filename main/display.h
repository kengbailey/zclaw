#ifndef DISPLAY_H
#define DISPLAY_H

#include "esp_err.h"

typedef enum {
    DISPLAY_STATE_BOOTING,
    DISPLAY_STATE_CONNECTING,
    DISPLAY_STATE_IDLE,
    DISPLAY_STATE_THINKING,
    DISPLAY_STATE_TOOL_EXEC,
    DISPLAY_STATE_LISTENING,
    DISPLAY_STATE_ERROR,
} display_state_t;

esp_err_t display_init(void);
void display_set_state(display_state_t state);
void display_add_user_message(const char *text);
void display_add_agent_message(const char *text);
void display_set_tool_status(const char *tool_name);
void display_set_wifi_status(const char *ip_addr);
void display_scroll_conversation(int pixels);
void display_set_battery(int percent);

#endif // DISPLAY_H
