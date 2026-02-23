#include "voice.h"
#include "config.h"
#include "messages.h"
#include "display.h"
#include "haptic.h"
#include "lcd_touch_bsp.h"

#include "driver/i2s_pdm.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "memory.h"
#include "nvs_keys.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "voice";

// I2S PDM microphone pins
#define PDM_CLK_PIN   45
#define PDM_DIN_PIN   46

// Recording parameters
#define SAMPLE_RATE       16000
#define RECORD_CHANNELS   1
#define BITS_PER_SAMPLE   16
#define MAX_RECORD_SEC    VOICE_MAX_RECORD_SEC
#define RECORD_BUF_SIZE   (SAMPLE_RATE * RECORD_CHANNELS * (BITS_PER_SAMPLE / 8) * MAX_RECORD_SEC)

// I2S read chunk size
#define I2S_READ_CHUNK    2048

// Whisper config
#ifndef CONFIG_ZCLAW_WHISPER_URL
#define CONFIG_ZCLAW_WHISPER_URL "https://api.openai.com/v1/audio/transcriptions"
#endif
#ifndef CONFIG_ZCLAW_WHISPER_MODEL
#define CONFIG_ZCLAW_WHISPER_MODEL "whisper-1"
#endif

// WAV file header (44 bytes)
typedef struct __attribute__((packed)) {
    char     riff_header[4];    // "RIFF"
    uint32_t wav_size;          // file size - 8
    char     wave_header[4];    // "WAVE"
    char     fmt_header[4];     // "fmt "
    uint32_t fmt_chunk_size;    // 16 for PCM
    uint16_t audio_format;      // 1 for PCM
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char     data_header[4];    // "data"
    uint32_t data_bytes;
} wav_header_t;

typedef enum {
    VOICE_IDLE,
    VOICE_RECORDING,
    VOICE_TRANSCRIBING,
} voice_state_t;

static i2s_chan_handle_t s_rx_chan = NULL;
static int16_t *s_record_buf = NULL;
static QueueHandle_t s_input_queue = NULL;
static volatile voice_state_t s_state = VOICE_IDLE;

// --- WAV header ---

static void build_wav_header(wav_header_t *hdr, uint32_t data_bytes)
{
    memcpy(hdr->riff_header, "RIFF", 4);
    hdr->wav_size = data_bytes + sizeof(wav_header_t) - 8;
    memcpy(hdr->wave_header, "WAVE", 4);
    memcpy(hdr->fmt_header, "fmt ", 4);
    hdr->fmt_chunk_size = 16;
    hdr->audio_format   = 1;
    hdr->num_channels   = RECORD_CHANNELS;
    hdr->sample_rate    = SAMPLE_RATE;
    hdr->byte_rate      = SAMPLE_RATE * RECORD_CHANNELS * (BITS_PER_SAMPLE / 8);
    hdr->block_align    = RECORD_CHANNELS * (BITS_PER_SAMPLE / 8);
    hdr->bits_per_sample = BITS_PER_SAMPLE;
    memcpy(hdr->data_header, "data", 4);
    hdr->data_bytes = data_bytes;
}

// --- Touch helpers ---

static bool touch_is_pressed(void)
{
    uint16_t x, y;
    return tpGetCoordinates(&x, &y) != 0;
}

// --- Whisper HTTP client ---

static char *s_http_resp = NULL;
static size_t s_http_resp_len = 0;

static esp_err_t whisper_http_event(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        if (!s_http_resp) {
            s_http_resp = malloc(evt->data_len + 1);
            if (s_http_resp) {
                memcpy(s_http_resp, evt->data, evt->data_len);
                s_http_resp_len = evt->data_len;
            }
        } else {
            char *new_buf = realloc(s_http_resp, s_http_resp_len + evt->data_len + 1);
            if (new_buf) {
                s_http_resp = new_buf;
                memcpy(s_http_resp + s_http_resp_len, evt->data, evt->data_len);
                s_http_resp_len += evt->data_len;
            }
        }
        if (s_http_resp) {
            s_http_resp[s_http_resp_len] = '\0';
        }
    }
    return ESP_OK;
}

static esp_err_t whisper_transcribe(const uint8_t *wav_data, size_t wav_size,
                                     char *out, size_t out_size)
{
    esp_err_t ret = ESP_OK;

    if (s_http_resp) {
        free(s_http_resp);
        s_http_resp = NULL;
    }
    s_http_resp_len = 0;

    const char *boundary = "----ZclawVoiceBoundary";

    // Build multipart parts
    char hdr_model[256];
    char hdr_file[256];
    char footer[64];

    snprintf(hdr_model, sizeof(hdr_model),
             "--%s\r\n"
             "Content-Disposition: form-data; name=\"model\"\r\n\r\n"
             "%s\r\n",
             boundary, CONFIG_ZCLAW_WHISPER_MODEL);

    snprintf(hdr_file, sizeof(hdr_file),
             "--%s\r\n"
             "Content-Disposition: form-data; name=\"file\"; filename=\"audio.wav\"\r\n"
             "Content-Type: audio/wav\r\n\r\n",
             boundary);

    snprintf(footer, sizeof(footer), "\r\n--%s--\r\n", boundary);

    size_t model_len  = strlen(hdr_model);
    size_t file_h_len = strlen(hdr_file);
    size_t foot_len   = strlen(footer);
    size_t total      = model_len + file_h_len + wav_size + foot_len;

    // Allocate in PSRAM — WAV data can be ~320 KB
    char *body = heap_caps_malloc(total, MALLOC_CAP_SPIRAM);
    if (!body) {
        ESP_LOGE(TAG, "Failed to allocate multipart body (%u bytes)", (unsigned)total);
        return ESP_ERR_NO_MEM;
    }

    size_t off = 0;
    memcpy(body + off, hdr_model, model_len);  off += model_len;
    memcpy(body + off, hdr_file, file_h_len);  off += file_h_len;
    memcpy(body + off, wav_data, wav_size);     off += wav_size;
    memcpy(body + off, footer, foot_len);

    char content_type[128];
    snprintf(content_type, sizeof(content_type),
             "multipart/form-data; boundary=%s", boundary);

    esp_http_client_config_t cfg = {
        .url               = CONFIG_ZCLAW_WHISPER_URL,
        .method            = HTTP_METHOD_POST,
        .event_handler     = whisper_http_event,
        .timeout_ms        = 30000,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client) {
        heap_caps_free(body);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", content_type);

    // Add Bearer auth if STT API key is provisioned
    char stt_key[128] = {0};
    if (memory_get(NVS_KEY_STT_API_KEY, stt_key, sizeof(stt_key)) && stt_key[0] != '\0') {
        char auth_header[192];
        snprintf(auth_header, sizeof(auth_header), "Bearer %s", stt_key);
        esp_http_client_set_header(client, "Authorization", auth_header);
    }

    esp_http_client_set_post_field(client, body, (int)total);

    ESP_LOGI(TAG, "Sending %u bytes to Whisper API", (unsigned)total);
    ret = esp_http_client_perform(client);

    if (ret == ESP_OK) {
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Whisper HTTP %d", status);

        if (status == 200 && s_http_resp) {
            cJSON *root = cJSON_Parse(s_http_resp);
            if (root) {
                cJSON *text = cJSON_GetObjectItem(root, "text");
                if (text && cJSON_IsString(text)) {
                    strncpy(out, text->valuestring, out_size - 1);
                    out[out_size - 1] = '\0';
                    ESP_LOGI(TAG, "Transcription: %s", out);
                } else {
                    ESP_LOGE(TAG, "No 'text' field in Whisper response");
                    ret = ESP_FAIL;
                }
                cJSON_Delete(root);
            } else {
                ESP_LOGE(TAG, "Failed to parse Whisper JSON");
                ret = ESP_FAIL;
            }
        } else {
            ESP_LOGE(TAG, "Whisper request failed (HTTP %d)", status);
            if (s_http_resp) ESP_LOGE(TAG, "Response: %s", s_http_resp);
            ret = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "Whisper HTTP error: %s", esp_err_to_name(ret));
    }

    esp_http_client_cleanup(client);
    heap_caps_free(body);

    if (s_http_resp) {
        free(s_http_resp);
        s_http_resp = NULL;
    }
    s_http_resp_len = 0;

    return ret;
}

// --- Voice task ---

static void voice_task(void *arg)
{
    ESP_LOGI(TAG, "Voice task started");

    for (;;) {
        // Wait for touch press
        if (!touch_is_pressed()) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Debounce: require held for VOICE_TOUCH_HOLD_MS
        int held_ms = 0;
        while (touch_is_pressed() && held_ms < VOICE_TOUCH_HOLD_MS) {
            vTaskDelay(pdMS_TO_TICKS(50));
            held_ms += 50;
        }

        if (held_ms < VOICE_TOUCH_HOLD_MS) {
            // Short tap — not a voice trigger
            continue;
        }

        // --- Start recording ---
        s_state = VOICE_RECORDING;
        display_set_state(DISPLAY_STATE_LISTENING);
        haptic_play(HAPTIC_CLICK);

        memset(s_record_buf, 0, RECORD_BUF_SIZE);
        size_t recorded_samples = 0;
        size_t max_samples = RECORD_BUF_SIZE / sizeof(int16_t);

        esp_err_t err = i2s_channel_enable(s_rx_chan);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable I2S: %s", esp_err_to_name(err));
            s_state = VOICE_IDLE;
            display_set_state(DISPLAY_STATE_IDLE);
            continue;
        }

        int64_t start_us = esp_timer_get_time();
        int64_t max_us = (int64_t)MAX_RECORD_SEC * 1000000LL;

        while (recorded_samples < max_samples) {
            // Check timeout
            if ((esp_timer_get_time() - start_us) >= max_us) {
                ESP_LOGI(TAG, "Max recording time reached");
                break;
            }

            // Check touch released → stop recording
            if (!touch_is_pressed()) {
                ESP_LOGI(TAG, "Touch released, stopping recording");
                break;
            }

            size_t remaining = (max_samples - recorded_samples) * sizeof(int16_t);
            size_t to_read = (remaining < I2S_READ_CHUNK) ? remaining : I2S_READ_CHUNK;
            size_t bytes_read = 0;

            err = i2s_channel_read(s_rx_chan,
                                   (uint8_t *)(s_record_buf + recorded_samples),
                                   to_read, &bytes_read, 1000);
            if (err == ESP_OK) {
                recorded_samples += bytes_read / sizeof(int16_t);
            } else {
                ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(err));
                break;
            }
        }

        i2s_channel_disable(s_rx_chan);

        float duration = (esp_timer_get_time() - start_us) / 1000000.0f;
        ESP_LOGI(TAG, "Recorded %.2fs, %u samples", duration, (unsigned)recorded_samples);

        if (recorded_samples < SAMPLE_RATE / 4) {
            // Less than 250ms of audio — too short
            ESP_LOGW(TAG, "Recording too short, discarding");
            s_state = VOICE_IDLE;
            display_set_state(DISPLAY_STATE_IDLE);
            continue;
        }

        // --- Transcribe ---
        s_state = VOICE_TRANSCRIBING;
        haptic_play(HAPTIC_DOUBLE_CLICK);
        display_set_state(DISPLAY_STATE_THINKING);

        uint32_t data_bytes = recorded_samples * sizeof(int16_t);
        size_t wav_size = sizeof(wav_header_t) + data_bytes;

        uint8_t *wav_data = heap_caps_malloc(wav_size, MALLOC_CAP_SPIRAM);
        if (!wav_data) {
            ESP_LOGE(TAG, "Failed to allocate WAV buffer (%u bytes)", (unsigned)wav_size);
            s_state = VOICE_IDLE;
            display_set_state(DISPLAY_STATE_IDLE);
            continue;
        }

        wav_header_t hdr;
        build_wav_header(&hdr, data_bytes);
        memcpy(wav_data, &hdr, sizeof(hdr));
        memcpy(wav_data + sizeof(hdr), s_record_buf, data_bytes);

        ESP_LOGI(TAG, "WAV packaged: %u bytes", (unsigned)wav_size);

        char transcription[CHANNEL_RX_BUF_SIZE];
        transcription[0] = '\0';

        err = whisper_transcribe(wav_data, wav_size,
                                  transcription, sizeof(transcription));
        heap_caps_free(wav_data);

        if (err == ESP_OK && transcription[0] != '\0') {
            display_add_user_message(transcription);

            channel_msg_t msg = {0};
            strncpy(msg.text, transcription, CHANNEL_RX_BUF_SIZE - 1);
            msg.source = MSG_SOURCE_VOICE;
            msg.chat_id = 0;

            if (xQueueSend(s_input_queue, &msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
                ESP_LOGW(TAG, "Failed to queue voice message");
            }
        } else {
            ESP_LOGW(TAG, "Transcription failed or empty");
            haptic_play(HAPTIC_ERROR);
            display_set_state(DISPLAY_STATE_IDLE);
        }

        s_state = VOICE_IDLE;
        // display_set_state(DISPLAY_STATE_IDLE) will be called by agent after processing
    }
}

// --- Public API ---

esp_err_t voice_init(void)
{
    ESP_LOGI(TAG, "Initializing voice (I2S PDM mic)");

    // Allocate recording buffer in PSRAM
    s_record_buf = heap_caps_malloc(RECORD_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_record_buf) {
        ESP_LOGE(TAG, "Failed to allocate recording buffer (%d bytes)", RECORD_BUF_SIZE);
        return ESP_ERR_NO_MEM;
    }

    // Create I2S PDM RX channel
    i2s_chan_config_t rx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&rx_cfg, NULL, &s_rx_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S channel: %s", esp_err_to_name(err));
        heap_caps_free(s_record_buf);
        s_record_buf = NULL;
        return err;
    }

    // Configure PDM RX mode
    i2s_pdm_rx_config_t pdm_cfg = {
        .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = PDM_CLK_PIN,
            .din = PDM_DIN_PIN,
            .invert_flags = { .clk_inv = false },
        },
    };

    err = i2s_channel_init_pdm_rx_mode(s_rx_chan, &pdm_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init PDM RX: %s", esp_err_to_name(err));
        i2s_del_channel(s_rx_chan);
        heap_caps_free(s_record_buf);
        s_record_buf = NULL;
        return err;
    }

    ESP_LOGI(TAG, "Voice initialized (PDM mic on GPIO %d/%d, %dHz mono)",
             PDM_CLK_PIN, PDM_DIN_PIN, SAMPLE_RATE);
    return ESP_OK;
}

esp_err_t voice_start(QueueHandle_t input_queue)
{
    s_input_queue = input_queue;

    if (xTaskCreate(voice_task, "voice", VOICE_TASK_STACK_SIZE, NULL,
                    VOICE_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create voice task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Voice task started");
    return ESP_OK;
}
