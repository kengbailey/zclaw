# AGENTS.md

Guidance for AI coding agents working on the zclaw codebase.

## What is zclaw

zclaw is an AI personal assistant that runs on ESP32 microcontrollers. Written in C on ESP-IDF/FreeRTOS. It receives input (serial, Telegram, voice), calls an LLM with tool-calling, and executes hardware actions (GPIO, I2C, scheduling, persistent memory).

This branch (`knob-claw-port`) targets the **Waveshare ESP32-S3-Knob-Touch-LCD-1.8**: ESP32-S3 with 16 MB flash, 8 MB PSRAM, 360x360 round touch LCD, rotary encoder, DRV2605 haptic motor, PDM microphone, and speaker.

## Build & Flash

ESP-IDF v5.5.1 must be sourced before any build/flash/monitor command:

```bash
. ~/esp/v5.5.1/esp-idf/export.sh
idf.py build                                    # build
idf.py -p /dev/cu.usbmodem1101 flash            # flash
idf.py -p /dev/cu.usbmodem1101 monitor          # serial monitor (needs TTY)
```

Or wrap in bash -c:
```bash
bash -c '. ~/esp/v5.5.1/esp-idf/export.sh > /dev/null 2>&1 && idf.py build'
```

Provisioning (WiFi + API keys into NVS):
```bash
bash -c '. ~/esp/v5.5.1/esp-idf/export.sh > /dev/null 2>&1 && ./scripts/provision.sh --port /dev/cu.usbmodem1101'
```

## Architecture

### Message Flow

```
Input sources:
  Serial (channel.c) ──┐
  Telegram (telegram.c) ┤
  Cron (cron.c) ────────┤── channel_msg_t ──> input_queue ──> agent_task (agent.c)
  Voice STT (voice.c) ──┘                                         │
                                                                   ▼
                                                         LLM API call (llm.c)
                                                                   │
                                                         ┌─────────┴─────────┐
                                                         ▼                   ▼
                                                    Tool calls          Text response
                                                   (tools.c)        → output queues
                                                                    → display
```

All input sources push `channel_msg_t` (512-byte text + source enum + chat_id) into a single FreeRTOS queue. The agent task processes them identically regardless of source.

### Message Sources (messages.h)

```c
MSG_SOURCE_CHANNEL  = 0   // USB serial
MSG_SOURCE_TELEGRAM = 1   // Telegram bot
MSG_SOURCE_CRON     = 2   // Scheduled task
MSG_SOURCE_VOICE    = 3   // Voice STT transcription
```

To add a new input source: add enum value, create module with `xxx_start(QueueHandle_t input_queue)`, push `channel_msg_t` to queue. No agent.c changes needed.

### FreeRTOS Tasks

| Task | Stack | Priority | Module |
|------|-------|----------|--------|
| agent | 8 KB | 5 | agent.c |
| channel_read | 4 KB | 5 | channel.c |
| channel_write | 4 KB | 5 | channel.c |
| telegram_poll | 4 KB | 5 | telegram.c |
| cron | 4 KB | 4 | cron.c |
| voice | 6 KB | 4 | voice.c |
| lvgl | 6 KB | 2 | display.c |
| input | 3 KB | 2 | input.c |

### Init Order (main.c)

1. NVS (memory_init)
2. OTA check
3. Display (LVGL, LCD, touch, I2C bus)
4. Input (encoder knob)
5. Haptic (DRV2605)
6. Voice (I2S PDM mic)
7. Factory reset check
8. Boot loop protection
9. WiFi connect
10. Cron/NTP, LLM, rate limiter, Telegram, tools, channel
11. Create queues
12. Start tasks: channel, telegram, voice, agent, cron

## Key Files

### Core

| File | Purpose |
|------|---------|
| `main/main.c` | Boot sequence, WiFi, queue creation, task startup |
| `main/config.h` | All compile-time constants (buffers, timeouts, system prompt) |
| `main/messages.h` | `channel_msg_t`, `message_source_t` enums |
| `main/agent.c` | Agent loop: receive message → LLM → tools → respond |
| `main/llm.c` | HTTP client for LLM APIs (Anthropic, OpenAI, OpenRouter, Ollama) |
| `main/llm_auth.c` | API key management and auth headers |
| `main/channel.c` | USB serial read/write tasks |

### Tools

| File | Purpose |
|------|---------|
| `main/tools.c` | Tool registry, lookup, dispatch |
| `main/tools_gpio.c` | GPIO read/write with pin allowlist |
| `main/tools_i2c.c` | I2C scan/read/write |
| `main/tools_memory.c` | Persistent KV store (NVS) |
| `main/tools_cron.c` | Schedule management |
| `main/tools_persona.c` | Persona get/set/reset |
| `main/tools_system.c` | System info, reboot |
| `main/user_tools.c` | User-created custom tools |

### Hardware (Knob-Touch-LCD variant)

| File | Purpose |
|------|---------|
| `main/display.c` | LVGL UI: status bar, WiFi info, scrollable conversation |
| `main/display.h` | Display states: BOOTING, CONNECTING, IDLE, THINKING, TOOL_EXEC, LISTENING, ERROR |
| `main/input.c` | Rotary encoder → conversation scrolling |
| `main/haptic.c` | DRV2605 haptic effects (CLICK, TICK, ERROR, SUCCESS, etc.) |
| `main/voice.c` | PDM mic recording → WAV → Whisper API → queue injection |

### BSP Components (components/)

| Component | Purpose |
|-----------|---------|
| `i2c_bsp` | I2C master bus init, device handles (touch 0x15, DRV2605 0x5A) |
| `lcd_touch_bsp` | Touch coordinate reading via `tpGetCoordinates()` |
| `lcd_bl_pwm_bsp` | LCD backlight PWM control |
| `user_encoder_bsp` | Quadrature rotary encoder driver |

### Configuration

| File | Purpose |
|------|---------|
| `main/Kconfig.projbuild` | Menuconfig options (WiFi, GPIO safety, voice/STT, stubs) |
| `sdkconfig.defaults` | Default build config (ESP32-S3, PSRAM, LVGL, GPIO pins) |
| `partitions.csv` | Flash layout: 6 MB x2 OTA, 24 KB NVS, ~4 MB storage |

## Hardware Pin Map (Waveshare ESP32-S3-Knob-Touch-LCD-1.8)

| Function | GPIO | Notes |
|----------|------|-------|
| Audio enable (PCM5100A) | 0 | HIGH = speaker on |
| SD card | 2,3,4,5,6,42 | 4-bit SDMMC |
| Encoder B / A | 7 / 8 | Quadrature knob (no push button) |
| Touch INT / RST | 9 / 10 | Capacitive touch |
| I2C SDA / SCL | 11 / 12 | Touch (0x15) + DRV2605 (0x5A) |
| LCD SPI | 13,14,15,16,17,18,21 | SH8601 QSPI display |
| USB D-/D+ | 19 / 20 | Internal USB |
| I2S BCLK/WS/DOUT | 39,40,41 | Speaker output |
| PDM CLK/DIN | 45 / 46 | Microphone input |
| LCD backlight | 47 | PWM |

**Safe for agent GPIO tools:** 1, 35, 36, 37, 38, 43, 44, 48

## Adding Features

### New Tool

1. Create `main/tools_xxx.c` with handler function matching `tool_handler_t` signature
2. Register in `tools.c` → `tools_init()` with name, description, parameter schema
3. Add to `main/CMakeLists.txt` SRCS list
4. Tool results are strings (max 512 bytes, `TOOL_RESULT_BUF_SIZE`)

### New Input Source

1. Add enum to `message_source_t` in `messages.h`
2. Create module with `xxx_init()` and `xxx_start(QueueHandle_t input_queue)`
3. Push `channel_msg_t` with your source enum to `input_queue`
4. Call init in `main.c`, call start after queue creation
5. Add .c to `CMakeLists.txt`

### New Display State

1. Add to `display_state_t` enum in `display.h`
2. Add case to `display_set_state()` switch in `display.c` (color + label text)
3. All LVGL access must be wrapped in `lvgl_lock()`/`lvgl_unlock()`

## Conventions

- **Language**: C (no C++), ESP-IDF v5.5.1 APIs
- **Error handling**: Return `esp_err_t`. Non-fatal failures log warning and continue. Fatal failures call `fail_fast_startup()` which restarts.
- **Memory**: Large buffers (>1 KB) go in PSRAM via `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`. Task stacks and small buffers use internal RAM.
- **NVS keys**: Max 15 chars. Defined in `nvs_keys.h`. Namespace `zclaw` for general, `zc_cron`/`zc_tools`/`zc_config` for subsystems.
- **Thread safety**: LVGL access requires `s_lvgl_mux` mutex. I2C bus is shared (touch, DRV2605, tools_i2c) — reads are idempotent but concurrent writes need care.
- **Kconfig**: Runtime-configurable settings go in `Kconfig.projbuild`. Compile-time constants go in `config.h`.
- **Init pattern**: `xxx_init()` allocates resources, `xxx_start(queue)` creates the FreeRTOS task. Init happens early (before WiFi), start happens after queue creation.

## Common Pitfalls

- **Must source ESP-IDF** before build/flash/provision commands
- **Boot loop protection**: After 4 consecutive boot failures, device enters safe mode. If NVS boot_count gets stuck, erase NVS: `python -m esptool --chip esp32s3 -p PORT erase_region 0x9000 0x6000` (then reprovision)
- **I2C bus sharing**: Touch, DRV2605, and tools_i2c all share I2C_NUM_0 on GPIO 11/12. The bus is initialized once in `display_init()` → `i2c_master_Init()`.
- **PSRAM for large allocations**: HTTP bodies, audio buffers, and WAV data must use PSRAM. Internal RAM is ~200 KB and shared with task stacks, DMA buffers, TLS.
- **TLS heap pressure**: Each HTTPS connection needs ~40-60 KB internal RAM for TLS handshake. Serialize STT → LLM → TTS to avoid concurrent TLS sessions.
- **No encoder button**: The rotary encoder has NO push button GPIO. Use touch screen gestures for button-like interactions.
- **Factory reset pin**: Disabled (set to -1) because GPIO 9 is used by touch INT. Factory reset via NVS erase only.
- **Clone boards**: Some clone Waveshare boards have defective DRV2605 chips (I2C reads work but writes silently fail). Use genuine Waveshare boards.

## Testing

- **Host tests**: `./scripts/test.sh` (runs test/host/ suite, no device needed)
- **QEMU emulation**: `./scripts/emulate.sh` (stub LLM, no WiFi)
- **On-device**: Flash, monitor via serial, verify boot log shows all subsystems initialized
- **CI**: GitHub Actions runs multi-target builds (C3/S3/C6), size guard (888 KiB), stack guard, and host tests
