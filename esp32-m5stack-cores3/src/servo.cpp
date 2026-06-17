// Minimal SCS0009 servo driver — see servo.h for API.
//
// Hardware notes for Stack-chan on M5Stack CoreS3:
//   - Servos are 5V SCS0009 on UART_NUM_1 (GPIO 6 TX, GPIO 7 RX, 1 Mbps).
//   - **CRITICAL**: servo power is gated by a PY32 I2C IO Expander
//     (addr 0x6F on the internal I2C bus, GPIO 11 SCL / 12 SDA).
//     Pin 0 of the PY32 = VM_EN. It must be set to OUTPUT, pull-up enabled,
//     and driven HIGH before servos see any voltage. Without this,
//     UART bytes go nowhere.
//
// Protocol summary (Feetech SCS / Waveshare SCS0009):
//   Frame:  0xFF 0xFF [ID] [LEN] [CMD] [PARAM...] [CHECKSUM]
//     LEN = (PARAM bytes) + 2
//     CHECKSUM = ~(ID + LEN + CMD + PARAM_SUM) & 0xFF
//   CMD 0x03 = WRITE; ADDR 0x2A = Goal Position L
//   Position 0..1023 (10-bit), 512 = center, ~300° range → 1° ≈ 3.4 ticks.

#include "servo.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "pipecat_servo"

#define SERVO_UART          UART_NUM_1
#define SERVO_TX_PIN        6
#define SERVO_RX_PIN        7
#define SERVO_BAUDRATE      1000000
#define SERVO_RX_BUF_SIZE   256

#define SERVO_YAW_ID        1
#define SERVO_PITCH_ID      2

#define CMD_WRITE           0x03
#define ADDR_GOAL_POSITION  0x2A

// PY32 I2C IO Expander (controls VM_EN on Stack-chan + the RGB LEDs)
#define PY32_ADDR           0x6F
#define PY32_REG_GPIO_M_L   0x03  // Direction low byte (1 = output)
#define PY32_REG_GPIO_O_L   0x05  // Output level low byte
#define PY32_REG_GPIO_PU_L  0x09  // Pull-up low byte
#define PY32_REG_GPIO_PD_L  0x0B  // Pull-down low byte
#define PY32_VM_EN_PIN      0     // Pin 0 = servo power switch

// LED registers (same PY32). LEDs are NOT GPIO-controlled SK6812; they're driven
// by the PY32 chip itself, exposed via these registers (stackchan-arduino source):
//   REG_LED_CFG = 0x24  bits[5:0] = count (max 32), bit 6 = refresh trigger
//   REG_LED_RAM_START = 0x30  16 bytes per LED... no, 2 bytes per LED (RGB565 LE)
// Stack-chan kit ships with 3 LEDs (cheek-L / cheek-R / center).
#define PY32_REG_LED_CFG       0x24
#define PY32_REG_LED_RAM_START 0x30

#define I2C_SCL_PIN         GPIO_NUM_11
#define I2C_SDA_PIN         GPIO_NUM_12
#define I2C_FREQ_HZ         100000  // 100 kHz (PY32 reliability — see stackchan-mcp notes)

// SCS0009 ticks per degree (10-bit / ~300°)
// Stack-chan physical mounting offset:
//   yaw=460 / pitch=716 are the raw positions where the head physically faces forward.
//   (boot self-test confirmed 716 = 物理 forward; the previous code "looked correct"
//    because deg_to_ticks_pitch(45) clamped to 30 → 620 + 30*3.2 = 716 by accident.)
// TICKS_PER_DEG = 16/5 from stackchan-mcp.
static const int YAW_CENTER_POS   = 460;   // raw position for yaw 0°
static const int PITCH_CENTER_POS = 716;   // raw position for pitch 0° (physical forward)
static const float TICKS_PER_DEG  = 16.0f / 5.0f;  // = 3.2 ticks per degree

static bool servo_initialized = false;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t py32_dev = NULL;

// Convert degrees to raw servo position for a specific axis.
// Each axis has its own physical center (mounting offset); see
// YAW_CENTER_POS / PITCH_CENTER_POS above.
static int deg_to_ticks_yaw(int deg) {
    if (deg < -60) deg = -60;
    if (deg > 60) deg = 60;
    return YAW_CENTER_POS + (int)(deg * TICKS_PER_DEG);
}

static int deg_to_ticks_pitch(int deg) {
    if (deg < -30) deg = -30;
    if (deg > 30) deg = 30;
    return PITCH_CENTER_POS + (int)(deg * TICKS_PER_DEG);
}

// I2C helper: write one byte to a PY32 register (with light retry)
static bool py32_write_reg(uint8_t reg, uint8_t value) {
    if (!py32_dev) return false;
    uint8_t buf[2] = {reg, value};
    for (int attempt = 0; attempt < 3; attempt++) {
        esp_err_t err = i2c_master_transmit(py32_dev, buf, 2, 100);
        if (err == ESP_OK) return true;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return false;
}

// Read-modify-write a single bit in a register (used for pin-level config)
static bool py32_set_bit(uint8_t reg, uint8_t pin, bool level) {
    if (!py32_dev) return false;
    uint8_t cur = 0;
    esp_err_t err = i2c_master_transmit_receive(py32_dev, &reg, 1, &cur, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "py32 read reg 0x%02X failed: %d", reg, err);
        return false;
    }
    if (level) {
        cur |= (1 << pin);
    } else {
        cur &= ~(1 << pin);
    }
    return py32_write_reg(reg, cur);
}

// Initialize PY32 IO Expander and turn on VM_EN to power the servos.
// This is the missing piece that makes UART servo writes actually do anything.
static bool init_servo_power(void) {
    // Internal I2C bus on CoreS3 (controller 1, GPIO 11/12)
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = (i2c_port_t)1,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { .enable_internal_pullup = 1 },
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %d", err);
        return false;
    }

    // Add PY32 device at 100 kHz (M5 default — PY32 mis-behaves at 400 kHz)
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PY32_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 0,
        .flags = { .disable_ack_check = 0 },
    };
    err = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &py32_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "py32 add_device failed: %d", err);
        return false;
    }

    // Pin 0 = VM_EN: direction=output, pull-up=enabled, output=HIGH
    bool ok_dir   = py32_set_bit(PY32_REG_GPIO_M_L, PY32_VM_EN_PIN, true);
    bool ok_pull  = py32_set_bit(PY32_REG_GPIO_PU_L, PY32_VM_EN_PIN, true);
    bool ok_pd    = py32_set_bit(PY32_REG_GPIO_PD_L, PY32_VM_EN_PIN, false);
    bool ok_write = py32_set_bit(PY32_REG_GPIO_O_L, PY32_VM_EN_PIN, true);

    if (!ok_dir || !ok_pull || !ok_pd || !ok_write) {
        ESP_LOGE(TAG, "PY32 VM_EN setup failed (dir=%d pull=%d pd=%d write=%d)",
                 ok_dir, ok_pull, ok_pd, ok_write);
        return false;
    }
    ESP_LOGI(TAG, "PY32 VM_EN driven HIGH (servo power ON)");
    vTaskDelay(pdMS_TO_TICKS(200));  // give servos time to boot
    return true;
}

// Send "WRITE goal position" command to one servo
// Note: SCS0009 expects small time values (~30ms) per step + speed parameter.
// time too large or speed=0 with non-zero time causes the servo to ignore the move.
// stackchan-mcp uses time=30ms speed=0 for smooth interpolation across many writes.
//
// CRITICAL: SCS0009 sends a 6-byte ACK packet after every WRITE. If we don't
// drain the ACK before the next WRITE, the bus state goes wrong and all
// subsequent WRITEs are silently dropped ("starts moving once then never
// moves again" — known bug, see stackchan-mcp comment about Level=0). Also
// SCS0009 uses half-duplex: TX and RX share one wire, so we MUST flush RX
// after every send. uart_flush is the simplest robust approach here.
static void write_pos(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed) {
    if (!servo_initialized) return;

    uint8_t buf[14];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 9;  // LEN: 7 params + 2
    buf[4] = CMD_WRITE;
    buf[5] = ADDR_GOAL_POSITION;
    // SCS0009 protocol uses BIG-ENDIAN multi-byte fields (End=1 default).
    // The Feetech reference driver names the bytes "DataL/DataH" but with
    // End=1 it actually puts the HIGH byte first. We MUST send high then low.
    // Sending little-endian instead makes the servo interpret the position
    // as a completely different value — head spins to a random pose with
    // no error reported. (This is the cause of the "head goes backward
    // and screen up" symptom we saw with little-endian.)
    buf[6] = (position >> 8) & 0xFF;  // position HIGH
    buf[7] = position & 0xFF;          // position LOW
    buf[8] = (time_ms >> 8) & 0xFF;    // time HIGH
    buf[9] = time_ms & 0xFF;            // time LOW
    buf[10] = (speed >> 8) & 0xFF;     // speed HIGH
    buf[11] = speed & 0xFF;             // speed LOW

    uint16_t sum = 0;
    for (int i = 2; i <= 11; i++) sum += buf[i];
    buf[12] = (~sum) & 0xFF;

    int written = uart_write_bytes(SERVO_UART, (const char*)buf, 13);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(100));

    // DEBUG: dump TX bytes so we can see exactly what we sent.
    ESP_LOGI(TAG, "TX[id=%d pos=%d t=%d sp=%d] wrote=%d %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             id, position, time_ms, speed, written,
             buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
             buf[7], buf[8], buf[9], buf[10], buf[11], buf[12]);

    // Drain ACK reply (6 bytes: 0xFF 0xFF ID LEN ERR CHK). Without this,
    // every subsequent WritePos is silently dropped — the SCS0009 known
    // "first move OK, then never moves again" symptom.
    uint8_t ack[16] = {0};
    int rx = uart_read_bytes(SERVO_UART, ack, sizeof(ack), pdMS_TO_TICKS(20));
    if (rx > 0) {
        ESP_LOGI(TAG, "ACK[%d]: %02X %02X %02X %02X %02X %02X",
                 rx, ack[0], ack[1], ack[2], ack[3], ack[4], ack[5]);
    } else {
        ESP_LOGW(TAG, "ACK: no reply (rx=%d)", rx);
    }
}

// Calibration sweep: slowly walk yaw 0..1023 and pitch 0..1023 so user
// can identify the raw position where the head physically faces forward.
// Watch the serial log: each position is printed for 1.5s before moving on.
void pipecat_servo_calibration_sweep(void) {
    ESP_LOGI(TAG, "=== CALIBRATION SWEEP START ===");
    ESP_LOGI(TAG, "Moving YAW through full range. Note position where head faces FORWARD.");

    // Sweep yaw across 8 positions: 100, 200, 300, ... 800, 900
    int yaw_positions[] = {100, 200, 300, 400, 500, 600, 700, 800, 900};
    for (int i = 0; i < 9; i++) {
        ESP_LOGI(TAG, ">>> YAW = %d", yaw_positions[i]);
        write_pos(SERVO_YAW_ID, yaw_positions[i], 0, 200);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    write_pos(SERVO_YAW_ID, 460, 0, 200);  // back to nominal center
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "Moving PITCH. Note position where head faces FORWARD (level).");
    int pitch_positions[] = {200, 400, 500, 600, 700, 800, 900};
    for (int i = 0; i < 7; i++) {
        ESP_LOGI(TAG, ">>> PITCH = %d", pitch_positions[i]);
        write_pos(SERVO_PITCH_ID, pitch_positions[i], 0, 200);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
    write_pos(SERVO_PITCH_ID, 620, 0, 200);
    ESP_LOGI(TAG, "=== CALIBRATION SWEEP DONE ===");
}

void pipecat_servo_init(void) {
    if (servo_initialized) return;

    // Step 1: power up servos via PY32 IO Expander (CRITICAL!)
    if (!init_servo_power()) {
        ESP_LOGE(TAG, "servo power init failed — servos will not respond");
        // continue anyway so UART init still happens (for diagnostics)
    }

    // Step 2: configure UART
    uart_config_t cfg = {
        .baud_rate = SERVO_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t err = uart_param_config(SERVO_UART, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %d", err);
        return;
    }
    err = uart_set_pin(SERVO_UART, SERVO_TX_PIN, SERVO_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
        return;
    }
    err = uart_driver_install(SERVO_UART, SERVO_RX_BUF_SIZE, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
        return;
    }

    servo_initialized = true;
    ESP_LOGI(TAG, "Servo UART %d ready (TX=%d RX=%d %d bps)",
             SERVO_UART, SERVO_TX_PIN, SERVO_RX_PIN, SERVO_BAUDRATE);

    // PY32 LED 初始化（套件标配 3 LED）
    pipecat_py32_led_init(3);

    // Small delay to let bus settle, then center to known good pose
    vTaskDelay(pdMS_TO_TICKS(100));
    pipecat_servo_center();
}

void pipecat_servo_move(int pan_deg, int tilt_deg) {
    int yaw_ticks   = deg_to_ticks_yaw(pan_deg);
    // tilt_deg 直接传给 deg_to_ticks_pitch（正=往上看，负=往下看）。
    // PITCH_CENTER_POS=716 已经是物理"看正前方"，不需要 +45° offset。
    // 之前 `45 - tilt_deg` 然后 clamp 到 ±30，导致 tilt 永远 clamp 飞，pitch_ticks
    // 总是 716（跟 boot center 那次的 deg_to_ticks_pitch(45) clamp 后值相同）。
    int pitch_ticks = deg_to_ticks_pitch(tilt_deg);
    ESP_LOGI(TAG, "move pan=%d tilt=%d -> yaw_ticks=%d pitch_ticks=%d",
             pan_deg, tilt_deg, yaw_ticks, pitch_ticks);
    write_pos(SERVO_YAW_ID, yaw_ticks, 0, 300);
    write_pos(SERVO_PITCH_ID, pitch_ticks, 0, 300);
}

void pipecat_servo_center(void) {
    // 物理"看正前方"对应:
    //   yaw_ticks = YAW_CENTER_POS = 460
    //   pitch_ticks = PITCH_CENTER_POS = 716
    int yaw_pos = deg_to_ticks_yaw(0);
    int pitch_pos = deg_to_ticks_pitch(0);
    ESP_LOGI(TAG, "center (yaw=%d pitch=%d)", yaw_pos, pitch_pos);
    write_pos(SERVO_YAW_ID, yaw_pos, 0, 300);
    write_pos(SERVO_PITCH_ID, pitch_pos, 0, 300);
}

void pipecat_servo_nod(void) {
    ESP_LOGI(TAG, "nod");
    int neutral = deg_to_ticks_pitch(0);   // forward
    int down = deg_to_ticks_pitch(-15);    // 15° down
    int up   = deg_to_ticks_pitch(15);     // 15° up

    // Need longer delay between SCS0009 commands than the move duration —
    // otherwise the servo's still mid-move when next WRITE arrives and
    // the new target sometimes gets ignored.
    write_pos(SERVO_PITCH_ID, down, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(500));
    write_pos(SERVO_PITCH_ID, up, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(500));
    write_pos(SERVO_PITCH_ID, down, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(500));
    write_pos(SERVO_PITCH_ID, neutral, 0, 400);
    vTaskDelay(pdMS_TO_TICKS(500));
}

void pipecat_servo_shake(void) {
    ESP_LOGI(TAG, "shake");
    int neutral = deg_to_ticks_yaw(0);
    int left  = deg_to_ticks_yaw(-30);
    int right = deg_to_ticks_yaw(30);
    write_pos(SERVO_YAW_ID, left, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(500));
    write_pos(SERVO_YAW_ID, right, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(500));
    write_pos(SERVO_YAW_ID, left, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(500));
    write_pos(SERVO_YAW_ID, neutral, 0, 400);
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ── PY32 LED API (复用 servo.cpp 已经初始化好的 py32_dev I2C handle) ────────
//
// stackchan-arduino 源码确认：stack-chan 套件的 LED 不是 GPIO SK6812，而是 PY32
// IO Expander 自带的 LED 驱动器 — 通过 I2C 写 REG_LED_CFG 设置数量、写
// REG_LED_RAM_START 起的 RAM (每 LED 2 字节 RGB565 little-endian)，然后
// 写 REG_LED_CFG bit 6 = 1 触发刷新。
//
// 套件出厂带 3 LEDs (cheek-L, cheek-R, optional center)，所以 default count=3。

static uint8_t s_led_count = 0;

void pipecat_py32_led_init(uint8_t count) {
    if (!py32_dev) {
        ESP_LOGW(TAG, "LED init: py32_dev not ready (call pipecat_servo_init first)");
        return;
    }
    if (count > 32) count = 32;
    s_led_count = count;
    uint8_t buf[2] = {PY32_REG_LED_CFG, (uint8_t)(count & 0x3F)};
    esp_err_t err = i2c_master_transmit(py32_dev, buf, 2, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PY32 LED count set to %d (write CFG=0x%02X OK)", count, count);
    } else {
        ESP_LOGE(TAG, "PY32 LED setLedCount failed: %d", err);
    }
    // Read back to verify
    uint8_t reg = PY32_REG_LED_CFG;
    uint8_t cur = 0;
    err = i2c_master_transmit_receive(py32_dev, &reg, 1, &cur, 1, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "PY32 LED CFG read-back = 0x%02X", cur);
    } else {
        ESP_LOGW(TAG, "PY32 LED CFG read-back failed: %d", err);
    }
}

void pipecat_py32_led_set(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    if (!py32_dev || index >= 32) return;
    // RGB888 → RGB565: RRRRRGGG_GGGBBBBB
    uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    // 写 RAM: addr = REG_LED_RAM_START + index*2; 然后 LE 2 bytes
    uint8_t buf[3] = {
        (uint8_t)(PY32_REG_LED_RAM_START + index * 2),
        (uint8_t)(color565 & 0xFF),
        (uint8_t)((color565 >> 8) & 0xFF),
    };
    esp_err_t err = i2c_master_transmit(py32_dev, buf, 3, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "LED[%d]=rgb(%d,%d,%d)=0x%04X OK", index, r, g, b, color565);
    } else {
        ESP_LOGW(TAG, "LED[%d] set rgb(%d,%d,%d) failed: %d", index, r, g, b, err);
    }
}

void pipecat_py32_led_set_all(uint8_t r, uint8_t g, uint8_t b) {
    uint8_t n = s_led_count == 0 ? 3 : s_led_count;
    for (uint8_t i = 0; i < n; i++) {
        pipecat_py32_led_set(i, r, g, b);
    }
}

void pipecat_py32_led_refresh(void) {
    if (!py32_dev) return;
    // Read REG_LED_CFG, set bit 6, write back to trigger refresh.
    uint8_t reg = PY32_REG_LED_CFG;
    uint8_t cur = 0;
    esp_err_t err = i2c_master_transmit_receive(py32_dev, &reg, 1, &cur, 1, 100);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LED refresh: read CFG failed: %d", err);
        return;
    }
    uint8_t buf[2] = {PY32_REG_LED_CFG, (uint8_t)(cur | (1 << 6))};
    err = i2c_master_transmit(py32_dev, buf, 2, 100);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "LED refresh OK (CFG was 0x%02X, wrote 0x%02X)", cur, cur | 0x40);
    } else {
        ESP_LOGW(TAG, "LED refresh: write CFG failed: %d", err);
    }
}
