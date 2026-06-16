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

// PY32 I2C IO Expander (controls VM_EN on Stack-chan)
#define PY32_ADDR           0x6F
#define PY32_REG_GPIO_M_L   0x03  // Direction low byte (1 = output)
#define PY32_REG_GPIO_O_L   0x05  // Output level low byte
#define PY32_REG_GPIO_PU_L  0x09  // Pull-up low byte
#define PY32_REG_GPIO_PD_L  0x0B  // Pull-down low byte
#define PY32_VM_EN_PIN      0     // Pin 0 = servo power switch

#define I2C_SCL_PIN         GPIO_NUM_11
#define I2C_SDA_PIN         GPIO_NUM_12
#define I2C_FREQ_HZ         100000  // 100 kHz (PY32 reliability — see stackchan-mcp notes)

// SCS0009 ticks per degree (10-bit / ~300°)
// Stack-chan calibration values — physical mounting offset on the SCS0009
// gear differs from the protocol "512 = center". stackchan-mcp ships these
// values for the same hardware; we adopt them directly.
static const int YAW_CENTER_POS   = 460;   // raw position for yaw 0°
static const int PITCH_CENTER_POS = 620;   // raw position for pitch 0°
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
static void write_pos(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed) {
    if (!servo_initialized) return;

    uint8_t buf[14];
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 9;  // LEN: 7 params + 2
    buf[4] = CMD_WRITE;
    buf[5] = ADDR_GOAL_POSITION;
    buf[6] = position & 0xFF;
    buf[7] = (position >> 8) & 0xFF;
    buf[8] = time_ms & 0xFF;
    buf[9] = (time_ms >> 8) & 0xFF;
    buf[10] = speed & 0xFF;
    buf[11] = (speed >> 8) & 0xFF;

    uint16_t sum = 0;
    for (int i = 2; i <= 11; i++) sum += buf[i];
    buf[12] = (~sum) & 0xFF;

    uart_write_bytes(SERVO_UART, (const char*)buf, 13);
    uart_wait_tx_done(SERVO_UART, pdMS_TO_TICKS(100));
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

    // Calibration mode: slowly sweep so user can find the FORWARD raw positions
    vTaskDelay(pdMS_TO_TICKS(500));
    pipecat_servo_calibration_sweep();
}

void pipecat_servo_move(int pan_deg, int tilt_deg) {
    int yaw_ticks   = deg_to_ticks_yaw(pan_deg);
    // Stack-chan: forward-facing pitch is 45°. tilt=0 means look forward,
    // tilt>0 means look up (subtract from 45°).
    int pitch_ticks = deg_to_ticks_pitch(45 - tilt_deg);
    ESP_LOGI(TAG, "move pan=%d tilt=%d -> yaw_ticks=%d pitch_ticks=%d",
             pan_deg, tilt_deg, yaw_ticks, pitch_ticks);
    write_pos(SERVO_YAW_ID, yaw_ticks, 0, 300);
    write_pos(SERVO_PITCH_ID, pitch_ticks, 0, 300);
}

void pipecat_servo_center(void) {
    // Stack-chan physical "looking forward" pose:
    //   yaw = 0° (face the user directly)
    //   pitch = 45° (raises the face up so the screen points forward, not at the floor)
    // This matches stackchan-mcp's BOOT_INIT_YAW_DEG=0 / BOOT_INIT_PITCH_DEG=45.
    int yaw_pos = deg_to_ticks_yaw(0);
    int pitch_pos = deg_to_ticks_pitch(45);
    ESP_LOGI(TAG, "center (yaw=0deg=%d pitch=45deg=%d)", yaw_pos, pitch_pos);
    // SCS0009: time=0 + non-zero speed = constant-speed move at 'speed' ticks/s.
    // Large time values can be ignored or misinterpreted on some firmware revs;
    // stackchan-mcp uses time=30 with high-frequency interpolation. Here we
    // use simple speed-controlled moves to avoid the time-large bug.
    write_pos(SERVO_YAW_ID, yaw_pos, 0, 300);
    write_pos(SERVO_PITCH_ID, pitch_pos, 0, 300);
}

void pipecat_servo_nod(void) {
    ESP_LOGI(TAG, "nod");
    int neutral = deg_to_ticks_pitch(45);  // forward
    int down = deg_to_ticks_pitch(60);     // 15° down from forward
    int up   = deg_to_ticks_pitch(30);     // 15° up from forward
    write_pos(SERVO_PITCH_ID, down, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_PITCH_ID, up, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_PITCH_ID, down, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_PITCH_ID, neutral, 0, 400);
}

void pipecat_servo_shake(void) {
    ESP_LOGI(TAG, "shake");
    int neutral = deg_to_ticks_yaw(0);
    int left  = deg_to_ticks_yaw(-30);
    int right = deg_to_ticks_yaw(30);
    write_pos(SERVO_YAW_ID, left, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_YAW_ID, right, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_YAW_ID, left, 0, 600);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_YAW_ID, neutral, 0, 400);
}
