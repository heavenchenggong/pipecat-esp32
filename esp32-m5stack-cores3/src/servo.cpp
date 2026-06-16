// Minimal SCS0009 servo driver — see servo.h for API.
//
// Protocol summary (Feetech SCS / Waveshare SCS0009):
//   Frame:  0xFF 0xFF [ID] [LEN] [CMD] [PARAM...] [CHECKSUM]
//     LEN = (PARAM bytes) + 2
//     CHECKSUM = ~(ID + LEN + CMD + PARAM_SUM) & 0xFF
//
//   CMD 0x03 = WRITE
//   Goal Position register address = 0x2A (low) / 0x2B (high)
//   To write "go to position P over time T at speed S":
//     PARAM = [0x2A, P_low, P_high, T_low, T_high, S_low, S_high]
//
//   Position range: 0..1023 (10-bit), 512 = center, 0/1023 = ~±150°
//   For SCS0009 the working angle is ~300°, so 1° ≈ 3.4 ticks.

#include "servo.h"
#include "esp_log.h"
#include "driver/uart.h"
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

// SCS0009 ticks per degree (10-bit / ~300°)
static const float TICKS_PER_DEG = 1024.0f / 300.0f;
static const int CENTER_POS = 512;

static bool servo_initialized = false;

static int deg_to_ticks(int deg, int axis_min, int axis_max) {
    if (deg < axis_min) deg = axis_min;
    if (deg > axis_max) deg = axis_max;
    return CENTER_POS + (int)(deg * TICKS_PER_DEG);
}

// Send "WRITE goal position" command to one servo
static void write_pos(uint8_t id, uint16_t position, uint16_t time_ms, uint16_t speed) {
    if (!servo_initialized) return;

    // Frame: 0xFF 0xFF [ID] [LEN=9] [CMD=WRITE] [ADDR=0x2A] [P_l P_h T_l T_h S_l S_h] [CHK]
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

void pipecat_servo_init(void) {
    if (servo_initialized) return;

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

    // Small delay to let bus settle, then center
    vTaskDelay(pdMS_TO_TICKS(100));
    pipecat_servo_center();

    // Boot 自检：等 1 秒后 nod 一下，物理上能看到点头说明 servo 物理通
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "BOOT SELF-TEST: nod sequence");
    pipecat_servo_nod();
    ESP_LOGI(TAG, "BOOT SELF-TEST: done");
}

void pipecat_servo_move(int pan_deg, int tilt_deg) {
    int yaw_ticks   = deg_to_ticks(pan_deg, -60, 60);
    int pitch_ticks = deg_to_ticks(-tilt_deg, -30, 30);  // mechanical convention
    ESP_LOGI(TAG, "move pan=%d tilt=%d -> yaw_ticks=%d pitch_ticks=%d",
             pan_deg, tilt_deg, yaw_ticks, pitch_ticks);
    write_pos(SERVO_YAW_ID, yaw_ticks, 500, 0);
    write_pos(SERVO_PITCH_ID, pitch_ticks, 500, 0);
}

void pipecat_servo_center(void) {
    ESP_LOGI(TAG, "center");
    write_pos(SERVO_YAW_ID, CENTER_POS, 500, 0);
    write_pos(SERVO_PITCH_ID, CENTER_POS, 500, 0);
}

void pipecat_servo_nod(void) {
    ESP_LOGI(TAG, "nod");
    int down = deg_to_ticks(20, -30, 30);  // tilt down
    int up   = deg_to_ticks(-10, -30, 30); // tilt up
    write_pos(SERVO_PITCH_ID, down, 250, 0);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_PITCH_ID, up, 250, 0);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_PITCH_ID, down, 250, 0);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_PITCH_ID, CENTER_POS, 350, 0);
}

void pipecat_servo_shake(void) {
    ESP_LOGI(TAG, "shake");
    int left  = deg_to_ticks(-30, -60, 60);
    int right = deg_to_ticks(30, -60, 60);
    write_pos(SERVO_YAW_ID, left, 250, 0);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_YAW_ID, right, 250, 0);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_YAW_ID, left, 250, 0);
    vTaskDelay(pdMS_TO_TICKS(280));
    write_pos(SERVO_YAW_ID, CENTER_POS, 350, 0);
}
