// Minimal SCS0009 servo driver for pipecat-esp32 + Stack-chan
//
// Hardware:
//   StackChan with 2x SCS0009 servos:
//     ID 1 = yaw   (horizontal, pan)
//     ID 2 = pitch (vertical)
//   UART 1, GPIO 6 TX / GPIO 7 RX, 1Mbps
//
// Public API:
//   pipecat_servo_init()                         — init UART, center both
//   pipecat_servo_move(int pan_deg, int tilt_deg) — pan: -60..60, tilt: -30..30
//   pipecat_servo_nod()                           — nod (down-up-down-center)
//   pipecat_servo_shake()                         — shake (left-right-left-center)
//   pipecat_servo_center()                        — both back to 0,0

#ifndef PIPECAT_SERVO_H
#define PIPECAT_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void pipecat_servo_init(void);
void pipecat_servo_move(int pan_deg, int tilt_deg);
void pipecat_servo_nod(void);
void pipecat_servo_shake(void);
void pipecat_servo_center(void);

// Stack-chan 套件的 RGB LED 也挂在 PY32 IO Expander 上（不是 GPIO 控制的 SK6812）。
// LED control: I2C addr 0x6F, REG_LED_CFG=0x24 (count), REG_LED_RAM_START=0x30 (RGB565 ram).
// stackchan 套件 LED 数量是 3（左右脸颊 + 中间），每个 2 字节 RGB565 little-endian。
// 调用前必须先 pipecat_servo_init() 让 PY32 设备 handle 准备好。
void pipecat_py32_led_init(uint8_t count);
void pipecat_py32_led_set(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void pipecat_py32_led_set_all(uint8_t r, uint8_t g, uint8_t b);
void pipecat_py32_led_refresh(void);

#ifdef __cplusplus
}
#endif

#endif
