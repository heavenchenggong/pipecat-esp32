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

#ifdef __cplusplus
extern "C" {
#endif

void pipecat_servo_init(void);
void pipecat_servo_move(int pan_deg, int tilt_deg);
void pipecat_servo_nod(void);
void pipecat_servo_shake(void);
void pipecat_servo_center(void);

#ifdef __cplusplus
}
#endif

#endif
