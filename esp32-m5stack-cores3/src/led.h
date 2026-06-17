#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// CoreS3 没有内置 RGB LED（BUILTIN_LED_GPIO=NC），用 LCD 屏当大彩色灯。
// 颜色名: "red"/"green"/"blue"/"yellow"/"purple"/"white"/"off"。
// 不识别的颜色当 off 处理（黑屏）。
void pipecat_led_set(const char *color);

#ifdef __cplusplus
}
#endif
