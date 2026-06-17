// Stack-chan 套件的 RGB LED 是接在 PY32 IO Expander 上的（不是 GPIO SK6812）。
// 由 servo.cpp 持有 PY32 i2c handle，这里只做颜色名 → RGB888 的映射。
//
// 参考: stackchan-arduino src/drivers/PY32IOExpander_Class — REG_LED_CFG 0x24
// + REG_LED_RAM_START 0x30 (每 LED 2 bytes RGB565 LE) + bit 6 of CFG = refresh.

#include "led.h"
#include "servo.h"
#include "esp_log.h"
#include <string.h>
#include <M5Unified.h>

#define TAG "pipecat_led"

extern "C" void pipecat_led_set(const char *color) {
    if (color == NULL) return;
    ESP_LOGI(TAG, "set color = %s", color);

    uint8_t r = 0, g = 0, b = 0;
    if (strcmp(color, "red") == 0)         { r = 255; g = 0;   b = 0;   }
    else if (strcmp(color, "green") == 0)  { r = 0;   g = 255; b = 0;   }
    else if (strcmp(color, "blue") == 0)   { r = 0;   g = 0;   b = 255; }
    else if (strcmp(color, "yellow") == 0) { r = 255; g = 200; b = 0;   }
    else if (strcmp(color, "purple") == 0) { r = 200; g = 0;   b = 255; }
    else if (strcmp(color, "white") == 0)  { r = 255; g = 255; b = 255; }
    else if (strcmp(color, "cyan") == 0)   { r = 0;   g = 255; b = 255; }
    else if (strcmp(color, "orange") == 0) { r = 255; g = 100; b = 0;   }
    else if (strcmp(color, "pink") == 0)   { r = 255; g = 100; b = 180; }
    else if (strcmp(color, "off") == 0 || strcmp(color, "black") == 0) {
        r = 0; g = 0; b = 0;
    } else {
        ESP_LOGW(TAG, "unknown color '%s', off", color);
        r = 0; g = 0; b = 0;
    }

    // 写所有 LED 同色 + 触发刷新
    pipecat_py32_led_set_all(r, g, b);
    pipecat_py32_led_refresh();

    // Bonus: 同时让屏幕背景换色，让用户更直观（PY32 LED 是脸颊小灯）
    uint16_t color565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    M5.Display.fillScreen(color565);
}
