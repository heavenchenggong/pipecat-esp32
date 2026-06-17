// CoreS3 没有内置 RGB LED；用 LCD 整屏 fillScreen 当大彩色灯。
// 显示亮一片纯色，比单 RGB LED 还醒目，且不需要外接硬件。

#include "led.h"
#include "esp_log.h"
#include <string.h>
#include <M5Unified.h>

#define TAG "pipecat_led"

extern "C" void pipecat_led_set(const char *color) {
    if (color == NULL) return;
    ESP_LOGI(TAG, "set color = %s", color);

    // M5GFX color 常量：M5GFX.h 里定义为 565 格式
    // 简单 if-else 匹配，避免拉 std::map 增加 binary
    if (strcmp(color, "red") == 0) {
        M5.Display.fillScreen(RED);
    } else if (strcmp(color, "green") == 0) {
        M5.Display.fillScreen(GREEN);
    } else if (strcmp(color, "blue") == 0) {
        M5.Display.fillScreen(BLUE);
    } else if (strcmp(color, "yellow") == 0) {
        M5.Display.fillScreen(YELLOW);
    } else if (strcmp(color, "purple") == 0) {
        M5.Display.fillScreen(PURPLE);
    } else if (strcmp(color, "white") == 0) {
        M5.Display.fillScreen(WHITE);
    } else if (strcmp(color, "cyan") == 0) {
        M5.Display.fillScreen(CYAN);
    } else if (strcmp(color, "orange") == 0) {
        M5.Display.fillScreen(ORANGE);
    } else if (strcmp(color, "pink") == 0) {
        M5.Display.fillScreen(PINK);
    } else if (strcmp(color, "off") == 0 || strcmp(color, "black") == 0) {
        M5.Display.fillScreen(BLACK);
    } else {
        ESP_LOGW(TAG, "unknown color '%s', off", color);
        M5.Display.fillScreen(BLACK);
    }
}
