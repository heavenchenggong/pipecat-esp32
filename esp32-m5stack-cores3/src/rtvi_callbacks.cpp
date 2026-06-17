#include <esp_log.h>
#include <cJSON.h>
#include <string.h>

#include "main.h"
#include "servo.h"
#include "led.h"

static void on_bot_started_speaking() {
  // pipecat_screen_new_log();
}

static void on_bot_stopped_speaking() {
  // pipecat_screen_log("\n");
}

static void on_bot_tts_text(const char *text) {
  // pipecat_screen_log(text);
  // pipecat_screen_log(" ");
}

// 自定义 app message 处理 — 来自主机端 Pipecat tool 的设备控制命令
// data JSON 例如:
//   {"action":"nod"}
//   {"action":"shake_head"}
//   {"action":"move_head","pan":-30,"tilt":0}
//   {"action":"center"}
static void on_app_message(const char *json) {
  if (json == NULL) return;
  ESP_LOGI(LOG_TAG, "app_message: %s", json);

  cJSON *root = cJSON_Parse(json);
  if (root == NULL) {
    ESP_LOGW(LOG_TAG, "failed to parse app_message JSON");
    return;
  }

  cJSON *j_action = cJSON_GetObjectItem(root, "action");
  if (j_action == NULL || !cJSON_IsString(j_action)) {
    cJSON_Delete(root);
    return;
  }

  const char *action = j_action->valuestring;
  if (strcmp(action, "nod") == 0) {
    pipecat_servo_nod();
  } else if (strcmp(action, "shake_head") == 0) {
    pipecat_servo_shake();
  } else if (strcmp(action, "center") == 0) {
    pipecat_servo_center();
  } else if (strcmp(action, "move_head") == 0) {
    cJSON *j_pan = cJSON_GetObjectItem(root, "pan");
    cJSON *j_tilt = cJSON_GetObjectItem(root, "tilt");
    int pan = (j_pan && cJSON_IsNumber(j_pan)) ? j_pan->valueint : 0;
    int tilt = (j_tilt && cJSON_IsNumber(j_tilt)) ? j_tilt->valueint : 0;
    pipecat_servo_move(pan, tilt);
  } else if (strcmp(action, "set_led") == 0) {
    cJSON *j_color = cJSON_GetObjectItem(root, "color");
    const char *color = (j_color && cJSON_IsString(j_color)) ? j_color->valuestring : "off";
    pipecat_led_set(color);
  } else {
    ESP_LOGW(LOG_TAG, "unknown action: %s", action);
  }

  cJSON_Delete(root);
}

rtvi_callbacks_t pipecat_rtvi_callbacks = {
    .on_bot_started_speaking = on_bot_started_speaking,
    .on_bot_stopped_speaking = on_bot_stopped_speaking,
    .on_bot_tts_text = on_bot_tts_text,
    .on_app_message = on_app_message,
};
