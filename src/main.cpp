#include "obd_can.h"

#include <driver/twai.h>

namespace {
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_21;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_22;
constexpr uint32_t CAN_REQUEST_ID = 0x7DF;
constexpr uint32_t CAN_REPLY_ID = 0x7E8;

bool canStarted = false;
}

bool obdCanInit() {
  if (canStarted) {
    return true;
  }

  twai_general_config_t gcfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t tcfg = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t fcfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&gcfg, &tcfg, &fcfg) != ESP_OK) {
    Serial.println("[CAN] Failed to install TWAI driver");
    return false;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("[CAN] Failed to start TWAI driver");
    return false;
  }

  canStarted = true;
  Serial.println("[CAN] OBD-II CAN initialized");
  return true;
}

void obdCanRequestPid(uint8_t pid) {
  if (!canStarted) {
    return;
  }

  twai_message_t frame = {};
  frame.identifier = CAN_REQUEST_ID;
  frame.extended_id = false;
  frame.data_length_code = 8;
  frame.data[0] = 0x02;
  frame.data[1] = 0x01;
  frame.data[2] = pid;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;
  frame.data[5] = 0x00;
  frame.data[6] = 0x00;
  frame.data[7] = 0x00;

  if (twai_transmit(&frame, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.println("[CAN] Request transmit failed");
  }
}

void obdCanPoll(VehicleData &data) {
  data.valid = false;

  twai_message_t message = {};
  if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
    if (message.identifier == CAN_REPLY_ID && message.data_length_code >= 8) {
      const uint8_t mode = message.data[0];
      const uint8_t pid = message.data[1];

      if (mode == 0x41) {
        switch (pid) {
          case 0x0C: {
            uint16_t rpmRaw = ((uint16_t)message.data[3] << 8) | message.data[4];
            data.rpm = (int)(rpmRaw / 4.0f);
            data.valid = true;
            break;
          }
          case 0x0D: {
            data.speedKph = message.data[3];
            data.valid = true;
            break;
          }
          case 0x05: {
            data.coolantTempC = (int)message.data[3] - 40;
            data.valid = true;
            break;
          }
          case 0x0B: {
            data.mapKpa = message.data[3] * 256 + message.data[4];
            data.valid = true;
            break;
          }
          case 0x0F: {
            data.intakeTempC = (int)message.data[3] - 40;
            data.valid = true;
            break;
          }
          default:
            break;
        }
      }
    }
  }
}

void obdCanDemoValues(VehicleData &data) {
  static uint32_t lastDemoMs = 0;
  const uint32_t now = millis();

  if (now - lastDemoMs > 100) {
    lastDemoMs = now;
    const float phase = now / 1000.0f;
    data.rpm = 1400 + (int)(2600.0f * (0.5f + 0.5f * sin(phase * 1.8f)));
    data.speedKph = 40 + (int)(50.0f * (0.5f + 0.5f * sin(phase * 0.9f)));
    data.coolantTempC = 82 + (int)(10.0f * (0.5f + 0.5f * sin(phase * 0.4f)));
    data.mapKpa = 100 + (int)(55.0f * (0.5f + 0.5f * sin(phase * 0.7f)));
    data.intakeTempC = 24 + (int)(10.0f * (0.5f + 0.5f * sin(phase * 0.5f)));
    data.valid = true;
  }
}

#include <Arduino.h>

#include <lvgl.h>
#include <TFT_eSPI.h>

#include "obd_can.h"

#define LV_TICK_PERIOD_MS 5
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 272

static TFT_eSPI tft = TFT_eSPI();

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 40];

static void display_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
  uint32_t width = lv_area_get_width(area);
  uint32_t height = lv_area_get_height(area);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, width, height);

  uint16_t *buffer = (uint16_t *)px_map;
  for (uint32_t y = 0; y < height; y++) {
    tft.pushPixels(&buffer[y * width], width);
  }

  tft.endWrite();
  lv_display_flush_ready(display);
}

static void setup_lvgl() {
  lv_init();

  lv_display_t *display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(display, display_flush_cb);
  lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_display_set_rotation(display, LV_DISPLAY_ROT_90);

  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x02070a), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

  lv_obj_t *title = lv_label_create(screen);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0xF8F8F8), 0);
  lv_label_set_text(title, "MEGANE RS");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

  lv_obj_t *rpmArc = lv_arc_create(screen);
  lv_obj_set_size(rpmArc, 340, 340);
  lv_obj_center(rpmArc);
  lv_arc_set_range(rpmArc, 0, 8000);
  lv_arc_set_value(rpmArc, 2500);
  lv_arc_set_bg_angles(rpmArc, 135, 45);
  lv_arc_set_rotation(rpmArc, 180);
  lv_obj_set_style_arc_color(rpmArc, lv_color_hex(0x2f7d32), LV_PART_MAIN);
  lv_obj_set_style_arc_width(rpmArc, 14, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rpmArc, lv_color_hex(0x1b1b1b), LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(rpmArc, LV_OPA_40, LV_PART_INDICATOR);

  lv_obj_t *rpmValue = lv_label_create(screen);
  lv_obj_set_style_text_font(rpmValue, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(rpmValue, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text_fmt(rpmValue, "%d", 2500);
  lv_obj_align(rpmValue, LV_ALIGN_CENTER, 0, -8);

  lv_obj_t *rpmUnit = lv_label_create(screen);
  lv_obj_set_style_text_font(rpmUnit, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(rpmUnit, lv_color_hex(0xBBBBBB), 0);
  lv_label_set_text(rpmUnit, "RPM");
  lv_obj_align_to(rpmUnit, rpmValue, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

  lv_obj_t *speedBox = lv_obj_create(screen);
  lv_obj_set_size(speedBox, 190, 104);
  lv_obj_align(speedBox, LV_ALIGN_CENTER, 0, 88);
  lv_obj_set_style_bg_color(speedBox, lv_color_hex(0x071922), 0);
  lv_obj_set_style_border_color(speedBox, lv_color_hex(0x26d7ff), 0);
  lv_obj_set_style_border_width(speedBox, 2, 0);
  lv_obj_set_style_radius(speedBox, 8, 0);

  lv_obj_t *speedLabel = lv_label_create(speedBox);
  lv_obj_set_style_text_font(speedLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(speedLabel, lv_color_hex(0xE0F7FF), 0);
  lv_label_set_text(speedLabel, "VITESSE");
  lv_obj_align(speedLabel, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *speedValue = lv_label_create(speedBox);
  lv_obj_set_style_text_font(speedValue, &lv_font_montserrat_36, 0);
  lv_obj_set_style_text_color(speedValue, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text_fmt(speedValue, "%d", 82);
  lv_obj_align(speedValue, LV_ALIGN_CENTER, 0, 8);

  lv_obj_t *speedUnit = lv_label_create(speedBox);
  lv_obj_set_style_text_font(speedUnit, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(speedUnit, lv_color_hex(0xC7E7FF), 0);
  lv_label_set_text(speedUnit, "km/h");
  lv_obj_align(speedUnit, LV_ALIGN_BOTTOM_MID, 0, -10);

  lv_obj_t *status = lv_label_create(screen);
  lv_obj_set_style_text_font(status, &lv_font_montserrat_26, 0);
  lv_obj_set_style_text_color(status, lv_color_hex(0xA7FF7A), 0);
  lv_label_set_text(status, "OBD CONNECTE");
  lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -12);

  (void)rpmArc;
  (void)rpmValue;
  (void)rpmUnit;
  (void)speedBox;
  (void)speedLabel;
  (void)speedValue;
  (void)speedUnit;
  (void)status;
}

void update_dashboard(const VehicleData &data) {
  static lv_obj_t *rpmArc = nullptr;
  static lv_obj_t *rpmValue = nullptr;
  static lv_obj_t *speedValue = nullptr;
  static lv_obj_t *tempValue = nullptr;
  static lv_obj_t *boostValue = nullptr;

  if (rpmArc == nullptr) {
    lv_obj_t *screen = lv_screen_active();
    rpmArc = lv_obj_get_child(screen, 0);
    // The UI creation above is tiny; in a real project, this would use the object IDs directly.
    // This placeholder avoids a full UI object graph. The actual app should keep references.
    (void)screen;
  }

  if (rpmValue == nullptr) {
    lv_obj_t *screen = lv_screen_active();
    (void)screen;
  }

  (void)data;
  (void)rpmArc;
  (void)rpmValue;
  (void)speedValue;
  (void)tempValue;
  (void)boostValue;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  setup_lvgl();

  if (!obdCanInit()) {
    Serial.println("[APP] CAN init failed, demo mode active");
  }

  obdCanRequestPid(0x0C);
  obdCanRequestPid(0x0D);
  obdCanRequestPid(0x05);
  obdCanRequestPid(0x0B);
}

void loop() {
  static uint32_t lastRequestMs = 0;
  static uint32_t lastUiMs = 0;

  VehicleData data;
  obdCanPoll(data);
  if (!data.valid) {
    obdCanDemoValues(data);
  }

  if (millis() - lastRequestMs > 250) {
    lastRequestMs = millis();
    obdCanRequestPid(0x0C);
    obdCanRequestPid(0x0D);
    obdCanRequestPid(0x05);
    obdCanRequestPid(0x0B);
  }

  if (millis() - lastUiMs > 50) {
    lastUiMs = millis();
    update_dashboard(data);
  }

  lv_timer_handler();
  delay(LV_TICK_PERIOD_MS);
}

#include <Arduino.h>

#include <lvgl.h>
#include <TFT_eSPI.h>

#include "obd_can.h"

#define LV_TICK_PERIOD_MS 5
#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 272

static TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 40];

static void display_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
  uint32_t width = lv_area_get_width(area);
  uint32_t height = lv_area_get_height(area);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, width, height);

  uint16_t *buffer = (uint16_t *)px_map;
  for (uint32_t y = 0; y < height; y++) {
    tft.pushPixels(&buffer[y * width], width);
  }

  tft.endWrite();
  lv_display_flush_ready(display);
}

static void setup_lvgl() {
  lv_init();

  lv_display_t *display = lv_display_create(SCREEN_WIDTH, SCREEN_HEIGHT);
  lv_display_set_flush_cb(display, display_flush_cb);
  lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_rotation(display, LV_DISPLAY_ROT_90);

  lv_obj_t *screen = lv_screen_active();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x02070a), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

  lv_obj_t *title = lv_label_create(screen);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title, lv_color_hex(0xF8F8F8), 0);
  lv_label_set_text(title, "MEGANE RS");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

  lv_obj_t *rpmArc = lv_arc_create(screen);
  lv_obj_set_size(rpmArc, 340, 340);
  lv_obj_center(rpmArc);
  lv_arc_set_range(rpmArc, 0, 8000);
  lv_arc_set_value(rpmArc, 2500);
  lv_arc_set_bg_angles(rpmArc, 135, 45);
  lv_arc_set_rotation(rpmArc, 180);
  lv_obj_set_style_arc_color(rpmArc, lv_color_hex(0x2f7d32), LV_PART_MAIN);
  lv_obj_set_style_arc_width(rpmArc, 14, LV_PART_MAIN);
  lv_obj_set_style_arc_color(rpmArc, lv_color_hex(0x1b1b1b), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(rpmArc, 14, LV_PART_INDICATOR);

  lv_obj_t *rpmValue = lv_label_create(screen);
  lv_obj_set_style_text_font(rpmValue, &lv_font_montserrat_40, 0);
  lv_obj_set_style_text_color(rpmValue, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text_fmt(rpmValue, "%d", 2500);
  lv_obj_align(rpmValue, LV_ALIGN_CENTER, 0, -8);

  lv_obj_t *rpmUnit = lv_label_create(screen);
  lv_obj_set_style_text_font(rpmUnit, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(rpmUnit, lv_color_hex(0xBBBBBB), 0);
  lv_label_set_text(rpmUnit, "RPM");
  lv_obj_align_to(rpmUnit, rpmValue, LV_ALIGN_OUT_BOTTOM_MID, 0, 8);

  lv_obj_t *speedBox = lv_obj_create(screen);
  lv_obj_set_size(speedBox, 190, 104);
  lv_obj_align(speedBox, LV_ALIGN_CENTER, 0, 88);
  lv_obj_set_style_bg_color(speedBox, lv_color_hex(0x071922), 0);
  lv_obj_set_style_border_color(speedBox, lv_color_hex(0x26d7ff), 0);
  lv_obj_set_style_border_width(speedBox, 2, 0);
  lv_obj_set_style_radius(speedBox, 8, 0);

  lv_obj_t *speedLabel = lv_label_create(speedBox);
  lv_obj_set_style_text_font(speedLabel, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(speedLabel, lv_color_hex(0xE0F7FF), 0);
  lv_label_set_text(speedLabel, "VITESSE");
  lv_obj_align(speedLabel, LV_ALIGN_TOP_MID, 0, 10);

  lv_obj_t *speedValue = lv_label_create(speedBox);
  lv_obj_set_style_text_font(speedValue, &lv_font_montserrat_36, 0);
  lv_obj_set_style_text_color(speedValue, lv_color_hex(0xFFFFFF), 0);
  lv_label_set_text_fmt(speedValue, "%d", 82);
  lv_obj_align(speedValue, LV_ALIGN_CENTER, 0, 8);

  lv_obj_t *speedUnit = lv_label_create(speedBox);
  lv_obj_set_style_text_font(speedUnit, &lv_font_montserrat_18, 0);
  lv_obj_set_style_text_color(speedUnit, lv_color_hex(0xC7E7FF), 0);
  lv_label_set_text(speedUnit, "km/h");
  lv_obj_align(speedUnit, LV_ALIGN_BOTTOM_MID, 0, -10);

  lv_obj_t *status = lv_label_create(screen);
  lv_obj_set_style_text_font(status, &lv_font_montserrat_26, 0);
  lv_obj_set_style_text_color(status, lv_color_hex(0xA7FF7A), 0);
  lv_label_set_text(status, "OBD CONNECTE");
  lv_obj_align(status, LV_ALIGN_BOTTOM_MID, 0, -12);

  (void)rpmArc;
  (void)rpmValue;
  (void)rpmUnit;
  (void)speedBox;
  (void)speedLabel;
  (void)speedValue;
  (void)speedUnit;
  (void)status;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  setup_lvgl();

  if (!obdCanInit()) {
    Serial.println("[APP] CAN init failed — demo mode enabled");
  }
}

void loop() {
  static uint32_t lastPidRequest = 0;
  VehicleData data;

  if (millis() - lastPidRequest > 250) {
    lastPidRequest = millis();
    obdCanRequestPid(0x0C);
    obdCanRequestPid(0x0D);
    obdCanRequestPid(0x05);
    obdCanRequestPid(0x0B);
  }

  obdCanPoll(data);
  if (!data.valid) {
    obdCanDemoValues(data);
  }

  (void)data;

  lv_timer_handler();
  delay(LV_TICK_PERIOD_MS);
}
