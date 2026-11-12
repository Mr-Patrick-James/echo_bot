// Edge Impulse FOMO Object Detection for TrashCam
// Include this ONLY in CameraWebServer.ino — NOT in app_httpd.cpp
// Classes: bio, non_bio, recyclable

#pragma once

#include <object_detection_inferencing.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <HTTPClient.h>

// ── Main bot IP — set this to your echo_bot's static IP ──
#define BOT_IP "10.60.114.42"

// ── Cooldown: minimum ms between HTTP sends to the bot ───
#define TRACK_COOLDOWN_MS 800

// ── Confidence threshold for triggering the bot ──────────
#define TRACK_MIN_CONFIDENCE 0.55f

// ── BB width fraction of frame that means "close enough" ─
// 0.35 = trash occupies ~35% of frame width → close enough to sort
#define TRACK_CLOSE_THRESHOLD 0.35f

// ── Single best detection result ─────────────────────────
#define EI_MAX_BOXES 1

struct TrashBox {
  char  label[32];
  float confidence;
  float x, y, w, h;  // normalised 0-1
};

struct TrashResult {
  char     label[32];   // best label (kept for backward compat)
  float    confidence;
  bool     valid;
  float    bbox_x, bbox_y, bbox_w, bbox_h;
  TrashBox boxes[EI_MAX_BOXES];
  int      box_count;
};

static TrashResult       last_result;
static SemaphoreHandle_t ei_mutex = NULL;

// ── Color per label ───────────────────────────────────────
const char* ei_label_color(const char* lbl) {
  if (strcmp(lbl, "bio") == 0)        return "#4ade80";
  if (strcmp(lbl, "non_bio") == 0)    return "#ef4444";
  if (strcmp(lbl, "recyclable") == 0) return "#3b82f6";
  return "#888888";
}
static const char* label_color(const char* lbl) { return ei_label_color(lbl); }

// ── Buffers ───────────────────────────────────────────────
static uint8_t *ei_resize_buf = NULL;
static uint8_t *ei_rgb_buf    = NULL;
static int      ei_src_w      = 0;
static int      ei_src_h      = 0;

// ── Feature callback ─────────────────────────────────────
static int ei_get_feature(size_t offset, size_t length, float *out) {
  for (size_t i = offset; i < offset + length; i++) {
    int idx = i * 3;
    out[i - offset] = (float)((ei_resize_buf[idx]   << 16)
                             | (ei_resize_buf[idx+1] << 8)
                             |  ei_resize_buf[idx+2]);
  }
  return 0;
}

// ── Resize + grayscale ────────────────────────────────────
static bool prepare_model_input(uint8_t *src, int sw, int sh) {
  const int DW = EI_CLASSIFIER_INPUT_WIDTH;
  const int DH = EI_CLASSIFIER_INPUT_HEIGHT;
  ei_resize_buf = (uint8_t*)heap_caps_malloc(DW * DH * 3,
                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!ei_resize_buf) ei_resize_buf = (uint8_t*)malloc(DW * DH * 3);
  if (!ei_resize_buf) return false;
  for (int y = 0; y < DH; y++) {
    for (int x = 0; x < DW; x++) {
      int sx = x * sw / DW;
      int sy = y * sh / DH;
      int si = (sy * sw + sx) * 3;
      uint8_t g = (uint8_t)(0.299f*src[si] + 0.587f*src[si+1] + 0.114f*src[si+2]);
      int di = (y * DW + x) * 3;
      ei_resize_buf[di] = ei_resize_buf[di+1] = ei_resize_buf[di+2] = g;
    }
  }
  return true;
}

// ── HTTP sender to main bot ───────────────────────────────
static unsigned long last_track_send_ms  = 0;
static char          last_track_label[32] = "";

static void send_track_to_bot(const char* label, float bbX, float bbW, float bbH) {
  unsigned long now = millis();

  // Cooldown check
  bool same_label = (strcmp(label, last_track_label) == 0);
  bool too_soon   = (now - last_track_send_ms < TRACK_COOLDOWN_MS);
  if (same_label && too_soon) return;

  // steer: -100 (far left) to +100 (far right), 0 = centered
  float bbCenterX = bbX + (bbW / 2.0f);
  int steer = (int)((bbCenterX - 0.5f) * 200.0f);  // -100 to 100
  steer = steer < -100 ? -100 : (steer > 100 ? 100 : steer);

  // close: bounding box width > threshold of frame width
  int isClose = (bbW >= TRACK_CLOSE_THRESHOLD) ? 1 : 0;

  // map label to bot endpoint label (non_bio → nonbio)
  char botLabel[32];
  if (strcmp(label, "non_bio") == 0)        strncpy(botLabel, "nonbio",    sizeof(botLabel)-1);
  else if (strcmp(label, "recyclable") == 0) strncpy(botLabel, "recycle",   sizeof(botLabel)-1);
  else                                        strncpy(botLabel, label,       sizeof(botLabel)-1);

  String url = String("http://") + BOT_IP + "/trackTrash"
             + "?label=" + String(botLabel)
             + "&steer=" + String(steer)
             + "&close=" + String(isClose);

  HTTPClient http;
  http.begin(url);
  http.setTimeout(300);
  int code = http.GET();
  http.end();

  Serial.printf("[CAM→BOT] label=%s steer=%d close=%d -> HTTP %d\n",
                botLabel, steer, isClose, code);

  strncpy(last_track_label, label, sizeof(last_track_label)-1);
  last_track_send_ms = now;
}

static void send_track_lost() {
  // Only send lost if we previously had a valid track
  if (last_track_label[0] == '\0') return;
  unsigned long now = millis();
  if (now - last_track_send_ms < TRACK_COOLDOWN_MS) return;

  HTTPClient http;
  http.begin(String("http://") + BOT_IP + "/trackLost");
  http.setTimeout(300);
  http.GET();
  http.end();

  last_track_label[0] = '\0';
  last_track_send_ms  = now;
  Serial.println("[CAM→BOT] trackLost sent");
}

// ── Inference task ────────────────────────────────────────
static void ei_inference_task(void *arg) {
  Serial.printf("EI: task on core %d heap=%d psram=%d\n",
                xPortGetCoreID(), ESP.getFreeHeap(), ESP.getFreePsram());
  // Initialise last_result
  memset(&last_result, 0, sizeof(last_result));
  strncpy(last_result.label, "waiting...", sizeof(last_result.label)-1);

  vTaskDelay(2000 / portTICK_PERIOD_MS); // short 2s startup wait

  for (;;) {
  vTaskDelay(800 / portTICK_PERIOD_MS);

    // 1. Grab frame
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) continue;

    ei_src_w = (int)fb->width;
    ei_src_h = (int)fb->height;
    size_t rgb_len = ei_src_w * ei_src_h * 3;

    // 2. Decode JPEG → RGB888
    ei_rgb_buf = (uint8_t*)heap_caps_malloc(rgb_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ei_rgb_buf) { esp_camera_fb_return(fb); continue; }

    bool ok = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, ei_rgb_buf);
    esp_camera_fb_return(fb);
    if (!ok) { free(ei_rgb_buf); ei_rgb_buf = NULL; continue; }

    // 3. Resize → 96x96 grayscale
    bool prep = prepare_model_input(ei_rgb_buf, ei_src_w, ei_src_h);
    free(ei_rgb_buf); ei_rgb_buf = NULL;
    if (!prep) continue;

    // 4. Run classifier
    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data     = &ei_get_feature;

    ei_impulse_result_t result = {0};
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);
    free(ei_resize_buf); ei_resize_buf = NULL;
    if (err != EI_IMPULSE_OK) continue;

    // 5. Find single best box above threshold
    TrashResult next;
    memset(&next, 0, sizeof(next));
    next.box_count  = 0;
    next.valid      = false;
    next.confidence = 0.0f;

    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
      auto bb = result.bounding_boxes[i];
      if (bb.value < 0.3f) continue;

      // Keep only the highest confidence detection
      if (bb.value > next.confidence) {
        next.confidence = bb.value;
        strncpy(next.label, bb.label, sizeof(next.label)-1);
        next.bbox_x = (float)bb.x / EI_CLASSIFIER_INPUT_WIDTH;
        next.bbox_y = (float)bb.y / EI_CLASSIFIER_INPUT_HEIGHT;
        next.bbox_w = (float)(bb.width  > 0 ? bb.width  : 8) / EI_CLASSIFIER_INPUT_WIDTH;
        next.bbox_h = (float)(bb.height > 0 ? bb.height : 8) / EI_CLASSIFIER_INPUT_HEIGHT;
        next.valid  = true;

        // Store as single box for web UI overlay
        next.box_count = 1;
        strncpy(next.boxes[0].label, bb.label, sizeof(next.boxes[0].label)-1);
        next.boxes[0].confidence = bb.value;
        next.boxes[0].x = next.bbox_x;
        next.boxes[0].y = next.bbox_y;
        next.boxes[0].w = next.bbox_w;
        next.boxes[0].h = next.bbox_h;
      }
    }

    next.valid = (next.box_count > 0);
    if (!next.valid) {
      strncpy(next.label, "uncertain", sizeof(next.label)-1);
      next.bbox_x = 0.10f; next.bbox_y = 0.10f;
      next.bbox_w = 0.80f; next.bbox_h = 0.80f;
    }

    Serial.printf("EI: %d boxes detected\n", next.box_count);

    // ── Notify main bot ──────────────────────────────────
    if (next.valid && next.confidence >= TRACK_MIN_CONFIDENCE) {
      send_track_to_bot(next.label, next.bbox_x, next.bbox_w, next.bbox_h);
    } else {
      send_track_lost();
    }

    // 6. Store result
    xSemaphoreTake(ei_mutex, portMAX_DELAY);
    last_result = next;
    xSemaphoreGive(ei_mutex);
  }
}

// ── Start task ────────────────────────────────────────────
static void ei_start_inference_task() {
  ei_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    ei_inference_task, "ei_inf",
    24 * 1024, NULL, 1, NULL, 0
  );
  Serial.println("EI: inference task created");
}

// ── Safe read ─────────────────────────────────────────────
TrashResult ei_get_result_shared() {
  TrashResult r;
  memset(&r, 0, sizeof(r));
  strncpy(r.label, "loading...", sizeof(r.label)-1);
  if (!ei_mutex) return r;
  xSemaphoreTake(ei_mutex, portMAX_DELAY);
  r = last_result;
  xSemaphoreGive(ei_mutex);
  return r;
}
static TrashResult ei_get_result() { return ei_get_result_shared(); }
