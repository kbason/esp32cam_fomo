#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_camera.h"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static const char *TAG = "fomo";

// Podglad przez WiFi (wifi_preview.cpp): wpisz w przegladarce IP z logu.
#include "wifi_preview.h"

// Kamera zawsze pracuje w 96x96 (najmniejszy rozmiar w sterowniku esp32-camera).
// Jesli model ma inne wejscie (np. 48x48), klatka jest skalowana programowo,
// wiec zmiana modelu = zmiana jednej nazwy w REQUIRES w main/CMakeLists.txt.
#define CAM_WIDTH   96
#define CAM_HEIGHT  96

#define CAM_PIN_PWDN   32
#define CAM_PIN_RESET  -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD   26
#define CAM_PIN_SIOC   27
#define CAM_PIN_D7     35
#define CAM_PIN_D6     34
#define CAM_PIN_D5     39
#define CAM_PIN_D4     36
#define CAM_PIN_D3     21
#define CAM_PIN_D2     19
#define CAM_PIN_D1     18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC  25
#define CAM_PIN_HREF   23
#define CAM_PIN_PCLK   22

#define EI_W  EI_CLASSIFIER_INPUT_WIDTH
#define EI_H  EI_CLASSIFIER_INPUT_HEIGHT

static esp_err_t camera_init(void)
{
    camera_config_t cfg = {};
    cfg.pin_pwdn = CAM_PIN_PWDN;
    cfg.pin_reset = CAM_PIN_RESET;
    cfg.pin_xclk = CAM_PIN_XCLK;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;
    cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_d7 = CAM_PIN_D7;
    cfg.pin_d6 = CAM_PIN_D6;
    cfg.pin_d5 = CAM_PIN_D5;
    cfg.pin_d4 = CAM_PIN_D4;
    cfg.pin_d3 = CAM_PIN_D3;
    cfg.pin_d2 = CAM_PIN_D2;
    cfg.pin_d1 = CAM_PIN_D1;
    cfg.pin_d0 = CAM_PIN_D0;
    cfg.pin_vsync = CAM_PIN_VSYNC;
    cfg.pin_href = CAM_PIN_HREF;
    cfg.pin_pclk = CAM_PIN_PCLK;

    // 10 MHz: przy 20 MHz DMA nie nadaza i klatki 96x96 sa obciete (FB-SIZE: 6144 != 9216)
    cfg.xclk_freq_hz = 10000000;
    cfg.ledc_timer = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;

    cfg.pixel_format = PIXFORMAT_GRAYSCALE;
    cfg.frame_size = FRAMESIZE_96X96;
    cfg.fb_count = 2;                       // kamera robi kolejna klatke, gdy liczymy poprzednia
    cfg.fb_location = CAMERA_FB_IN_DRAM;    // 2 x 9 kB, szybciej niz PSRAM
    cfg.grab_mode = CAMERA_GRAB_LATEST;     // zawsze najswiezsza klatka
    return esp_camera_init(&cfg);
}

// bufor wejsciowy dla EI (float, 0xRRGGBB na piksel; dla grayscale R=G=B)
// (w PSRAM: 96x96x4 B = 36 kB, szkoda na to wewnetrznego RAM-u, ktorego brakuje WiFi)
static float *ei_buf = nullptr;

// kopia surowych bajtow grayscale (po skalowaniu), do podgladu przez WiFi
static uint8_t gray_raw[EI_W * EI_H];

static inline void store_pixel(int idx, uint8_t g)
{
    gray_raw[idx] = g;
    ei_buf[idx] = (float)(((uint32_t)g << 16) | ((uint32_t)g << 8) | g);
}

// Skaluje klatke grayscale (sw x sh) do rozmiaru modelu (EI_W x EI_H)
// i zapisuje ja w ei_buf. Zmniejszanie: usrednianie blokow (area average).
// Powiekszanie: najblizszy sasiad (blok ma wtedy min. 1 piksel).
static void gray_to_ei(const uint8_t *src, int sw, int sh)
{
    if (sw == EI_W && sh == EI_H) {
        for (int i = 0; i < EI_W * EI_H; i++) {
            store_pixel(i, src[i]);
        }
        return;
    }

    for (int y = 0; y < EI_H; y++) {
        int y0 = y * sh / EI_H;
        int y1 = (y + 1) * sh / EI_H;
        if (y1 <= y0) y1 = y0 + 1;

        for (int x = 0; x < EI_W; x++) {
            int x0 = x * sw / EI_W;
            int x1 = (x + 1) * sw / EI_W;
            if (x1 <= x0) x1 = x0 + 1;

            uint32_t sum = 0;
            for (int yy = y0; yy < y1; yy++) {
                const uint8_t *row = src + yy * sw;
                for (int xx = x0; xx < x1; xx++) {
                    sum += row[xx];
                }
            }
            store_pixel(y * EI_W + x, (uint8_t)(sum / (uint32_t)((y1 - y0) * (x1 - x0))));
        }
    }
}

static int ei_get_data(size_t offset, size_t length, float *out_ptr)
{
    memcpy(out_ptr, ei_buf + offset, length * sizeof(float));
    return 0;
}

// Cala petla inferencji na core 1 (kamera i app_main siedza na core 0).
static void inference_task(void *arg)
{
    // punkt odniesienia: ile najmniej wolnej pamieci bylo do tej pory
    size_t int_min0 = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t ps_min0  = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    bool first = true;

    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "fb_get failed");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (fb->width != CAM_WIDTH || fb->height != CAM_HEIGHT) {
            ESP_LOGE(TAG, "nieoczekiwany rozmiar klatki: %dx%d (oczekiwano %dx%d)",
                     (int)fb->width, (int)fb->height, CAM_WIDTH, CAM_HEIGHT);
            esp_camera_fb_return(fb);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        gray_to_ei(fb->buf, fb->width, fb->height);
        esp_camera_fb_return(fb);

        signal_t signal;
        signal.total_length = EI_W * EI_H;
        signal.get_data = &ei_get_data;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        if (res != EI_IMPULSE_OK) {
            ESP_LOGE(TAG, "run_classifier failed: %d", res);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // klatka + ramki do przegladarki
        {
            preview_box_t pb[16];
            int npb = 0;
#if EI_CLASSIFIER_OBJECT_DETECTION
            for (size_t i = 0; i < result.bounding_boxes_count && npb < 16; i++) {
                const ei_impulse_result_bounding_box_t &bb = result.bounding_boxes[i];
                if (bb.value == 0) continue;
                pb[npb++] = { bb.label, bb.value, bb.x, bb.y, bb.width, bb.height };
            }
#endif
            wifi_preview_update(gray_raw, pb, npb);
        }

        if (first) {
            first = false;
            ESP_LOGI(TAG, "pamiec zajeta przez 1. inferencje: internal %u B, psram %u B",
                     (unsigned)(int_min0 - heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)),
                     (unsigned)(ps_min0  - heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM)));
        }

        ESP_LOGI(TAG, "inference: DSP %lld us, classification %lld us",
                 (long long)result.timing.dsp_us, (long long)result.timing.classification_us);

#if EI_CLASSIFIER_OBJECT_DETECTION
        // FOMO / detekcja obiektow
        if (result.bounding_boxes_count == 0) {
            ESP_LOGI(TAG, "brak detekcji");
        }
        for (size_t i = 0; i < result.bounding_boxes_count; i++) {
            ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
            if (bb.value == 0) continue;
            ESP_LOGI(TAG, "%s (%.2f) x=%lu y=%lu w=%lu h=%lu",
                     bb.label, bb.value,
                     (unsigned long)bb.x, (unsigned long)bb.y,
                     (unsigned long)bb.width, (unsigned long)bb.height);
        }
#else
        // zwykly klasyfikator
        for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
            ESP_LOGI(TAG, "%s: %.2f", result.classification[i].label, result.classification[i].value);
        }
#endif

        // 1 tick: oddaje czas zadaniu IDLE1, zeby nie wyzwolic watchdoga
        vTaskDelay(1);
    }
}

extern "C" void app_main(void)
{
    esp_err_t err = camera_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed: 0x%x", err);
        return;
    }
    ei_buf = (float *)heap_caps_malloc(EI_W * EI_H * sizeof(float), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ei_buf) {
        ESP_LOGE(TAG, "brak PSRAM na bufor wejsciowy");
        return;
    }
    ESP_LOGI(TAG, "camera ready: %dx%d -> model %dx%d", CAM_WIDTH, CAM_HEIGHT, EI_W, EI_H);
#ifdef EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE
    ESP_LOGI(TAG, "arena modelu: %d B", (int)EI_CLASSIFIER_TFLITE_LARGEST_ARENA_SIZE);
#endif

    // Najpierw WiFi + serwer HTTP (potrzebuja pamieci wewnetrznej), dopiero potem model.
    wifi_preview_start(EI_W, EI_H);   // IP pojawi sie w logu

    if (xTaskCreatePinnedToCore(inference_task, "inference", 16384, nullptr, 5, nullptr, 1) != pdPASS) {
        ESP_LOGE(TAG, "nie udalo sie utworzyc zadania inferencji");
    }
}