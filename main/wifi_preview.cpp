// Podglad z kamery + ramki FOMO przez WiFi: wpisz w przegladarce IP wyswietlone w logu.
// UWAGA: ten plik NIE moze dolaczac ei_run_classifier.h (SDK EI definiuje funkcje w naglowkach,
// dolaczenie w dwoch plikach .cpp konczy sie "multiple definition" przy linkowaniu).
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "wifi_preview.h"

#include "wifi_secrets.h"   // WIFI_SSID, WIFI_PASS (plik w .gitignore)

static const char *TAG = "wifi_prev";
static EventGroupHandle_t s_wifi_events;
#define GOT_IP_BIT BIT0

static SemaphoreHandle_t s_mutex;
static int s_w = 0, s_h = 0;
static uint8_t *s_frame = nullptr;
static uint8_t *s_copy = nullptr;
static char s_json[1024] = "[]";

static const char PAGE[] =
"<!doctype html><html><head><meta charset=utf-8><title>ESP32-CAM FOMO</title>"
"<style>body{background:#111;color:#ddd;font:14px monospace;text-align:center}"
"canvas{image-rendering:pixelated;margin-top:10px}</style></head><body>"
"<canvas id=c></canvas><div id=i></div>"
"<script>"
"const W=%d,H=%d,S=%d,c=document.getElementById('c'),x=c.getContext('2d');"
"c.width=W*S;c.height=H*S;x.imageSmoothingEnabled=false;"
"const o=document.createElement('canvas');o.width=W;o.height=H;const ox=o.getContext('2d');"
"let t=performance.now();"
"async function loop(){try{"
"const [f,d]=await Promise.all([fetch('/frame').then(r=>r.arrayBuffer()),fetch('/det').then(r=>r.json())]);"
"const g=new Uint8Array(f),im=ox.createImageData(W,H);"
"for(let i=0;i<W*H;i++){im.data[4*i]=im.data[4*i+1]=im.data[4*i+2]=g[i];im.data[4*i+3]=255;}"
"ox.putImageData(im,0,0);x.drawImage(o,0,0,W*S,H*S);"
"x.strokeStyle='#0f0';x.fillStyle='#0f0';x.lineWidth=2;x.font='12px monospace';"
"for(const b of d){x.strokeRect(b.x*S,b.y*S,b.w*S,b.h*S);x.fillText(b.l+' '+b.v.toFixed(2),b.x*S,b.y*S-3);}"
"const n=performance.now();document.getElementById('i').textContent=(1000/(n-t)).toFixed(1)+' fps, det: '+d.length;t=n;"
"}catch(e){await new Promise(r=>setTimeout(r,500));}"
"requestAnimationFrame(loop);}loop();"
"</script></body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    size_t cap = sizeof(PAGE) + 32;
    char *html = (char *)malloc(cap);
    if (!html) return ESP_FAIL;
    int scale = 480 / s_w;
    if (scale < 1) scale = 1;
    int n = snprintf(html, cap, PAGE, s_w, s_h, scale);
    httpd_resp_set_type(req, "text/html");
    esp_err_t r = httpd_resp_send(req, html, n);
    free(html);
    return r;
}

static esp_err_t frame_handler(httpd_req_t *req)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(s_copy, s_frame, s_w * s_h);
    xSemaphoreGive(s_mutex);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)s_copy, s_w * s_h);
}

static esp_err_t det_handler(httpd_req_t *req)
{
    char copy[sizeof(s_json)];
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strcpy(copy, s_json);
    xSemaphoreGive(s_mutex);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, copy, HTTPD_RESP_USE_STRLEN);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t *)data;
        ESP_LOGW(TAG, "rozlaczono, powod=%d (201=nie znaleziono SSID, 202/15/204=zle haslo/handshake), ponawiam", d->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "==> otworz w przegladarce: http://" IPSTR "/", IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_events, GOT_IP_BIT);
    }
}

void wifi_preview_start(int width, int height)
{
    s_w = width;
    s_h = height;
    s_frame = (uint8_t *)calloc(width * height, 1);
    s_copy  = (uint8_t *)calloc(width * height, 1);
    s_mutex = xSemaphoreCreateMutex();
    s_wifi_events = xEventGroupCreate();
    if (!s_frame || !s_copy) {
        ESP_LOGE(TAG, "brak pamieci na bufor podgladu");
        return;
    }

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t icfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&icfg);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, nullptr);

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid, WIFI_SSID, sizeof(wcfg.sta.ssid) - 1);
    strncpy((char *)wcfg.sta.password, WIFI_PASS, sizeof(wcfg.sta.password) - 1);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);   // mniejsze opoznienia

    xEventGroupWaitBits(s_wifi_events, GOT_IP_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.core_id = 0;   // inferencja jest na core 1
    hcfg.stack_size = 4096;          // domyslnie 4096, ale jawnie, zeby nie rosnac
    hcfg.max_open_sockets = 3;       // domyslnie 7; kazde gniazdo = pamiec wewnetrzna
    hcfg.lru_purge_enable = true;    // zamykaj najstarsze polaczenie zamiast odrzucac nowe
    httpd_handle_t srv = nullptr;
    if (httpd_start(&srv, &hcfg) == ESP_OK) {
        httpd_uri_t u1 = {}; u1.uri = "/";      u1.method = HTTP_GET; u1.handler = index_handler;
        httpd_uri_t u2 = {}; u2.uri = "/frame"; u2.method = HTTP_GET; u2.handler = frame_handler;
        httpd_uri_t u3 = {}; u3.uri = "/det";   u3.method = HTTP_GET; u3.handler = det_handler;
        httpd_register_uri_handler(srv, &u1);
        httpd_register_uri_handler(srv, &u2);
        httpd_register_uri_handler(srv, &u3);
    }
}

void wifi_preview_update(const uint8_t *gray, const preview_box_t *boxes, int n_boxes)
{
    if (!s_mutex || !s_frame) return;
    char tmp[sizeof(s_json)];
    size_t n = 0;
    tmp[n++] = '[';
    for (int i = 0; i < n_boxes; i++) {
        const preview_box_t &b = boxes[i];
        int w = snprintf(tmp + n, sizeof(tmp) - n - 2,
                         "%s{\"l\":\"%s\",\"v\":%.2f,\"x\":%lu,\"y\":%lu,\"w\":%lu,\"h\":%lu}",
                         i ? "," : "", b.label, b.value,
                         (unsigned long)b.x, (unsigned long)b.y,
                         (unsigned long)b.w, (unsigned long)b.h);
        if (w < 0 || (size_t)w >= sizeof(tmp) - n - 2) break;
        n += w;
    }
    tmp[n++] = ']';
    tmp[n] = 0;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(s_frame, gray, s_w * s_h);
    strcpy(s_json, tmp);
    xSemaphoreGive(s_mutex);
}