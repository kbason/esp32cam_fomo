#pragma once
#include <stdint.h>

typedef struct {
    const char *label;
    float value;
    uint32_t x, y, w, h;
} preview_box_t;

// width/height = rozmiar obrazu modelu (EI_W x EI_H)
void wifi_preview_start(int width, int height);
// gray: width*height bajtow, boxes: ramki z FOMO (moze byc n_boxes = 0)
void wifi_preview_update(const uint8_t *gray, const preview_box_t *boxes, int n_boxes);
