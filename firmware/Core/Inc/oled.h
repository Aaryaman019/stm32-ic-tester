#ifndef OLED_H
#define OLED_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

/* 128x64 I2C OLED, SH1106 (1.3") or SSD1306 (0.96"), 5x7 font, page-mode framebuffer.
 * oled_flush() only sends the 128-byte pages that changed since the last flush. */
bool oled_begin(I2C_HandleTypeDef *hi2c, uint8_t addr7, bool sh1106);
void oled_clear(void);
void oled_text(int16_t x, int16_t y, const char *s, uint8_t size);
void oled_rect(int16_t x, int16_t y, int16_t w, int16_t h, bool fill);
void oled_hline(int16_t x, int16_t y, int16_t w);
bool oled_flush(void);          /* false if any page write failed */
void oled_invalidate(void);   /* force full redraw on next flush */
bool oled_reinit(void);       /* after I2C recovery: re-init panel, keep framebuffer */
void oled_touch_page(uint8_t page);   /* force one page to be re-sent on next flush (self-heal) */

#endif /* OLED_H */
