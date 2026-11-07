/* Optional SSD1306 128x64 I2C OLED status display (address 0x3C).
 * SDA on D3/GPIO7, SCL on D10/GPIO10: both header pads are free unless
 * EXPERIMENTAL Clock Pin D10 is on, which disables the display.
 *
 * Memory: no frame buffer and no task of its own. Four text lines of 16 px
 * (two SSD1306 pages each) are rendered straight from the font into a
 * 129-byte stack buffer per page and sent by the console task, only when a
 * line changed. The I2C bus is created after boot (outside the boot heap
 * peak) and only when a display answers. */
#pragma once

/* Console task, every loop (~100 ms): attaches the display once the
 * receiver is up, probes again every 5 s while none answers, and redraws
 * the changed lines twice a second. */
void oled_poll(void);

/* OLED line for T. */
void oled_print(void);
