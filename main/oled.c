/* Optional SSD1306 status OLED, see oled.h. */
#include "oled.h"
#include <stdio.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "experimental.h"
#include "oled_font.h"
#include "receiver.h"
#include "rf.h"

#define OLED_SDA_GPIO     7      /* XIAO D3 */
#define OLED_SCL_GPIO     10     /* XIAO D10 */
#define OLED_ADDR         0x3C
#define OLED_I2C_HZ       400000
#define OLED_COL_OFFSET   0      /* 2 for SH1106 (1.3" modules) */
#define OLED_W            128
#define OLED_LINES        4      /* 16 px each: two pages */
#define OLED_CHARS        (OLED_W / OLED_FONT_W)   /* 18 */
#define OLED_PROBE_US     5000000
#define OLED_REFRESH_US   500000
#define OLED_TIMEOUT_MS   20

extern volatile const char *s_boot_stage;   /* main.c */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static bool s_attached;
static int64_t s_next_probe_us, s_next_draw_us;
static char s_shown[OLED_LINES][OLED_CHARS + 1];
static uint32_t s_frames, s_errors, s_attaches;
static uint32_t s_pic_found, s_pic_expected;   /* last refresh, for the sync % */

static bool cmds(const uint8_t *c, size_t n)
{
    uint8_t buf[1 + 8];
    if (n > sizeof(buf) - 1) return false;
    buf[0] = 0x00;   /* Co = 0, D/C = 0: command stream */
    memcpy(buf + 1, c, n);
    return i2c_master_transmit(s_dev, buf, n + 1, OLED_TIMEOUT_MS) == ESP_OK;
}

/* One text line: two pages, each [0x40, 128 columns]. */
static bool draw_line(unsigned line, const char *text)
{
    uint8_t buf[1 + OLED_W];
    for (unsigned half = 0; half < 2u; ++half) {
        const unsigned page = line * 2u + half;
        const uint8_t addr[3] = {(uint8_t)(0xB0 | page), (uint8_t)(OLED_COL_OFFSET & 0x0F),
                                 (uint8_t)(0x10 | (OLED_COL_OFFSET >> 4))};
        if (!cmds(addr, sizeof(addr))) return false;
        buf[0] = 0x40;   /* data stream */
        memset(buf + 1, 0, OLED_W);
        bool end = false;
        for (unsigned k = 0; k < OLED_CHARS; ++k) {
            char c = end ? ' ' : text[k];
            if (!c) { end = true; c = ' '; }
            if (c < OLED_FONT_FIRST || c > OLED_FONT_LAST) c = '?';
            const uint8_t *g = s_oled_font[c - OLED_FONT_FIRST];
            for (unsigned x = 0; x < OLED_FONT_W; ++x) buf[1 + k * OLED_FONT_W + x] = g[x * 2u + half];
        }
        if (i2c_master_transmit(s_dev, buf, sizeof(buf), OLED_TIMEOUT_MS) != ESP_OK) return false;
    }
    return true;
}

static bool display_init(void)
{
    static const uint8_t init[][3] = {
        {0xAE},             /* display off */
        {0xD5, 0x80},       /* clock divide */
        {0xA8, 0x3F},       /* multiplex 64 */
        {0xD3, 0x00},       /* display offset */
        {0x40},             /* start line 0 */
        {0x8D, 0x14},       /* charge pump on */
        {0x20, 0x02},       /* page addressing */
        {0xA0},             /* segment column 0 = SEG0 (rotated 180 deg) */
        {0xC0},             /* COM scan ascending (rotated 180 deg) */
        {0xDA, 0x12},       /* COM pins */
        {0x81, 0xCF},       /* contrast */
        {0xD9, 0xF1},       /* precharge */
        {0xDB, 0x40},       /* VCOMH */
        {0xA4},             /* follow RAM */
        {0xA6},             /* normal (not inverted) */
    };
    static const uint8_t len[] = {1, 2, 2, 2, 1, 2, 2, 1, 1, 2, 2, 2, 2, 1, 1};
    for (unsigned k = 0; k < sizeof(len); ++k)
        if (!cmds(init[k], len[k])) return false;
    for (unsigned l = 0; l < OLED_LINES; ++l)
        if (!draw_line(l, "")) return false;
    static const uint8_t on = 0xAF;
    return cmds(&on, 1);
}

static void detach(void)
{
    if (s_dev) { (void)i2c_master_bus_rm_device(s_dev); s_dev = NULL; }
    if (s_bus) { (void)i2c_del_master_bus(s_bus); s_bus = NULL; }
    s_attached = false;
}

/* The bus exists only while a display answers: a board without one keeps
 * the memory. */
static void attach(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 1,
        .trans_queue_depth = 0,   /* synchronous: no queue memory */
        .flags.enable_internal_pullup = true,   /* modules carry their own as well */
    };
    if (i2c_new_master_bus(&bus_cfg, &s_bus) != ESP_OK) { s_bus = NULL; return; }
    if (i2c_master_probe(s_bus, OLED_ADDR, OLED_TIMEOUT_MS) != ESP_OK) { detach(); return; }
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = OLED_I2C_HZ,
    };
    if (i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev) != ESP_OK) { s_dev = NULL; detach(); return; }
    for (unsigned l = 0; l < OLED_LINES; ++l) { s_shown[l][0] = '\x01'; s_shown[l][1] = '\0'; }   /* redraw all */
    if (!display_init()) { ++s_errors; detach(); return; }
    s_attached = true;
    ++s_attaches;
    printf("OLED attached addr=0x%02X sda=%d scl=%d\n", OLED_ADDR, OLED_SDA_GPIO, OLED_SCL_GPIO);
}

#define OLED_BUF 32   /* compose room; lines are cut to OLED_CHARS */
static void compose(char out[OLED_LINES][OLED_BUF])
{
    const fpv_channel_t *ch = rf_channel();
    const bool video = s_last_video_us && esp_timer_get_time() - s_last_video_us < 500000;
    snprintf(out[0], OLED_BUF, "%-3s %4u %s", ch->name, (unsigned)rf_frequency_mhz(),
             !video ? "NO VIDEO" : !s_detected_std_valid ? "VIDEO" : s_detected_std == VIDEO_STD_PAL ? "PAL" : "NTSC");

    const rx_level_t lv = receiver_level();
    if (lv.valid)
        snprintf(out[1], OLED_BUF, "%4ddBm CN%3d.%d", (lv.carrier_dbm_x10 - 5) / 10,
                 lv.cn_x10 / 10, (lv.cn_x10 < 0 ? -lv.cn_x10 : lv.cn_x10) % 10);
    else
        snprintf(out[1], OLED_BUF, "RF --  clip %u", (unsigned)lv.clipped_pm);

    /* Sync pulses found over the expected ones since the last refresh: the
     * share of lines the decoder saw, what a monitor blanks on. */
    uint32_t found = 0, expected = 0;
    control_picture_counts(&found, &expected);
    const uint32_t df = found - s_pic_found, de = expected - s_pic_expected;
    s_pic_found = found; s_pic_expected = expected;
    const unsigned sync_pct = de ? (unsigned)((df > de ? de : df) * 100u / de) : 0u;
    snprintf(out[2], OLED_BUF, "%s G%-2u SYNC %3u%%", s_afc_locked ? "LCK" : "---", (unsigned)s_current_gain,
             sync_pct);

    if (s_menu_active)
        snprintf(out[3], OLED_BUF, "MENU");
    else if (s_scan_active)
        snprintf(out[3], OLED_BUF, "SCAN %3u%%", s_scan_progress);
    else {
        vq_t vq;
        control_video_quality(&vq);
        const unsigned up = (unsigned)(esp_timer_get_time() / 1000000);
        if (vq.known)
            snprintf(out[3], OLED_BUF, "Q%3d.%d  %3u:%02u", vq.snr_x10 / 10,
                     (vq.snr_x10 < 0 ? -vq.snr_x10 : vq.snr_x10) % 10, up / 60u, up % 60u);
        else
            snprintf(out[3], OLED_BUF, "Q --    %3u:%02u", up / 60u, up % 60u);
    }
}

void oled_poll(void)
{
    const int64_t now = esp_timer_get_time();
    if (!s_attached) {
        /* After boot only (outside the splash + task heap peak), never with
         * the D10 sample clock (D3 and D10 are then DAC bit 2 and the clock). */
        if (now < s_next_probe_us || strcmp((const char *)s_boot_stage, "ready") != 0 || experimental_clock_d10())
            return;
        s_next_probe_us = now + OLED_PROBE_US;
        esp_log_level_set("i2c.master", ESP_LOG_NONE);   /* no NACK spam without a display */
        attach();
        if (!s_attached) return;
    }
    if (now < s_next_draw_us) return;
    s_next_draw_us = now + OLED_REFRESH_US;
    char text[OLED_LINES][OLED_BUF];
    compose(text);
    for (unsigned l = 0; l < OLED_LINES; ++l) text[l][OLED_CHARS] = '\0';
    for (unsigned l = 0; l < OLED_LINES; ++l) {
        if (!strcmp(text[l], s_shown[l])) continue;
        if (!draw_line(l, text[l])) {
            ++s_errors;
            printf("OLED lost (I2C error); probing again in 5 s\n");
            detach();
            s_next_probe_us = now + OLED_PROBE_US;
            return;
        }
        memcpy(s_shown[l], text[l], OLED_CHARS + 1);
    }
    ++s_frames;
}

void oled_print(void)
{
    printf("OLED attached=%u addr=0x%02X sda=%d scl=%d attaches=%lu frames=%lu errors=%lu disabled_by_d10_clock=%u\n",
           (unsigned)s_attached, OLED_ADDR, OLED_SDA_GPIO, OLED_SCL_GPIO, (unsigned long)s_attaches,
           (unsigned long)s_frames, (unsigned long)s_errors, (unsigned)experimental_clock_d10());
}
