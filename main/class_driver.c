/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * ESP32-P4 ADS-B Receiver — ATC Terminal Edition
 */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "es8311.h"
#include "rtl-sdr.h"
#include "mode-s.h"
#include "esp_task_wdt.h"

/* ── build config ────────────────────────────────────────────────────────── */
#define CLIENT_NUM_EVENT_MSG  5
#define MAX_PACKET_SIZE       16384
#define DEFAULT_BUF_LENGTH    (MAX_PACKET_SIZE * 2)
#define MAX_TRACKED           16
#define LOG_LINES             7
#define TUI_REFRESH_MS        150

/* ── audio pins (Waveshare ESP32-P4-NANO + ES8311) ───────────────────────── */
#define I2C_SCL_PIN     8
#define I2C_SDA_PIN     7
#define I2S_MCLK_PIN    13
#define I2S_BCK_PIN     12
#define I2S_WS_PIN      10
#define I2S_DOUT_PIN    9
#define I2S_DIN_PIN     11
#define PA_EN_PIN       53
#define AUDIO_RATE_HZ   16000
#define MCLK_MULTIPLE   384

/* ── USB driver types ────────────────────────────────────────────────────── */
typedef enum {
    ACTION_OPEN_DEV        = (1 << 0),
    ACTION_GET_DEV_INFO    = (1 << 1),
    ACTION_GET_DEV_DESC    = (1 << 2),
    ACTION_GET_CONFIG_DESC = (1 << 3),
    ACTION_GET_STR_DESC    = (1 << 4),
    ACTION_CLOSE_DEV       = (1 << 5),
} action_t;

#define DEV_MAX_COUNT 128

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t                  dev_addr;
    usb_device_handle_t      dev_hdl;
    action_t                 actions;
} usb_device_t;

typedef struct {
    struct {
        union {
            struct {
                uint8_t unhandled_devices : 1;
                uint8_t shutdown          : 1;
                uint8_t reserved6         : 6;
            };
            uint8_t val;
        } flags;
        usb_device_t device[DEV_MAX_COUNT];
    } mux_protected;
    struct {
        usb_host_client_handle_t client_hdl;
        SemaphoreHandle_t        mux_lock;
    } constant;
} class_driver_t;

/* ── CPR frame ───────────────────────────────────────────────────────────── */
typedef struct {
    int     raw_lat;
    int     raw_lon;
    int64_t ts_us;
    bool    valid;
} cpr_frame_t;

/* ── aircraft record ─────────────────────────────────────────────────────── */
typedef struct {
    uint32_t    icao;
    char        callsign[9];
    int         altitude;
    int         velocity;
    int         heading;
    float       lat;
    float       lon;
    bool        pos_valid;
    int         ew_velocity;
    int         ns_velocity;
    int         vert_rate;
    int         msg_count;
    int64_t     last_seen_us;
    cpr_frame_t cpr_even;
    cpr_frame_t cpr_odd;
    bool        active;
} aircraft_t;

/* ── event log ───────────────────────────────────────────────────────────── */
typedef struct {
    char    text[80];
    uint8_t color;
} log_entry_t;

/* ── audio events ────────────────────────────────────────────────────────── */
typedef enum {
    AUDIO_EVT_NONE = 0,
    AUDIO_EVT_BOOT,
    AUDIO_EVT_NEW_CONTACT,
    AUDIO_EVT_LOST_CONTACT,
    AUDIO_EVT_POSITION,
} audio_evt_t;

/* ── globals ─────────────────────────────────────────────────────────────── */
static const char *TAG = "CLASS";
static rtlsdr_dev_t   *rtldev       = NULL;
static class_driver_t *s_driver_obj;
static mode_s_t        state;

static aircraft_t  s_aircraft[MAX_TRACKED];
static log_entry_t s_log[LOG_LINES];
static int         s_log_head    = 0;
static int         s_msg_count   = 0;
static int         s_msg_rate    = 0;
static int         s_msg_bucket  = 0;
static int64_t     s_rate_ts     = 0;
static int64_t     s_start_us    = 0;
static int         s_good        = 0;
static int         s_crc_err     = 0;
static int64_t     s_last_draw   = 0;
static bool        s_dirty       = false;
static float       s_decode_smooth = 0.0f;
static float       s_crc_smooth    = 0.0f;

/* audio */
static i2s_chan_handle_t  s_i2s_tx  = NULL;
static volatile int       s_volume  = 70;
static volatile bool      s_muted   = false;
static QueueHandle_t      s_audio_q = NULL;

/* ── ANSI / phosphor-green ATC palette ───────────────────────────────────── */
#define CLS      "\033[2J\033[H"
#define RESET    "\033[0m"
#define BOLD     "\033[1m"
#define DIM      "\033[2m"

/* 24-bit RGB phosphor green palette */
#define PH_HI    "\033[38;2;0;255;80m"     /* bright phosphor    */
#define PH_MID   "\033[38;2;0;200;60m"     /* mid phosphor       */
#define PH_DIM   "\033[38;2;0;100;30m"     /* dim phosphor       */
#define PH_SCAN  "\033[38;2;140;255;140m"  /* scan highlight     */
#define PH_GRID  "\033[38;2;0;60;20m"      /* grid lines         */

/* accent colours */
#define AC_AMBER "\033[38;2;255;180;0m"    /* warning amber      */
#define AC_RED   "\033[38;2;255;60;60m"    /* alert red          */
#define AC_CYAN  "\033[38;2;0;220;220m"    /* info cyan          */
#define AC_WHITE "\033[38;2;220;255;220m"  /* near-white         */

/* box drawing UTF-8 */
#define HL   "\xe2\x94\x80"
#define VL   "\xe2\x94\x82"
#define TL   "\xe2\x94\x8c"
#define TR   "\xe2\x94\x90"
#define BL   "\xe2\x94\x94"
#define BR   "\xe2\x94\x98"
#define TR_  "\xe2\x94\x9c"
#define TL_  "\xe2\x94\xa4"
#define T_UP "\xe2\x94\xb4"
#define CROSS "\xe2\x94\xbc"
#define BLK  "\xe2\x96\x88"
#define BBLK "\xe2\x96\x91"

/* ═══════════════════════════════════════════════════════════════════════════
 * AUDIO
 * ═══════════════════════════════════════════════════════════════════════════ */

static void audio_write_mono(const int16_t *samples, int n)
{
    if (!s_i2s_tx || s_muted) return;
    static int16_t stereo[512];
    int written = 0;
    while (written < n) {
        int chunk = n - written;
        if (chunk > 256) chunk = 256;
        float vol = s_volume / 100.0f;
        for (int i = 0; i < chunk; i++) {
            int16_t s = (int16_t)(samples[written + i] * vol);
            stereo[i*2]   = s;
            stereo[i*2+1] = s;
        }
        size_t bytes_out = 0;
        i2s_channel_write(s_i2s_tx, stereo, chunk * 4,
                          &bytes_out, pdMS_TO_TICKS(20));
        written += chunk;
    }
}

static void audio_tone(float freq, float dur_s, float amp)
{
    if (!s_i2s_tx || s_muted) return;
    int total = (int)(AUDIO_RATE_HZ * dur_s);
    static int16_t buf[256];
    for (int i = 0; i < total; i += 256) {
        int chunk = total - i;
        if (chunk > 256) chunk = 256;
        for (int j = 0; j < chunk; j++) {
            float t = (float)(i + j) / AUDIO_RATE_HZ;
            buf[j] = (int16_t)(sinf(2.0f * (float)M_PI * freq * t) * amp);
        }
        audio_write_mono(buf, chunk);
    }
}

static void snd_boot(void)
{
    audio_tone(440.0f, 0.06f, 7000.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    audio_tone(660.0f, 0.06f, 7000.0f);
    vTaskDelay(pdMS_TO_TICKS(20));
    audio_tone(880.0f, 0.10f, 7000.0f);
}

static void snd_new_contact(void)
{
    audio_tone(1200.0f, 0.03f, 5000.0f);
    vTaskDelay(pdMS_TO_TICKS(40));
    audio_tone(1200.0f, 0.03f, 5000.0f);
}

static void snd_lost_contact(void)
{
    audio_tone(800.0f, 0.05f, 4000.0f);
    vTaskDelay(pdMS_TO_TICKS(15));
    audio_tone(600.0f, 0.08f, 3000.0f);
}

static void snd_position_fix(void)
{
    audio_tone(1800.0f, 0.02f, 3000.0f);
}

static void audio_task(void *arg)
{
    audio_evt_t evt;
    while (1) {
        if (xQueueReceive(s_audio_q, &evt, portMAX_DELAY) == pdTRUE) {
            switch (evt) {
                case AUDIO_EVT_BOOT:        snd_boot();         break;
                case AUDIO_EVT_NEW_CONTACT: snd_new_contact();  break;
                case AUDIO_EVT_LOST_CONTACT:snd_lost_contact(); break;
                case AUDIO_EVT_POSITION:    snd_position_fix(); break;
                default: break;
            }
        }
    }
}

void audio_play(audio_evt_t evt)
{
    if (s_audio_q) xQueueSend(s_audio_q, &evt, 0);
}

esp_err_t audio_init(void)
{
    const i2c_config_t i2c_cfg = {
        .sda_io_num    = I2C_SDA_PIN,
        .scl_io_num    = I2C_SCL_PIN,
        .mode          = I2C_MODE_MASTER,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &i2c_cfg);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    es8311_handle_t es = es8311_create(I2C_NUM_0, 0x18);
    if (!es) {
        ESP_LOGW(TAG, "ES8311 not found — audio disabled");
        return ESP_FAIL;
    }

    const es8311_clock_config_t clk = {
        .mclk_inverted      = false,
        .sclk_inverted      = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency     = AUDIO_RATE_HZ * MCLK_MULTIPLE,
        .sample_frequency   = AUDIO_RATE_HZ,
    };
    es8311_init(es, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16);
    es8311_sample_frequency_config(es, AUDIO_RATE_HZ * MCLK_MULTIPLE,
                                   AUDIO_RATE_HZ);
    es8311_voice_volume_set(es, s_volume, NULL);
    es8311_microphone_config(es, false);

    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    i2s_chan_handle_t rx_tmp;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_tx, &rx_tmp));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN, .bclk = I2S_BCK_PIN,
            .ws   = I2S_WS_PIN,   .dout = I2S_DOUT_PIN,
            .din  = I2S_DIN_PIN,
            .invert_flags = {0},
        },
    };
    std_cfg.clk_cfg.mclk_multiple = MCLK_MULTIPLE;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_tx, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_tx));

    gpio_config_t pa = { .pin_bit_mask = (1ULL << PA_EN_PIN),
                         .mode = GPIO_MODE_OUTPUT };
    gpio_config(&pa);
    gpio_set_level(PA_EN_PIN, 1);

    s_audio_q = xQueueCreate(8, sizeof(audio_evt_t));
    xTaskCreatePinnedToCore(audio_task, "audio", 4096, NULL, 6, NULL, 1);
    ESP_LOGI(TAG, "Audio OK  vol=%d", s_volume);
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CPR DECODE
 * ═══════════════════════════════════════════════════════════════════════════ */

static int cpr_nl(double lat)
{
    if (lat < 0) lat = -lat;
    if (lat < 10.47047130) return 59;
    if (lat < 14.82817437) return 58;
    if (lat < 18.18626357) return 57;
    if (lat < 21.02939493) return 56;
    if (lat < 23.54504487) return 55;
    if (lat < 25.82924707) return 54;
    if (lat < 27.93898710) return 53;
    if (lat < 29.91135686) return 52;
    if (lat < 31.77209708) return 51;
    if (lat < 33.53993436) return 50;
    if (lat < 35.22899598) return 49;
    if (lat < 36.85025108) return 48;
    if (lat < 38.41241892) return 47;
    if (lat < 39.92256684) return 46;
    if (lat < 41.38651832) return 45;
    if (lat < 42.80914012) return 44;
    if (lat < 44.19454951) return 43;
    if (lat < 45.54626723) return 42;
    if (lat < 46.86733252) return 41;
    if (lat < 48.16039128) return 40;
    if (lat < 49.42776439) return 39;
    if (lat < 50.67150166) return 38;
    if (lat < 51.89342469) return 37;
    if (lat < 53.09516153) return 36;
    if (lat < 54.27817472) return 35;
    if (lat < 55.44378444) return 34;
    if (lat < 56.59318756) return 33;
    if (lat < 57.72747354) return 32;
    if (lat < 58.84763776) return 31;
    if (lat < 59.95459277) return 30;
    if (lat < 61.04917774) return 29;
    if (lat < 62.13216659) return 28;
    if (lat < 63.20427479) return 27;
    if (lat < 64.26616523) return 26;
    if (lat < 65.31845310) return 25;
    if (lat < 66.36171008) return 24;
    if (lat < 67.39646774) return 23;
    if (lat < 68.42322022) return 22;
    if (lat < 69.44242631) return 21;
    if (lat < 70.45451075) return 20;
    if (lat < 71.45986473) return 19;
    if (lat < 72.45884545) return 18;
    if (lat < 73.45177442) return 17;
    if (lat < 74.43893416) return 16;
    if (lat < 75.42056257) return 15;
    if (lat < 76.39684391) return 14;
    if (lat < 77.36789461) return 13;
    if (lat < 78.33374083) return 12;
    if (lat < 79.29428225) return 11;
    if (lat < 80.24923213) return 10;
    if (lat < 81.19801349) return 9;
    if (lat < 82.13956981) return 8;
    if (lat < 83.07199445) return 7;
    if (lat < 83.99173563) return 6;
    if (lat < 84.89166191) return 5;
    if (lat < 85.75541621) return 4;
    if (lat < 86.53536998) return 3;
    if (lat < 87.00000000) return 2;
    return 1;
}

static bool cpr_decode(aircraft_t *a)
{
    if (!a->cpr_even.valid || !a->cpr_odd.valid) return false;
    int64_t dt = a->cpr_even.ts_us - a->cpr_odd.ts_us;
    if (dt < 0) dt = -dt;
    if (dt > 10000000LL) return false;

    double rlat0 = a->cpr_even.raw_lat / 131072.0;
    double rlat1 = a->cpr_odd.raw_lat  / 131072.0;
    double rlon0 = a->cpr_even.raw_lon / 131072.0;
    double rlon1 = a->cpr_odd.raw_lon  / 131072.0;

    double dlat0 = 360.0 / 60.0;
    double dlat1 = 360.0 / 59.0;
    double j     = floor(59.0 * rlat0 - 60.0 * rlat1 + 0.5);

    double lat0 = dlat0 * (fmod(j, 60.0) + rlat0);
    double lat1 = dlat1 * (fmod(j, 59.0) + rlat1);
    if (lat0 >= 270.0) lat0 -= 360.0;
    if (lat1 >= 270.0) lat1 -= 360.0;

    if (cpr_nl(lat0) != cpr_nl(lat1)) return false;

    double lat, rlon, dlon;
    int nl;
    if (a->cpr_even.ts_us >= a->cpr_odd.ts_us) {
        lat  = lat0; nl = cpr_nl(lat0); rlon = rlon0;
    } else {
        lat  = lat1; nl = cpr_nl(lat1); if (nl > 0) nl--; rlon = rlon1;
    }
    dlon = 360.0 / (nl > 0 ? nl : 1);

    double m   = floor(rlon0 * (cpr_nl(lat) - 1) -
                       rlon1 *  cpr_nl(lat)       + 0.5);
    double lon = dlon * (fmod(m, (nl > 0 ? nl : 1)) + rlon);
    if (lon >= 180.0) lon -= 360.0;

    a->lat = (float)lat;
    a->lon = (float)lon;
    a->pos_valid = true;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * LOG / AIRCRAFT HELPERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void tui_log(uint8_t color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_log[s_log_head % LOG_LINES].text,
              sizeof(s_log[0].text), fmt, ap);
    va_end(ap);
    s_log[s_log_head % LOG_LINES].color = color;
    s_log_head++;
    s_dirty = true;
}

static aircraft_t *find_or_create(uint32_t icao)
{
    aircraft_t *empty = NULL;
    for (int i = 0; i < MAX_TRACKED; i++) {
        if (s_aircraft[i].active && s_aircraft[i].icao == icao)
            return &s_aircraft[i];
        if (!s_aircraft[i].active && !empty)
            empty = &s_aircraft[i];
    }
    if (empty) {
        memset(empty, 0, sizeof(aircraft_t));
        empty->icao   = icao;
        empty->active = true;
        tui_log(1, "CONTACT  %06lX  first squawk", (unsigned long)icao);
        audio_play(AUDIO_EVT_NEW_CONTACT);
    }
    return empty;
}

static int active_count(void)
{
    int n = 0;
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < MAX_TRACKED; i++) {
        if (!s_aircraft[i].active) continue;
        if (now - s_aircraft[i].last_seen_us > 60000000LL) {
            tui_log(4, "LOST     %06lX  (%s)",
                    (unsigned long)s_aircraft[i].icao,
                    s_aircraft[i].callsign[0] ?
                        s_aircraft[i].callsign : "--------");
            s_aircraft[i].active = false;
            audio_play(AUDIO_EVT_LOST_CONTACT);
        } else {
            n++;
        }
    }
    return n;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * TUI LAYOUT CONFIG
 * TERM_W = visible chars INSIDE the two border │ characters.
 * Your terminal window should be at least TERM_W+2 columns wide.
 * ═══════════════════════════════════════════════════════════════════════════ */
#define TERM_W      154
#define RADAR_COLS   41   /* must be odd; right panel visible width          */
#define RADAR_ROWS   20
#define TABLE_ROWS   20
#define LOG_SHOW      7
/* inner border │ takes 1 char, space before panel takes 1 char = 2 overhead */
#define LEFT_W       (TERM_W - RADAR_COLS - 2)
#define EL           "\033[K"

/* ── draw helpers ─────────────────────────────────────────────────────────*/

static void print_bar(const char *color, float value, float max, int width)
{
    int filled = (max > 0.0f) ? (int)(value * width / max) : 0;
    if (filled > width) filled = width;
    if (filled < 0)     filled = 0;
    printf("%s", color);
    for (int i = 0; i < width; i++)
        printf("%s", i < filled ? BLK : BBLK);
    printf(RESET);
}

static void hline(int w)
{
    printf(PH_GRID);
    for (int i = 0; i < w; i++) printf(HL);
    printf(RESET);
}

/* Every content row is:  │  <LEFT_W chars>  │  <RADAR_COLS chars>  │  \n
 * row_begin/end wrap the outer borders only.
 * The inner │ is printed manually between the two panels.               */
static void row_begin(void) { printf(PH_GRID VL RESET); }
static void row_end(void)   { printf(PH_GRID VL EL "\n" RESET); }

static void sep_full(void)
{
    /* ├────────────────────────────────────────────────┤ */
    printf(PH_GRID TR_); hline(TERM_W); printf(TL_ EL "\n" RESET);
}

static void sep_split(void)
{
    /* ├─── left ───┼─── right ───┤ */
    printf(PH_GRID TR_);
    hline(LEFT_W);
    printf(CROSS);
    hline(RADAR_COLS);
    printf(TL_ EL "\n" RESET);
}

static void sp(int n) { for (int i = 0; i < n; i++) putchar(' '); }

static void fmt_uptime(char *buf, size_t len)
{
    int64_t s = (esp_timer_get_time() - s_start_us) / 1000000LL;
    snprintf(buf, len, "%02d:%02d:%02d",
             (int)(s/3600), (int)((s%3600)/60), (int)(s%60));
}

/* ═══════════════════════════════════════════════════════════════════════════
 * RADAR — real aircraft positions, tight sweep line, two range rings
 * ═══════════════════════════════════════════════════════════════════════════ */

#define RADAR_RANGE_KM  400.0f
#define RADAR_CX        (RADAR_COLS / 2)
#define RADAR_CY        (RADAR_ROWS / 2)

static int  s_sweep_angle = 0;

/* panel mode: 0 = radar, 1 = waterfall */
static int  s_panel_mode  = 0;

/* waterfall: store last RADAR_ROWS signal-strength rows */
#define WF_BINS  RADAR_COLS
static uint8_t s_wf[RADAR_ROWS][WF_BINS];   /* 0-255 intensity per bin     */
static int     s_wf_row = 0;                 /* next row to write (ring buf)*/

static void latlon_to_xy(float clat, float clon,
                          float alat, float alon,
                          int *ox, int *oy)
{
    float dlat  = alat - clat;
    float dlon  = (alon - clon) * cosf(clat * (float)M_PI / 180.0f);
    float dx_km =  dlon * 111.0f;
    float dy_km = -dlat * 111.0f;
    float rx = (float)RADAR_CX * 0.90f;
    float ry = (float)RADAR_CY * 0.90f;
    *ox = RADAR_CX + (int)(dx_km / RADAR_RANGE_KM * rx);
    *oy = RADAR_CY + (int)(dy_km / RADAR_RANGE_KM * ry);
}

static void render_radar(char panel[RADAR_ROWS][RADAR_COLS + 1])
{
    for (int r = 0; r < RADAR_ROWS; r++) {
        memset(panel[r], ' ', RADAR_COLS);
        panel[r][RADAR_COLS] = '\0';
    }

    /* three range rings — keep outermost inside the panel boundary */
    for (int r = 0; r < RADAR_ROWS; r++) {
        for (int c = 0; c < RADAR_COLS; c++) {
            float dx = (float)(c - RADAR_CX);
            float dy = (float)(r - RADAR_CY) * 2.2f; /* aspect ratio fix */
            float d  = sqrtf(dx*dx + dy*dy);
            float r1 = (float)RADAR_CX * 0.30f;
            float r2 = (float)RADAR_CX * 0.60f;
            float r3 = (float)RADAR_CX * 0.90f;  /* pulled in from border */
            if (fabsf(d - r1) < 0.55f) panel[r][c] = '.';
            if (fabsf(d - r2) < 0.55f) panel[r][c] = '.';
            if (fabsf(d - r3) < 0.55f) panel[r][c] = ':';
        }
    }

    /* crosshair */
    panel[RADAR_CY][RADAR_CX] = '+';
    if (RADAR_CY > 0)             panel[RADAR_CY-1][RADAR_CX] = '|';
    if (RADAR_CY+1 < RADAR_ROWS)  panel[RADAR_CY+1][RADAR_CX] = '|';
    if (RADAR_CX > 1)             panel[RADAR_CY][RADAR_CX-1] = '-';
    if (RADAR_CX+1 < RADAR_COLS)  panel[RADAR_CY][RADAR_CX+1] = '-';

    /* cardinals */
    panel[0][RADAR_CX]            = 'N';
    panel[RADAR_ROWS-1][RADAR_CX] = 'S';
    panel[RADAR_CY][0]            = 'W';
    panel[RADAR_CY][RADAR_COLS-1] = 'E';

    /* tight sweep line — threshold 0.07 rad ≈ 4° */
    float sa = s_sweep_angle * (float)M_PI / 180.0f;
    float sweep_limit = (float)RADAR_CX * 0.90f;
    for (int r = 0; r < RADAR_ROWS; r++) {
        for (int c = 0; c < RADAR_COLS; c++) {
            float dx = (float)(c - RADAR_CX);
            float dy = (float)(r - RADAR_CY) * 2.2f;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d > sweep_limit || d < 1.0f) continue;
            float angle = atan2f(dy, dx);
            float diff  = angle - sa;
            while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
            while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
            if (fabsf(diff) < 0.07f)
                panel[r][c] = '/';          /* tight bright line */
            else if (fabsf(diff) < 0.18f && panel[r][c] == ' ')
                panel[r][c] = ',';          /* narrow fade */
        }
    }

    /* aircraft blips */
    float clat = 53.3498f, clon = -6.2603f;
    int pc = 0; float sl = 0, sn = 0;
    for (int i = 0; i < MAX_TRACKED; i++) {
        if (s_aircraft[i].active && s_aircraft[i].pos_valid) {
            sl += s_aircraft[i].lat; sn += s_aircraft[i].lon; pc++;
        }
    }
    if (pc > 0) { clat = sl / pc; clon = sn / pc; }

    for (int i = 0; i < MAX_TRACKED; i++) {
        if (!s_aircraft[i].active || !s_aircraft[i].pos_valid) continue;
        int px, py;
        latlon_to_xy(clat, clon, s_aircraft[i].lat, s_aircraft[i].lon, &px, &py);
        if (px < 0 || px >= RADAR_COLS || py < 0 || py >= RADAR_ROWS) continue;
        panel[py][px] = '*';
        /* show up to 3 callsign chars after blip */
        if (s_aircraft[i].callsign[0]) {
            for (int ci = 0; ci < 3 && (px + 1 + ci) < RADAR_COLS; ci++) {
                if (!s_aircraft[i].callsign[ci]) break;
                panel[py][px + 1 + ci] = s_aircraft[i].callsign[ci];
            }
        }
    }
}

/* ── waterfall ─────────────────────────────────────────────────────────────
 * Updated from the IQ magnitude data each demodulate() call.
 * We bucket the magnitude vector into WF_BINS bins and push a new row.
 * Uses running peak tracking for auto-gain so narrow signals are visible. */
static uint32_t s_wf_peak = 256;  /* running peak for auto-scale */

void waterfall_push(const uint16_t *mag, int mag_len)
{
    if (!mag || mag_len <= 0) return;
    int bin_size = mag_len / WF_BINS;
    if (bin_size < 1) bin_size = 1;
    int row = s_wf_row % RADAR_ROWS;

    /* first pass: compute bins and track peak */
    uint32_t bins[WF_BINS];
    uint32_t frame_peak = 1;
    for (int b = 0; b < WF_BINS; b++) {
        int start = b * bin_size;
        int end   = start + bin_size;
        if (end > mag_len) end = mag_len;
        uint32_t sum = 0;
        for (int j = start; j < end; j++) sum += mag[j];
        bins[b] = sum / (uint32_t)(end - start);
        if (bins[b] > frame_peak) frame_peak = bins[b];
    }

    /* IIR track peak with slow decay so display stays responsive */
    if (frame_peak > s_wf_peak)
        s_wf_peak = frame_peak;
    else
        s_wf_peak = s_wf_peak - (s_wf_peak >> 6) + (frame_peak >> 6);
    if (s_wf_peak < 256) s_wf_peak = 256;

    /* second pass: normalize to 0-255 using tracked peak */
    for (int b = 0; b < WF_BINS; b++) {
        uint32_t v = bins[b] * 255 / s_wf_peak;
        if (v > 255) v = 255;
        s_wf[row][b] = (uint8_t)v;
    }
    s_wf_row++;
}

static void render_waterfall(char panel[RADAR_ROWS][RADAR_COLS + 1])
{
    /* We still fill the text panel with density chars for the non-coloured path,
     * but the real display comes from the coloured render in tui_draw().       */
    static const char dens[] = " .,:;=+*#%@";
    for (int r = 0; r < RADAR_ROWS; r++) {
        int src = ((s_wf_row - 1 - r) % RADAR_ROWS + RADAR_ROWS) % RADAR_ROWS;
        panel[r][RADAR_COLS] = '\0';
        for (int c = 0; c < RADAR_COLS; c++) {
            int v = s_wf[src][c];
            int idx = v * (int)(sizeof(dens) - 2) / 255;
            if (idx < 0) idx = 0;
            if (idx > (int)(sizeof(dens) - 2)) idx = (int)(sizeof(dens) - 2);
            panel[r][c] = dens[idx];
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * TUI DRAW
 * ═══════════════════════════════════════════════════════════════════════════ */

static void tui_draw(void)
{
    int64_t now = esp_timer_get_time();
    if (!s_dirty && (now - s_last_draw) < (TUI_REFRESH_MS * 1000LL)) return;
    s_last_draw = now;
    s_dirty     = false;

    if (now - s_rate_ts >= 1000000LL) {
        s_msg_rate   = s_msg_bucket;
        s_msg_bucket = 0;
        s_rate_ts    = now;
    }

    s_sweep_angle = (s_sweep_angle + 4) % 360;

    int   ac    = active_count();
    int   total = s_good + s_crc_err;
    float dp    = total > 0 ? (s_good    * 100.0f / total) : 0.0f;
    float cp    = total > 0 ? (s_crc_err * 100.0f / total) : 0.0f;
    s_decode_smooth += (dp - s_decode_smooth) * 0.12f;
    s_crc_smooth    += (cp - s_crc_smooth)    * 0.12f;

    char uptime[12];
    fmt_uptime(uptime, sizeof(uptime));

    printf("\033[H");

    /* ┌─────────────────────────────────────────────────┐ */
    printf(PH_GRID TL); hline(TERM_W); printf(TR EL "\n" RESET);

    /* header — single full-width row */
    row_begin();
    {
        int n = (int)strlen("  ATC TERMINAL  //  ESP32-P4 ADS-B RECEIVER"
                            "  //  1090.000 MHz  //  2 MSPS");
        printf(PH_HI BOLD "  ATC TERMINAL" RESET
               PH_GRID "  //  " RESET PH_SCAN "ESP32-P4 ADS-B RECEIVER" RESET
               PH_GRID "  //  " RESET PH_HI "1090.000 MHz" RESET
               PH_GRID "  //  " RESET PH_MID "2 MSPS" RESET);
        sp(TERM_W - n);
    }
    row_end();
    sep_full();

    /* status */
    row_begin();
    {
        char vol_str[8];
        snprintf(vol_str, sizeof(vol_str), "%3d%%", s_muted ? 0 : s_volume);
        /* compute visible width by snprintf to scratch buffer */
        char scratch[256];
        int n = snprintf(scratch, sizeof(scratch),
            "  UP %-9s  ACFT %-3d  MSG/S %-5d  TOTAL %-8d"
            "  DEC %5.1f%%  ERR %5.1f%%  VOL %s",
            uptime, ac, s_msg_rate, s_msg_count,
            s_decode_smooth, s_crc_smooth, vol_str);
        if (n > TERM_W) n = TERM_W;
        printf(PH_DIM "  UP " RESET PH_HI "%-9s" RESET
               PH_DIM "  ACFT " RESET PH_HI BOLD "%-3d" RESET
               PH_DIM "  MSG/S " RESET PH_SCAN "%-5d" RESET
               PH_DIM "  TOTAL " RESET PH_MID "%-8d" RESET
               PH_DIM "  DEC " RESET PH_HI "%5.1f%%" RESET
               PH_DIM "  ERR " RESET AC_AMBER "%5.1f%%" RESET
               PH_DIM "  VOL " RESET "%s%s" RESET,
               uptime, ac, s_msg_rate, s_msg_count,
               s_decode_smooth, s_crc_smooth,
               s_muted ? AC_RED : PH_HI, vol_str);
        sp(TERM_W - n);
    }
    row_end();

    /* bars */
    row_begin();
    {
        /* overhead: "  DECODE [" = 10, "] ERR [" = 7, "]" = 1 → 18 total */
        int bar_w = (TERM_W - 18) / 2;
        if (bar_w < 10) bar_w = 10;
        int n = 10 + bar_w + 7 + bar_w + 1;
        printf(PH_DIM "  DECODE [" RESET);
        print_bar(PH_HI, s_decode_smooth, 100.0f, bar_w);
        printf(PH_DIM "] ERR [" RESET);
        print_bar(AC_AMBER, s_crc_smooth, 100.0f, bar_w);
        printf(PH_DIM "]" RESET);
        sp(TERM_W - n);
    }
    row_end();

    /* ├─── left ───┼─── right ───┤ */
    sep_split();

    /* panel header row */
    row_begin();
    {
        /* left: column labels */
        char hdr[256];
        int n = snprintf(hdr, sizeof(hdr),
            "  %-8s  %-9s  %8s  %7s  %-5s  %9s  %9s  %5s  %4s ",
            "ICAO","CALLSIGN","ALT ft","SPD kt","HDG","LAT","LON","V/S","MSGS");
        printf(PH_DIM "%s" RESET, hdr);
        sp(LEFT_W - n);
        /* inner border */
        printf(PH_GRID VL RESET);
        /* right: panel title */
        const char *title = s_panel_mode == 0 ? " RADAR  [R]" : " WFALL  [R]";
        printf(PH_DIM " %-*s" RESET, RADAR_COLS - 1, title);
    }
    row_end();
    sep_split();

    /* render panel */
    char panel[RADAR_ROWS][RADAR_COLS + 1];
    if (s_panel_mode == 0)
        render_radar(panel);
    else
        render_waterfall(panel);

    /* aircraft rows + panel */
    int ac_idx = 0;
    for (int row = 0; row < TABLE_ROWS; row++) {
        aircraft_t *a = NULL;
        for (; ac_idx < MAX_TRACKED; ac_idx++) {
            if (s_aircraft[ac_idx].active) { a = &s_aircraft[ac_idx++]; break; }
        }

        row_begin();

        if (a) {
            const char *alt_col = PH_MID;
            char vs_plain[16] = "  --";
            char vs_col[64]   = "  --";
            if (a->vert_rate > 200) {
                alt_col = PH_HI;
                snprintf(vs_plain, sizeof(vs_plain), "+%d", a->vert_rate);
                snprintf(vs_col,   sizeof(vs_col),   PH_HI "+%d" RESET, a->vert_rate);
            } else if (a->vert_rate < -200) {
                alt_col = AC_AMBER;
                snprintf(vs_plain, sizeof(vs_plain), "%d", a->vert_rate);
                snprintf(vs_col,   sizeof(vs_col),   AC_AMBER "%d" RESET, a->vert_rate);
            }
            char lat_s[12] = "   ------";
            char lon_s[12] = "   ------";
            if (a->pos_valid) {
                snprintf(lat_s, sizeof(lat_s), "%+9.4f", a->lat);
                snprintf(lon_s, sizeof(lon_s), "%+9.4f", a->lon);
            }
            const char *dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
            int didx = (int)(((float)a->heading + 22.5f) / 45.0f) % 8;
            const char *cs = a->callsign[0] ? a->callsign : "--------";

            /* measure visible width of left panel content */
            int vis = 2+8+2+9+2+8+2+7+2+(int)strlen(dirs[didx])+1+3
                      +2+9+2+9+2+(int)strlen(vs_plain)+2+4+1;

            printf("  " AC_CYAN "%-8lX" RESET
                   "  " PH_HI   "%-9s" RESET
                   "  " "%s%8d" RESET
                   "  " PH_MID  "%7d" RESET
                   "  " PH_MID  "%s" RESET PH_DIM "-" RESET PH_HI "%03d" RESET
                   "  " PH_DIM  "%9s" RESET
                   "  " PH_DIM  "%9s" RESET
                   "  " "%s"
                   "  " PH_DIM  "%4d" RESET " ",
                   (unsigned long)a->icao, cs,
                   alt_col, a->altitude,
                   a->velocity,
                   dirs[didx], a->heading,
                   lat_s, lon_s,
                   vs_col,
                   a->msg_count);
            sp(LEFT_W - vis);
        } else {
            printf(PH_DIM "  ---" RESET);
            sp(LEFT_W - 5);
        }

        /* inner border + panel line */
        printf(PH_GRID VL RESET);
        if (row < RADAR_ROWS) {
            if (s_panel_mode == 0) {
                /* radar: colour each char individually */
                for (int c = 0; c < RADAR_COLS; c++) {
                    char ch = panel[row][c];
                    if      (ch == '*')                                        printf(PH_HI BOLD "%c" RESET, ch);
                    else if (ch == '/')                                        printf(PH_SCAN "%c" RESET, ch);
                    else if (ch == ',')                                        printf(PH_DIM "%c" RESET, ch);
                    else if (ch=='N'||ch=='S'||ch=='E'||ch=='W'||ch=='+'||ch=='|') printf(PH_MID "%c" RESET, ch);
                    else if (ch == ':')                                        printf(PH_GRID "%c" RESET, ch);
                    else if (ch == '-' || ch == '.')                           printf(PH_DIM "%c" RESET, ch);
                    else if (ch != ' ')                                        printf(PH_MID "%c" RESET, ch);
                    else putchar(' ');
                }
            } else {
                /* waterfall: heat-map colour gradient green→amber→white */
                int src = ((s_wf_row - 1 - row) % RADAR_ROWS + RADAR_ROWS) % RADAR_ROWS;
                for (int c = 0; c < RADAR_COLS; c++) {
                    int v = s_wf[src][c];
                    int rr, gg, bb;
                    if (v < 85) {
                        /* black → dark green */
                        rr = 0; gg = 20 + v * 2; bb = 0;
                    } else if (v < 170) {
                        /* dark green → bright green/amber */
                        int t = v - 85;
                        rr = t * 2; gg = 190 + t; bb = 0;
                    } else {
                        /* amber → white-hot */
                        int t = v - 170;
                        rr = 170 + t; gg = 255; bb = t * 3;
                        if (bb > 255) bb = 255;
                        if (rr > 255) rr = 255;
                    }
                    /* use block char for filled look */
                    char ch = panel[row][c];
                    if (ch == ' ' || ch == '.') ch = ' ';
                    printf("\033[38;2;%d;%d;%dm%c" RESET, rr, gg, bb, ch);
                }
            }
        } else {
            sp(RADAR_COLS);
        }
        row_end();
    }

    sep_split();

    /* event log */
    row_begin();
    printf(PH_DIM "  EVENT LOG"); sp(LEFT_W - 11);
    printf(PH_GRID VL RESET); sp(RADAR_COLS);
    row_end();

    static const char *log_cols[] = { PH_DIM, PH_HI, AC_AMBER, AC_CYAN, AC_RED };
    for (int i = LOG_SHOW - 1; i >= 0; i--) {
        int idx = (s_log_head - 1 - i + LOG_LINES * 2) % LOG_LINES;
        row_begin();
        printf("  ");
        if (s_log[idx].text[0]) {
            int tlen = (int)strnlen(s_log[idx].text, sizeof(s_log[0].text));
            printf("%s%s" RESET, log_cols[s_log[idx].color], s_log[idx].text);
            sp(LEFT_W - 2 - tlen);
        } else {
            printf(PH_DIM "~" RESET); sp(LEFT_W - 3);
        }
        /* right side of log rows: blank panel column */
        printf(PH_GRID VL RESET); sp(RADAR_COLS);
        row_end();
    }

    /* └─────────────────────────────────────────────────┘ */
    printf(PH_GRID BL); hline(LEFT_W); printf(T_UP); hline(RADAR_COLS);
    printf(BR EL "\n" RESET);
    printf(PH_DIM "  R828D  " PH_GRID "|" RESET
           PH_DIM "  RAFAEL MICRO  " PH_GRID "|" RESET
           PH_DIM "  ctrl+] EXIT  " PH_GRID "|" RESET
           PH_DIM "  [M]UTE  " PH_GRID "|" RESET
           PH_DIM "  [+/-] VOL  " PH_GRID "|" RESET
           PH_DIM "  [R] RADAR/WFALL" EL "\n" RESET);

    fflush(stdout);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * DEMODULATE  (also feeds waterfall)
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ═══════════════════════════════════════════════════════════════════════════
 * ON_MSG CALLBACK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void on_msg(mode_s_t *self, struct mode_s_msg *mm)
{
    s_msg_count++;
    s_msg_bucket++;
    s_dirty = true;

    uint32_t icao = ((uint32_t)mm->aa1 << 16) |
                    ((uint32_t)mm->aa2 <<  8) |
                     (uint32_t)mm->aa3;

    if (!mm->crcok) {
        s_crc_err++;
        tui_log(4, "CRC ERR  %06lX  df=%d", (unsigned long)icao, mm->msgtype);
        tui_draw();
        return;
    }
    s_good++;

    aircraft_t *a = find_or_create(icao);
    if (!a) { tui_draw(); return; }

    a->last_seen_us = esp_timer_get_time();
    a->msg_count++;

    if (mm->flight[0]) {
        strncpy(a->callsign, mm->flight, 8);
        a->callsign[8] = '\0';
        for (int i = 7; i >= 0 && a->callsign[i] == ' '; i--)
            a->callsign[i] = '\0';
        tui_log(2, "IDENT    %06lX  %s", (unsigned long)icao, a->callsign);
    }

    if (mm->altitude)         a->altitude  = mm->altitude;
    if (mm->heading_is_valid) a->heading   = mm->heading;
    if (mm->velocity)         a->velocity  = mm->velocity;
    if (mm->ew_velocity)
        a->ew_velocity = mm->ew_dir ? -mm->ew_velocity : mm->ew_velocity;
    if (mm->ns_velocity)
        a->ns_velocity = mm->ns_dir ? -mm->ns_velocity : mm->ns_velocity;
    if (mm->vert_rate)
        a->vert_rate = mm->vert_rate_sign ? -mm->vert_rate : mm->vert_rate;

    /* CPR position */
    if (mm->msgtype == 17 && mm->metype >= 9 && mm->metype <= 18
        && mm->raw_latitude != 0) {
        int64_t ts = esp_timer_get_time();
        if (mm->fflag == 0)
            a->cpr_even = (cpr_frame_t){ mm->raw_latitude, mm->raw_longitude, ts, true };
        else
            a->cpr_odd  = (cpr_frame_t){ mm->raw_latitude, mm->raw_longitude, ts, true };
        bool was_valid = a->pos_valid;
        if (cpr_decode(a) && !was_valid) {
            tui_log(3, "FIX      %06lX  %+.4f  %+.4f",
                    (unsigned long)icao, a->lat, a->lon);
            audio_play(AUDIO_EVT_POSITION);
        }
    }

    if (mm->msgtype == 17) {
        if (mm->metype >= 9 && mm->metype <= 18 && mm->altitude)
            tui_log(0, "ALT      %06lX  %d ft", (unsigned long)icao, a->altitude);
        else if (mm->metype >= 19 && mm->metype <= 22 && mm->velocity)
            tui_log(0, "VEL      %06lX  %d kt  hdg=%d  vs=%d",
                    (unsigned long)icao, a->velocity, a->heading, a->vert_rate);
    }

    tui_draw();
}

/* forward declaration satisfied above — no duplicate needed */

void demodulate(uint8_t *source, int length)
{
    if (!source || length <= 0) return;
    int mag_len = length / 2;
    uint16_t *mag = malloc(mag_len * sizeof(uint16_t));
    if (!mag) { tui_log(4, "OOM  magnitude buffer"); return; }
    mode_s_compute_magnitude_vector(source, mag, length);
    waterfall_push(mag, mag_len);       /* feed real IQ energy into wfall  */
    mode_s_detect(&state, mag, mag_len, on_msg);
    free(mag);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ADSB RX TASK  — also polls UART0 for keystrokes
 * ═══════════════════════════════════════════════════════════════════════════ */

void adsb_rx_task(void *arg)
{
    s_start_us = esp_timer_get_time();
    s_rate_ts  = s_start_us;

    tui_log(1, "INIT     adsb_rx running on CPU1");
    tui_log(1, "INIT     R828D locked  1090.000 MHz  2 MSPS");

    uint8_t *buffer = malloc(DEFAULT_BUF_LENGTH);
    if (!buffer) { tui_log(4, "OOM rx buffer"); vTaskDelete(NULL); return; }

    int  n_read      = 0;
    bool full_buffer = false;
    mode_s_init(&state);

    /* configure UART0 for non-blocking key reads */
    uart_config_t uart_cfg = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_cfg);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    printf(CLS);
    tui_draw();

    while (true) {
        /* ── non-blocking keyread ── */
        uint8_t key = 0;
        if (uart_read_bytes(UART_NUM_0, &key, 1, 0) > 0) {
            switch (key) {
                case 'r': case 'R':
                    s_panel_mode = (s_panel_mode + 1) % 2;
                    tui_log(3, "PANEL    switched to %s",
                            s_panel_mode == 0 ? "RADAR" : "WATERFALL");
                    s_dirty = true;
                    break;
                case 'm': case 'M':
                    s_muted = !s_muted;
                    tui_log(2, "AUDIO    %s", s_muted ? "muted" : "unmuted");
                    s_dirty = true;
                    break;
                case '+': case '=':
                    s_volume += 10;
                    if (s_volume > 100) s_volume = 100;
                    s_dirty = true;
                    break;
                case '-':
                    s_volume -= 10;
                    if (s_volume < 0) s_volume = 0;
                    s_dirty = true;
                    break;
                default: break;
            }
        }

        full_buffer = true;
        for (int i = 0; i < DEFAULT_BUF_LENGTH; i += MAX_PACKET_SIZE) {
            int r = rtlsdr_read_sync(rtldev, &buffer[i],
                                     MAX_PACKET_SIZE, &n_read);
            if (r < 0) {
                tui_log(4, "READ ERR  r=%d", r);
                full_buffer = false;
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            if ((uint32_t)n_read < MAX_PACKET_SIZE) {
                tui_log(3, "SHORT READ  got=%d", n_read);
                full_buffer = false;
                break;
            }
        }

        if (full_buffer) demodulate(buffer, DEFAULT_BUF_LENGTH);

        int64_t now = esp_timer_get_time();
        if (now - s_last_draw > (TUI_REFRESH_MS * 1000LL)) {
            s_dirty = true;
            tui_draw();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buffer);
    vTaskDelete(NULL);
}


/* ═══════════════════════════════════════════════════════════════════════════
 * RTLSDR SETUP TASK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void rtlsdr_setup_task(void *arg)
{
    uint8_t dev_addr = (uint8_t)(uint32_t)arg;

    int r = rtlsdr_open(&rtldev, dev_addr, s_driver_obj->constant.client_hdl);
    if (r < 0) { ESP_LOGE(TAG, "rtlsdr_open failed"); vTaskDelete(NULL); return; }

    rtlsdr_set_center_freq(rtldev, 1090000000);
    rtlsdr_set_sample_rate(rtldev, 2000000);
    rtlsdr_set_tuner_gain_mode(rtldev, 0);
    rtlsdr_reset_buffer(rtldev);

    /* dongle found — audio already init'd in app_main, just play the sound */
    audio_play(AUDIO_EVT_NEW_CONTACT);

    xTaskCreatePinnedToCore(adsb_rx_task, "adsb_rx", 16384, NULL, 5, NULL, 1);
    vTaskDelete(NULL);
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CLIENT EVENT CALLBACK
 * ═══════════════════════════════════════════════════════════════════════════ */

static void client_event_cb(const usb_host_client_event_msg_t *event_msg,
                             void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
    case USB_HOST_CLIENT_EVENT_NEW_DEV:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        driver_obj->mux_protected.device[event_msg->new_dev.address].dev_addr =
            event_msg->new_dev.address;
        xTaskCreatePinnedToCore(rtlsdr_setup_task, "rtlsdr_setup", 8192,
                                (void *)(uint32_t)event_msg->new_dev.address,
                                4, NULL, 0);
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;
    case USB_HOST_CLIENT_EVENT_DEV_GONE:
        xSemaphoreTake(driver_obj->constant.mux_lock, portMAX_DELAY);
        for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
            if (driver_obj->mux_protected.device[i].dev_hdl ==
                event_msg->dev_gone.dev_hdl) {
                driver_obj->mux_protected.device[i].actions = ACTION_CLOSE_DEV;
                driver_obj->mux_protected.flags.unhandled_devices = 1;
            }
        }
        xSemaphoreGive(driver_obj->constant.mux_lock);
        break;
    default: abort();
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * ACTION HANDLERS
 * ═══════════════════════════════════════════════════════════════════════════ */

static void action_open_dev(usb_device_t *d)
{
    assert(d->dev_addr != 0);
    ESP_ERROR_CHECK(usb_host_device_open(d->client_hdl, d->dev_addr,
                                          &d->dev_hdl));
    d->actions |= ACTION_GET_DEV_INFO;
}
static void action_get_info(usb_device_t *d)
{
    usb_device_info_t i;
    ESP_ERROR_CHECK(usb_host_device_info(d->dev_hdl, &i));
    if (i.parent.dev_hdl) {
        usb_device_info_t p;
        ESP_ERROR_CHECK(usb_host_device_info(i.parent.dev_hdl, &p));
    }
    d->actions |= ACTION_GET_DEV_DESC;
}
static void action_get_dev_desc(usb_device_t *d)
{
    const usb_device_desc_t *dd;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(d->dev_hdl, &dd));
    d->actions |= ACTION_GET_CONFIG_DESC;
}
static void action_get_config_desc(usb_device_t *d)
{
    const usb_config_desc_t *cd;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(d->dev_hdl, &cd));
    d->actions |= ACTION_GET_STR_DESC;
}
static void action_get_str_desc(usb_device_t *d)
{
    usb_device_info_t i;
    ESP_ERROR_CHECK(usb_host_device_info(d->dev_hdl, &i));
}
static void action_close_dev(usb_device_t *d)
{
    ESP_ERROR_CHECK(usb_host_device_close(d->client_hdl, d->dev_hdl));
    d->dev_hdl = NULL; d->dev_addr = 0;
}

static void class_driver_device_handle(usb_device_t *d)
{
    uint8_t actions = d->actions;
    d->actions = 0;
    while (actions) {
        if (actions & ACTION_OPEN_DEV)        action_open_dev(d);
        if (actions & ACTION_GET_DEV_INFO)    action_get_info(d);
        if (actions & ACTION_GET_DEV_DESC)    action_get_dev_desc(d);
        if (actions & ACTION_GET_CONFIG_DESC) action_get_config_desc(d);
        if (actions & ACTION_GET_STR_DESC)    action_get_str_desc(d);
        if (actions & ACTION_CLOSE_DEV)       action_close_dev(d);
        actions = d->actions; d->actions = 0;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 * CLASS DRIVER TASK
 * ═══════════════════════════════════════════════════════════════════════════ */

void class_driver_task(void *arg)
{
    class_driver_t           obj = {0};
    usb_host_client_handle_t hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");
    SemaphoreHandle_t mux = xSemaphoreCreateMutex();
    if (!mux) { ESP_LOGE(TAG, "mutex fail"); vTaskSuspend(NULL); return; }

    usb_host_client_config_t cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = { .client_event_callback = client_event_cb,
                   .callback_arg          = (void *)&obj },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&cfg, &hdl));

    obj.constant.mux_lock   = mux;
    obj.constant.client_hdl = hdl;
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
        obj.mux_protected.device[i].client_hdl = hdl;
    s_driver_obj = &obj;

    while (1) {
        if (obj.mux_protected.flags.unhandled_devices) {
            xSemaphoreTake(obj.constant.mux_lock, portMAX_DELAY);
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
                if (obj.mux_protected.device[i].actions)
                    class_driver_device_handle(&obj.mux_protected.device[i]);
            obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(obj.constant.mux_lock);
        } else {
            if (!obj.mux_protected.flags.shutdown)
                usb_host_client_handle_events(hdl, portMAX_DELAY);
            else break;
        }
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(hdl));
    if (mux) vSemaphoreDelete(mux);
    vTaskSuspend(NULL);
}

void class_driver_client_deregister(void)
{
    xSemaphoreTake(s_driver_obj->constant.mux_lock, portMAX_DELAY);
    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
        if (s_driver_obj->mux_protected.device[i].dev_hdl != NULL) {
            s_driver_obj->mux_protected.device[i].actions |= ACTION_CLOSE_DEV;
            s_driver_obj->mux_protected.flags.unhandled_devices = 1;
        }
    }
    s_driver_obj->mux_protected.flags.shutdown = 1;
    xSemaphoreGive(s_driver_obj->constant.mux_lock);
    ESP_ERROR_CHECK(usb_host_client_unblock(s_driver_obj->constant.client_hdl));
}
