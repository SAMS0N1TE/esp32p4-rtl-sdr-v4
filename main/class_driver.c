/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "usb/usb_host.h"
#include "rtl-sdr.h"
#include "mode-s.h"
#include "esp_task_wdt.h"

#define CLIENT_NUM_EVENT_MSG  5
#define MAX_PACKET_SIZE       16384
#define DEFAULT_BUF_LENGTH    (MAX_PACKET_SIZE * 2)
#define MAX_TRACKED           16
#define LOG_LINES             6

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

/* ── tracked aircraft ───────────────────────────────────────────────────── */
typedef struct {
    uint32_t icao;
    char     callsign[9];
    int      altitude;
    int      velocity;      /* computed ground speed kt */
    int      heading;
    int      raw_lat;       /* raw CPR latitude  */
    int      raw_lon;       /* raw CPR longitude */
    int      ew_velocity;
    int      ns_velocity;
    int      vert_rate;
    int      msg_count;
    int64_t  last_seen_us;
    bool     active;
} aircraft_t;

/* ── event log ring buffer ──────────────────────────────────────────────── */
typedef struct {
    char    text[72];
    uint8_t color; /* 0=white 1=green 2=yellow 3=cyan 4=red */
} log_entry_t;

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

/* ── ANSI ────────────────────────────────────────────────────────────────── */
#define CLS        "\033[2J\033[H"
#define RESET      "\033[0m"
#define BOLD       "\033[1m"
#define FG_WHITE   "\033[97m"
#define FG_CYAN    "\033[96m"
#define FG_GREEN   "\033[92m"
#define FG_YELLOW  "\033[93m"
#define FG_RED     "\033[91m"
#define FG_BLUE    "\033[94m"
#define FG_GRAY    "\033[90m"

/* ── log helper ─────────────────────────────────────────────────────────── */
static void tui_log(uint8_t color, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(s_log[s_log_head % LOG_LINES].text,
              sizeof(s_log[0].text), fmt, ap);
    va_end(ap);
    s_log[s_log_head % LOG_LINES].color = color;
    s_log_head++;
}

/* ── aircraft helpers ───────────────────────────────────────────────────── */
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
        tui_log(1, "NEW  %06lX  first contact", (unsigned long)icao);
    }
    return empty;
}

static int active_count(void)
{
    int     n   = 0;
    int64_t now = esp_timer_get_time();
    for (int i = 0; i < MAX_TRACKED; i++) {
        if (!s_aircraft[i].active) continue;
        if (now - s_aircraft[i].last_seen_us > 60000000LL) {
            tui_log(3, "LOST %06lX  (%s)  timeout",
                    (unsigned long)s_aircraft[i].icao,
                    s_aircraft[i].callsign);
            s_aircraft[i].active = false;
        } else {
            n++;
        }
    }
    return n;
}

/* ── bar renderer ───────────────────────────────────────────────────────── */
static void print_bar(const char *color, int value, int max, int width)
{
    int filled = (max > 0) ? (value * width / max) : 0;
    if (filled > width) filled = width;
    printf("%s", color);
    for (int i = 0; i < width; i++)
        printf("%s", i < filled ? "\xe2\x96\x88" : "\xe2\x96\x91"); /* █ ░ */
    printf(RESET);
}

/* ── uptime ─────────────────────────────────────────────────────────────── */
static void fmt_uptime(char *buf, size_t len)
{
    int64_t s   = (esp_timer_get_time() - s_start_us) / 1000000LL;
    int     h   = s / 3600;
    int     m   = (s % 3600) / 60;
    int     sec = s % 60;
    snprintf(buf, len, "%02d:%02d:%02d", h, m, sec);
}

/* ── full TUI redraw ─────────────────────────────────────────────────────── */
static void tui_draw(void)
{
    char uptime[12];
    fmt_uptime(uptime, sizeof(uptime));

    int ac          = active_count();
    int total       = s_good + s_crc_err;
    int decode_pct  = total > 0 ? (s_good    * 100 / total) : 0;
    int crc_pct     = total > 0 ? (s_crc_err * 100 / total) : 0;

    /* update rate bucket */
    int64_t now = esp_timer_get_time();
    if (now - s_rate_ts >= 1000000LL) {
        s_msg_rate   = s_msg_bucket;
        s_msg_bucket = 0;
        s_rate_ts    = now;
    }

    printf(CLS);

    /* header */
    printf(FG_CYAN BOLD
           "╔══════════════════════════════════════════════════════════════════╗\n"
           "║  " RESET FG_WHITE BOLD
           "ESP32-P4  //  ADS-B RECEIVER  //  1090.000 MHz  //  2 MSPS"
           RESET FG_CYAN BOLD "  ║\n"
           "╚══════════════════════════════════════════════════════════════════╝\n"
           RESET);

    /* metrics row */
    printf(FG_GRAY  "  uptime "   RESET FG_WHITE  BOLD "%-10s" RESET
           FG_GRAY  "  aircraft " RESET FG_GREEN  BOLD "%-4d"  RESET
           FG_GRAY  "  msgs/s "   RESET FG_YELLOW BOLD "%-5d"  RESET
           FG_GRAY  "  total "    RESET FG_WHITE  BOLD "%-8d"  RESET
           FG_GRAY  "  decode "   RESET FG_GREEN  BOLD "%d%%"  RESET
           FG_GRAY  "  crc err "  RESET FG_RED    BOLD "%d%%\n" RESET,
           uptime, ac, s_msg_rate, s_msg_count, decode_pct, crc_pct);

    /* bars */
    printf(FG_GRAY "  decode  [" RESET);
    print_bar(FG_GREEN, decode_pct, 100, 20);
    printf(FG_GRAY "]  crc err [" RESET);
    print_bar(FG_RED, crc_pct, 100, 20);
    printf(FG_GRAY "]\n" RESET);

    printf(FG_CYAN
           "  ────────────────────────────────────────────────────────────────\n"
           RESET);

    /* table header */
    printf(FG_GRAY BOLD
           "  %-8s %-9s %7s %6s %4s %10s %10s %6s %5s\n" RESET,
           "ICAO", "CALLSIGN", "ALT(ft)", "SPD(kt)", "HDG",
           "EW vel", "NS vel", "V/S", "MSGS");
    printf(FG_GRAY
           "  ──────── ───────── ─────── ────── ──── ────────── ────────── "
           "────── ─────\n" RESET);

    /* aircraft rows */
    int rows = 0;
    for (int i = 0; i < MAX_TRACKED && rows < 10; i++) {
        if (!s_aircraft[i].active) continue;
        aircraft_t *a = &s_aircraft[i];

        const char *alt_color = FG_WHITE;
        char        vsbuf[32] = "    --";
        if (a->vert_rate > 100) {
            alt_color = FG_GREEN;
            snprintf(vsbuf, sizeof(vsbuf), FG_GREEN "+%d" RESET, a->vert_rate);
        } else if (a->vert_rate < -100) {
            alt_color = FG_RED;
            snprintf(vsbuf, sizeof(vsbuf), FG_RED  "%d"  RESET, a->vert_rate);
        }

        /* EW/NS direction arrows */
        char ew_buf[16], ns_buf[16];
        snprintf(ew_buf, sizeof(ew_buf), "%s%d",
                 a->ew_velocity ? (a->ew_velocity < 0 ? "W" : "E") : "",
                 abs(a->ew_velocity));
        snprintf(ns_buf, sizeof(ns_buf), "%s%d",
                 a->ns_velocity ? (a->ns_velocity < 0 ? "S" : "N") : "",
                 abs(a->ns_velocity));

        const char *cs = a->callsign[0] ? a->callsign : "--------";

        printf("  " FG_BLUE  "%-8lX" RESET
               " " FG_WHITE  "%-9s"  RESET
               " " "%s%7d"   RESET
               " " FG_WHITE  "%6d"   RESET
               " " FG_GRAY   "%03d"  RESET
               " " FG_GRAY   "%10s"  RESET
               " " FG_GRAY   "%10s"  RESET
               " %s"
               " " FG_GRAY   "%5d"   RESET "\n",
               (unsigned long)a->icao, cs,
               alt_color, a->altitude,
               a->velocity,
               a->heading,
               ew_buf,
               ns_buf,
               vsbuf,
               a->msg_count);
        rows++;
    }

    /* pad to 10 rows so log stays anchored */
    for (; rows < 10; rows++)
        printf("  " FG_GRAY "        ---\n" RESET);

    printf(FG_CYAN
           "  ────────────────────────────────────────────────────────────────\n"
           RESET);

    /* event log */
    printf(FG_GRAY BOLD "  event log\n" RESET);
    static const char *log_colors[] = {
        FG_WHITE, FG_GREEN, FG_YELLOW, FG_CYAN, FG_RED
    };
    for (int i = LOG_LINES - 1; i >= 0; i--) {
        int idx = (s_log_head - 1 - i + LOG_LINES * 2) % LOG_LINES;
        if (s_log[idx].text[0])
            printf("  %s%-70s" RESET "\n",
                   log_colors[s_log[idx].color], s_log[idx].text);
        else
            printf("  " FG_GRAY "~\n" RESET);
    }

    printf(FG_CYAN
           "  ────────────────────────────────────────────────────────────────\n"
           RESET FG_GRAY
           "  R828D tuner  |  Rafael Micro  |  ctrl+] to exit monitor\n"
           RESET);

    fflush(stdout);
}

/* ── on_msg ──────────────────────────────────────────────────────────────── */
void on_msg(mode_s_t *self, struct mode_s_msg *mm)
{
    s_msg_count++;
    s_msg_bucket++;

    uint32_t icao = ((uint32_t)mm->aa1 << 16) |
                    ((uint32_t)mm->aa2 <<  8) |
                     (uint32_t)mm->aa3;

    if (!mm->crcok) {
        s_crc_err++;
        tui_log(4, "CRC  %06lX  df=%d", (unsigned long)icao, mm->msgtype);
        tui_draw();
        return;
    }
    s_good++;

    aircraft_t *a = find_or_create(icao);
    if (!a) { tui_draw(); return; }

    a->last_seen_us = esp_timer_get_time();
    a->msg_count++;

    /* callsign — metype 1-4 */
    if (mm->flight[0]) {
        strncpy(a->callsign, mm->flight, 8);
        a->callsign[8] = '\0';
        for (int i = 7; i >= 0 && a->callsign[i] == ' '; i--)
            a->callsign[i] = '\0';
        tui_log(2, "IDNT %06lX  %-8s", (unsigned long)icao, a->callsign);
    }

    /* altitude */
    if (mm->altitude)
        a->altitude = mm->altitude;

    /* heading */
    if (mm->heading_is_valid)
        a->heading = mm->heading;

    /* velocity — metype 19 */
    if (mm->velocity)
        a->velocity = mm->velocity;

    /* EW / NS components */
    if (mm->ew_velocity) {
        a->ew_velocity = mm->ew_dir ? -mm->ew_velocity : mm->ew_velocity;
    }
    if (mm->ns_velocity) {
        a->ns_velocity = mm->ns_dir ? -mm->ns_velocity : mm->ns_velocity;
    }

    /* vertical rate */
    if (mm->vert_rate) {
        a->vert_rate = mm->vert_rate_sign ? -mm->vert_rate : mm->vert_rate;
    }

    /* raw CPR position */
    if (mm->raw_latitude)  a->raw_lat = mm->raw_latitude;
    if (mm->raw_longitude) a->raw_lon = mm->raw_longitude;

    /* log notable events */
    if (mm->msgtype == 17) {
        if (mm->metype >= 9 && mm->metype <= 18)
            tui_log(0, "ALRT %06lX  alt=%d ft", (unsigned long)icao, a->altitude);
        else if (mm->metype >= 19 && mm->metype <= 22)
            tui_log(3, "VELO %06lX  spd=%d kt  hdg=%d  vs=%d",
                    (unsigned long)icao, a->velocity, a->heading, a->vert_rate);
    }

    tui_draw();
}

/* ── demodulate ──────────────────────────────────────────────────────────── */
void demodulate(uint8_t *source, int length)
{
    if (!source || length <= 0) return;

    uint16_t *mag = malloc((length / 2) * sizeof(uint16_t));
    if (!mag) {
        tui_log(4, "OOM  magnitude buffer alloc failed");
        return;
    }
    mode_s_compute_magnitude_vector(source, mag, length);
    mode_s_detect(&state, mag, length / 2, on_msg);
    free(mag);
}

/* ── adsb_rx_task ────────────────────────────────────────────────────────── */
void adsb_rx_task(void *arg)
{
    s_start_us = esp_timer_get_time();
    s_rate_ts  = s_start_us;

    tui_log(1, "INIT  adsb_rx_task started on CPU1");
    tui_log(1, "INIT  R828D locked  1090.000 MHz  2 MSPS");

    uint8_t *buffer = malloc(DEFAULT_BUF_LENGTH);
    if (!buffer) {
        tui_log(4, "OOM  rx buffer alloc failed");
        vTaskDelete(NULL);
        return;
    }

    int  n_read      = 0;
    bool full_buffer = false;
    mode_s_init(&state);

    tui_draw();

    while (true) {
        full_buffer = true;
        for (int i = 0; i < DEFAULT_BUF_LENGTH; i += MAX_PACKET_SIZE) {
            int r = rtlsdr_read_sync(rtldev, &buffer[i], MAX_PACKET_SIZE, &n_read);
            if (r < 0) {
                tui_log(4, "READ  sync failed  r=%d", r);
                full_buffer = false;
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            if ((uint32_t)n_read < MAX_PACKET_SIZE) {
                tui_log(3, "READ  short read  got=%d", n_read);
                full_buffer = false;
                break;
            }
        }

        if (full_buffer)
            demodulate(buffer, DEFAULT_BUF_LENGTH);

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    free(buffer);
    vTaskDelete(NULL);
}

/* ── rtlsdr_setup_task ───────────────────────────────────────────────────── */
static void rtlsdr_setup_task(void *arg)
{
    uint8_t dev_addr = (uint8_t)(uint32_t)arg;

    int r = rtlsdr_open(&rtldev, dev_addr, s_driver_obj->constant.client_hdl);
    if (r < 0) {
        ESP_LOGE(TAG, "rtlsdr_open failed");
        vTaskDelete(NULL);
        return;
    }

    rtlsdr_set_center_freq(rtldev, 1090000000);
    rtlsdr_set_sample_rate(rtldev, 2000000);
    rtlsdr_set_tuner_gain_mode(rtldev, 0);
    rtlsdr_reset_buffer(rtldev);

    xTaskCreatePinnedToCore(adsb_rx_task, "adsb_rx", 16384, NULL, 5, NULL, 1);
    vTaskDelete(NULL);
}

/* ── client_event_cb ─────────────────────────────────────────────────────── */
static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
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
    default:
        abort();
    }
}

/* ── action handlers ─────────────────────────────────────────────────────── */
static void action_open_dev(usb_device_t *device_obj)
{
    assert(device_obj->dev_addr != 0);
    ESP_LOGI(TAG, "Opening device at address %d", device_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(device_obj->client_hdl,
                                         device_obj->dev_addr,
                                         &device_obj->dev_hdl));
    device_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));
    if (dev_info.parent.dev_hdl) {
        usb_device_info_t p;
        ESP_ERROR_CHECK(usb_host_device_info(dev_info.parent.dev_hdl, &p));
    }
    device_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(device_obj->dev_hdl, &dev_desc));
    device_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(device_obj->dev_hdl,
                                                           &config_desc));
    device_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(usb_device_t *device_obj)
{
    assert(device_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(device_obj->dev_hdl, &dev_info));
}

static void action_close_dev(usb_device_t *device_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(device_obj->client_hdl,
                                           device_obj->dev_hdl));
    device_obj->dev_hdl  = NULL;
    device_obj->dev_addr = 0;
}

static void class_driver_device_handle(usb_device_t *device_obj)
{
    uint8_t actions = device_obj->actions;
    device_obj->actions = 0;
    while (actions) {
        if (actions & ACTION_OPEN_DEV)        action_open_dev(device_obj);
        if (actions & ACTION_GET_DEV_INFO)    action_get_info(device_obj);
        if (actions & ACTION_GET_DEV_DESC)    action_get_dev_desc(device_obj);
        if (actions & ACTION_GET_CONFIG_DESC) action_get_config_desc(device_obj);
        if (actions & ACTION_GET_STR_DESC)    action_get_str_desc(device_obj);
        if (actions & ACTION_CLOSE_DEV)       action_close_dev(device_obj);
        actions = device_obj->actions;
        device_obj->actions = 0;
    }
}

/* ── class_driver_task ───────────────────────────────────────────────────── */
void class_driver_task(void *arg)
{
    class_driver_t           driver_obj             = {0};
    usb_host_client_handle_t class_driver_client_hdl = NULL;

    ESP_LOGI(TAG, "Registering Client");

    SemaphoreHandle_t mux_lock = xSemaphoreCreateMutex();
    if (!mux_lock) {
        ESP_LOGE(TAG, "Unable to create class driver mutex");
        vTaskSuspend(NULL);
        return;
    }

    usb_host_client_config_t client_config = {
        .is_synchronous    = false,
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg          = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config,
                                              &class_driver_client_hdl));

    driver_obj.constant.mux_lock   = mux_lock;
    driver_obj.constant.client_hdl = class_driver_client_hdl;

    for (uint8_t i = 0; i < DEV_MAX_COUNT; i++)
        driver_obj.mux_protected.device[i].client_hdl = class_driver_client_hdl;

    s_driver_obj = &driver_obj;

    while (1) {
        if (driver_obj.mux_protected.flags.unhandled_devices) {
            xSemaphoreTake(driver_obj.constant.mux_lock, portMAX_DELAY);
            for (uint8_t i = 0; i < DEV_MAX_COUNT; i++) {
                if (driver_obj.mux_protected.device[i].actions)
                    class_driver_device_handle(
                        &driver_obj.mux_protected.device[i]);
            }
            driver_obj.mux_protected.flags.unhandled_devices = 0;
            xSemaphoreGive(driver_obj.constant.mux_lock);
        } else {
            if (!driver_obj.mux_protected.flags.shutdown)
                usb_host_client_handle_events(class_driver_client_hdl,
                                              portMAX_DELAY);
            else
                break;
        }
    }

    ESP_LOGI(TAG, "Deregistering Class Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(class_driver_client_hdl));
    if (mux_lock) vSemaphoreDelete(mux_lock);
    vTaskSuspend(NULL);
}

/* ── class_driver_client_deregister ─────────────────────────────────────── */
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