// M5Stack Tab5 (ESP32-P4) - MIPI-DSI display (ST7121/ST7123), I2C touch,
// external M5 I2C keyboard. Requires esp-idf v6.0.x (same as jc4880p443).
#define BUILD_ESP32

// MAX PSRAM is 32MB, allocate 28 MB for system and reserve max 2M for video memory
#define PSRAM_ALLOC_LEN ((30 * 1024 * 1024) - (2 * 1024 * 1024))

#define IRAM_ATTR_CPU_EXEC1

#define BPP 16
#define FULL_UPDATE
#define SWAPXY
#define USE_LCD_TAB5

/* Logical/landscape dimensions as seen by the guest - the panel is
 * 720x1280 portrait-native; vga.c rotates pixels in software (SWAPXY)
 * before they ever reach lcd_draw(). */
#define LCD_WIDTH  1280
#define LCD_HEIGHT 720

/* Native panel resolution (portrait) - used by lcd_tab5.c for the DSI/DPI
 * panel config and by the touch driver's x_max/y_max. */
#define TAB5_LCD_H_RES 720
#define TAB5_LCD_V_RES 1280

/* SD card (SDMMC 4-bit) - same pins/LDO channel as jc4880p443 */
#define SD_CLK 43
#define SD_CMD 44
#define SD_D0 39
#define SD_D1 40
#define SD_D2 41
#define SD_D3 42
#define SD_PWR_CTRL_LDO_IO_ID 4

#define USE_HOSTED_WIFI

/* No audio: ES8388 codec isn't supported by i2s.c (only ES8311) - leaving
 * I2S_MCLK/USE_ES8311 undefined makes i2s_main() a no-op. */

/* Internal system I2C bus (IO-expanders, touch) */
#define TAB5_I2C_SDA  31
#define TAB5_I2C_SCL  32
#define TAB5_I2C_PORT 0

/* Touch (ST7123 built-in, on the system I2C bus, addr 0x55) */
#define TAB5_TOUCH_INT 23

/* External M5 keyboard I2C bus (separate from system I2C) */
#define TAB5_KBD_SDA  0
#define TAB5_KBD_SCL  1
#define TAB5_KBD_PORT 1
#define TAB5_KBD_INT  50

/* Backlight PWM */
#define TAB5_LCD_BACKLIGHT_GPIO 22
