#ifdef USE_LCD_TAB5
/*
 * LCD + touch driver for the M5Stack Tab5 (ESP32-P4).
 *
 * Display: ST7121 panel (reports itself as ST7123) via MIPI-DSI, 2 lanes
 * @965Mbps, 70MHz DPI clock, 720x1280 portrait-native. tiny386 treats it
 * as a 1280x720 landscape framebuffer (LCD_WIDTH/LCD_HEIGHT) - the 90°
 * rotation is done in software by vga.c (SWAPXY), not by this driver.
 *
 * Touch: ST7123 integrated touch controller, I2C addr 0x55 on the system
 * I2C bus, polled (not interrupt-driven) like the elecrow7s3 GT911 driver.
 *
 * Critical quirk (from the picocalc reference port): on cold boot, two
 * PI4IOE5V6416 I2C IO-expanders (addr 0x43/0x44) reset to all-input/
 * high-impedance, which holds LCD_RST and TP_RST low. Without bringing
 * them up and pulsing those two lines, the DSI panel never responds and
 * panel init hangs forever (IDLE0 watchdog reset). This must happen
 * before the DSI bus is created.
 */

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7121.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_st7123.h"
#include "esp_ldo_regulator.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"

#include "common.h"

static const char *TAG = "lcd_tab5";

/* ---------- IO-expander init (PI4IOE5V6416 x2, system I2C bus) -------- */

#define PI4IOE1_I2C_ADDR 0x43
#define PI4IOE2_I2C_ADDR 0x44

#define PI4IOE_REG_CHIP_RESET 0x01
#define PI4IOE_REG_IO_DIR     0x03
#define PI4IOE_REG_OUT_SET    0x05
#define PI4IOE_REG_OUT_H_IM   0x07
#define PI4IOE_REG_IN_DEF_STA 0x09
#define PI4IOE_REG_PULL_EN    0x0B
#define PI4IOE_REG_PULL_SEL   0x0D
#define PI4IOE_REG_INT_MASK   0x11

#define PI4IOE_LCD_RST_BIT 4 /* PI4IOE1 P4 */
#define PI4IOE_TP_RST_BIT  5 /* PI4IOE1 P5 */

static i2c_master_bus_handle_t sys_i2c_handle;

static esp_err_t pi4ioe_write(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
	uint8_t buf[2] = { reg, val };
	return i2c_master_transmit(dev, buf, 2, 50);
}

static esp_err_t tab5_sys_i2c_init(void)
{
	if (sys_i2c_handle)
		return ESP_OK;
	i2c_master_bus_config_t cfg = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.sda_io_num = TAB5_I2C_SDA,
		.scl_io_num = TAB5_I2C_SCL,
		.i2c_port = TAB5_I2C_PORT,
		.flags.enable_internal_pullup = true,
	};
	return i2c_new_master_bus(&cfg, &sys_i2c_handle);
}

static esp_err_t tab5_io_expander_init(void)
{
	ESP_RETURN_ON_ERROR(tab5_sys_i2c_init(), TAG, "sys i2c init failed");

	uint8_t reg, val;
	i2c_master_dev_handle_t dev = NULL;

	/* PI4IOE1 (0x43): SPK_EN, EXT5V_EN, LCD_RST, TP_RST, CAM_RST */
	i2c_device_config_t dev_cfg1 = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = PI4IOE1_I2C_ADDR,
		.scl_speed_hz = 400000,
	};
	ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(sys_i2c_handle, &dev_cfg1, &dev),
			     TAG, "PI4IOE1 add device failed");

	pi4ioe_write(dev, PI4IOE_REG_CHIP_RESET, 0xFF);
	reg = PI4IOE_REG_CHIP_RESET;
	i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 50);
	pi4ioe_write(dev, PI4IOE_REG_IO_DIR, 0b01111111);
	pi4ioe_write(dev, PI4IOE_REG_OUT_H_IM, 0b00000000);
	pi4ioe_write(dev, PI4IOE_REG_PULL_SEL, 0b01111111);
	pi4ioe_write(dev, PI4IOE_REG_PULL_EN, 0b01111111);
	/* P1=SPK_EN, P2=EXT5V_EN, P4=LCD_RST, P5=TP_RST, P6=CAM_RST -> high */
	pi4ioe_write(dev, PI4IOE_REG_OUT_SET, 0b01110110);

	i2c_master_bus_rm_device(dev);

	/* PI4IOE2 (0x44): WLAN_PWR_EN, USB5V_EN, CHG_EN, power-off line */
	i2c_device_config_t dev_cfg2 = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = PI4IOE2_I2C_ADDR,
		.scl_speed_hz = 400000,
	};
	dev = NULL;
	ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(sys_i2c_handle, &dev_cfg2, &dev),
			     TAG, "PI4IOE2 add device failed");

	pi4ioe_write(dev, PI4IOE_REG_CHIP_RESET, 0xFF);
	reg = PI4IOE_REG_CHIP_RESET;
	i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 50);
	pi4ioe_write(dev, PI4IOE_REG_IO_DIR, 0b10111001);
	pi4ioe_write(dev, PI4IOE_REG_OUT_H_IM, 0b00000110);
	pi4ioe_write(dev, PI4IOE_REG_PULL_SEL, 0b10111001);
	pi4ioe_write(dev, PI4IOE_REG_PULL_EN, 0b11111001);
	pi4ioe_write(dev, PI4IOE_REG_IN_DEF_STA, 0b01000000);
	pi4ioe_write(dev, PI4IOE_REG_INT_MASK, 0b10111111);
	pi4ioe_write(dev, PI4IOE_REG_OUT_SET, 0b10001001);

	i2c_master_bus_rm_device(dev);
	return ESP_OK;
}

static esp_err_t tab5_reset_tp(void)
{
	ESP_RETURN_ON_ERROR(tab5_sys_i2c_init(), TAG, "sys i2c init failed");

	gpio_reset_pin((gpio_num_t) TAB5_TOUCH_INT);

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = PI4IOE1_I2C_ADDR,
		.scl_speed_hz = 400000,
	};
	i2c_master_dev_handle_t dev = NULL;
	ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(sys_i2c_handle, &dev_cfg, &dev),
			     TAG, "PI4IOE1 add device failed");

	uint8_t reg = PI4IOE_REG_OUT_SET;
	uint8_t val = 0;
	i2c_master_transmit_receive(dev, &reg, 1, &val, 1, 50);

	pi4ioe_write(dev, PI4IOE_REG_OUT_SET,
		     (uint8_t) (val & ~((1 << PI4IOE_LCD_RST_BIT) | (1 << PI4IOE_TP_RST_BIT))));
	vTaskDelay(pdMS_TO_TICKS(100));

	pi4ioe_write(dev, PI4IOE_REG_OUT_SET,
		     (uint8_t) (val | (1 << PI4IOE_LCD_RST_BIT) | (1 << PI4IOE_TP_RST_BIT)));
	vTaskDelay(pdMS_TO_TICKS(100));

	i2c_master_bus_rm_device(dev);
	return ESP_OK;
}

/* ---------- DSI PHY power --------------------------------------------- */

#define TAB5_DSI_PHY_LDO_CHAN 3
#define TAB5_DSI_PHY_LDO_MV   2500

static esp_err_t enable_dsi_phy_power(void)
{
	static esp_ldo_channel_handle_t chan = NULL;
	if (chan)
		return ESP_OK;
	esp_ldo_channel_config_t ldo_cfg = {
		.chan_id = TAB5_DSI_PHY_LDO_CHAN,
		.voltage_mv = TAB5_DSI_PHY_LDO_MV,
	};
	return esp_ldo_acquire_channel(&ldo_cfg, &chan);
}

/* ---------- backlight (LEDC PWM) -------------------------------------- */

static void backlight_on(void)
{
	const ledc_timer_config_t timer_cfg = {
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.duty_resolution = LEDC_TIMER_12_BIT,
		.timer_num = LEDC_TIMER_0,
		.freq_hz = 5000,
		.clk_cfg = LEDC_AUTO_CLK,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

	const ledc_channel_config_t ch_cfg = {
		.gpio_num = TAB5_LCD_BACKLIGHT_GPIO,
		.speed_mode = LEDC_LOW_SPEED_MODE,
		.channel = LEDC_CHANNEL_1,
		.intr_type = LEDC_INTR_DISABLE,
		.timer_sel = LEDC_TIMER_0,
		.duty = 4095,
		.hpoint = 0,
	};
	ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
}

/* ---------- lcd_draw() / mouse helpers --------------------------------- */

void pc_vga_step(void *o);

typedef struct PS2MouseState PS2MouseState;
void ps2_mouse_event(PS2MouseState *s, int dx, int dy, int dz, int buttons_state);

void lcd_draw(int x_start, int y_start, int x_end, int y_end, void *src)
{
	if (globals.panel) {
		ESP_ERROR_CHECK(
			esp_lcd_panel_draw_bitmap(globals.panel, x_start, y_start, x_end, y_end, src));
	}
}

/* ---------- touch polling / gesture-to-mouse bridge -------------------- */

#define TAP_MAX_MS 200
#define TAP_MAX_PX 10

static void touch_poll(esp_lcd_touch_handle_t tp)
{
	static bool was_touching = false;
	static int16_t prev_x, prev_y;
	static TickType_t touch_start_tick;
	static int16_t start_x, start_y;
	static bool was_two_finger = false;

	esp_lcd_touch_point_data_t pts[5];
	uint8_t cnt = 0;

	esp_lcd_touch_read_data(tp);
	if (esp_lcd_touch_get_data(tp, pts, &cnt, 5) != ESP_OK)
		cnt = 0;

	/* native portrait (px,py) -> logical landscape (lx,ly):
	 * lx = LCD_WIDTH - 1 - py, ly = px (90 deg rotation, matches the
	 * SWAPXY transform vga.c applies to the framebuffer). */
	int16_t x0 = 0, y0 = 0;
	if (cnt >= 1) {
		x0 = (int16_t) (LCD_WIDTH - 1 - pts[0].y);
		y0 = (int16_t) pts[0].x;
	}

	if (cnt >= 2)
		was_two_finger = true;

	if (cnt >= 1) {
		if (was_touching) {
			int dx = (int) x0 - (int) prev_x;
			int dy = (int) y0 - (int) prev_y;
			if (dx != 0 || dy != 0)
				ps2_mouse_event(globals.mouse, dx, dy, 0, 0);
		} else {
			touch_start_tick = xTaskGetTickCount();
			start_x = x0;
			start_y = y0;
			was_two_finger = (cnt >= 2);
		}
		prev_x = x0;
		prev_y = y0;
		was_touching = true;
	} else if (was_touching) {
		TickType_t dur = xTaskGetTickCount() - touch_start_tick;
		int move_x = abs((int) prev_x - (int) start_x);
		int move_y = abs((int) prev_y - (int) start_y);

		if (pdTICKS_TO_MS(dur) < TAP_MAX_MS && move_x < TAP_MAX_PX && move_y < TAP_MAX_PX) {
			if (was_two_finger) {
				ps2_mouse_event(globals.mouse, 0, 0, 0, 2);
				ps2_mouse_event(globals.mouse, 0, 0, 0, 0);
			} else {
				ps2_mouse_event(globals.mouse, 0, 0, 0, 1);
				ps2_mouse_event(globals.mouse, 0, 0, 0, 0);
			}
		}
		was_touching = false;
		was_two_finger = false;
	}
}

static esp_lcd_touch_handle_t touch_init(void)
{
	esp_lcd_panel_io_handle_t io_handle = NULL;
	esp_lcd_panel_io_i2c_config_t tp_io_cfg = {
		.dev_addr = 0x55,
		.control_phase_bytes = 1,
		.dc_bit_offset = 0,
		.lcd_cmd_bits = 16,
		.lcd_param_bits = 0,
		.flags.disable_control_phase = 1,
		.scl_speed_hz = 100000,
	};

	esp_err_t err = esp_lcd_new_panel_io_i2c(sys_i2c_handle, &tp_io_cfg, &io_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "touch I2C IO init failed: %s", esp_err_to_name(err));
		return NULL;
	}

	esp_lcd_touch_config_t tp_cfg = {
		.x_max = TAB5_LCD_H_RES,
		.y_max = TAB5_LCD_V_RES,
		.rst_gpio_num = GPIO_NUM_NC,
		.int_gpio_num = (gpio_num_t) TAB5_TOUCH_INT,
	};

	esp_lcd_touch_handle_t tp = NULL;
	err = esp_lcd_touch_new_i2c_st7123(io_handle, &tp_cfg, &tp);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "ST7123 touch init failed: %s", esp_err_to_name(err));
		esp_lcd_panel_io_del(io_handle);
		return NULL;
	}

	ESP_LOGI(TAG, "touch ready (ST7123, portrait %dx%d)", TAB5_LCD_H_RES, TAB5_LCD_V_RES);
	return tp;
}

/* ---------- main display + input task ---------------------------------- */

void tab5_kbd_init(void);
int tab5_kbd_poll(void);

void vga_task(void *arg)
{
	int core_id = esp_cpu_get_core_id();
	fprintf(stderr, "vga runs on core %d\n", core_id);

	ESP_LOGI(TAG, "Init IO expanders / panel reset");
	ESP_ERROR_CHECK(tab5_io_expander_init());
	ESP_ERROR_CHECK(tab5_reset_tp());

	ESP_LOGI(TAG, "Init display");
	ESP_ERROR_CHECK(enable_dsi_phy_power());

	esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
	esp_lcd_dsi_bus_config_t bus_cfg = {
		.bus_id = 0,
		.num_data_lanes = 2,
		.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
		.lane_bit_rate_mbps = 965,
	};
	ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_cfg, &mipi_dsi_bus));
	vTaskDelay(pdMS_TO_TICKS(50));

	esp_lcd_panel_io_handle_t io;
	esp_lcd_dbi_io_config_t dbi_cfg = {
		.virtual_channel = 0,
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8,
	};
	ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_cfg, &io));

	esp_lcd_dpi_panel_config_t dpi_cfg = {
		.virtual_channel = 0,
		.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
		.dpi_clock_freq_mhz = 70,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
		.in_color_format = LCD_COLOR_FMT_RGB565,
#else
		.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
#endif
		.num_fbs = 1,
		.video_timing = {
			.h_size = TAB5_LCD_H_RES,
			.v_size = TAB5_LCD_V_RES,
			.hsync_pulse_width = 2,
			.hsync_back_porch = 40,
			.hsync_front_porch = 40,
			.vsync_pulse_width = 20,
			.vsync_back_porch = 24,
			.vsync_front_porch = 200,
		},
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(6, 0, 0)
		.flags = { .use_dma2d = true },
#endif
	};

	st7121_vendor_config_t vendor_cfg = {
		.mipi_config = {
			.dsi_bus = mipi_dsi_bus,
			.dpi_config = &dpi_cfg,
		},
	};

	esp_lcd_panel_dev_config_t dev_cfg = {
		.reset_gpio_num = -1, /* reset already pulsed via I2C IO-expander */
		.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
		.bits_per_pixel = 16,
		.vendor_config = &vendor_cfg,
	};

	esp_lcd_panel_handle_t panel = NULL;
	ESP_ERROR_CHECK(esp_lcd_new_panel_st7121(io, &dev_cfg, &panel));
	ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
	ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
	ESP_ERROR_CHECK(esp_lcd_dpi_panel_enable_dma2d(panel));
#endif
	ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

	backlight_on();

	globals.panel = panel;
	xEventGroupSetBits(global_event_group, BIT1);
	xEventGroupWaitBits(global_event_group, BIT0, pdFALSE, pdFALSE, portMAX_DELAY);

	ESP_LOGI(TAG, "Init touch");
	esp_lcd_touch_handle_t tp = touch_init();

	ESP_LOGI(TAG, "Init keyboard");
	tab5_kbd_init();

	while (1) {
		pc_vga_step(globals.pc);
		if (tp)
			touch_poll(tp);
		tab5_kbd_poll();
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}
#endif /* USE_LCD_TAB5 */
