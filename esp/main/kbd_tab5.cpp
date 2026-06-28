#ifdef USE_LCD_TAB5
/*
 * Wrapper around the M5 Tab5 external I2C keyboard component, translating
 * its HID-mode events into tiny386's generic PS/2 keycode injection API
 * (ps2_put_keycode(), expects Linux input layer KEY_* codes - see i8042.h).
 *
 * HID mode only reports a single "current" regular key + a modifier
 * bitmask per event, and signals "all keys released" with hid_key_code==0
 * (it never reports which specific key went up). So this wrapper tracks
 * the previously-held regular key and modifier bitmask itself and
 * synthesizes the missing up/down transitions by diffing against the new
 * state - same simplification the picocalc reference port makes.
 */

#include "m5_tab5_keyboard.h"
#include "board_tab5.h"

extern "C" {
typedef struct PS2KbdState PS2KbdState;
void ps2_put_keycode(PS2KbdState *s, int is_down, int keycode);
}

#include "common.h"

/* Linux input-event-codes.h values (avoid pulling in the kernel header). */
#define KEY_ESC        1
#define KEY_1          2
#define KEY_2          3
#define KEY_3          4
#define KEY_4          5
#define KEY_5          6
#define KEY_6          7
#define KEY_7          8
#define KEY_8          9
#define KEY_9          10
#define KEY_0          11
#define KEY_MINUS      12
#define KEY_EQUAL      13
#define KEY_BACKSPACE  14
#define KEY_TAB        15
#define KEY_Q          16
#define KEY_W          17
#define KEY_E          18
#define KEY_R          19
#define KEY_T          20
#define KEY_Y          21
#define KEY_U          22
#define KEY_I          23
#define KEY_O          24
#define KEY_P          25
#define KEY_LEFTBRACE  26
#define KEY_RIGHTBRACE 27
#define KEY_ENTER      28
#define KEY_LEFTCTRL   29
#define KEY_A          30
#define KEY_S          31
#define KEY_D          32
#define KEY_F          33
#define KEY_G          34
#define KEY_H          35
#define KEY_J          36
#define KEY_K          37
#define KEY_L          38
#define KEY_SEMICOLON  39
#define KEY_APOSTROPHE 40
#define KEY_GRAVE      41
#define KEY_LEFTSHIFT  42
#define KEY_BACKSLASH  43
#define KEY_Z          44
#define KEY_X          45
#define KEY_C          46
#define KEY_V          47
#define KEY_B          48
#define KEY_N          49
#define KEY_M          50
#define KEY_COMMA      51
#define KEY_DOT        52
#define KEY_SLASH      53
#define KEY_RIGHTSHIFT 54
#define KEY_KPASTERISK 55
#define KEY_LEFTALT    56
#define KEY_SPACE      57
#define KEY_CAPSLOCK   58
#define KEY_F1         59
#define KEY_F2         60
#define KEY_F3         61
#define KEY_F4         62
#define KEY_F5         63
#define KEY_F6         64
#define KEY_F7         65
#define KEY_F8         66
#define KEY_F9         67
#define KEY_F10        68
#define KEY_F11        87
#define KEY_F12        88
#define KEY_RIGHTCTRL  97
#define KEY_RIGHTALT   100
#define KEY_HOME       102
#define KEY_UP         103
#define KEY_PAGEUP     104
#define KEY_LEFT       105
#define KEY_RIGHT      106
#define KEY_END        107
#define KEY_DOWN       108
#define KEY_PAGEDOWN   109
#define KEY_INSERT     110
#define KEY_DELETE     111
#define KEY_LEFTMETA   125
#define KEY_RIGHTMETA  126

/* USB HID Usage Page 0x07 (Keyboard/Keypad) -> Linux KEY_*. Covers the
 * whole "boot protocol" keyboard range (0x04-0x65); returns 0 (ignored)
 * for anything unmapped. A switch instead of a designated-initializer
 * array because g++ doesn't support sparse array designated initializers. */
static int hid_to_linux_key(uint8_t hid)
{
	switch (hid) {
	case 0x04: return KEY_A;         case 0x05: return KEY_B;
	case 0x06: return KEY_C;         case 0x07: return KEY_D;
	case 0x08: return KEY_E;         case 0x09: return KEY_F;
	case 0x0A: return KEY_G;         case 0x0B: return KEY_H;
	case 0x0C: return KEY_I;         case 0x0D: return KEY_J;
	case 0x0E: return KEY_K;         case 0x0F: return KEY_L;
	case 0x10: return KEY_M;         case 0x11: return KEY_N;
	case 0x12: return KEY_O;         case 0x13: return KEY_P;
	case 0x14: return KEY_Q;         case 0x15: return KEY_R;
	case 0x16: return KEY_S;         case 0x17: return KEY_T;
	case 0x18: return KEY_U;         case 0x19: return KEY_V;
	case 0x1A: return KEY_W;         case 0x1B: return KEY_X;
	case 0x1C: return KEY_Y;         case 0x1D: return KEY_Z;
	case 0x1E: return KEY_1;         case 0x1F: return KEY_2;
	case 0x20: return KEY_3;         case 0x21: return KEY_4;
	case 0x22: return KEY_5;         case 0x23: return KEY_6;
	case 0x24: return KEY_7;         case 0x25: return KEY_8;
	case 0x26: return KEY_9;         case 0x27: return KEY_0;
	case 0x28: return KEY_ENTER;     case 0x29: return KEY_ESC;
	case 0x2A: return KEY_BACKSPACE; case 0x2B: return KEY_TAB;
	case 0x2C: return KEY_SPACE;     case 0x2D: return KEY_MINUS;
	case 0x2E: return KEY_EQUAL;     case 0x2F: return KEY_LEFTBRACE;
	case 0x30: return KEY_RIGHTBRACE;
	case 0x31: return KEY_BACKSLASH; case 0x33: return KEY_SEMICOLON;
	case 0x34: return KEY_APOSTROPHE;
	case 0x35: return KEY_GRAVE;     case 0x36: return KEY_COMMA;
	case 0x37: return KEY_DOT;       case 0x38: return KEY_SLASH;
	case 0x39: return KEY_CAPSLOCK;
	case 0x3A: return KEY_F1;        case 0x3B: return KEY_F2;
	case 0x3C: return KEY_F3;        case 0x3D: return KEY_F4;
	case 0x3E: return KEY_F5;        case 0x3F: return KEY_F6;
	case 0x40: return KEY_F7;        case 0x41: return KEY_F8;
	case 0x42: return KEY_F9;        case 0x43: return KEY_F10;
	case 0x44: return KEY_F11;       case 0x45: return KEY_F12;
	case 0x49: return KEY_INSERT;    case 0x4A: return KEY_HOME;
	case 0x4B: return KEY_PAGEUP;    case 0x4C: return KEY_DELETE;
	case 0x4D: return KEY_END;       case 0x4E: return KEY_PAGEDOWN;
	case 0x4F: return KEY_RIGHT;     case 0x50: return KEY_LEFT;
	case 0x51: return KEY_DOWN;      case 0x52: return KEY_UP;
	case 0x55: return KEY_KPASTERISK;
	case 0x58: return KEY_ENTER;
	default: return 0;
	}
}

/* USB HID modifier bitmask (byte 0 of the boot keyboard report) -> KEY_*. */
static const int modifier_keys[8] = {
	KEY_LEFTCTRL, KEY_LEFTSHIFT, KEY_LEFTALT, KEY_LEFTMETA,
	KEY_RIGHTCTRL, KEY_RIGHTSHIFT, KEY_RIGHTALT, KEY_RIGHTMETA,
};

static m5::M5Tab5Keyboard kbd;

static volatile uint8_t s_modifier;
static volatile uint8_t s_keycode;
static volatile int s_ready;

static void on_key(m5_tab5_key_event_t ev, void *)
{
	/* HID mode never sets ev.pressed; hid_key_code==0 means "no key
	 * currently held" (release-all), not which specific key went up. */
	s_modifier = ev.hid_modifier;
	s_keycode = ev.hid_key_code;
	s_ready = 1;
}

extern "C" void tab5_kbd_init(void)
{
	kbd.begin((i2c_port_t) TAB5_KBD_PORT, M5_TAB5_KB_DEFAULT_ADDR,
		  TAB5_KBD_SDA, TAB5_KBD_SCL, M5_TAB5_KB_I2C_FREQ_100K,
		  TAB5_KBD_INT, M5_TAB5_KB_INT_MODE_HARDWARE);
	kbd.enableHIDMode(on_key, nullptr);
}

extern "C" int tab5_kbd_poll(void)
{
	static uint8_t held_modifier;
	static uint8_t held_keycode;

	if (!s_ready)
		return 0;
	s_ready = 0;

	PS2KbdState *kbd_state = (PS2KbdState *) globals.kbd;
	uint8_t new_modifier = s_modifier;
	uint8_t new_keycode = s_keycode;

	/* Diff the modifier bitmask, emit up/down for whatever changed. */
	for (int i = 0; i < 8; i++) {
		int bit = 1 << i;
		if ((new_modifier & bit) != (held_modifier & bit))
			ps2_put_keycode(kbd_state, (new_modifier & bit) != 0, modifier_keys[i]);
	}
	held_modifier = new_modifier;

	if (new_keycode != held_keycode) {
		int old_key = hid_to_linux_key(held_keycode);
		int new_key = hid_to_linux_key(new_keycode);
		if (held_keycode != 0 && old_key)
			ps2_put_keycode(kbd_state, 0, old_key);
		if (new_keycode != 0 && new_key)
			ps2_put_keycode(kbd_state, 1, new_key);
		held_keycode = new_keycode;
	}
	return 1;
}
#endif /* USE_LCD_TAB5 */
