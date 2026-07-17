/*
 * Internal VGA state and declarations shared between vga.c (common VGA
 * logic), vga_bochs.c (Bochs VBE extension) and vga_cirrus.c (Cirrus
 * Logic GD5430 extension). Not part of the public API - see vga.h for
 * that.
 */
#ifndef VGA_INTERNAL_H
#define VGA_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#include "vga.h"

#ifdef BUILD_ESP32
#include "esp_attr.h"
void *pcmalloc(long size);
#define RETRACE_INTERVAL_US 5000
#else
#define IRAM_ATTR
#define pcmalloc malloc
#define RETRACE_INTERVAL_US 15000
#endif

//#define DEBUG_VBE
//#define DEBUG_VGA_REG
//#define DEBUG_CIRRUS
/* DEBUG_BAND: traces every VRAM store landing in the watch ranges (vga.c),
 * used to hunt the Win3.11 stale-band bug; DEBUG_BAND_PC covers the LFB
 * fast paths in pc.c. Heavy on stdout - can slow the guest enough to trip
 * Windows-internal timeouts (32-bit disk access), keep off normally. */
//#define DEBUG_BAND
//#define DEBUG_BAND_PC

#define MSR_COLOR_EMULATION 0x01
#define MSR_PAGE_SELECT     0x20

#define ST01_V_RETRACE      0x08
#define ST01_DISP_ENABLE    0x01

#define VBE_DISPI_INDEX_ID              0x0
#define VBE_DISPI_INDEX_XRES            0x1
#define VBE_DISPI_INDEX_YRES            0x2
#define VBE_DISPI_INDEX_BPP             0x3
#define VBE_DISPI_INDEX_ENABLE          0x4
#define VBE_DISPI_INDEX_BANK            0x5
#define VBE_DISPI_INDEX_VIRT_WIDTH      0x6
#define VBE_DISPI_INDEX_VIRT_HEIGHT     0x7
#define VBE_DISPI_INDEX_X_OFFSET        0x8
#define VBE_DISPI_INDEX_Y_OFFSET        0x9
#define VBE_DISPI_INDEX_VIDEO_MEMORY_64K 0xa
#define VBE_DISPI_INDEX_NB              0xb

#define VBE_DISPI_ID0                   0xB0C0
#define VBE_DISPI_ID1                   0xB0C1
#define VBE_DISPI_ID2                   0xB0C2
#define VBE_DISPI_ID3                   0xB0C3
#define VBE_DISPI_ID4                   0xB0C4
#define VBE_DISPI_ID5                   0xB0C5

#define VBE_DISPI_DISABLED              0x00
#define VBE_DISPI_ENABLED               0x01
#define VBE_DISPI_GETCAPS               0x02
#define VBE_DISPI_8BIT_DAC              0x20
#define VBE_DISPI_LFB_ENABLED           0x40
#define VBE_DISPI_NOCLEARMEM            0x80

#define FB_ALLOC_ALIGN (1 << 20)

#define MAX_TEXT_WIDTH 132
#define MAX_TEXT_HEIGHT 60

/* VGA CRT controller register indices */
#define VGA_CRTC_H_TOTAL        0
#define VGA_CRTC_H_DISP         1
#define VGA_CRTC_H_BLANK_START  2
#define VGA_CRTC_H_BLANK_END    3
#define VGA_CRTC_H_SYNC_START   4
#define VGA_CRTC_H_SYNC_END     5
#define VGA_CRTC_V_TOTAL        6
#define VGA_CRTC_OVERFLOW       7
#define VGA_CRTC_PRESET_ROW     8
#define VGA_CRTC_MAX_SCAN       9
#define VGA_CRTC_CURSOR_START   0x0A
#define VGA_CRTC_CURSOR_END     0x0B
#define VGA_CRTC_START_HI       0x0C
#define VGA_CRTC_START_LO       0x0D
#define VGA_CRTC_CURSOR_HI      0x0E
#define VGA_CRTC_CURSOR_LO      0x0F
#define VGA_CRTC_V_SYNC_START   0x10
#define VGA_CRTC_V_SYNC_END     0x11
#define VGA_CRTC_V_DISP_END     0x12
#define VGA_CRTC_OFFSET         0x13
#define VGA_CRTC_UNDERLINE      0x14
#define VGA_CRTC_V_BLANK_START  0x15
#define VGA_CRTC_V_BLANK_END    0x16
#define VGA_CRTC_MODE           0x17
#define VGA_CRTC_LINE_COMPARE   0x18
#define VGA_CRTC_REGS           VGA_CRT_C

/* VGA sequencer register indices */
#define VGA_SEQ_RESET           0x00
#define VGA_SEQ_CLOCK_MODE      0x01
#define VGA_SEQ_PLANE_WRITE     0x02
#define VGA_SEQ_CHARACTER_MAP   0x03
#define VGA_SEQ_MEMORY_MODE     0x04

/* VGA sequencer register bit masks */
#define VGA_SR01_CHAR_CLK_8DOTS 0x01 /* bit 0: character clocks 8 dots wide are generated */
#define VGA_SR01_SCREEN_OFF     0x20 /* bit 5: Screen is off */
#define VGA_SR02_ALL_PLANES     0x0F /* bits 3-0: enable access to all planes */
#define VGA_SR04_EXT_MEM        0x02 /* bit 1: allows complete mem access to 256K */
#define VGA_SR04_SEQ_MODE       0x04 /* bit 2: directs system to use a sequential addressing mode */
#define VGA_SR04_CHN_4M         0x08 /* bit 3: selects modulo 4 addressing for CPU access to display memory */

/* VGA graphics controller register indices */
#define VGA_GFX_SR_VALUE        0x00
#define VGA_GFX_SR_ENABLE       0x01
#define VGA_GFX_COMPARE_VALUE   0x02
#define VGA_GFX_DATA_ROTATE     0x03
#define VGA_GFX_PLANE_READ      0x04
#define VGA_GFX_MODE            0x05
#define VGA_GFX_MISC            0x06
#define VGA_GFX_COMPARE_MASK    0x07
#define VGA_GFX_BIT_MASK        0x08

/* VGA graphics controller bit masks */
#define VGA_GR06_GRAPHICS_MODE  0x01

struct FBDevice {
    /* the following is set by the device */
    int width;
    int height;
    int stride; /* current stride in bytes */
    uint8_t *fb_data; /* current pointer to the pixel data */
};

struct VGAState {
    FBDevice *fb_dev;
    int graphic_mode;
    uint32_t cursor_blink_time;
    int cursor_visible_phase;
    uint32_t retrace_time;
    int retrace_phase;
    int force_8dm;

    uint8_t *vga_ram;
    int vga_ram_size;

    uint8_t sr_index;
    uint8_t sr[8];
    uint8_t gr_index;
    uint8_t gr[16];
    uint8_t ar_index;
    /* Standard VGA only defines indices 0x00-0x14 (21 registers); Cirrus
     * extends the index mask to 0x1f but indices 0x15-0x1f have no
     * defined function either (vid_cl54xx.c just stores the raw byte
     * unconditionally before its index-specific switch) - sized to 32 so
     * those still round-trip instead of being silently dropped. */
    uint8_t ar[32];
    int ar_flip_flop;
    uint8_t cr_index;
    uint8_t cr[256]; /* CRT registers */
    uint8_t msr; /* Misc Output Register */
    uint8_t fcr; /* Feature Control Register */
    uint8_t st00; /* status 0 */
    uint8_t st01; /* status 1 */
    uint8_t dac_state;
    uint8_t dac_sub_index;
    uint8_t dac_read_index;
    uint8_t dac_write_index;
    uint8_t dac_8bit;
    uint8_t dac_cache[3]; /* used when writing */
    uint8_t palette[768];
    int32_t bank_offset;

    uint32_t latch;

    int comp_ntsc;

    /* text mode state */
    uint32_t last_palette[16];
#ifndef FULL_UPDATE
    uint16_t last_ch_attr[MAX_TEXT_WIDTH * MAX_TEXT_HEIGHT];
#endif
    uint32_t last_width;
    uint32_t last_height;
    uint16_t last_line_offset;
    uint16_t last_start_addr;
    uint16_t last_cursor_offset;
    uint8_t last_cursor_start;
    uint8_t last_cursor_end;

    /* VBE extension */
    uint16_t vbe_index;
    uint16_t vbe_regs[VBE_DISPI_INDEX_NB];
    uint32_t vbe_start_addr;
    uint32_t vbe_line_offset;

    /* Cirrus GD5430 extension */
    int card_type; /* VGA_CARD_BOCHS (default) or VGA_CARD_CIRRUS */
    /* On real GD5430 silicon (chip ID >= CIRRUS_ID_CLGD5429), the
     * extended-register lock is hardwired open from reset and SR06
     * writes never change it - only chips OLDER than 5429 derive the
     * lock from the SR06==0x12 key (86Box vid_cl54xx.c:769-770,
     * 4211-4214). It still must be readable/round-trippable for
     * detection code, see sr06_shadow below. */
    int cirrus_unlocked;
    /* GR0x09/0x0A/0x0B: legacy bank-select/extended-write-mode scratch
     * registers. Not part of the BLT engine and bank-switching itself
     * isn't implemented, but they must read back whatever was last
     * written (regardless of lock state, like on real hardware) -
     * guest hardware-detection code probes them for consistency before
     * trusting the rest of the Cirrus register set. */
    uint8_t cirrus_bank_reg[3];
    /* GR0x0C/0x0D: overlay color-key compare value/mask. GR0x0E: DPMS
     * control (5429+). GR0x0F: no defined function on real hardware
     * either, but 86Box still round-trips it via the raw gdcreg[] shadow
     * (vid_cl54xx.c:1614-1625, the gdcaddr<0x10 catch-all branch returns
     * the stored byte, NOT 0xff like the >=0x10-unmapped case does).
     * Video overlay compositing and DPMS power signaling aren't
     * implemented (no second video plane / no host power-state concept
     * to drive) - stored only so reads round-trip what was written, like
     * the VCLK/bus-config SR registers above. */
    uint8_t cirrus_gr_ext[4];
    /* Extended Sequencer registers 0x08-0xFF: raw round-trip storage
     * for everything we don't otherwise special-case (VCLK dividers,
     * DRAM control, bus-type/MMIO config, I2C/DDC, misc control) -
     * these don't affect tiny386's fixed-function rendering, but must
     * read back what was written for hardware detection. */
    uint8_t cirrus_sr_ext[256];
    struct {
        int ena;
        int large; /* cur_xsize/cur_ysize: 64 if set, 32 if clear */
        int x, y;
        uint32_t addr;
    } cirrus_cursor;
    uint8_t cirrus_ext_palette[16 * 3]; /* extended palette, 16 RGB entries (6-bit) */
    uint8_t cirrus_dac_state; /* hidden DAC register read state machine (0x3c6) */
    uint8_t cirrus_dac_hidden; /* hidden DAC control register value */
    /* I2C/DDC bit-bang state machine for SR0x08 (86Box i2c_gpio.c), plus
     * a virtual EDID EEPROM slave at address 0x50 (86Box i2c_eeprom.c).
     * Without a slave actually answering DDC reads, Windows falls back
     * to a conservative "Default Monitor" profile that caps the usable
     * resolution well below what the video card itself supports. */
    struct {
        uint8_t prev_scl, prev_sda;
        uint8_t started;
        uint8_t pos;
        uint8_t byte;
        uint8_t slave_addr; /* 0xff = none selected */
        uint8_t slave_read; /* 0/1=write/read transfer, 2=address phase, |0x80 flag */
        uint8_t slave_sda;  /* ack/data bit driven by the slave device */
        uint8_t edid_addr;  /* current EDID EEPROM read/write offset */
        uint8_t wrote_offset; /* the one offset-setting byte of a write transfer was consumed */
    } cirrus_i2c;
    struct cirrus_blt_s {
        uint32_t bg_col, fg_col;
        uint16_t width, height, dst_pitch, src_pitch;
        uint32_t dst_addr, src_addr;
        uint8_t mask, mode, rop, modeext;
        uint16_t trans_col, trans_mask;
        uint8_t status; /* bit0 BUSY, bit1 START, bit2 RESET, bit7 AUTOSTART */
        /* run-time state, valid only while a blit is executing */
        int dir;
        int pixel_width;
        int pattern_x;
        /* MEMSYSSRC streaming state: while ms_active is set, CPU
         * writes to the LFB/banked VRAM range are intercepted and
         * fed into the color-expand blit engine instead of being
         * written as normal pixels (86Box gd54xx_writeb_linear /
         * gd54xx_mem_sys_src). */
        int ms_active;
        uint32_t dst_addr_backup;
        int x_count, y_count;
        uint32_t sys_src32;
        int sys_cnt;
    } cirrus_blt;

#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
#ifndef LCD_WIDTH
#define LCD_WIDTH 2048
#endif
    uint8_t tmpbuf[(LCD_WIDTH > 720 ? LCD_WIDTH : 720) * 3 * 2];
#endif
};

uint32_t get_uticks();

/* shared helpers (vga.c) used by the card-specific extension files */
bool vbe_enabled(VGAState *s);
void vbe_fixup_regs(VGAState *s);
void vbe_update_vgaregs(VGAState *s);
unsigned int rgb_to_pixel(unsigned int r, unsigned int g, unsigned int b);
int c6_to_8(int v);

extern const uint8_t sr_mask[8];
extern const uint8_t gr_mask[16];
extern const uint32_t mask16[16];

/* Cirrus extension (vga_cirrus.c), called from the common dispatch code
 * in vga.c. */
void cirrus_cursor_draw(VGAState *s, FBDevice *fb_dev, int i0);
void cirrus_get_extended_addr(VGAState *s, uint32_t *start_addr, uint32_t *line_offset);
uint32_t cirrus_get_vram_wrap_mask(VGAState *s);
void cirrus_get_svga_depth(VGAState *s, int *bpp, int *xdiv, uint32_t *line_offset, int hdisp, int dispend);
uint32_t cirrus_banked_addr(VGAState *s, uint32_t addr);

uint32_t cirrus_ioport_read(VGAState *s, uint32_t addr, int *handled);
int cirrus_ioport_write(VGAState *s, uint32_t addr, uint32_t val);
int cirrus_ext_palette_active(VGAState *s);

void vga_set_card_type_cirrus(VGAState *s);

/* MEMSYSSRC streaming blit interception (CPU writes to VRAM feed the
 * BitBlt color-expand engine instead of writing pixels directly while
 * armed). Returns 1 if the byte was consumed by the blit engine. */
int cirrus_mem_sys_src_write(VGAState *s, uint8_t val);

#endif /* VGA_INTERNAL_H */
