/*
 * Dummy VGA device
 * 
 * Copyright (c) 2003-2017 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <assert.h>

#include "vga.h"
#include "pci.h"

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
#define DEBUG_CIRRUS

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
    uint8_t ar[21];
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
     * control (5429+). Video overlay compositing and DPMS power
     * signaling aren't implemented (no second video plane / no host
     * power-state concept to drive) - stored only so reads round-trip
     * what was written, like the VCLK/bus-config SR registers above. */
    uint8_t cirrus_gr_ext[3];
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
    } cirrus_blt;

#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
#ifndef LCD_WIDTH
#define LCD_WIDTH 2048
#endif
    uint8_t tmpbuf[(LCD_WIDTH > 720 ? LCD_WIDTH : 720) * 3 * 2];
#endif
};

uint32_t get_uticks();
static int after_eq(uint32_t a, uint32_t b)
{
    return (a - b) < (1u << 31);
}

#if BPP == 32
static void vga_draw_glyph8(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol)
{
    uint32_t font_data, xorcol;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint32_t *)d)[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        ((uint32_t *)d)[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[7] = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}

static void vga_draw_glyph9(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol,
                            int dup9)
{
    uint32_t font_data, xorcol, v;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint32_t *)d)[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        ((uint32_t *)d)[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        v = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        ((uint32_t *)d)[7] = v;
        if (dup9)
            ((uint32_t *)d)[8] = v;
        else
            ((uint32_t *)d)[8] = bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}
#elif BPP == 16
static void vga_draw_glyph8(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol)
{
    uint32_t font_data, xorcol;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint16_t *)d)[0] = (-((font_data >> 7)) & xorcol) ^ bgcol;
        ((uint16_t *)d)[1] = (-((font_data >> 6) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[2] = (-((font_data >> 5) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[3] = (-((font_data >> 4) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[4] = (-((font_data >> 3) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[5] = (-((font_data >> 2) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[6] = (-((font_data >> 1) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[7] = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}

static void vga_draw_glyph9(uint8_t *d, int linesize,
                            const uint8_t *font_ptr, int h,
                            uint32_t fgcol, uint32_t bgcol,
                            int dup9)
{
    uint32_t font_data, xorcol, v;

    xorcol = bgcol ^ fgcol;
    do {
        font_data = font_ptr[0];
        ((uint16_t *)d)[0] = ((-((font_data >> 7)) & xorcol) ^ bgcol);
        ((uint16_t *)d)[1] = ((-((font_data >> 6) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[2] = ((-((font_data >> 5) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[3] = ((-((font_data >> 4) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[4] = ((-((font_data >> 3) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[5] = ((-((font_data >> 2) & 1) & xorcol) ^ bgcol);
        ((uint16_t *)d)[6] = ((-((font_data >> 1) & 1) & xorcol) ^ bgcol);
        v = (-((font_data >> 0) & 1) & xorcol) ^ bgcol;
        ((uint16_t *)d)[7] = v;
        if (dup9)
            ((uint16_t *)d)[8] = v;
        else
            ((uint16_t *)d)[8] = bgcol;
        font_ptr += 4;
        d += linesize;
    } while (--h);
}

static inline uint32_t c69(uint16_t c)
{
    // 0000 0000 0000 rrrr rggg gggb bbbb
    // 0000 0rrr rr00 0ggg ggg0 000b bbbb
    return (c & 0x1f) | ((c & 0x7e0) << 4) | ((c & 0xf800) << 7);
}

static inline uint16_t c96(uint32_t c)
{
    // 0000 0rrr rr00 0ggg ggg0 000b bbbb
    // 0000 0000 0000 rrrr rggg gggb bbbb
    uint16_t t = (c & 0x1f) | ((c & 0x7e00) >> 4) | ((c & 0x7c0000) >> 7);
#ifdef SWAP_BYTEORDER_BPP16
    return (t << 8) | (t >> 8);
#else
    return t;
#endif
}

static void scale_3_2(uint8_t *dst, int dst_stride, uint8_t *src, int w)
{
   const static int shift[4][4] = {
       { 2, 0, 1, 0 },
       { 1, 2, 0, 0 },
       { 0, 0, 2, 1 },
       { 0, 1, 0, 2 }
   };
   int ww = w / 3 * 2;
   int idx = 0;
   for (int j = 0; j < 2; j++, idx ^= 2,
#ifdef SWAPXY
                dst += BPP / 8
#else
                dst += dst_stride
#endif
           ) {
       uint8_t *dst1 = dst;
       uint8_t *src1 = src + (j * w) * (BPP / 8);
       for (int k = 0; k < ww; k++, idx ^= 1) {
           int kk = k / 2 * 3 + (k & 1);
           uint16_t *p0 = (uint16_t *) (src1 + kk * (BPP / 8));
           uint16_t *p1 = p0 + 1;
           uint16_t *p2 = p0 + w;
           uint16_t *p3 = p1 + w;
           int sh0 = shift[idx][0];
           int sh1 = shift[idx][1];
           int sh2 = shift[idx][2];
           int sh3 = shift[idx][3];
           *(uint16_t *)dst1 = c96(((c69(*p0) << sh0) + (c69(*p1) << sh1) +
                                    (c69(*p2) << sh2) + (c69(*p3) << sh3)) >> 3);
#ifdef SWAPXY
           dst1 += dst_stride;
#else
           dst1 += BPP / 8;
#endif
       }
   }
}

static void scale_3_3(uint8_t *dst, int dst_stride, uint8_t *src, int w)
{
   for (int j = 0; j < 3; j++,
#ifdef SWAPXY
                dst += BPP / 8
#else
                dst += dst_stride
#endif
           ) {
       uint8_t *dst1 = dst;
       uint8_t *src1 = src + (j * w) * (BPP / 8);
       for (int k = 0; k < w; k++) {
           *(uint16_t *)dst1 = *(uint16_t *) (src1 + k * (BPP / 8));
#ifdef SWAPXY
           dst1 += dst_stride;
#else
           dst1 += BPP / 8;
#endif
       }
   }
}

static void scale_2_1(uint8_t *dst, int dst_stride, uint8_t *src, int w)
{
    int ww = w / 2;
    uint8_t *dst1 = dst;
    for (int k = 0; k < ww; k++) {
        int kk = k * 2;
        uint16_t *p0 = (uint16_t *) (src + kk * (BPP / 8));
        uint16_t *p1 = p0 + 1;
        uint16_t *p2 = p0 + w;
        uint16_t *p3 = p1 + w;
        *(uint16_t *)dst1 = c96((c69(*p0) + c69(*p1) +
                                 c69(*p2) + c69(*p3)) >> 2);
#ifdef SWAPXY
        dst1 += dst_stride;
#else
        dst1 += BPP / 8;
#endif
    }
}

#else
#error "bad bpp"
#endif

static const uint8_t cursor_glyph[32] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

#if BPP == 32
#define COLOR(a) a
#elif BPP == 16
#define COLOR(a) (((a & 0xff) >> 3) | ((((a >> 8) & 0xff) >> 2) << 5) | ((((a >> 16) & 0xff) >> 3) << 11))
#endif
const static uint32_t ntsc_color_lut[16] = {
    COLOR(0x000000),
    COLOR(0x006E2D),
    COLOR(0x2D02FF),
    COLOR(0x008BFF),
    COLOR(0xA9002D),
    COLOR(0x777677),
    COLOR(0xEC09FF),
    COLOR(0xBB92FD),
    COLOR(0x2D5A00),
    COLOR(0x00DC00),
    COLOR(0x767777),
    COLOR(0x45F4B9),
    COLOR(0xEA6502),
    COLOR(0xBCE500),
    COLOR(0xFF80BC),
    COLOR(0xFFFFFF),
};
#undef COLOR

#if BPP == 32
static inline int c6_to_8(int v)
{
    int b;
    v &= 0x3f;
    b = v & 1;
    return (v << 2) | (b << 1) | b;
}

static inline unsigned int rgb_to_pixel(unsigned int r, unsigned int g,
                                        unsigned int b)
{
    return (r << 16) | (g << 8) | b;
}

static int update_palette256(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    v = 0;
    for(i = 0; i < 256; i++) {
        if (s->dac_8bit) {
          col = rgb_to_pixel(s->palette[v],
                             s->palette[v + 1],
                             s->palette[v + 2]);
        } else {
          col = rgb_to_pixel(c6_to_8(s->palette[v]),
                             c6_to_8(s->palette[v + 1]),
                             c6_to_8(s->palette[v + 2]));
        }
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
        v += 3;
    }
    return full_update;
}

static int update_palette16(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    for(i = 0; i < 16; i++) {
        v = s->ar[i];
        if (s->ar[0x10] & 0x80)
            v = ((s->ar[0x14] & 0xf) << 4) | (v & 0xf);
        else
            v = ((s->ar[0x14] & 0xc) << 4) | (v & 0x3f);
        v = v * 3;
        col = (c6_to_8(s->palette[v]) << 16) |
            (c6_to_8(s->palette[v + 1]) << 8) |
            c6_to_8(s->palette[v + 2]);
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
    }
    return full_update;
}
#elif BPP == 16
static int update_palette256(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    v = 0;
    for(i = 0; i < 256; i++) {
        if (s->dac_8bit) {
            col = ((s->palette[v + 2] >> 3)) |
                ((s->palette[v + 1] >> 2) << 5) |
                ((s->palette[v] >> 3) << 11);
        } else {
            col = (s->palette[v + 2] >> 1) |
                ((s->palette[v + 1]) << 5) |
                ((s->palette[v] >> 1) << 11);
        }
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
        v += 3;
    }
    return full_update;
}

static int update_palette16(VGAState *s, uint32_t *palette)
{
    int full_update, i;
    uint32_t v, col;

    full_update = 0;
    for(i = 0; i < 16; i++) {
        v = s->ar[i];
        if (s->ar[0x10] & 0x80)
            v = ((s->ar[0x14] & 0xf) << 4) | (v & 0xf);
        else
            v = ((s->ar[0x14] & 0xc) << 4) | (v & 0x3f);
        v = v * 3;
        col = (s->palette[v + 2] >> 1) |
              ((s->palette[v + 1]) << 5) |
              ((s->palette[v] >> 1) << 11);
        if (col != palette[i]) {
            full_update = 1;
            palette[i] = col;
        }
    }
    return full_update;
}
#else
#error "bad bpp"
#endif

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

static bool vbe_enabled(VGAState *s)
{
    return s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED;
}

/*
 * Sanity check vbe register writes.
 *
 * As we don't have a way to signal errors to the guest in the bochs
 * dispi interface we'll go adjust the registers to the closest valid
 * value.
 */
static void vbe_fixup_regs(VGAState *s)
{
    uint16_t *r = s->vbe_regs;
    uint32_t bits, linelength, /*maxy,*/ offset;

    if (!vbe_enabled(s)) {
        /* vbe is turned off -- nothing to do */
        return;
    }

    /* check depth */
    switch (r[VBE_DISPI_INDEX_BPP]) {
    case 4:
    case 8:
    case 16:
    case 24:
    case 32:
        bits = r[VBE_DISPI_INDEX_BPP];
        break;
    case 15:
        bits = 16;
        break;
    default:
        bits = r[VBE_DISPI_INDEX_BPP] = 8;
        break;
    }

    /* check width */
    r[VBE_DISPI_INDEX_XRES] &= ~7u;
    if (r[VBE_DISPI_INDEX_XRES] == 0) {
        r[VBE_DISPI_INDEX_XRES] = 8;
    }
//    if (r[VBE_DISPI_INDEX_XRES] > VBE_DISPI_MAX_XRES) {
//        r[VBE_DISPI_INDEX_XRES] = VBE_DISPI_MAX_XRES;
//    }
    r[VBE_DISPI_INDEX_VIRT_WIDTH] &= ~7u;
//    if (r[VBE_DISPI_INDEX_VIRT_WIDTH] > VBE_DISPI_MAX_XRES) {
//        r[VBE_DISPI_INDEX_VIRT_WIDTH] = VBE_DISPI_MAX_XRES;
//    }
    if (r[VBE_DISPI_INDEX_VIRT_WIDTH] < r[VBE_DISPI_INDEX_XRES]) {
        r[VBE_DISPI_INDEX_VIRT_WIDTH] = r[VBE_DISPI_INDEX_XRES];
    }

    /* check height */
    linelength = r[VBE_DISPI_INDEX_VIRT_WIDTH] * bits / 8;
//    maxy = s->vbe_size / linelength;
    if (r[VBE_DISPI_INDEX_YRES] == 0) {
        r[VBE_DISPI_INDEX_YRES] = 1;
    }
//    if (r[VBE_DISPI_INDEX_YRES] > VBE_DISPI_MAX_YRES) {
//        r[VBE_DISPI_INDEX_YRES] = VBE_DISPI_MAX_YRES;
//    }
//    if (r[VBE_DISPI_INDEX_YRES] > maxy) {
//        r[VBE_DISPI_INDEX_YRES] = maxy;
//    }

    /* check offset */
//    if (r[VBE_DISPI_INDEX_X_OFFSET] > VBE_DISPI_MAX_XRES) {
//        r[VBE_DISPI_INDEX_X_OFFSET] = VBE_DISPI_MAX_XRES;
//    }
//    if (r[VBE_DISPI_INDEX_Y_OFFSET] > VBE_DISPI_MAX_YRES) {
//        r[VBE_DISPI_INDEX_Y_OFFSET] = VBE_DISPI_MAX_YRES;
//    }
    offset = r[VBE_DISPI_INDEX_X_OFFSET] * bits / 8;
    offset += r[VBE_DISPI_INDEX_Y_OFFSET] * linelength;
//    if (offset + r[VBE_DISPI_INDEX_YRES] * linelength > s->vbe_size) {
//        r[VBE_DISPI_INDEX_Y_OFFSET] = 0;
//        offset = r[VBE_DISPI_INDEX_X_OFFSET] * bits / 8;
//        if (offset + r[VBE_DISPI_INDEX_YRES] * linelength > s->vbe_size) {
//            r[VBE_DISPI_INDEX_X_OFFSET] = 0;
//            offset = 0;
//        }
//    }

    /* update vga state */
//    r[VBE_DISPI_INDEX_VIRT_HEIGHT] = maxy;
    s->vbe_line_offset = linelength;
    s->vbe_start_addr  = offset / 4;
}

static void vbe_update_vgaregs(VGAState *s)
{
    int h, shift_control;

    if (!vbe_enabled(s)) {
        /* vbe is turned off -- nothing to do */
        return;
    }

    /* graphic mode + memory map 1 */
    s->gr[VGA_GFX_MISC] = (s->gr[VGA_GFX_MISC] & ~0x0c) | 0x04 |
        VGA_GR06_GRAPHICS_MODE;
    s->cr[VGA_CRTC_MODE] |= 3; /* no CGA modes */
    s->cr[VGA_CRTC_OFFSET] = s->vbe_line_offset >> 3;
    /* width */
    s->cr[VGA_CRTC_H_DISP] =
        (s->vbe_regs[VBE_DISPI_INDEX_XRES] >> 3) - 1;
    /* height (only meaningful if < 1024) */
    h = s->vbe_regs[VBE_DISPI_INDEX_YRES] - 1;
    s->cr[VGA_CRTC_V_DISP_END] = h;
    s->cr[VGA_CRTC_OVERFLOW] = (s->cr[VGA_CRTC_OVERFLOW] & ~0x42) |
        ((h >> 7) & 0x02) | ((h >> 3) & 0x40);
    /* line compare to 1023 */
    s->cr[VGA_CRTC_LINE_COMPARE] = 0xff;
    s->cr[VGA_CRTC_OVERFLOW] |= 0x10;
    s->cr[VGA_CRTC_MAX_SCAN] |= 0x40;

    if (s->vbe_regs[VBE_DISPI_INDEX_BPP] == 4) {
        shift_control = 0;
        s->sr/*_vbe*/[VGA_SEQ_CLOCK_MODE] &= ~8; /* no double line */
    } else {
        shift_control = 2;
        /* set chain 4 mode */
        s->sr/*_vbe*/[VGA_SEQ_MEMORY_MODE] |= VGA_SR04_CHN_4M;
        /* activate all planes */
        s->sr/*_vbe*/[VGA_SEQ_PLANE_WRITE] |= VGA_SR02_ALL_PLANES;
    }
    s->gr[VGA_GFX_MODE] = (s->gr[VGA_GFX_MODE] & ~0x60) |
        (shift_control << 5);
    s->cr[VGA_CRTC_MAX_SCAN] &= ~0x9f; /* no double scan */
}

/* the text refresh is just for debugging and initial boot message, so
   it is very incomplete */
static void vga_text_refresh(VGAState *s,
                             SimpleFBDrawFunc *redraw_func, void *opaque,
                             int full_update)
{
    FBDevice *fb_dev = s->fb_dev;
    int width, height, cwidth, cheight, cy, cx, x1, y1, width1, height1;
    int cx_min, cx_max, dup9;
    uint32_t ch_attr, line_offset, start_addr, ch_addr, ch_addr1, ch, cattr;
    uint8_t *vga_ram, *dst;
    const uint8_t *font_ptr;
    uint32_t fgcol, bgcol, cursor_offset, cursor_start, cursor_end;
    uint32_t now = get_uticks();
    if (after_eq(now, s->cursor_blink_time)) {
        s->cursor_blink_time = now + 133333;
        s->cursor_visible_phase = !s->cursor_visible_phase;
    }

    full_update = full_update || update_palette16(s, s->last_palette);

    vga_ram = s->vga_ram;

    const uint8_t *font_base[2];
    uint32_t v = s->sr[0x3];
    font_base[0] = vga_ram + (((v >> 4) & 1) | ((v << 1) & 6)) * 8192 * 4 + 2;
    font_base[1] = vga_ram + (((v >> 5) & 1) | ((v >> 1) & 6)) * 8192 * 4 + 2;
    
    line_offset = s->cr[0x13];
    if (s->card_type == VGA_CARD_CIRRUS)
        line_offset |= (s->cr[0x1b] & 0x10) << 4;
    line_offset <<= 3;

    start_addr = s->cr[0x0d] | (s->cr[0x0c] << 8);
    if (s->card_type == VGA_CARD_CIRRUS)
        start_addr |= ((s->cr[0x1b] & 0x01) << 16) | ((s->cr[0x1b] & 0x0c) << 15);

    cheight = (s->cr[9] & 0x1f) + 1;
    cwidth = 8;
    if (!s->force_8dm && !(s->sr[1] & 0x01))
        cwidth++;

    width = (s->cr[0x01] + 1);
    height = s->cr[0x12] |
        ((s->cr[0x07] & 0x02) << 7) |
        ((s->cr[0x07] & 0x40) << 3);
    height = (height + 1) / cheight;
    
    width1 = width * cwidth;
    height1 = height * cheight;
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
#if defined(SCALE_3_2)
    if (fb_dev->width * 3 / 2 < width1 || fb_dev->height * 3 / 2 < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT || cheight > 16)
        return; /* not enough space */
    x1 = (fb_dev->width * 3 / 2 - width1) / 3;
    y1 = (fb_dev->height * 3 / 2 - height1) / 3;
    full_update = 1;
#elif defined(SCALE_2_1)
    if (fb_dev->width * 2 < width1 || fb_dev->height * 2 < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT || cheight > 16)
        return; /* not enough space */
    x1 = (fb_dev->width * 2 - width1) / 4;
    y1 = (fb_dev->height * 2 - height1) / 4;
    full_update = 1;
#else
    if (fb_dev->width < width1 || fb_dev->height < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT || cheight > 16)
        return; /* not enough space */
    x1 = (fb_dev->width - width1) / 2;
    y1 = (fb_dev->height - height1) / 2;
    full_update = 1;
#endif
#else
    if (fb_dev->width < width1 || fb_dev->height < height1 ||
        width > MAX_TEXT_WIDTH || height > MAX_TEXT_HEIGHT)
        return; /* not enough space */
    x1 = (fb_dev->width - width1) / 2;
    y1 = (fb_dev->height - height1) / 2;
    int stride = fb_dev->stride;
#endif
    if (s->last_line_offset != line_offset ||
        s->last_start_addr != start_addr ||
        s->last_width != width ||
        s->last_height != height) {
        s->last_line_offset = line_offset;
        s->last_start_addr = start_addr;
        s->last_width = width;
        s->last_height = height;
        full_update = 1;
    }
       
    /* update cursor position */
    cursor_offset = ((s->cr[0x0e] << 8) | s->cr[0x0f]) - start_addr;
    cursor_start = s->cr[0xa];
    cursor_end = s->cr[0xb];
    if (cursor_offset != s->last_cursor_offset ||
        cursor_start != s->last_cursor_start ||
        cursor_end != s->last_cursor_end) {
#ifndef FULL_UPDATE
        /* force refresh of characters with the cursor */
        if (s->last_cursor_offset < MAX_TEXT_WIDTH * MAX_TEXT_HEIGHT)
            s->last_ch_attr[s->last_cursor_offset] = -1;
        if (cursor_offset < MAX_TEXT_WIDTH * MAX_TEXT_HEIGHT)
            s->last_ch_attr[cursor_offset] = -1;
#endif
        s->last_cursor_offset = cursor_offset;
        s->last_cursor_start = cursor_start;
        s->last_cursor_end = cursor_end;
    }

    ch_addr1 = (start_addr * 4);
    cursor_offset = (start_addr + cursor_offset) * 4;
    
#if 0
    printf("text refresh %dx%d font=%dx%d start_addr=0x%x line_offset=0x%x\n",
           width, height, cwidth, cheight, start_addr, line_offset);
#endif
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
    int cb = 6;
    int nb = (width + cb - 1) / cb;
    int cxbegin = 0;
    int cxend = cb > width ? width : cb;
    int stride = (cxend - cxbegin) * cwidth * (BPP / 8);
    for (int b = 0; b < nb; b++)
    {
    int yt = 0;
    int yy = 0;
    ch_addr1 = (start_addr * 4) + cxbegin * 4;
#endif
    for(cy = 0; cy < height; cy++) {
        ch_addr = ch_addr1;
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
        dst = s->tmpbuf + yt * stride;
#else
        dst = fb_dev->fb_data + (y1 + cy * cheight) * stride + x1 * (BPP / 8);
#endif
        cx_min = width;
        cx_max = -1;
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
        for(cx = 0; cx < cxend - cxbegin; cx++) {
#else
        for(cx = 0; cx < width; cx++) {
#endif
            ch_attr = *(uint16_t *)(vga_ram + (ch_addr & 0x1fffe));
#ifdef FULL_UPDATE
            if (1) {
#else
            if (full_update || ch_attr != s->last_ch_attr[cy * width + cx] || cursor_offset == ch_addr) {
                s->last_ch_attr[cy * width + cx] = ch_attr;
#endif
                cx_min = cx_min > cx ? cx : cx_min;
                cx_max = cx_max < cx ? cx : cx_max;
                ch = ch_attr & 0xff;
                cattr = ch_attr >> 8;

                font_ptr = font_base[(cattr >> 3) & 1] + 32 * 4 * ch;
                bgcol = s->last_palette[cattr >> 4];
                fgcol = s->last_palette[cattr & 0x0f];
                if (cwidth == 8) {
                    vga_draw_glyph8(dst, stride, font_ptr, cheight,
                                    fgcol, bgcol);
                } else {
                    dup9 = 0;
                    if (ch >= 0xb0 && ch <= 0xdf && (s->ar[0x10] & 0x04))
                        dup9 = 1;
                    vga_draw_glyph9(dst, stride, font_ptr, cheight,
                                    fgcol, bgcol, dup9);
                }
                /* cursor display */
                if (cursor_offset == ch_addr && !(cursor_start & 0x20) && s->cursor_visible_phase) {
                    int line_start, line_last, h;
                    uint8_t *dst1;
                    line_start = cursor_start & 0x1f;
                    line_last = cursor_end & 0x1f;
                    if (line_last > cheight - 1)
                        line_last = cheight - 1;

                    if (line_last >= line_start && line_start < cheight) {
                        h = line_last - line_start + 1;
                        dst1 = dst + stride * line_start;
                        if (cwidth == 8) {
                            vga_draw_glyph8(dst1, stride,
                                            cursor_glyph,
                                            h, fgcol, bgcol);
                        } else {
                            vga_draw_glyph9(dst1, stride,
                                            cursor_glyph,
                                            h, fgcol, bgcol, 1);
                        }
                    }
                }
            }
            ch_addr += 4;
            dst += (BPP / 8) * cwidth;
        }
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
#if defined(SCALE_2_1)
#define KLINE 2
#else
#define KLINE 3
#endif
        int k;
        for (k = 0; k < yt + cheight - 1; k += KLINE) {
#if defined(SCALE_3_2)
#ifdef SWAPXY
                int ii0 = (BPP / 8) * ((y1 + yy) + (x1 + cxbegin * cwidth * 2 / 3) * fb_dev->height);
#else
                int ii0 = (BPP / 8) * ((y1 + yy) * fb_dev->width + x1 + cxbegin * cwidth * 2 / 3);
#endif
                scale_3_2(fb_dev->fb_data + ii0, fb_dev->stride,
                          s->tmpbuf + k * stride, stride / (BPP / 8));
                yy += 2;
#elif defined(SCALE_2_1)
#ifdef SWAPXY
                int ii0 = (BPP / 8) * ((y1 + yy) + (x1 + cxbegin * cwidth / 2) * fb_dev->height);
#else
                int ii0 = (BPP / 8) * ((y1 + yy) * fb_dev->width + x1 + cxbegin * cwidth / 2);
#endif
                scale_2_1(fb_dev->fb_data + ii0, fb_dev->stride,
                          s->tmpbuf + k * stride, stride / (BPP / 8));
                yy += 1;
#else
#ifdef SWAPXY
                int ii0 = (BPP / 8) * ((y1 + yy) + (x1 + cxbegin * cwidth) * fb_dev->height);
#else
                int ii0 = (BPP / 8) * ((y1 + yy) * fb_dev->width + x1 + cxbegin * cwidth);
#endif
                scale_3_3(fb_dev->fb_data + ii0, fb_dev->stride,
                          s->tmpbuf + k * stride, stride / (BPP / 8));
                yy += 3;
#endif
        }
        yt = k - (yt + cheight - 1);
        if (yt != 0) {
                yt = KLINE - yt;
                memcpy(s->tmpbuf, s->tmpbuf + (k - KLINE) * stride, yt * stride);
        }
#undef KLINE
#endif
//        if (cx_max >= cx_min) {
//            redraw_func(opaque,
//                        x1 + cx_min * cwidth, y1 + cy * cheight,
//                        (cx_max - cx_min + 1) * cwidth, cheight);
//        }
        ch_addr1 += line_offset;
    }
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
    cxbegin += cb;
    cxend += cb;
    if (cxend > width) cxend = width;
    stride = (cxend - cxbegin) * cwidth * (BPP / 8);
    }
#endif
    redraw_func(opaque, 0, 0, fb_dev->width, fb_dev->height);
}

/* Cirrus hardware cursor overlay, drawn as a final pass over the
 * already-rendered frame (86Box vid_cl54xx.c:2075-2137,
 * gd54xx_hwcursor_draw). 2bpp-per-pixel format: byte 0 of each 8-pixel
 * group is the AND mask, byte 1 the XOR mask (offset +8 within the row
 * for the 64x64 size, +0x80 - the fixed 32x32 slot size - for 32x32),
 * comb = (XOR<<0)|(AND<<1) selects: 0=passthrough, 1=bg color,
 * 2=XOR-invert, 3=fg color. Colors come from the extended palette
 * entries 0 and 0xf (gd54xx->extpallook[0]/[0xf]). */
static void cirrus_cursor_draw(VGAState *s, FBDevice *fb_dev, int i0)
{
    if (!s->cirrus_cursor.ena)
        return;
    int size = s->cirrus_cursor.large ? 64 : 32;
    int pitch = s->cirrus_cursor.large ? 16 : 4;
    int xor_off = s->cirrus_cursor.large ? 8 : 0x80;
#if BPP == 32
    uint32_t bgcol = rgb_to_pixel(c6_to_8(s->cirrus_ext_palette[0]),
                                  c6_to_8(s->cirrus_ext_palette[1]),
                                  c6_to_8(s->cirrus_ext_palette[2]));
    uint32_t fgcol = rgb_to_pixel(c6_to_8(s->cirrus_ext_palette[0xf * 3]),
                                  c6_to_8(s->cirrus_ext_palette[0xf * 3 + 1]),
                                  c6_to_8(s->cirrus_ext_palette[0xf * 3 + 2]));
#else
    uint32_t bgcol = (s->cirrus_ext_palette[2] >> 1) |
        (s->cirrus_ext_palette[1] << 5) | ((s->cirrus_ext_palette[0] >> 1) << 11);
    uint32_t fgcol = (s->cirrus_ext_palette[0xf * 3 + 2] >> 1) |
        (s->cirrus_ext_palette[0xf * 3 + 1] << 5) |
        ((s->cirrus_ext_palette[0xf * 3] >> 1) << 11);
#endif
    for (int row = 0; row < size; row++) {
        int yy = s->cirrus_cursor.y + row;
        if (yy < 0 || yy >= fb_dev->height)
            continue;
        uint32_t row_addr = s->cirrus_cursor.addr + row * pitch;
        for (int xb = 0; xb < size; xb += 8) {
            uint32_t a0 = row_addr + (xb >> 3);
            uint32_t a1 = row_addr + xor_off + (xb >> 3);
            uint8_t dat0 = (a0 < (uint32_t)s->vga_ram_size) ? s->vga_ram[a0] : 0;
            uint8_t dat1 = (a1 < (uint32_t)s->vga_ram_size) ? s->vga_ram[a1] : 0;
            for (int xx = 0; xx < 8; xx++) {
                int b0 = (dat0 >> (7 - xx)) & 1;
                int b1 = (dat1 >> (7 - xx)) & 1;
                int comb = b1 | (b0 << 1);
                int xpos = s->cirrus_cursor.x + xb + xx;
                if (comb == 0 || xpos < 0 || xpos >= fb_dev->width)
                    continue;
                int idx = (BPP / 8) * (yy * fb_dev->width + xpos) + i0;
                uint32_t color;
                switch (comb) {
                case 1: color = bgcol; break;
                case 3: color = fgcol; break;
                default: /* 2: XOR-invert the existing pixel */
                    color = 0;
                    for (int k = 0; k < BPP / 8; k++)
                        color |= (uint32_t)fb_dev->fb_data[idx + k] << (8 * k);
                    color ^= (BPP == 32) ? 0xffffffu : 0xffffu;
                    break;
                }
                for (int k = 0; k < BPP / 8; k++)
                    fb_dev->fb_data[idx + k] = color >> (8 * k);
            }
        }
    }
}

static void vga_graphic_refresh(VGAState *s,
                                SimpleFBDrawFunc *redraw_func, void *opaque,
                                int full_update)
{
    FBDevice *fb_dev = s->fb_dev;
    int w = (s->cr[0x01] + 1) * 8;
    int h = s->cr[0x12] |
        ((s->cr[0x07] & 0x02) << 7) |
        ((s->cr[0x07] & 0x40) << 3);
    h++;

    int shift_control = (s->gr[0x05] >> 5) & 3;
#ifdef DEBUG_CIRRUS
    {
        static int last_w = -1, last_h = -1, last_sc = -1, last_vbe = -1;
        if (w != last_w || h != last_h || shift_control != last_sc || vbe_enabled(s) != last_vbe) {
            printf("cirrus: graphic_refresh w=%d h=%d shift_control=%d vbe=%d fbw=%d fbh=%d sr1=0x%02x gr5=0x%02x cr1=0x%02x\n",
                   w, h, shift_control, vbe_enabled(s), s->fb_dev->width, s->fb_dev->height,
                   s->sr[1], s->gr[5], s->cr[1]);
            last_w = w; last_h = h; last_sc = shift_control; last_vbe = vbe_enabled(s);
        }
    }
#endif
    int double_scan = (s->cr[0x09] >> 7);
    int multi_scan, multi_run;
    if (!double_scan) {
        multi_scan = (((s->cr[0x09] & 0x1f) + 1) << double_scan) - 1;
    } else {
        /* in CGA modes, multi_scan is ignored */
        /* XXX: is it correct ? */
        multi_scan = double_scan;
    }
    multi_run = multi_scan;

    uint32_t start_addr = s->cr[0x0d] | (s->cr[0x0c] << 8);
    if (s->card_type == VGA_CARD_CIRRUS)
        start_addr |= ((s->cr[0x1b] & 0x01) << 16) | ((s->cr[0x1b] & 0x0c) << 15);
    uint32_t line_offset = s->cr[0x13];
    if (s->card_type == VGA_CARD_CIRRUS)
        line_offset |= (s->cr[0x1b] & 0x10) << 4;
    line_offset <<= 3;
//    uint32_t line_compare = s->cr[0x18] |
//        ((s->cr[0x07] & 0x10) << 4) |
//        ((s->cr[0x09] & 0x40) << 3);
    if (vbe_enabled(s)) {
        line_offset = s->vbe_line_offset;
        start_addr = s->vbe_start_addr;
//        line_compare = 65535;
    }
    uint32_t addr1 = 4 * start_addr;
    /* CR0x1B bit1: select the display-memory wrap mask (86Box
     * vid_cl54xx.c:2056). Only bounds the start of each scanline, not
     * every per-pixel fetch within it - in practice this never differs
     * from "no mask" for any vga_mem_size we actually configure (a few
     * MB), so the approximation is harmless. */
    if (s->card_type == VGA_CARD_CIRRUS) {
        uint32_t vram_mask = (s->cr[0x1b] & 0x02) ? (uint32_t)(s->vga_ram_size - 1) : 0x3ffff;
        addr1 &= vram_mask;
    }
    uint8_t *vram = s->vga_ram;
    uint32_t palette[256];
    int xdiv = 1;
    int bpp = 4;
    if (shift_control == 0 || shift_control == 1) {
        update_palette16(s, palette);
        if (s->sr[0x01] & 8) {
            xdiv = 2;
            if (shift_control == 1) // XXX
                w *= 2;
        }
    } else {
        if (!vbe_enabled(s) && s->card_type == VGA_CARD_CIRRUS) {
            /* Native Cirrus SVGA color depth: derived from the hidden DAC
             * control register (0x3c6) + SR0x07 bpp bits, not from the
             * Bochs VBE registers (86Box vid_cl54xx.c:1898-2040). SR0x07
             * bit0 (CIRRUS_SR7_BPP_SVGA) also selects full vs halved dot
             * clock (xdiv). */
            int true_svga = s->sr[7] & 0x01;
            uint8_t ctrl = s->cirrus_dac_hidden;
            bpp = 8;
            if (ctrl & 0x80) {
                if (ctrl & 0x40) {
                    switch (ctrl & 0x0f) {
                    case 0: bpp = 15; break;
                    case 1: bpp = 16; break;
                    case 5: bpp = 24; break;
                    case 8: case 9: bpp = 8; break;
                    case 0xf:
                        switch (s->sr[7] & 0x0e) {
                        case 0x08: bpp = 32; break;
                        case 0x04: bpp = 24; break;
                        case 0x06: case 0x02: bpp = 16; break;
                        default: bpp = 8; break;
                        }
                        break;
                    default: break;
                    }
                } else {
                    bpp = 15;
                }
            }
            xdiv = true_svga ? 1 : 2;
            if (bpp == 32)
                line_offset *= 2;
            if (bpp == 8)
                update_palette256(s, palette);
        } else if (!vbe_enabled(s)) {
            update_palette256(s, palette);
            xdiv = 2;
            bpp = 8;
        } else {
            bpp = s->vbe_regs[VBE_DISPI_INDEX_BPP];
            if (bpp == 8)
                update_palette256(s, palette);
        }
    }

    int y1 = 0;
    int i0 = 0;
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
#if defined(SCALE_3_2)
    int hx = fb_dev->height * 3 / 2;
    int wx = fb_dev->width * 3 / 2;
    if (h < hx)
#ifdef SWAPXY
        i0 += (hx - h) / 3 * (BPP / 8);
#else
        i0 += (hx - h) / 3 * fb_dev->stride;
#endif
    else
        h = hx;
    if (w < wx)
#ifdef SWAPXY
        i0 += (wx - w) / 3 * fb_dev->stride;
#else
        i0 += (wx - w) / 3 * (BPP / 8);
#endif
    else
        w = wx;
#elif defined(SCALE_2_1)
    int hx = fb_dev->height * 2;
    int wx = fb_dev->width * 2;
    if (h < hx)
#ifdef SWAPXY
        i0 += (hx - h) / 4 * (BPP / 8);
#else
        i0 += (hx - h) / 4 * fb_dev->stride;
#endif
    else
        h = hx;
    if (w < wx)
#ifdef SWAPXY
        i0 += (wx - w) / 4 * fb_dev->stride;
#else
        i0 += (wx - w) / 4 * (BPP / 8);
#endif
    else
        w = wx;
#else
    int hx = fb_dev->height;
    int wx = fb_dev->width;
    if (h < hx)
        i0 += (hx - h) / 2 * (BPP / 8);
    else
        h = hx;
    if (w < wx)
        i0 += (wx - w) / 2 * fb_dev->stride;
    else
        w = wx;
#endif
    int yyt = 0;
    int yy = 0;
#else
    int hx = fb_dev->height;
    int wx = fb_dev->width;
    if (h < hx)
        i0 += (hx - h) / 2 * fb_dev->stride;
    else
        h = hx;
    if (w < wx)
        i0 += (wx - w) / 2 * (BPP / 8);
    else
        w = wx;
#endif
    uint32_t plane_mask = s->ar[0x12];
    for (int y = 0; y < h; y++) {
        uint32_t addr = addr1;
        if (!(s->cr[0x17] & 1)) {
            int shift;
            /* CGA compatibility handling */
            shift = 14 + ((s->cr[0x17] >> 6) & 1);
            addr = (addr & ~(1 << shift)) | ((y1 & 1) << shift);
        }
        if (!(s->cr[0x17] & 2)) {
            addr = (addr & ~0x8000) | ((y1 & 2) << 14);
        }

        uint32_t color_comp = 0;
        for (int x = 0; x < w; x++) {
            int x1 = x / xdiv;
            uint32_t color;
            if (shift_control == 0) {
                if (plane_mask == 1) {
                    if (s->comp_ntsc) {
                        if (!(x1 & 3)) {
                            int k = vram[addr + 4 * (x1 >> 3)];
                            if (!(x1 & 4))
                                k >>= 4;
                            color_comp = ntsc_color_lut[k & 0xf];
                        }
                        color = color_comp;
                    } else {
                        int k = ((vram[addr + 4 * (x1 >> 3)] >> (7 - (x1 & 7))) & 1);
                        color = palette[k];
                    }
                } else {
                    int k = ((vram[addr + 4 * (x1 >> 3)] >> (7 - (x1 & 7))) & 1) << 0;
                    k |= ((vram[addr + 4 * (x1 >> 3) + 1] >> (7 - (x1 & 7))) & 1) << 1;
                    k |= ((vram[addr + 4 * (x1 >> 3) + 2] >> (7 - (x1 & 7))) & 1) << 2;
                    k |= ((vram[addr + 4 * (x1 >> 3) + 3] >> (7 - (x1 & 7))) & 1) << 3;
                    color = palette[k];
                }
            } else if (shift_control == 1) {
                int k = ((vram[addr + 4 * (x1 >> 3) + ((x1 & 4) >> 2)] >>
                          (6 - 2 * (x1 & 3))) & 3);
                color = palette[k];
            } else
#if BPP == 32
            {
                switch (bpp) {
                case 8: {
                    int k = vram[addr + x1];
                    color = palette[k];
                    break;
                }
                case 15: {
                    int k = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    int b = (k & ((1 << 5) - 1)) << 3;
                    int g = ((k >> 5) & ((1 << 5) - 1)) << 3;
                    int r = ((k >> 10) & ((1 << 5) - 1)) << 3;
                    color = b | (g << 8) | (r << 16);
                    break;
                }
                case 16: {
                    int k = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    int b = (k & ((1 << 5) - 1)) << 3;
                    int g = ((k >> 5) & ((1 << 6) - 1)) << 2;
                    int r = ((k >> 11) & ((1 << 5) - 1)) << 3;
                    color = b | (g << 8) | (r << 16);
                    break;
                }
                case 24: {
                    color = vram[addr + 3 * x1] |
                        (vram[addr + 3 * x1 + 1] << 8) |
                        (vram[addr + 3 * x1 + 2] << 16);
                    break;
                }
                case 32: {
                    color = vram[addr + 4 * x1] |
                        (vram[addr + 4 * x1 + 1] << 8) |
                        (vram[addr + 4 * x1 + 2] << 16) |
                        (vram[addr + 4 * x1 + 3] << 24);
                    break;
                }
                default:
                    fprintf(stderr, "vga bpp is %d\n", bpp);
                    abort();
                }
            }
            int i = (BPP / 8) * (y * fb_dev->width + x) + i0;
            fb_dev->fb_data[i + 0] = color;
            fb_dev->fb_data[i + 1] = color >> 8;
            fb_dev->fb_data[i + 2] = color >> 16;
            fb_dev->fb_data[i + 3] = color >> 24;
#elif BPP == 16
            {
                switch (bpp) {
                case 8: {
                    color = palette[vram[addr + x1]];
                    break;
                }
                case 15: {
                    int k = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    color = (k & 0x1f) | ((k & ~0x1f) << 1);
                    break;
                }
                case 16: {
                    color = vram[addr + 2 * x1] | (vram[addr + 2 * x1 + 1] << 8);
                    break;
                }
                case 24: {
                    color = ((vram[addr + 3 * x1] >> 3)) |
                        ((vram[addr + 3 * x1 + 1] >> 2) << 5) |
                        ((vram[addr + 3 * x1 + 2] >> 3) << 11);
                    break;
                }
                case 32: {
                    color = ((vram[addr + 4 * x1] >> 3)) |
                        ((vram[addr + 4 * x1 + 1] >> 2) << 5) |
                        ((vram[addr + 4 * x1 + 2] >> 3) << 11);
                    break;
                }
                default:
                    fprintf(stderr, "vga bpp is %d\n", bpp);
                    abort();
                }
            }
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
            int i = (BPP / 8) * (yyt * w + x);
            s->tmpbuf[i + 0] = color;
            s->tmpbuf[i + 1] = color >> 8;
#else
            int i = (BPP / 8) * (y * fb_dev->width + x) + i0;
            fb_dev->fb_data[i + 0] = color;
            fb_dev->fb_data[i + 1] = color >> 8;
#endif
#else
#error "bad bpp"
#endif
        }
        if (!multi_run) {
            int mask = (s->cr[0x17] & 3) ^ 3;
            if ((y1 & mask) == mask)
                addr1 += line_offset;
            y1++;
            multi_run = multi_scan;
        } else {
            multi_run--;
        }
#if defined(SCALE_3_2) || defined(SCALE_2_1) || defined(SWAPXY)
#if defined(SCALE_2_1)
#define KLINE 2
#else
#define KLINE 3
#endif
        yyt++;
        if (yyt == KLINE) {
#ifdef SWAPXY
            int ii0 = (BPP / 8) * yy + i0;
#else
            int ii0 = (BPP / 8) * (yy * fb_dev->width) + i0;
#endif
#if defined(SCALE_3_2)
            scale_3_2(fb_dev->fb_data + ii0, fb_dev->stride, s->tmpbuf, w);
            yyt = 0;
            yy += 2;
#elif defined(SCALE_2_1)
            scale_2_1(fb_dev->fb_data + ii0, fb_dev->stride, s->tmpbuf, w);
            yyt = 0;
            yy += 1;
#else
            scale_3_3(fb_dev->fb_data + ii0, fb_dev->stride, s->tmpbuf, w);
            yyt = 0;
            yy += 3;
#endif
        }
#undef KLINE
#endif
    }
#if !defined(SCALE_3_2) && !defined(SCALE_2_1) && !defined(SWAPXY)
    if (s->card_type == VGA_CARD_CIRRUS)
        cirrus_cursor_draw(s, fb_dev, i0);
#endif
    redraw_func(opaque, 0, 0, fb_dev->width, fb_dev->height);
}

static void simplefb_clear(FBDevice *fb_dev,
               SimpleFBDrawFunc *redraw_func, void *opaque)
{
    memset(fb_dev->fb_data, 0, fb_dev->width * fb_dev->height * (BPP / 8));
}

int vga_step(VGAState *s)
{
    uint32_t now = get_uticks();
    int ret = 0;
    if (after_eq(now, s->retrace_time)) {
        if (s->retrace_phase == 0) {
            s->st01 |= ST01_DISP_ENABLE;
            s->retrace_phase = 1;
            s->retrace_time = now + 833;
        } else if (s->retrace_phase == 1) {
            s->st01 |= ST01_V_RETRACE;
            s->retrace_phase = 2;
            s->retrace_time = now + 833;
            ret = 1;
        } else {
            s->st01 &= ~(ST01_V_RETRACE | ST01_DISP_ENABLE);
            s->retrace_phase = 0;
            s->retrace_time = now + RETRACE_INTERVAL_US;
        }
    }
    return ret;
}

void vga_refresh(VGAState *s,
                 SimpleFBDrawFunc *redraw_func, void *opaque, int full_update)
{
    FBDevice *fb_dev = s->fb_dev;
    int graphic_mode;
    if (!(s->ar_index & 0x20)) {
        /* blank */
        graphic_mode = 0;
    } else if (s->gr[0x06] & 1) {
        /* graphic mode */
        graphic_mode = 2;
    } else {
        /* text mode */
        graphic_mode = 1;
    }

    if (graphic_mode != s->graphic_mode) {
        s->graphic_mode = graphic_mode;
        full_update = 1;
        s->cursor_blink_time = get_uticks();
        simplefb_clear(fb_dev, redraw_func, opaque);
    }

    if (s->graphic_mode == 2) {
        vga_graphic_refresh(s, redraw_func, opaque, full_update);
    } else if (s->graphic_mode == 1) {
        vga_text_refresh(s, redraw_func, opaque, full_update);
    }
}

/* force some bits to zero */
static const uint8_t sr_mask[8] = {
    (uint8_t)~0xfc,
    (uint8_t)~0xc2,
    (uint8_t)~0xf0,
    (uint8_t)~0xc0,
    (uint8_t)~0xf1,
    (uint8_t)~0xff,
    (uint8_t)~0xff,
    (uint8_t)~0x00,
};

static const uint8_t gr_mask[16] = {
    (uint8_t)~0xf0, /* 0x00 */
    (uint8_t)~0xf0, /* 0x01 */
    (uint8_t)~0xf0, /* 0x02 */
    (uint8_t)~0xe0, /* 0x03 */
    (uint8_t)~0xfc, /* 0x04 */
    (uint8_t)~0x84, /* 0x05 */
    (uint8_t)~0xf0, /* 0x06 */
    (uint8_t)~0xf0, /* 0x07 */
    (uint8_t)~0x00, /* 0x08 */
    (uint8_t)~0xff, /* 0x09 */
    (uint8_t)~0xff, /* 0x0a */
    (uint8_t)~0xff, /* 0x0b */
    (uint8_t)~0xff, /* 0x0c */
    (uint8_t)~0xff, /* 0x0d */
    (uint8_t)~0xff, /* 0x0e */
    (uint8_t)~0xff, /* 0x0f */
};

/*
 * Cirrus Logic GD5430 BitBlt 2D acceleration engine.
 *
 * Only active when s->card_type == VGA_CARD_CIRRUS. Registers are kept in
 * s->cirrus_blt and are accessible both through the classic extended GR
 * ports (0x3ce/0x3cf, index > 8) and through a fixed MMIO window at offset
 * 0xb8000 of the linear framebuffer BAR (see cirrus_blt_mmio_read8/write8
 * and pc.c). Both paths share the same canonical offset numbering (0x00-
 * 0x21, plus 0x40 for the status/trigger register), taken from the real
 * GD5430 MMIO register map.
 *
 * Implemented: screen-to-screen / solid-fill BitBlt (cirrus_normal_blit,
 * with a memmove() fast path for the common SRCCOPY case) and 8x8 pattern
 * fill incl. monochrome color-expand solid fill (cirrus_pattern_copy) -
 * these cover the BitBlt operations GDI actually issues for window
 * drag/scroll and background/brush fills. CPU<->VRAM streaming blits
 * (MEMSYSSRC/MEMSYSDEST, used for font color-expand and readback) and
 * the full ROP/transparency matrix are deliberately not implemented yet;
 * such requests complete as a no-op rather than hang the driver.
 */
#define CIRRUS_BLTMODE_BACKWARDS       0x01
#define CIRRUS_BLTMODE_MEMSYSDEST      0x02
#define CIRRUS_BLTMODE_MEMSYSSRC       0x04
#define CIRRUS_BLTMODE_TRANSPARENTCOMP 0x08
#define CIRRUS_BLTMODE_PIXELWIDTHMASK  0x30
#define CIRRUS_BLTMODE_PIXELWIDTH8     0x00
#define CIRRUS_BLTMODE_PIXELWIDTH16    0x10
#define CIRRUS_BLTMODE_PIXELWIDTH24    0x20
#define CIRRUS_BLTMODE_PIXELWIDTH32    0x30
#define CIRRUS_BLTMODE_PATTERNCOPY     0x40
#define CIRRUS_BLTMODE_COLOREXPAND     0x80

#define CIRRUS_BLT_BUSY      0x01
#define CIRRUS_BLT_START     0x02
#define CIRRUS_BLT_RESET     0x04
#define CIRRUS_BLT_FIFOUSED  0x10
#define CIRRUS_BLT_PAUSED    0x20
#define CIRRUS_BLT_APERTURE2 0x40
#define CIRRUS_BLT_AUTOSTART 0x80

#define CIRRUS_BLTMODEEXT_DWORDGRANULARITY 0x01
#define CIRRUS_BLTMODEEXT_COLOREXPINV      0x02
#define CIRRUS_BLTMODEEXT_SOLIDFILL        0x04
#define CIRRUS_BLTMODEEXT_BACKGROUNDONLY   0x08

static uint8_t cirrus_rop(VGAState *s, uint8_t dst, uint8_t src)
{
    switch (s->cirrus_blt.rop) {
    case 0x00: return 0x00;
    case 0x05: return src & dst;
    case 0x06: return dst;
    case 0x09: return src & ~dst;
    case 0x0b: return ~dst;
    case 0x0d: return src;
    case 0x0e: return 0xff;
    case 0x50: return ~src & dst;
    case 0x59: return src ^ dst;
    case 0x6d: return src | dst;
    case 0x90: return ~(src | dst);
    case 0x95: return ~(src ^ dst);
    case 0xad: return src | ~dst;
    case 0xd0: return ~src;
    case 0xd6: return ~src | dst;
    case 0xda: return ~(src & dst);
    default: return dst;
    }
}

static int cirrus_get_pixel_width(VGAState *s)
{
    switch (s->cirrus_blt.mode & CIRRUS_BLTMODE_PIXELWIDTHMASK) {
    case CIRRUS_BLTMODE_PIXELWIDTH16: return 2;
    case CIRRUS_BLTMODE_PIXELWIDTH24: return 3;
    case CIRRUS_BLTMODE_PIXELWIDTH32: return 4;
    default: return 1;
    }
}

static uint8_t cirrus_color_expand(VGAState *s, int mask, int shift)
{
    if (mask)
        return s->cirrus_blt.fg_col >> (shift << 3);
    else
        return s->cirrus_blt.bg_col >> (shift << 3);
}

/* mirrors gd54xx_blit(): writes *dst = target unless skipped by the
 * pattern's left clip (skip) or, for transparent/color-expand modes,
 * by the pattern bit (mask). */
static void cirrus_blt_write_pixel(VGAState *s, int mask, uint8_t *dst,
                                   uint8_t target, int skip)
{
    int is_transp = s->cirrus_blt.mode & CIRRUS_BLTMODE_TRANSPARENTCOMP;
    int is_bgonly = s->cirrus_blt.modeext & CIRRUS_BLTMODEEXT_BACKGROUNDONLY;

    if (is_transp) {
        if ((s->cirrus_blt.mode & CIRRUS_BLTMODE_COLOREXPAND) &&
            (s->cirrus_blt.modeext & CIRRUS_BLTMODEEXT_COLOREXPINV))
            mask = !mask;
        if (mask && !skip)
            *dst = target;
    } else if ((s->cirrus_blt.mode & CIRRUS_BLTMODE_COLOREXPAND) && is_bgonly) {
        if (mask || !skip)
            *dst = target;
    } else {
        if (!skip)
            *dst = target;
    }
}

/* Screen-to-screen / solid BitBlt. Handles SRCCOPY and any ROP, plus
 * monochrome color-expand reading the pattern from src VRAM. Transparent
 * comparison and CPU<->VRAM streaming are out of scope for now (see
 * cirrus_blt_start). */
static void cirrus_normal_blit(VGAState *s)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;
    uint8_t *vram = (uint8_t *)s->vga_ram;
    uint32_t vram_size = (uint32_t)s->vga_ram_size;
    int row;

    /* fast path: plain SRCCOPY screen-to-screen/fill, no color expand */
    if (b->rop == 0x0d && !(b->mode & CIRRUS_BLTMODE_COLOREXPAND)) {
        for (row = 0; row <= b->height; row++) {
            int64_t doff = (int64_t)b->dst_addr + (int64_t)row * b->dst_pitch * b->dir;
            int64_t soff = (int64_t)b->src_addr + (int64_t)row * b->src_pitch * b->dir;
            uint32_t dbase = (uint32_t)(b->dir > 0 ? doff : doff - b->width);
            uint32_t sbase = (uint32_t)(b->dir > 0 ? soff : soff - b->width);
            uint32_t len = (uint32_t)b->width + 1;
            if (dbase + len > vram_size || sbase + len > vram_size)
                break;
            memmove(vram + dbase, vram + sbase, len);
        }
        return;
    }

    for (row = 0; row <= b->height; row++) {
        uint32_t dst_addr = (uint32_t)(b->dst_addr + (int64_t)row * b->dst_pitch * b->dir);
        uint32_t src_addr = (uint32_t)(b->src_addr + (int64_t)row * b->src_pitch * b->dir);
        int x_count = 0;
        int col;
        for (col = 0; col <= b->width; col++) {
            uint8_t src, dst, target;
            int mask;

            if (dst_addr >= vram_size || src_addr >= vram_size)
                break;

            if (b->mode & CIRRUS_BLTMODE_COLOREXPAND) {
                int shift = x_count % b->pixel_width;
                mask = vram[src_addr] & (0x80 >> (x_count / b->pixel_width));
                src = cirrus_color_expand(s, mask, shift);
            } else {
                src = vram[src_addr];
                mask = 1;
            }

            dst = vram[dst_addr];
            target = cirrus_rop(s, dst, src);

            cirrus_blt_write_pixel(s, mask, &vram[dst_addr], target, 0);

            dst_addr += b->dir;
            if (!(b->mode & CIRRUS_BLTMODE_COLOREXPAND))
                src_addr += b->dir;

            x_count++;
            if (x_count == (b->pixel_width << 3)) {
                x_count = 0;
                if (b->mode & CIRRUS_BLTMODE_COLOREXPAND)
                    src_addr += b->dir;
            }
        }
    }
}

/* 8x8 pattern fill, including monochrome color-expand patterns; with
 * CIRRUS_BLTMODEEXT_SOLIDFILL this is how GDI does accelerated solid
 * color fills (the pattern source is then ignored, fg_col is used
 * directly). Mirrors gd54xx_pattern_fill(). */
static void cirrus_pattern_copy(VGAState *s)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;
    uint8_t *vram = (uint8_t *)s->vga_ram;
    uint32_t vram_size = (uint32_t)s->vga_ram_size;
    int pattern_pitch = b->pixel_width << 3;
    uint32_t dsta = b->dst_addr;
    int pattern_y = b->src_addr & 0x07;
    uint32_t srca = b->src_addr & ~0x07u;
    int y;

    if (b->pixel_width == 3)
        pattern_pitch = 32;
    if (b->mode & CIRRUS_BLTMODE_COLOREXPAND)
        pattern_pitch = 1;

    for (y = 0; y <= b->height; y++) {
        uint32_t srca2 = srca + (uint32_t)(pattern_y * pattern_pitch);
        int pixel = 0;
        int x;
        for (x = 0; x <= b->width; x += b->pixel_width) {
            int bitmask = 1;
            int xx;

            if (b->mode & CIRRUS_BLTMODE_COLOREXPAND) {
                if (b->modeext & CIRRUS_BLTMODEEXT_SOLIDFILL)
                    bitmask = 1;
                else if (srca2 < vram_size)
                    bitmask = vram[srca2] & (0x80 >> pixel);
                else
                    bitmask = 0;
            }
            for (xx = 0; xx < b->pixel_width; xx++) {
                uint32_t dstoff = dsta + x + xx;
                uint8_t src, target;
                int skip;

                if (dstoff >= vram_size)
                    continue;

                if (b->mode & CIRRUS_BLTMODE_COLOREXPAND) {
                    src = cirrus_color_expand(s, bitmask, xx);
                } else {
                    uint32_t srcoff = srca2 + (x % (b->pixel_width << 3)) + xx;
                    if (srcoff >= vram_size)
                        continue;
                    src = vram[srcoff];
                }

                target = cirrus_rop(s, vram[dstoff], src);
                skip = (b->pixel_width == 3) ? ((x + xx) < b->pattern_x)
                                              : (x < b->pattern_x);
                cirrus_blt_write_pixel(s, bitmask, &vram[dstoff], target, skip);
            }
            pixel = (pixel + 1) & 7;
        }
        pattern_y = (pattern_y + 1) & 7;
        dsta += b->dst_pitch;
    }
}

/* Dispatches a triggered blit. Always synchronous: by the time this
 * returns, BUSY/START are already cleared. */
static void cirrus_blt_start(VGAState *s)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;

    if ((b->mode & CIRRUS_BLTMODE_BACKWARDS) &&
        !(b->mode & (CIRRUS_BLTMODE_PATTERNCOPY | CIRRUS_BLTMODE_COLOREXPAND)) &&
        !(b->mode & CIRRUS_BLTMODE_TRANSPARENTCOMP))
        b->dir = -1;
    else
        b->dir = 1;

    b->pixel_width = cirrus_get_pixel_width(s);

    if (b->mode & (CIRRUS_BLTMODE_PATTERNCOPY | CIRRUS_BLTMODE_COLOREXPAND)) {
        if (b->pixel_width == 3)
            b->pattern_x = b->mask & 0x1f;
        else
            b->pattern_x = (b->mask & 0x07) * b->pixel_width;
    } else {
        b->pattern_x = 0;
    }

    if (b->mode & (CIRRUS_BLTMODE_MEMSYSSRC | CIRRUS_BLTMODE_MEMSYSDEST)) {
        /* CPU<->VRAM streaming blits aren't implemented; complete as a
         * no-op so the driver doesn't wait forever on BUSY. */
    } else if (b->mode & CIRRUS_BLTMODE_PATTERNCOPY) {
        cirrus_pattern_copy(s);
    } else {
        cirrus_normal_blit(s);
    }

    b->status &= ~(CIRRUS_BLT_START | CIRRUS_BLT_BUSY);
}

static void cirrus_blt_trigger(VGAState *s, uint8_t val)
{
    uint8_t old = s->cirrus_blt.status;
    s->cirrus_blt.status = val;
#ifdef DEBUG_CIRRUS
    printf("cirrus: trigger status=0x%02x mode=0x%02x rop=0x%02x modeext=0x%02x "
           "w=%d h=%d dst=0x%x dpitch=%d src=0x%x spitch=%d\n",
           val, s->cirrus_blt.mode, s->cirrus_blt.rop, s->cirrus_blt.modeext,
           s->cirrus_blt.width, s->cirrus_blt.height,
           s->cirrus_blt.dst_addr, s->cirrus_blt.dst_pitch,
           s->cirrus_blt.src_addr, s->cirrus_blt.src_pitch);
#endif
    if (!(old & CIRRUS_BLT_RESET) && (val & CIRRUS_BLT_RESET)) {
        s->cirrus_blt.status &= ~(CIRRUS_BLT_START | CIRRUS_BLT_BUSY | CIRRUS_BLT_RESET);
    } else if (!(old & CIRRUS_BLT_START) && (val & CIRRUS_BLT_START)) {
        s->cirrus_blt.status |= CIRRUS_BLT_BUSY;
        cirrus_blt_start(s);
    }
}

/* canonical register space: 0x00-0x21 plus 0x40 (status/trigger),
 * matching the real GD5430 MMIO BitBlt register map. Both the classic
 * extended-GR port path and the MMIO window go through these. */
static void cirrus_blt_reg_write8(VGAState *s, uint32_t off, uint8_t val)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;
#ifdef DEBUG_CIRRUS
    printf("cirrus: reg_write8 off=0x%02x val=0x%02x\n", off, val);
#endif
    switch (off) {
    case 0x00: b->bg_col = (b->bg_col & 0xffffff00) | val; break;
    case 0x01: b->bg_col = (b->bg_col & 0xffff00ff) | ((uint32_t)val << 8); break;
    case 0x02: b->bg_col = (b->bg_col & 0xff00ffff) | ((uint32_t)val << 16); break;
    case 0x03: b->bg_col = (b->bg_col & 0x00ffffff) | ((uint32_t)val << 24); break;
    case 0x04: b->fg_col = (b->fg_col & 0xffffff00) | val; break;
    case 0x05: b->fg_col = (b->fg_col & 0xffff00ff) | ((uint32_t)val << 8); break;
    case 0x06: b->fg_col = (b->fg_col & 0xff00ffff) | ((uint32_t)val << 16); break;
    case 0x07: b->fg_col = (b->fg_col & 0x00ffffff) | ((uint32_t)val << 24); break;
    case 0x08: b->width = (b->width & 0xff00) | val; break;
    case 0x09: b->width = ((b->width & 0x00ff) | ((uint16_t)val << 8)) & 0x1fff; break;
    case 0x0a: b->height = (b->height & 0xff00) | val; break;
    case 0x0b: b->height = ((b->height & 0x00ff) | ((uint16_t)val << 8)) & 0x07ff; break;
    case 0x0c: b->dst_pitch = (b->dst_pitch & 0xff00) | val; break;
    case 0x0d: b->dst_pitch = ((b->dst_pitch & 0x00ff) | ((uint16_t)val << 8)) & 0x1fff; break;
    case 0x0e: b->src_pitch = (b->src_pitch & 0xff00) | val; break;
    case 0x0f: b->src_pitch = ((b->src_pitch & 0x00ff) | ((uint16_t)val << 8)) & 0x1fff; break;
    case 0x10: b->dst_addr = (b->dst_addr & 0xffff00) | val; break;
    case 0x11: b->dst_addr = (b->dst_addr & 0xff00ff) | ((uint32_t)val << 8); break;
    case 0x12: b->dst_addr = ((b->dst_addr & 0x00ffff) | ((uint32_t)val << 16)) & 0x3fffff; break;
    case 0x14: b->src_addr = (b->src_addr & 0xffff00) | val; break;
    case 0x15: b->src_addr = (b->src_addr & 0xff00ff) | ((uint32_t)val << 8); break;
    case 0x16: b->src_addr = ((b->src_addr & 0x00ffff) | ((uint32_t)val << 16)) & 0x3fffff; break;
    case 0x17: b->mask = val; break;
    case 0x18: b->mode = val; break;
    case 0x1a: b->rop = val; break;
    case 0x1b: b->modeext = val; break;
    case 0x1c: b->trans_col = (b->trans_col & 0xff00) | val; break;
    case 0x1d: b->trans_col = (b->trans_col & 0x00ff) | ((uint16_t)val << 8); break;
    case 0x20: b->trans_mask = (b->trans_mask & 0xff00) | val; break;
    case 0x21: b->trans_mask = (b->trans_mask & 0x00ff) | ((uint16_t)val << 8); break;
    case 0x40: cirrus_blt_trigger(s, val); break;
    default: break;
    }
}

static uint8_t cirrus_blt_reg_read8(VGAState *s, uint32_t off)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;
    switch (off) {
    case 0x00: return b->bg_col & 0xff;
    case 0x01: return (b->bg_col >> 8) & 0xff;
    case 0x02: return (b->bg_col >> 16) & 0xff;
    case 0x03: return (b->bg_col >> 24) & 0xff;
    case 0x04: return b->fg_col & 0xff;
    case 0x05: return (b->fg_col >> 8) & 0xff;
    case 0x06: return (b->fg_col >> 16) & 0xff;
    case 0x07: return (b->fg_col >> 24) & 0xff;
    case 0x08: return b->width & 0xff;
    case 0x09: return (b->width >> 8) & 0xff;
    case 0x0a: return b->height & 0xff;
    case 0x0b: return (b->height >> 8) & 0xff;
    case 0x0c: return b->dst_pitch & 0xff;
    case 0x0d: return (b->dst_pitch >> 8) & 0xff;
    case 0x0e: return b->src_pitch & 0xff;
    case 0x0f: return (b->src_pitch >> 8) & 0xff;
    case 0x10: return b->dst_addr & 0xff;
    case 0x11: return (b->dst_addr >> 8) & 0xff;
    case 0x12: return (b->dst_addr >> 16) & 0xff;
    case 0x14: return b->src_addr & 0xff;
    case 0x15: return (b->src_addr >> 8) & 0xff;
    case 0x16: return (b->src_addr >> 16) & 0xff;
    case 0x17: return b->mask;
    case 0x18: return b->mode;
    case 0x1a: return b->rop;
    case 0x1b: return b->modeext;
    case 0x1c: return b->trans_col & 0xff;
    case 0x1d: return (b->trans_col >> 8) & 0xff;
    case 0x20: return b->trans_mask & 0xff;
    case 0x21: return (b->trans_mask >> 8) & 0xff;
    case 0x40: return b->status;
    default: return 0xff;
    }
}

uint8_t cirrus_blt_mmio_read8(VGAState *s, uint32_t off)
{
    return cirrus_blt_reg_read8(s, off);
}

void cirrus_blt_mmio_write8(VGAState *s, uint32_t off, uint8_t val)
{
    cirrus_blt_reg_write8(s, off, val);
}

/* SR0x17 bit2 (CIRRUS_MMIO_ENABLE, 86Box vid_cl54xx.c:109/1796): the BLT
 * MMIO window only intercepts VRAM accesses once the driver explicitly
 * turns it on. Before that, the same byte range is plain VRAM - without
 * this gate, a guest memory self-test that happens to touch that range
 * before enabling MMIO would have its writes silently swallowed by the
 * BLT registers instead of landing in VRAM. */
int vga_cirrus_mmio_active(VGAState *s)
{
    return s->card_type == VGA_CARD_CIRRUS && (s->cirrus_sr_ext[0x17] & 0x04);
}

/* SR0x17 bit6 (CIRRUS_MMIO_USE_PCIADDR, 86Box vid_cl54xx.c:110): selects
 * which of the two real-hardware MMIO addressing schemes is active when
 * vga_cirrus_mmio_active() is true - the last 256 bytes of the LFB
 * aperture (set) vs. the fixed absolute physical address 0xb8000, outside
 * the LFB entirely (clear). See gd543x_recalc_mapping()/gd54xx_*_linear()
 * in 86Box for both. */
int vga_cirrus_mmio_use_pciaddr(VGAState *s)
{
    return s->cirrus_sr_ext[0x17] & 0x40;
}

/* translation table for the classic extended-GR port path (0x3ce/0x3cf,
 * index > 8) to the canonical MMIO offset space above. */
static int cirrus_gr_to_mmio_off(int gr_index)
{
    switch (gr_index) {
    case 0x10: return 0x01;
    case 0x11: return 0x05;
    case 0x12: return 0x02;
    case 0x13: return 0x06;
    case 0x14: return 0x03;
    case 0x15: return 0x07;
    case 0x20: return 0x08;
    case 0x21: return 0x09;
    case 0x22: return 0x0a;
    case 0x23: return 0x0b;
    case 0x24: return 0x0c;
    case 0x25: return 0x0d;
    case 0x26: return 0x0e;
    case 0x27: return 0x0f;
    case 0x28: return 0x10;
    case 0x29: return 0x11;
    case 0x2a: return 0x12;
    case 0x2c: return 0x14;
    case 0x2d: return 0x15;
    case 0x2e: return 0x16;
    case 0x2f: return 0x17;
    case 0x30: return 0x18;
    case 0x31: return 0x40;
    case 0x32: return 0x1a;
    case 0x33: return 0x1b;
    case 0x34: return 0x1c;
    case 0x35: return 0x1d;
    case 0x38: return 0x20;
    case 0x39: return 0x21;
    default: return -1;
    }
}

/* Extended Sequencer registers (index > 7). Formulas match 86Box
 * vid_cl54xx.c:760-857 (gd54xx_out, case 0x3c5). Memory-size scratch
 * pads (0x0a/0x15), VCLK dividers (0x0b-0x0e/0x1b-0x1e), DRAM control
 * (0x0f), bus-type/MMIO config (0x17), misc control (0x18) and I2C/DDC
 * (0x08) don't affect tiny386's fixed-function rendering - they are
 * stored as plain round-trip state so hardware-detection code that
 * reads them back doesn't see stale/wrong values. The hardware cursor
 * (0x10/0x11 + 7 alias banks, 0x12, 0x13) is functional, see
 * cirrus_cursor_draw(). */
static void cirrus_sr_ext_write(VGAState *s, int index, uint8_t val)
{
    s->cirrus_sr_ext[index & 0xff] = val;
    switch (index) {
    case 0x10: case 0x30: case 0x50: case 0x70:
    case 0x90: case 0xb0: case 0xd0: case 0xf0:
        s->cirrus_cursor.x = ((int)val << 3) | (index >> 5);
        break;
    case 0x11: case 0x31: case 0x51: case 0x71:
    case 0x91: case 0xb1: case 0xd1: case 0xf1:
        s->cirrus_cursor.y = ((int)val << 3) | (index >> 5);
        break;
    case 0x12:
        s->cirrus_cursor.ena = val & 0x01; /* CIRRUS_CURSOR_SHOW */
        s->cirrus_cursor.large = val & 0x04; /* CIRRUS_CURSOR_LARGE */
        if (s->cirrus_cursor.large)
            s->cirrus_cursor.addr = (s->vga_ram_size - 0x4000) +
                (s->cirrus_sr_ext[0x13] & 0x3c) * 256;
        else
            s->cirrus_cursor.addr = (s->vga_ram_size - 0x4000) +
                (s->cirrus_sr_ext[0x13] & 0x3f) * 256;
        break;
    case 0x13:
        if (s->cirrus_cursor.large)
            s->cirrus_cursor.addr = (s->vga_ram_size - 0x4000) + (val & 0x3c) * 256;
        else
            s->cirrus_cursor.addr = (s->vga_ram_size - 0x4000) + (val & 0x3f) * 256;
        break;
    default:
        break;
    }
}

static uint8_t cirrus_sr_ext_read(VGAState *s, int index)
{
    /* Memory-size scratch pads and bus-type/DRAM-width fields report the
     * actual configured VRAM size / bus type so guest detection code
     * sees consistent values, matching the exact bit encodings 86Box
     * uses for chip ID >= CIRRUS_ID_CLGD5430 (vid_cl54xx.c:1324-1422). */
    int kb = s->vga_ram_size >> 10;
    switch (index) {
    case 0x08: { /* I2C/DDC GPIO read-back (different bit positions
                  * than the write side - 86Box vid_cl54xx.c:1315-1323) */
        uint8_t ret = s->cirrus_sr_ext[0x08] & 0x7b;
        if (s->cirrus_sr_ext[0x08] & 0x01) ret |= 0x04; /* SCL */
        if (s->cirrus_sr_ext[0x08] & 0x02) ret |= 0x80; /* SDA */
        return ret;
    }
    case 0x0a: { /* Scratch Pad 1 (memory size, 5402/542x-style) */
        uint8_t ret = s->cirrus_sr_ext[0x0a] & ~0x1a;
        if (kb == 512) ret |= 0x08;
        else if (kb == 1024) ret |= 0x10;
        else if (kb == 2048) ret |= 0x18;
        return ret;
    }
    case 0x0f: { /* DRAM control (bus width) */
        uint8_t ret = s->cirrus_sr_ext[0x0f] & ~0x98;
        if (kb == 512) ret |= 0x08;
        else if (kb == 1024) ret |= 0x10;
        else if (kb == 2048) ret |= 0x18;
        else if (kb == 4096) ret |= 0x98;
        return ret;
    }
    case 0x15: { /* Scratch Pad 3 (memory size, 543x-style) */
        uint8_t ret = s->cirrus_sr_ext[0x15] & ~0x0f;
        int mb = s->vga_ram_size >> 20;
        if (mb == 1) ret |= 0x02;
        else if (mb == 2) ret |= 0x03;
        else if (mb == 4) ret |= 0x04;
        return ret;
    }
    case 0x17: /* bus type: we're always the PCI variant */
        return (s->cirrus_sr_ext[0x17] & ~(7 << 3)) | (4 << 3);
    case 0x18:
        return s->cirrus_sr_ext[0x18] & 0xfe;
    default:
        return s->cirrus_sr_ext[index & 0xff];
    }
}

static void cirrus_gr_write(VGAState *s, int index, uint8_t val)
{
    if (index <= 8) {
        if (index < 16)
            s->gr[index] = val & gr_mask[index];
        if (index == 0)
            cirrus_blt_reg_write8(s, 0x00, val);
        else if (index == 1)
            cirrus_blt_reg_write8(s, 0x04, val);
        return;
    }
    if (index >= 0x09 && index <= 0x0b) {
        s->cirrus_bank_reg[index - 0x09] = val;
        return;
    }
    if (index >= 0x0c && index <= 0x0e) {
        s->cirrus_gr_ext[index - 0x0c] = val;
        return;
    }
    /* Real hardware (and 86Box) gates SR06-unlock only on the legacy
     * shadow-register readback, not on whether the underlying extended
     * register actually does its job - the BLT engine and banking
     * registers are always live. Gating this dispatch on cirrus_unlocked
     * (as this code used to do) made the real Cirrus option ROM's own
     * POST routine - which pokes several of these registers before it
     * ever unlocks SR06 - silently no-op, which is what was driving it
     * into a reset/retry loop. */
    {
        int off = cirrus_gr_to_mmio_off(index);
        if (off >= 0)
            cirrus_blt_reg_write8(s, off, val);
#ifdef DEBUG_CIRRUS
        else
            printf("cirrus: gr_write unknown index=0x%02x val=0x%02x\n", index, val);
#endif
    }
}

static uint8_t cirrus_gr_read(VGAState *s, int index)
{
    /* GR0/GR1 are read back through the BLT engine's bg_col/fg_col
     * (86Box vid_cl54xx.c:1618-1621, gd543x_mmio_read of 0xb8000/
     * 0xb8004), not the plain shadow array - a guest that sets a color
     * via direct MMIO and reads it back via the legacy port must see
     * the same value. s->gr[0]/[1] still get written for symmetry but
     * are never the read source. */
    if (index == 0)
        return cirrus_blt_reg_read8(s, 0x00);
    if (index == 1)
        return cirrus_blt_reg_read8(s, 0x04);
    if (index <= 8)
        return index < 16 ? s->gr[index] : 0xff;
    if (index >= 0x09 && index <= 0x0b)
        return s->cirrus_bank_reg[index - 0x09];
    if (index >= 0x0c && index <= 0x0e)
        return s->cirrus_gr_ext[index - 0x0c];
    {
        int off = cirrus_gr_to_mmio_off(index);
        if (off >= 0)
            return cirrus_blt_reg_read8(s, off);
    }
    return 0xff;
}

uint32_t vga_ioport_read(VGAState *s, uint32_t addr)
{
    int val, index;

    /* check port range access depending on color/monochrome mode */
    if ((addr >= 0x3b0 && addr <= 0x3bf && (s->msr & MSR_COLOR_EMULATION)) ||
        (addr >= 0x3d0 && addr <= 0x3df && !(s->msr & MSR_COLOR_EMULATION))) {
        val = 0xff;
    } else {
        switch(addr) {
        case 0x3c0:
            if (s->ar_flip_flop == 0) {
                val = s->ar_index;
            } else {
                val = 0;
            }
            break;
        case 0x3c1:
            index = s->ar_index & 0x1f;
            if (index < 21)
                val = s->ar[index];
            else
                val = 0;
            break;
        case 0x3c2:
            val = s->st00;
            break;
        case 0x3c4:
            /* Reading the index port back while a cursor X/Y register
             * (0x10/0x11 or any of their 7 alias banks) is selected
             * and the chip is unlocked returns packed position bits
             * instead of the plain index (86Box vid_cl54xx.c:1291-
             * 1302) - an oddity of real silicon some detection code
             * checks for. */
            if (s->card_type == VGA_CARD_CIRRUS && s->sr[6] == 0x12 &&
                (s->sr_index & 0x1e) == 0x10) {
                if (s->sr_index & 1)
                    val = ((s->cirrus_cursor.y & 7) << 5) | 0x11;
                else
                    val = ((s->cirrus_cursor.x & 7) << 5) | 0x10;
            } else {
                val = s->sr_index;
            }
            break;
        case 0x3c5:
            if (s->card_type == VGA_CARD_CIRRUS && s->sr_index > 7)
                val = cirrus_sr_ext_read(s, s->sr_index);
            else
                val = s->sr[s->sr_index];
#ifdef DEBUG_VGA_REG
            printf("vga: read SR%x = 0x%02x\n", s->sr_index, val);
#endif
            break;
        case 0x3c6:
            if (s->card_type != VGA_CARD_CIRRUS) {
                val = 0x00; /* unhandled on the plain Bochs path, as before */
            } else if (s->cirrus_unlocked) {
                if (s->cirrus_dac_state == 4) {
                    s->cirrus_dac_state = 0; /* GD5430 isn't CLGD5428, always resets here */
                    val = s->cirrus_dac_hidden;
                } else {
                    s->cirrus_dac_state++;
                    val = (s->cirrus_dac_state == 4) ? s->cirrus_dac_hidden : 0xff;
                }
            } else {
                val = 0xff;
            }
            break;
        case 0x3c7:
            val = s->dac_state;
            s->cirrus_dac_state = 0;
            break;
        case 0x3c8:
            val = s->dac_write_index;
            s->cirrus_dac_state = 0;
            break;
        case 0x3c9:
            s->cirrus_dac_state = 0;
            if (s->card_type == VGA_CARD_CIRRUS && (s->cirrus_sr_ext[0x12] & 0x02)) {
                int idx = s->dac_read_index & 0x0f;
                val = s->cirrus_ext_palette[idx * 3 + s->dac_sub_index] & 0x3f;
            } else {
                val = s->palette[s->dac_read_index * 3 + s->dac_sub_index];
            }
            if (++s->dac_sub_index == 3) {
                s->dac_sub_index = 0;
                s->dac_read_index++;
            }
            break;
        case 0x3ca:
            val = s->fcr;
            break;
        case 0x3cc:
            val = s->msr;
            break;
        case 0x3ce:
            val = (s->card_type == VGA_CARD_CIRRUS) ? (s->gr_index & 0x3f) : s->gr_index;
            break;
        case 0x3cf:
            if (s->card_type == VGA_CARD_CIRRUS)
                val = cirrus_gr_read(s, s->gr_index);
            else
                val = s->gr[s->gr_index];
#ifdef DEBUG_VGA_REG
            printf("vga: read GR%x = 0x%02x\n", s->gr_index, val);
#endif
            break;
        case 0x3b4:
        case 0x3d4:
            val = s->cr_index;
            break;
        case 0x3b5:
        case 0x3d5:
            /* CR0x27 is the hardwired chip ID on real Cirrus silicon; the
             * in-box NT/9x drivers read it back to tell GD5430/34/36/46
             * apart. Nothing else sets this (no per-device option ROM in
             * tiny386), so without this the register reads back 0x00,
             * which isn't a valid ID and confuses the driver. */
            if (s->card_type == VGA_CARD_CIRRUS && s->cr_index == 0x27) {
                val = 0xa0; /* CIRRUS_ID_CLGD5430 */
            } else if (s->card_type == VGA_CARD_CIRRUS && s->cr_index == 0x28) {
                /* "Class ID" - always 0xff on real GD5430/5440 silicon
                 * (86Box vid_cl54xx.c:1652-1656). */
                val = 0xff;
            } else if (s->card_type == VGA_CARD_CIRRUS && s->cr_index == 0x22) {
                /* Graphics Data Latches Readback Register: byte
                 * (GR4 & 3) of the latch loaded by the last VRAM read
                 * (86Box vid_cl54xx.c:1639-1642). */
                val = (s->latch >> (8 * (s->gr[4] & 3))) & 0xff;
            } else if (s->card_type == VGA_CARD_CIRRUS && s->cr_index == 0x24) {
                /* Attribute controller flip-flop readback (86Box
                 * vid_cl54xx.c:1643-1645). */
                val = s->ar_flip_flop << 7;
            } else if (s->card_type == VGA_CARD_CIRRUS && s->cr_index == 0x26) {
                /* Attribute controller index readback (86Box
                 * vid_cl54xx.c:1646-1648). */
                val = s->ar_index & 0x3f;
            } else {
                val = s->cr[s->cr_index];
            }
#ifdef DEBUG_VGA_REG
            printf("vga: read CR%x = 0x%02x\n", s->cr_index, val);
#endif
            break;
        case 0x3ba:
        case 0x3da:
            /* just toggle to fool polling */
//            s->st01 ^= ST01_V_RETRACE | ST01_DISP_ENABLE;
            val = s->st01;
            s->ar_flip_flop = 0;
#ifdef DEBUG_VGA_REG
            printf("vga: read ST01 = 0x%02x\n", val);
#endif
            break;
        default:
            val = 0x00;
            break;
        }
    }
#if defined(DEBUG_VGA)
    printf("VGA: read addr=0x%04x data=0x%02x\n", addr, val);
#endif
    return val;
}

void vga_ioport_write(VGAState *s, uint32_t addr, uint32_t val)
{
    int index;

    /* check port range access depending on color/monochrome mode */
    if ((addr >= 0x3b0 && addr <= 0x3bf && (s->msr & MSR_COLOR_EMULATION)) ||
        (addr >= 0x3d0 && addr <= 0x3df && !(s->msr & MSR_COLOR_EMULATION)))
        return;

#ifdef DEBUG_VGA
    printf("VGA: write addr=0x%04x data=0x%02x\n", addr, val);
#endif

    switch(addr) {
    case 0x3c0:
        if (s->ar_flip_flop == 0) {
            val &= 0x3f;
            s->ar_index = val;
        } else {
            index = s->ar_index & 0x1f;
            switch(index) {
            case 0x00 ... 0x0f:
                s->ar[index] = val & 0x3f;
                break;
            case 0x10:
                s->ar[index] = val & ~0x10;
                break;
            case 0x11:
                s->ar[index] = val;
                break;
            case 0x12:
                s->ar[index] = val & ~0xc0;
                break;
            case 0x13:
                s->ar[index] = val & ~0xf0;
                break;
            case 0x14:
                s->ar[index] = val & ~0xf0;
                break;
            default:
                break;
            }
        }
        s->ar_flip_flop ^= 1;
        break;
    case 0x3c2:
        s->msr = val & ~0x10;
        break;
    case 0x3c4:
        s->sr_index = (s->card_type == VGA_CARD_CIRRUS) ? val : (val & 7);
        break;
    case 0x3c5:
#ifdef DEBUG_VGA_REG
        printf("vga: write SR%x = 0x%02x\n", s->sr_index, val);
#endif
        if (s->card_type == VGA_CARD_CIRRUS && s->sr_index == 6) {
            /* SR06 is the Cirrus unlock register, not the reserved
             * standard-VGA register sr_mask[6]=0x00 assumes. Real
             * silicon stores only 0x12 (exact key match) or 0x0f back,
             * never the raw written value (86Box vid_cl54xx.c:763-768).
             * GD5430's chip ID (0xa0) is >= CIRRUS_ID_CLGD5429 (0x9c),
             * so the extended-register lock is hardwired open from
             * reset on this chip and SR06 never actually changes it -
             * cirrus_unlocked is set once at card-type init and left
             * alone here; only the read-back shadow is updated. */
            uint8_t masked = val & 0x17;
            s->sr[6] = (masked == 0x12) ? 0x12 : 0x0f;
#ifdef DEBUG_CIRRUS
            printf("cirrus: SR06=0x%02x -> shadow=0x%02x unlocked=%d\n", val, s->sr[6], s->cirrus_unlocked);
#endif
        } else if (s->card_type == VGA_CARD_CIRRUS && s->sr_index > 7) {
            cirrus_sr_ext_write(s, s->sr_index, val);
        } else {
            s->sr[s->sr_index] = val & sr_mask[s->sr_index];
        }
        break;
    case 0x3c6:
        /* Hidden DAC control register: 4 consecutive reads of 0x3c6
         * (with no intervening 0x3c7/0x3c8/0x3c9 access) prime the
         * state machine, then a write captures the value (86Box
         * vid_cl54xx.c:865-874). Only meaningful when unlocked, which
         * is always true on GD5430 - see cirrus_unlocked above. We
         * store the register faithfully but don't wire it to a real
         * SVGA packed-pixel color-depth renderer (out of scope, same
         * as the rest of the BPP>8 path - see vga_set_card_type). */
        if (s->card_type == VGA_CARD_CIRRUS && s->cirrus_unlocked) {
            if (s->cirrus_dac_state == 4)
                s->cirrus_dac_hidden = val;
            s->cirrus_dac_state = 0;
        }
        break;
    case 0x3c7:
        s->dac_read_index = val;
        s->dac_sub_index = 0;
        s->dac_state = 3;
        s->cirrus_dac_state = 0;
        break;
    case 0x3c8:
        s->dac_write_index = val;
        s->dac_sub_index = 0;
        s->dac_state = 0;
        s->cirrus_dac_state = 0;
        break;
    case 0x3c9:
        s->dac_cache[s->dac_sub_index] = val;
        s->cirrus_dac_state = 0;
        if (++s->dac_sub_index == 3) {
            /* SR0x12 bit1: redirect palette writes to the 16-entry
             * extended palette instead of the standard 256-entry one
             * (86Box vid_cl54xx.c:895-908) - used for the hardware
             * cursor's background/foreground colors (entries 0/0xf). */
            if (s->card_type == VGA_CARD_CIRRUS && (s->cirrus_sr_ext[0x12] & 0x02)) {
                int idx = s->dac_write_index & 0x0f;
                memcpy(&s->cirrus_ext_palette[idx * 3], s->dac_cache, 3);
            } else {
                memcpy(&s->palette[s->dac_write_index * 3], s->dac_cache, 3);
            }
            s->dac_sub_index = 0;
            s->dac_write_index++;
        }
        break;
    case 0x3ce:
        s->gr_index = (s->card_type == VGA_CARD_CIRRUS) ? val : (val & 0x0f);
        break;
    case 0x3cf:
#ifdef DEBUG_VGA_REG
        printf("vga: write GR%x = 0x%02x\n", s->gr_index, val);
#endif
        if (s->gr_index == 0x06)
            s->comp_ntsc = 0;
        if (s->card_type == VGA_CARD_CIRRUS)
            cirrus_gr_write(s, s->gr_index, val);
        else
            s->gr[s->gr_index] = val & gr_mask[s->gr_index];
        break;
    case 0x3b4:
    case 0x3d4:
        s->cr_index = val;
        break;
    case 0x3b5:
    case 0x3d5:
#ifdef DEBUG_VGA_REG
        printf("vga: write CR%x = 0x%02x\n", s->cr_index, val);
#endif
        /* handle CR0-7 protection */
        if ((s->cr[0x11] & 0x80) && s->cr_index <= 7) {
            /* can always write bit 4 of CR7 */
            if (s->cr_index == 7)
                s->cr[7] = (s->cr[7] & ~0x10) | (val & 0x10);
            return;
        }
        switch(s->cr_index) {
        case 0x01: /* horizontal display end */
        case 0x07:
        case 0x09:
        case 0x0c:
        case 0x0d:
        case 0x12: /* vertical display end */
            s->cr[s->cr_index] = val;
            break;
        default:
            s->cr[s->cr_index] = val;
            break;
        }
        break;
    case 0x3ba:
    case 0x3da:
        s->fcr = val & 0x10;
        break;
    case 0x3d8:
        s->comp_ntsc = !(val & 4);
        fprintf(stderr, "comp_ntsc = %d\n", s->comp_ntsc);
        break;
    }
}

#define VGA_IO(base) \
static uint32_t vga_read_ ## base(void *opaque, uint32_t addr, int size_log2)\
{\
    return vga_ioport_read(opaque, base + addr);\
}\
static void vga_write_ ## base(void *opaque, uint32_t addr, uint32_t val, int size_log2)\
{\
    return vga_ioport_write(opaque, base + addr, val);\
}

void vbe_write(VGAState *s, uint32_t offset, uint32_t val)
{
    if (offset == 0) {
        s->vbe_index = val;
    } else {
#ifdef DEBUG_VBE
        printf("VBE write: index=0x%04x val=0x%04x\n", s->vbe_index, val);
#endif
        switch(s->vbe_index) {
        case VBE_DISPI_INDEX_ID:
            if (val >= VBE_DISPI_ID0 && val <= VBE_DISPI_ID5)
                s->vbe_regs[s->vbe_index] = val;
            break;
        case VBE_DISPI_INDEX_ENABLE:
            if ((val & VBE_DISPI_ENABLED) &&
                !(s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_ENABLED)) {
                s->vbe_regs[VBE_DISPI_INDEX_VIRT_WIDTH] =
                    s->vbe_regs[VBE_DISPI_INDEX_XRES];
                s->vbe_regs[VBE_DISPI_INDEX_VIRT_HEIGHT] =
                    s->vbe_regs[VBE_DISPI_INDEX_YRES];
                s->vbe_regs[VBE_DISPI_INDEX_X_OFFSET] = 0;
                s->vbe_regs[VBE_DISPI_INDEX_Y_OFFSET] = 0;
            } else {
                s->bank_offset = 0;
            }
            s->dac_8bit = (val & VBE_DISPI_8BIT_DAC) > 0;
            s->vbe_regs[s->vbe_index] = val;
            vbe_fixup_regs(s);
            vbe_update_vgaregs(s);
            /* clear the screen */
            if (!(val & VBE_DISPI_NOCLEARMEM)) {
                memset(s->vga_ram, 0,
                       s->vbe_regs[VBE_DISPI_INDEX_YRES] * s->vbe_line_offset);
            }
            break;
        case VBE_DISPI_INDEX_XRES:
        case VBE_DISPI_INDEX_YRES:
        case VBE_DISPI_INDEX_BPP:
        case VBE_DISPI_INDEX_VIRT_WIDTH:
        case VBE_DISPI_INDEX_VIRT_HEIGHT:
        case VBE_DISPI_INDEX_X_OFFSET:
        case VBE_DISPI_INDEX_Y_OFFSET:
            s->vbe_regs[s->vbe_index] = val;
            vbe_fixup_regs(s);
            vbe_update_vgaregs(s);
            break;
        case VBE_DISPI_INDEX_BANK:
            val &= (s->vga_ram_size >> 16) - 1;
            s->vbe_regs[s->vbe_index] = val;
            s->bank_offset = (val << 16);
            break;
        }
    }
}

uint32_t vbe_read(VGAState *s, uint32_t offset)
{
    uint32_t val;

    if (offset == 0) {
        val = s->vbe_index;
    } else {
        if (s->vbe_regs[VBE_DISPI_INDEX_ENABLE] & VBE_DISPI_GETCAPS) {
            switch(s->vbe_index) {
            case VBE_DISPI_INDEX_XRES:
#ifdef SCALE_3_2
                val = s->fb_dev->width * 3 / 2;
#else
                val = s->fb_dev->width;
#endif
                break;
            case VBE_DISPI_INDEX_YRES:
#ifdef SCALE_3_2
                val = s->fb_dev->height * 3 / 2;
#else
                val = s->fb_dev->height;
#endif
                break;
            case VBE_DISPI_INDEX_BPP:
                val = 32;
                break;
            default:
                goto read_reg;
            }
        } else {
        read_reg:
            if (s->vbe_index < VBE_DISPI_INDEX_NB)
                val = s->vbe_regs[s->vbe_index];
            else
                val = 0;
        }
#ifdef DEBUG_VBE
        printf("VBE read: index=0x%04x val=0x%04x\n", s->vbe_index, val);
#endif
    }
    return val;
}

#define cbswap_32(__x) \
((uint32_t)( \
                (((uint32_t)(__x) & (uint32_t)0x000000ffUL) << 24) | \
                (((uint32_t)(__x) & (uint32_t)0x0000ff00UL) <<  8) | \
                (((uint32_t)(__x) & (uint32_t)0x00ff0000UL) >>  8) | \
                (((uint32_t)(__x) & (uint32_t)0xff000000UL) >> 24) ))

#ifdef HOST_WORDS_BIGENDIAN
#define PAT(x) cbswap_32(x)
#else
#define PAT(x) (x)
#endif

#ifdef HOST_WORDS_BIGENDIAN
#define GET_PLANE(data, p) (((data) >> (24 - (p) * 8)) & 0xff)
#else
#define GET_PLANE(data, p) (((data) >> ((p) * 8)) & 0xff)
#endif

static const uint32_t mask16[16] = {
    PAT(0x00000000),
    PAT(0x000000ff),
    PAT(0x0000ff00),
    PAT(0x0000ffff),
    PAT(0x00ff0000),
    PAT(0x00ff00ff),
    PAT(0x00ffff00),
    PAT(0x00ffffff),
    PAT(0xff000000),
    PAT(0xff0000ff),
    PAT(0xff00ff00),
    PAT(0xff00ffff),
    PAT(0xffff0000),
    PAT(0xffff00ff),
    PAT(0xffffff00),
    PAT(0xffffffff),
};

#define VGA_SEQ_RESET           0x00
#define VGA_SEQ_CLOCK_MODE      0x01
#define VGA_SEQ_PLANE_WRITE     0x02
#define VGA_SEQ_CHARACTER_MAP   0x03
#define VGA_SEQ_MEMORY_MODE     0x04

#define VGA_SR01_CHAR_CLK_8DOTS 0x01 /* bit 0: character clocks 8 dots wide are generated */
#define VGA_SR01_SCREEN_OFF     0x20 /* bit 5: Screen is off */
#define VGA_SR02_ALL_PLANES     0x0F /* bits 3-0: enable access to all planes */
#define VGA_SR04_EXT_MEM        0x02 /* bit 1: allows complete mem access to 256K */
#define VGA_SR04_SEQ_MODE       0x04 /* bit 2: directs system to use a sequential addressing mode */
#define VGA_SR04_CHN_4M         0x08 /* bit 3: selects modulo 4 addressing for CPU access to display memory */

#define VGA_GFX_SR_VALUE        0x00
#define VGA_GFX_SR_ENABLE       0x01
#define VGA_GFX_COMPARE_VALUE   0x02
#define VGA_GFX_DATA_ROTATE     0x03
#define VGA_GFX_PLANE_READ      0x04
#define VGA_GFX_MODE            0x05
#define VGA_GFX_MISC            0x06
#define VGA_GFX_COMPARE_MASK    0x07
#define VGA_GFX_BIT_MASK        0x08

/* Cirrus extended bank registers (GR0x09/0x0A/0x0B) split the 64K window
 * at A0000 into two independently bankable 32K halves selected by addr
 * bit 15, per 86Box's gd54xx_recalc_banking()/gd54xx_write(). */
static inline uint32_t cirrus_banked_addr(VGAState *s, uint32_t addr)
{
    int gr0b = s->cirrus_bank_reg[2];
    int shift = (gr0b & 0x20) ? 14 : 12;
    uint32_t bank0 = s->cirrus_bank_reg[0] << shift;
    uint32_t bank1 = (gr0b & 0x01) ? (s->cirrus_bank_reg[1] << shift)
                                    : (bank0 + 0x8000);
    return (addr & 0x7fff) + (((addr >> 15) & 1) ? bank1 : bank0);
}

//#define DEBUG_VGA_MEM
//#define TARGET_FMT_plx "%x"
void IRAM_ATTR vga_mem_write16(VGAState *s, uint32_t addr, uint16_t val16)
{
    if (!(s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M)) {
        vga_mem_write(s, addr, val16);
        vga_mem_write(s, addr + 1, val16 >> 8);
        return;
    }
    uint32_t val = val16;

    int memory_map_mode, plane, mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif
    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return;
        addr = (s->card_type == VGA_CARD_CIRRUS) ? cirrus_banked_addr(s, addr) : (addr + s->bank_offset);
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return;
        break;
    }

    /* chain 4 mode : simplest access */
    plane = addr & 3;
    mask = (1 << plane);
    if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
        * (uint16_t *) &(s->vga_ram[addr]) = val;
    }
}

void IRAM_ATTR vga_mem_write32(VGAState *s, uint32_t addr, uint32_t val)
{
    if (!(s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M)) {
        vga_mem_write(s, addr, val);
        vga_mem_write(s, addr + 1, val >> 8);
        vga_mem_write(s, addr + 2, val >> 16);
        vga_mem_write(s, addr + 3, val >> 24);
        return;
    }

    int memory_map_mode, plane, mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif
    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return;
        addr = (s->card_type == VGA_CARD_CIRRUS) ? cirrus_banked_addr(s, addr) : (addr + s->bank_offset);
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return;
        break;
    }

    /* chain 4 mode : simplest access */
    plane = addr & 3;
    mask = (1 << plane);
    if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
        * (uint32_t *) &(s->vga_ram[addr]) = val;
    }
}

bool IRAM_ATTR vga_mem_write_string(VGAState *s, uint32_t addr, uint8_t *buf, int len)
{
    if (!(s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M)) {
        return false;
    }

    int memory_map_mode, plane, mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif
    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return false;
        addr = (s->card_type == VGA_CARD_CIRRUS) ? cirrus_banked_addr(s, addr) : (addr + s->bank_offset);
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return false;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return false;
        break;
    }

    /* chain 4 mode : simplest access */
    plane = addr & 3;
    mask = (1 << plane);
    if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
        memcpy(s->vga_ram + addr, buf, len);
        return true;
    }
    return false;
}

void IRAM_ATTR vga_mem_write(VGAState *s, uint32_t addr, uint8_t val8)
{
    uint32_t val = val8;

    int memory_map_mode, plane, write_mode, b, func_select, mask;
    uint32_t write_mask, bit_mask, set_mask;

#ifdef DEBUG_VGA_MEM
    printf("vga: [0x" TARGET_FMT_plx "] = 0x%02x\n", addr, val);
#endif
    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return;
        addr = (s->card_type == VGA_CARD_CIRRUS) ? cirrus_banked_addr(s, addr) : (addr + s->bank_offset);
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return;
        break;
    }

    if (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
        /* chain 4 mode : simplest access */
        plane = addr & 3;
        mask = (1 << plane);
        if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
            s->vga_ram[addr] = val;
#ifdef DEBUG_VGA_MEM
            printf("vga: chain4: [0x" TARGET_FMT_plx "]\n", addr);
#endif
//            s->plane_updated |= mask; /* only used to detect font change */
//            memory_region_set_dirty(&s->vram, addr, 1);
        }
    } else if (s->gr[VGA_GFX_MODE] & 0x10) {
        /* odd/even mode (aka text mode mapping) */
        plane = (s->gr[VGA_GFX_PLANE_READ] & 2) | (addr & 1);
        mask = (1 << plane);
        if (s->sr[VGA_SEQ_PLANE_WRITE] & mask) {
            addr = ((addr & ~1) << 1) | plane;
            if (addr >= s->vga_ram_size) {
                return;
            }
            s->vga_ram[addr] = val;
#ifdef DEBUG_VGA_MEM
            printf("vga: odd/even: [0x" TARGET_FMT_plx "]\n", addr);
#endif
//            s->plane_updated |= mask; /* only used to detect font change */
//            memory_region_set_dirty(&s->vram, addr, 1);
        }
    } else {
        /* standard VGA latched access */
        write_mode = s->gr[VGA_GFX_MODE] & 3;
        switch(write_mode) {
        default:
        case 0:
            /* rotate */
            b = s->gr[VGA_GFX_DATA_ROTATE] & 7;
            val = ((val >> b) | (val << (8 - b))) & 0xff;
            val |= val << 8;
            val |= val << 16;

            /* apply set/reset mask */
            set_mask = mask16[s->gr[VGA_GFX_SR_ENABLE]];
            val = (val & ~set_mask) |
                (mask16[s->gr[VGA_GFX_SR_VALUE]] & set_mask);
            bit_mask = s->gr[VGA_GFX_BIT_MASK];
            break;
        case 1:
            val = s->latch;
            goto do_write;
        case 2:
            val = mask16[val & 0x0f];
            bit_mask = s->gr[VGA_GFX_BIT_MASK];
            break;
        case 3:
            /* rotate */
            b = s->gr[VGA_GFX_DATA_ROTATE] & 7;
            val = (val >> b) | (val << (8 - b));

            bit_mask = s->gr[VGA_GFX_BIT_MASK] & val;
            val = mask16[s->gr[VGA_GFX_SR_VALUE]];
            break;
        }

        /* apply logical operation */
        func_select = s->gr[VGA_GFX_DATA_ROTATE] >> 3;
        switch(func_select) {
        case 0:
        default:
            /* nothing to do */
            break;
        case 1:
            /* and */
            val &= s->latch;
            break;
        case 2:
            /* or */
            val |= s->latch;
            break;
        case 3:
            /* xor */
            val ^= s->latch;
            break;
        }

        /* apply bit mask */
        bit_mask |= bit_mask << 8;
        bit_mask |= bit_mask << 16;
        val = (val & bit_mask) | (s->latch & ~bit_mask);

    do_write:
        /* mask data according to sr[2] */
        mask = s->sr[VGA_SEQ_PLANE_WRITE];
//        s->plane_updated |= mask; /* only used to detect font change */
        write_mask = mask16[mask];
        if (addr * sizeof(uint32_t) >= s->vga_ram_size) {
            return;
        }
        ((uint32_t *)s->vga_ram)[addr] =
            (((uint32_t *)s->vga_ram)[addr] & ~write_mask) |
            (val & write_mask);
#ifdef DEBUG_VGA_MEM
        printf("vga: latch: [0x" TARGET_FMT_plx "] mask=0x%08x val=0x%08x\n",
               addr * 4, write_mask, val);
#endif
//        memory_region_set_dirty(&s->vram, addr << 2, sizeof(uint32_t));
    }
}

uint8_t vga_mem_read(VGAState *s, uint32_t addr)
{
    int memory_map_mode, plane;
    uint32_t ret;

    /* convert to VGA memory offset */
    memory_map_mode = (s->gr[VGA_GFX_MISC] >> 2) & 3;
    addr &= 0x1ffff;
    switch(memory_map_mode) {
    case 0:
        break;
    case 1:
        if (addr >= 0x10000)
            return 0xff;
        addr = (s->card_type == VGA_CARD_CIRRUS) ? cirrus_banked_addr(s, addr) : (addr + s->bank_offset);
        break;
    case 2:
        addr -= 0x10000;
        if (addr >= 0x8000)
            return 0xff;
        break;
    default:
    case 3:
        addr -= 0x18000;
        if (addr >= 0x8000)
            return 0xff;
        break;
    }

    if (s->sr[VGA_SEQ_MEMORY_MODE] & VGA_SR04_CHN_4M) {
        /* chain 4 mode : simplest access */
//        assert(addr < s->vram_size);
        ret = s->vga_ram[addr];
    } else if (s->gr[VGA_GFX_MODE] & 0x10) {
        /* odd/even mode (aka text mode mapping) */
        plane = (s->gr[VGA_GFX_PLANE_READ] & 2) | (addr & 1);
        addr = ((addr & ~1) << 1) | plane;
        if (addr >= s->vga_ram_size) { // s->vram_size) {
            return 0xff;
        }
        ret = s->vga_ram[addr];
    } else {
        /* standard VGA latched access */
        if (addr * sizeof(uint32_t) >= s->vga_ram_size) {//s->vram_size) {
            return 0xff;
        }
        s->latch = ((uint32_t *)s->vga_ram)[addr];

        if (!(s->gr[VGA_GFX_MODE] & 0x08)) {
            /* read mode 0 */
            plane = s->gr[VGA_GFX_PLANE_READ];
            ret = GET_PLANE(s->latch, plane);
        } else {
            /* read mode 1 */
            ret = (s->latch ^ mask16[s->gr[VGA_GFX_COMPARE_VALUE]]) &
                mask16[s->gr[VGA_GFX_COMPARE_MASK]];
            ret |= ret >> 16;
            ret |= ret >> 8;
            ret = (~ret) & 0xff;
        }
    }
    return ret;
}

static void vga_initmode(VGAState *s);

VGAState *vga_init(char *vga_ram, int vga_ram_size,
                   uint8_t *fb, int width, int height)
{
    VGAState *s;

    s = pcmalloc(sizeof(*s));
    memset(s, 0, sizeof(*s));
    FBDevice *fb_dev = pcmalloc(sizeof(FBDevice));
    s->fb_dev = fb_dev;
    memset(s->fb_dev, 0, sizeof(FBDevice));
    s->graphic_mode = 0;
    s->cursor_blink_time = get_uticks();
    s->cursor_visible_phase = 1;
    s->retrace_time = get_uticks();
    s->retrace_phase = 0;
    fb_dev->width = width;
    fb_dev->height = height;
#ifdef SWAPXY
    fb_dev->stride = height * (BPP / 8);
#else
    fb_dev->stride = width * (BPP / 8);
#endif
    fb_dev->fb_data = fb;

    s->vga_ram = (uint8_t *) vga_ram;
    s->vga_ram_size = vga_ram_size;

    s->vbe_regs[VBE_DISPI_INDEX_ID] = VBE_DISPI_ID5;
    s->vbe_regs[VBE_DISPI_INDEX_VIDEO_MEMORY_64K] = s->vga_ram_size >> 16;

    vga_initmode(s);
    return s;
}

void vga_set_force_8dm(VGAState *s, int v)
{
    s->force_8dm = v;
}

void vga_set_card_type(VGAState *s, int card_type)
{
    s->card_type = card_type;
    /* GD5430 (CR0x27=0xa0) is >= CIRRUS_ID_CLGD5429: the extended
     * register lock is hardwired open from reset on real silicon and
     * SR06 writes never change it (86Box vid_cl54xx.c:4211-4214). */
    if (card_type == VGA_CARD_CIRRUS)
        s->cirrus_unlocked = 1;
}

PCIDevice *vga_pci_init(VGAState *s, PCIBus *bus,
                        void *o, void (*set_bar)(void *, int, uint32_t, bool))
{
    PCIDevice *d;
    if (s->card_type == VGA_CARD_CIRRUS) {
        /* Cirrus Logic GD5430 */
        d = pci_register_device(bus, "VGA", -1, 0x1013, 0x00a0, 0x00, 0x0300);
    } else {
        d = pci_register_device(bus, "VGA", -1, 0x1234, 0x1111, 0x00, 0x0300);
    }

    uint32_t bar_size;
    bar_size = 1;
    while (bar_size < s->vga_ram_size)
        bar_size <<= 1;
    pci_register_bar(d, 0, bar_size, PCI_ADDRESS_SPACE_MEM, o, set_bar);
    return d;
}

// from vgabios
// stdvga mode 2
const static uint8_t pal_ega[] = {
    0x00,0x00,0x00, 0x00,0x00,0x2a, 0x00,0x2a,0x00, 0x00,0x2a,0x2a,
    0x2a,0x00,0x00, 0x2a,0x00,0x2a, 0x2a,0x2a,0x00, 0x2a,0x2a,0x2a,
    0x00,0x00,0x15, 0x00,0x00,0x3f, 0x00,0x2a,0x15, 0x00,0x2a,0x3f,
    0x2a,0x00,0x15, 0x2a,0x00,0x3f, 0x2a,0x2a,0x15, 0x2a,0x2a,0x3f,
    0x00,0x15,0x00, 0x00,0x15,0x2a, 0x00,0x3f,0x00, 0x00,0x3f,0x2a,
    0x2a,0x15,0x00, 0x2a,0x15,0x2a, 0x2a,0x3f,0x00, 0x2a,0x3f,0x2a,
    0x00,0x15,0x15, 0x00,0x15,0x3f, 0x00,0x3f,0x15, 0x00,0x3f,0x3f,
    0x2a,0x15,0x15, 0x2a,0x15,0x3f, 0x2a,0x3f,0x15, 0x2a,0x3f,0x3f,
    0x15,0x00,0x00, 0x15,0x00,0x2a, 0x15,0x2a,0x00, 0x15,0x2a,0x2a,
    0x3f,0x00,0x00, 0x3f,0x00,0x2a, 0x3f,0x2a,0x00, 0x3f,0x2a,0x2a,
    0x15,0x00,0x15, 0x15,0x00,0x3f, 0x15,0x2a,0x15, 0x15,0x2a,0x3f,
    0x3f,0x00,0x15, 0x3f,0x00,0x3f, 0x3f,0x2a,0x15, 0x3f,0x2a,0x3f,
    0x15,0x15,0x00, 0x15,0x15,0x2a, 0x15,0x3f,0x00, 0x15,0x3f,0x2a,
    0x3f,0x15,0x00, 0x3f,0x15,0x2a, 0x3f,0x3f,0x00, 0x3f,0x3f,0x2a,
    0x15,0x15,0x15, 0x15,0x15,0x3f, 0x15,0x3f,0x15, 0x15,0x3f,0x3f,
    0x3f,0x15,0x15, 0x3f,0x15,0x3f, 0x3f,0x3f,0x15, 0x3f,0x3f,0x3f
};

const static uint8_t actl[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x14, 0x07,
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x0c, 0x00, 0x0f, 0x08 };

const static uint8_t sequ[] = { 0x00, 0x03, 0x00, 0x02 };

const static uint8_t grdc[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x0e, 0x0f, 0xff };

const static uint8_t crtc[] = {
    0x5f, 0x4f, 0x50, 0x82, 0x55, 0x81, 0xbf, 0x1f,
    0x00, 0x4f, 0x0d, 0x0e, 0x00, 0x00, 0x00, 0x00,
    0x9c, 0x8e, 0x8f, 0x28, 0x1f, 0x96, 0xb9, 0xa3,
    0xff };

const static uint8_t vgafont16[256 * 16] = {
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7e, 0x81, 0xa5, 0x81, 0x81, 0xbd, 0x99, 0x81, 0x81, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7e, 0xff, 0xdb, 0xff, 0xff, 0xc3, 0xe7, 0xff, 0xff, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x6c, 0xfe, 0xfe, 0xfe, 0xfe, 0x7c, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x7c, 0xfe, 0x7c, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x18, 0x3c, 0x3c, 0xe7, 0xe7, 0xe7, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x18, 0x3c, 0x7e, 0xff, 0xff, 0x7e, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3c, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe7, 0xc3, 0xc3, 0xe7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x66, 0x42, 0x42, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xc3, 0x99, 0xbd, 0xbd, 0x99, 0xc3, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x00, 0x00, 0x1e, 0x0e, 0x1a, 0x32, 0x78, 0xcc, 0xcc, 0xcc, 0xcc, 0x78, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3f, 0x33, 0x3f, 0x30, 0x30, 0x30, 0x30, 0x70, 0xf0, 0xe0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7f, 0x63, 0x7f, 0x63, 0x63, 0x63, 0x63, 0x67, 0xe7, 0xe6, 0xc0, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x18, 0x18, 0xdb, 0x3c, 0xe7, 0x3c, 0xdb, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x80, 0xc0, 0xe0, 0xf0, 0xf8, 0xfe, 0xf8, 0xf0, 0xe0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x02, 0x06, 0x0e, 0x1e, 0x3e, 0xfe, 0x3e, 0x1e, 0x0e, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7f, 0xdb, 0xdb, 0xdb, 0x7b, 0x1b, 0x1b, 0x1b, 0x1b, 0x1b, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x7c, 0xc6, 0x60, 0x38, 0x6c, 0xc6, 0xc6, 0x6c, 0x38, 0x0c, 0xc6, 0x7c, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0c, 0xfe, 0x0c, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x60, 0xfe, 0x60, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xc0, 0xc0, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x66, 0xff, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0x7c, 0x7c, 0x38, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x66, 0x66, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x6c, 0x6c, 0xfe, 0x6c, 0x6c, 0x6c, 0xfe, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x7c, 0xc6, 0xc2, 0xc0, 0x7c, 0x06, 0x06, 0x86, 0xc6, 0x7c, 0x18, 0x18, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xc2, 0xc6, 0x0c, 0x18, 0x30, 0x60, 0xc6, 0x86, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x76, 0xdc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x30, 0x30, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x0c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x30, 0x18, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x3c, 0xff, 0x3c, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc3, 0xc3, 0xdb, 0xdb, 0xc3, 0xc3, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x38, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x0c, 0x18, 0x30, 0x60, 0xc0, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0x06, 0x06, 0x3c, 0x06, 0x06, 0x06, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0c, 0x1c, 0x3c, 0x6c, 0xcc, 0xfe, 0x0c, 0x0c, 0x0c, 0x1e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0xc0, 0xc0, 0xc0, 0xfc, 0x06, 0x06, 0x06, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x60, 0xc0, 0xc0, 0xfc, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0xc6, 0x06, 0x06, 0x0c, 0x18, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0x7e, 0x06, 0x06, 0x06, 0x0c, 0x78, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x0c, 0x06, 0x0c, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0x0c, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xde, 0xde, 0xde, 0xdc, 0xc0, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x66, 0x66, 0x66, 0x66, 0xfc, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc2, 0xc0, 0xc0, 0xc0, 0xc0, 0xc2, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xf8, 0x6c, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x6c, 0xf8, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x62, 0x66, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc2, 0xc0, 0xc0, 0xde, 0xc6, 0xc6, 0x66, 0x3a, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1e, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0xcc, 0xcc, 0xcc, 0x78, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xe6, 0x66, 0x66, 0x6c, 0x78, 0x78, 0x6c, 0x66, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xf0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x62, 0x66, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xe7, 0xff, 0xff, 0xdb, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0xe6, 0xf6, 0xfe, 0xde, 0xce, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xd6, 0xde, 0x7c, 0x0c, 0x0e, 0x00, 0x00,
 0x00, 0x00, 0xfc, 0x66, 0x66, 0x66, 0x7c, 0x6c, 0x66, 0x66, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0x60, 0x38, 0x0c, 0x06, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xff, 0xdb, 0x99, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xdb, 0xdb, 0xff, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x18, 0x3c, 0x66, 0xc3, 0xc3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xff, 0xc3, 0x86, 0x0c, 0x18, 0x30, 0x60, 0xc1, 0xc3, 0xff, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x80, 0xc0, 0xe0, 0x70, 0x38, 0x1c, 0x0e, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x10, 0x38, 0x6c, 0xc6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00,
 0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xe0, 0x60, 0x60, 0x78, 0x6c, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc0, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1c, 0x0c, 0x0c, 0x3c, 0x6c, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x6c, 0x64, 0x60, 0xf0, 0x60, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0xcc, 0x78, 0x00,
 0x00, 0x00, 0xe0, 0x60, 0x60, 0x6c, 0x76, 0x66, 0x66, 0x66, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x06, 0x06, 0x00, 0x0e, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x66, 0x3c, 0x00,
 0x00, 0x00, 0xe0, 0x60, 0x60, 0x66, 0x6c, 0x78, 0x78, 0x6c, 0x66, 0xe6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xe6, 0xff, 0xdb, 0xdb, 0xdb, 0xdb, 0xdb, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xf0, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x7c, 0x0c, 0x0c, 0x1e, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xdc, 0x76, 0x66, 0x60, 0x60, 0x60, 0xf0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7c, 0xc6, 0x60, 0x38, 0x0c, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x10, 0x30, 0x30, 0xfc, 0x30, 0x30, 0x30, 0x30, 0x36, 0x1c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xc3, 0x66, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0xc3, 0xc3, 0xdb, 0xdb, 0xff, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc3, 0x66, 0x3c, 0x18, 0x3c, 0x66, 0xc3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7e, 0x06, 0x0c, 0xf8, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xcc, 0x18, 0x30, 0x60, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0e, 0x18, 0x18, 0x18, 0x70, 0x18, 0x18, 0x18, 0x18, 0x0e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x70, 0x18, 0x18, 0x18, 0x0e, 0x18, 0x18, 0x18, 0x18, 0x70, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3c, 0x66, 0xc2, 0xc0, 0xc0, 0xc0, 0xc2, 0x66, 0x3c, 0x0c, 0x06, 0x7c, 0x00, 0x00,
 0x00, 0x00, 0xcc, 0x00, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0c, 0x18, 0x30, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x10, 0x38, 0x6c, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xcc, 0x00, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x38, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x3c, 0x66, 0x60, 0x60, 0x66, 0x3c, 0x0c, 0x06, 0x3c, 0x00, 0x00, 0x00,
 0x00, 0x10, 0x38, 0x6c, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0x00, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x7c, 0xc6, 0xfe, 0xc0, 0xc0, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x66, 0x00, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x3c, 0x66, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xc6, 0x00, 0x10, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x38, 0x6c, 0x38, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x30, 0x60, 0x00, 0xfe, 0x66, 0x60, 0x7c, 0x60, 0x60, 0x66, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x6e, 0x3b, 0x1b, 0x7e, 0xd8, 0xdc, 0x77, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x3e, 0x6c, 0xcc, 0xcc, 0xfe, 0xcc, 0xcc, 0xcc, 0xcc, 0xce, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x10, 0x38, 0x6c, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x30, 0x78, 0xcc, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x60, 0x30, 0x18, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc6, 0x00, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7e, 0x06, 0x0c, 0x78, 0x00,
 0x00, 0xc6, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xc6, 0x00, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x18, 0x7e, 0xc3, 0xc0, 0xc0, 0xc0, 0xc3, 0x7e, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x64, 0x60, 0xf0, 0x60, 0x60, 0x60, 0x60, 0xe6, 0xfc, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xc3, 0x66, 0x3c, 0x18, 0xff, 0x18, 0xff, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xfc, 0x66, 0x66, 0x7c, 0x62, 0x66, 0x6f, 0x66, 0x66, 0x66, 0xf3, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0e, 0x1b, 0x18, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x18, 0x18, 0x18, 0xd8, 0x70, 0x00, 0x00,
 0x00, 0x18, 0x30, 0x60, 0x00, 0x78, 0x0c, 0x7c, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0c, 0x18, 0x30, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x30, 0x60, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x18, 0x30, 0x60, 0x00, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x76, 0xdc, 0x00, 0xdc, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,
 0x76, 0xdc, 0x00, 0xc6, 0xe6, 0xf6, 0xfe, 0xde, 0xce, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x3c, 0x6c, 0x6c, 0x3e, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x00, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x30, 0x30, 0x00, 0x30, 0x30, 0x60, 0xc0, 0xc6, 0xc6, 0x7c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x06, 0x06, 0x06, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xc0, 0xc0, 0xc2, 0xc6, 0xcc, 0x18, 0x30, 0x60, 0xce, 0x9b, 0x06, 0x0c, 0x1f, 0x00, 0x00,
 0x00, 0xc0, 0xc0, 0xc2, 0xc6, 0xcc, 0x18, 0x30, 0x66, 0xce, 0x96, 0x3e, 0x06, 0x06, 0x00, 0x00,
 0x00, 0x00, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x3c, 0x3c, 0x3c, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6c, 0xd8, 0x6c, 0x36, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x6c, 0x36, 0x6c, 0xd8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44, 0x11, 0x44,
 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa, 0x55, 0xaa,
 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77, 0xdd, 0x77,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xf6, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x18, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf6, 0x06, 0xf6, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x06, 0xf6, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf6, 0x06, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x18, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x30, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x30, 0x37, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf7, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xf7, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x37, 0x30, 0x37, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x36, 0x36, 0x36, 0x36, 0x36, 0xf7, 0x00, 0xf7, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x1f, 0x18, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x18, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0xff, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36, 0x36,
 0x18, 0x18, 0x18, 0x18, 0x18, 0xff, 0x18, 0xff, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0, 0xf0,
 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f,
 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xdc, 0xd8, 0xd8, 0xd8, 0xdc, 0x76, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x78, 0xcc, 0xcc, 0xcc, 0xd8, 0xcc, 0xc6, 0xc6, 0xc6, 0xcc, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0xfe, 0xc6, 0xc6, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xfe, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0xfe, 0xc6, 0x60, 0x30, 0x18, 0x30, 0x60, 0xc6, 0xfe, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xd8, 0xd8, 0xd8, 0xd8, 0xd8, 0x70, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7c, 0x60, 0x60, 0xc0, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x76, 0xdc, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x7e, 0x18, 0x3c, 0x66, 0x66, 0x66, 0x3c, 0x18, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xfe, 0xc6, 0xc6, 0x6c, 0x38, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x38, 0x6c, 0xc6, 0xc6, 0xc6, 0x6c, 0x6c, 0x6c, 0x6c, 0xee, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1e, 0x30, 0x18, 0x0c, 0x3e, 0x66, 0x66, 0x66, 0x66, 0x3c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x7e, 0xdb, 0xdb, 0xdb, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x03, 0x06, 0x7e, 0xdb, 0xdb, 0xf3, 0x7e, 0x60, 0xc0, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x1c, 0x30, 0x60, 0x60, 0x7c, 0x60, 0x60, 0x60, 0x30, 0x1c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x7c, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0xc6, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xfe, 0x00, 0x00, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7e, 0x18, 0x18, 0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x30, 0x18, 0x0c, 0x06, 0x0c, 0x18, 0x30, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x0c, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0c, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x0e, 0x1b, 0x1b, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0xd8, 0xd8, 0xd8, 0x70, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x7e, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xdc, 0x00, 0x76, 0xdc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x38, 0x6c, 0x6c, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x0f, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0xec, 0x6c, 0x6c, 0x3c, 0x1c, 0x00, 0x00, 0x00, 0x00,
 0x00, 0xd8, 0x6c, 0x6c, 0x6c, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x70, 0xd8, 0x30, 0x60, 0xc8, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void vga_initmode(VGAState *s)
{
    for (int i = 0; i < 64*3; i++)
        s->palette[i] = pal_ega[i];

    for (int i = 0; i <= 0x13; i++)
        s->ar[i] = actl[i];
    s->ar[0x14] = 0;

    s->sr[0] = 0x3;
    for (int i = 0; i < 4; i++)
        s->sr[i + 1] = sequ[i];

    for (int i = 0; i <= 8; i++)
        s->gr[i] = grdc[i];

    for (int i = 0; i <= 0x18; i++)
        s->cr[i] = crtc[i];

    s->msr = 0x67;

    // clear screen
    for (int i = 0; i < s->vga_ram_size / 4; i++) {
        s->vga_ram[i * 4] = 0x20;
        s->vga_ram[i * 4 + 1] = 0x07;
    }

    // load font
    for (int i = 0; i < 256; i++) {
        for (int j = 0; j < 16; j++) {
            s->vga_ram[i * 32 * 4 + j * 4 + 2] = vgafont16[i * 16 + j];
        }
    }

    s->ar_index = 0x20;
}
