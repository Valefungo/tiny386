/*
 * Cirrus Logic GD5430 VGA extension.
 *
 * Ported from 86Box's vid_cl54xx.c: extended SR/GR register space,
 * BitBlt engine (MMIO + legacy port access), hardware cursor, extended
 * palette, hidden DAC register, I2C/DDC GPIO, and native SVGA color-
 * depth/dot-clock detection for the non-VBE rendering path.
 *
 * Only active when s->card_type == VGA_CARD_CIRRUS. The common VGA
 * dispatch code in vga.c calls into this file through a small set of
 * hooks (cirrus_get_extended_addr, cirrus_get_vram_wrap_mask,
 * cirrus_get_svga_depth, cirrus_cursor_draw, cirrus_banked_addr,
 * cirrus_ioport_read/write) rather than branching on card_type inline.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "vga_internal.h"

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
    /* In TRANSPARENTCOMP mode the expanded byte is always fg_col - the
     * mask bit instead only decides (via cirrus_blt_write_pixel's
     * is_transp branch) whether that pixel gets written at all, so a
     * background color is never needed (86Box gd54xx_color_expand). */
    if ((s->cirrus_blt.mode & CIRRUS_BLTMODE_TRANSPARENTCOMP) || mask)
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

    {
        /* For COLOREXPAND the source is a packed monochrome bitstream
         * (1 bit/pixel, MSB first, byte-continuous across row
         * boundaries) - src_pitch does NOT apply to it, src_addr just
         * keeps advancing by 1 byte every 8 destination pixels for the
         * whole blit (86Box gd54xx_normal_blit, the non-XY-position
         * "while (count)" branch). Only the destination uses dst_pitch
         * to step to the next row. */
        uint32_t src_addr = b->src_addr;
        int x_count = 0;

    for (row = 0; row <= b->height; row++) {
        uint32_t dst_addr = (uint32_t)(b->dst_addr + (int64_t)row * b->dst_pitch * b->dir);
        if (!(b->mode & CIRRUS_BLTMODE_COLOREXPAND))
            src_addr = (uint32_t)(b->src_addr + (int64_t)row * b->src_pitch * b->dir);
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
        /* row ended mid-byte: consume the partial source byte too
         * before the next row starts (86Box: "if (x_count != 0)
         * src_addr++" at end-of-row). */
        if ((b->mode & CIRRUS_BLTMODE_COLOREXPAND) && x_count != 0)
            src_addr += b->dir;
        x_count = 0;
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

    if (b->mode & CIRRUS_BLTMODE_MEMSYSSRC) {
        /* CPU-to-VRAM streaming color-expand blit (used by GDI for font
         * glyphs): arm the engine and leave BUSY set - completion happens
         * byte-by-byte as the CPU streams data via cirrus_mem_sys_src_write,
         * driven by writes the driver makes to the LFB/banked VRAM range
         * (86Box gd54xx_mem_sys_src, non-XY-position branch). */
        b->ms_active = 1;
        b->dst_addr_backup = b->dst_addr;
        b->x_count = 0;
        b->y_count = 0;
        b->sys_src32 = 0;
        b->sys_cnt = 0;
#ifdef DEBUG_CIRRUS
        printf("cirrus: armed MEMSYSSRC mode=0x%02x modeext=0x%02x "
               "w=%d h=%d dst=0x%x dpitch=%d colorexpand=%d\n",
               b->mode, b->modeext, b->width, b->height, b->dst_addr, b->dst_pitch,
               !!(b->mode & CIRRUS_BLTMODE_COLOREXPAND));
#endif
        return;
    }

    /* Any non-MEMSYSSRC blit must disarm a previous, never-completed
     * streaming blit - otherwise ordinary VRAM writes that happen to
     * follow it (e.g. GDI loading an icon/pattern bitmap straight into
     * offscreen VRAM) would keep getting silently stolen by the stale
     * streaming engine instead of landing as normal pixel data. Real
     * hardware can't leave a blit half fed forever either: the next
     * BLT trigger always starts a fresh operation. */
    if (b->ms_active) {
#ifdef DEBUG_CIRRUS
        printf("cirrus: disarming stale MEMSYSSRC (x=%d y=%d w=%d h=%d) "
               "for new blit mode=0x%02x\n",
               b->x_count, b->y_count, b->width, b->height, b->mode);
#endif
        b->ms_active = 0;
    }

    if (b->mode & CIRRUS_BLTMODE_MEMSYSDEST) {
        /* VRAM-to-CPU readback isn't implemented; complete as a no-op so
         * the driver doesn't wait forever on BUSY. */
#ifdef DEBUG_CIRRUS
        printf("cirrus: UNIMPLEMENTED MEMSYSDEST blit mode=0x%02x modeext=0x%02x "
               "w=%d h=%d dst=0x%x dpitch=%d\n",
               b->mode, b->modeext, b->width, b->height, b->dst_addr, b->dst_pitch);
#endif
    } else if (b->mode & CIRRUS_BLTMODE_PATTERNCOPY) {
        cirrus_pattern_copy(s);
    } else {
        cirrus_normal_blit(s);
    }

    b->status &= ~(CIRRUS_BLT_START | CIRRUS_BLT_BUSY);
}

/* Feeds one CPU byte into an armed MEMSYSSRC color-expand blit (86Box
 * gd54xx_mem_sys_src, non-XY-position branch - the only one GD5430 uses).
 * Each bit of the byte, MSB first, is one source pixel, expanded to
 * pixel_width destination bytes (mirrors the bit/shift indexing
 * cirrus_normal_blit uses for its color-expand src path): color-expanded
 * via the existing fg/bg + ROP + transparency machinery and written to
 * the current destination position, which then advances across the row
 * (b->width+1 bytes) and down the rows (b->height+1), wrapping
 * dst_addr_backup by dst_pitch. Once the whole rectangle is filled the
 * blit completes and BUSY/ms_active are cleared. */
static void cirrus_mem_sys_src_feed_byte(VGAState *s, uint8_t val)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;
    uint8_t *vram = (uint8_t *)s->vga_ram;
    uint32_t vram_size = (uint32_t)s->vga_ram_size;
    int mask_shift;

    /* one CPU byte always maps to (the remainder of) a single row: once
     * x_count wraps (row complete), stop consuming this byte's
     * remaining bits even if unused - the next CPU byte starts a new
     * row (86Box gd54xx_mem_sys_src, "break" on x_count==0). */
    for (mask_shift = 7; mask_shift >= 0; mask_shift--) {
        int mask = val & (1 << mask_shift);
        int shift = 0; /* xx_count, always 0 for pixel_width==1 (8bpp) */
        uint8_t src, dst, target;
        uint32_t dst_addr;

        if (!b->ms_active)
            return;

        src = cirrus_color_expand(s, mask, shift);
        dst_addr = b->dst_addr_backup;
        if (dst_addr < vram_size) {
            dst = vram[dst_addr];
            target = cirrus_rop(s, dst, src);
            cirrus_blt_write_pixel(s, mask, &vram[dst_addr], target, 0);
        }

        b->dst_addr_backup += b->dir;
        b->x_count++;
        if (b->x_count > b->width) {
            b->x_count = 0;
            b->y_count++;
            if (b->y_count > b->height) {
                /* rectangle complete */
                b->ms_active = 0;
                b->status &= ~(CIRRUS_BLT_START | CIRRUS_BLT_BUSY);
            } else {
                b->dst_addr_backup = (uint32_t)(b->dst_addr +
                    (int64_t)b->y_count * b->dst_pitch * b->dir);
            }
            /* row done: stop blitting and wait for the next CPU byte,
             * discarding any unconsumed bits of this one. */
            return;
        }
    }
}

/* Entry point for VRAM-bound CPU writes while a MEMSYSSRC blit is armed
 * (86Box gd54xx_writeb_linear interception). Without DWORDGRANULARITY
 * each incoming byte is consumed immediately, one bit per pixel; with it
 * bytes are packed into a 32-bit word first (not currently exercised by
 * the Win9x/Win2000 GDI paths this was built against, but handled for
 * completeness). Returns 1 if the byte was consumed by the blit engine
 * (caller must not also perform a normal VRAM write), 0 otherwise. */
int cirrus_mem_sys_src_write(VGAState *s, uint8_t val)
{
    struct cirrus_blt_s *b = &s->cirrus_blt;

    if (!b->ms_active)
        return 0;

    if (b->modeext & CIRRUS_BLTMODEEXT_DWORDGRANULARITY) {
        b->sys_src32 |= (uint32_t)val << (8 * b->sys_cnt);
        b->sys_cnt++;
        if (b->sys_cnt == 4) {
            int i;
            for (i = 0; i < 4 && b->ms_active; i++)
                cirrus_mem_sys_src_feed_byte(s, (uint8_t)(b->sys_src32 >> (8 * i)));
            b->sys_src32 = 0;
            b->sys_cnt = 0;
        }
    } else {
        cirrus_mem_sys_src_feed_byte(s, val);
    }
    return 1;
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
        s->cirrus_blt.ms_active = 0;
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

/* Default monitor EDID (128-byte base block + 128-byte extension block),
 * byte-for-byte identical to 86Box's ddc_create_default_edid()
 * (vid_ddc.c) - established_timings/standard_timings/range-limits wide
 * enough that real display drivers offer resolutions well above
 * 1024x768, unlike the conservative built-in "Default Monitor" profile
 * Windows falls back to when DDC reads get no answer at all. */
static const uint8_t cirrus_edid[256] = {
0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x00,0x09,0xf8,0x00,0x00,
0x00,0x00,0x00,0x00,0x30,0x1e,0x01,0x04,0x0e,0x15,0x10,0x00,
0xeb,0x81,0xf1,0xa3,0x57,0x53,0x9f,0x27,0x0a,0x50,0x00,0xff,
0xff,0xff,0x81,0xc0,0x81,0x00,0x8b,0xc0,0x95,0x00,0xa9,0xc0,
0xa9,0x40,0xd1,0xc0,0xe1,0x40,0xa0,0x0f,0x20,0x00,0x31,0x58,
0x1c,0x20,0x28,0x80,0x14,0x00,0xd3,0x9e,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0xf7,0x00,0x0a,0xff,0xff,0xff,0xff,0xff,0xf0,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfd,0x00,0x2d,
0x7d,0x1e,0x73,0x1e,0x00,0x0a,0x20,0x20,0x20,0x20,0x20,0x20,
0x00,0x00,0x00,0xfc,0x00,0x38,0x36,0x42,0x6f,0x78,0x20,0x4d,
0x6f,0x6e,0x69,0x74,0x6f,0x72,0x01,0xa9,0x02,0x03,0x04,0x80,
0x66,0x21,0x56,0xaa,0x51,0x00,0x1e,0x30,0x46,0x8f,0x33,0x00,
0xd3,0x9e,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xfa,0x00,0x31,
0x59,0x45,0x59,0x61,0x59,0x81,0x99,0xa9,0x59,0xb3,0x00,0x0a,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x23,
};

/* Bit-bang I2C/DDC state machine for SR0x08, ported from 86Box's
 * i2c_gpio_set() (i2c_gpio.c), with a single hardwired read-only EDID
 * EEPROM slave at address 0x50 standing in for vid_ddc.c's
 * ddc_init()+i2c_eeprom_init(). Real DDC2B reads only ever do a single
 * "set offset, then read N sequential bytes" transaction, so only that
 * shape is modeled; a second write byte (which would normally write
 * EEPROM data) just NACKs since the EDID is read-only. */
static void cirrus_i2c_set(VGAState *s, int scl, int sda)
{
    scl = !!scl;
    sda = !!sda;
    if (s->cirrus_i2c.prev_scl && scl) {
        if (s->cirrus_i2c.prev_sda && !sda) {
            /* start (or repeated start) condition */
            s->cirrus_i2c.started = 1;
            s->cirrus_i2c.pos = 0;
            s->cirrus_i2c.slave_addr = 0xff;
            s->cirrus_i2c.slave_read = 2;
            s->cirrus_i2c.slave_sda = 1;
            s->cirrus_i2c.wrote_offset = 0;
        } else if (!s->cirrus_i2c.prev_sda && sda) {
            /* stop condition */
            s->cirrus_i2c.started = 0;
            s->cirrus_i2c.slave_addr = 0xff;
            s->cirrus_i2c.slave_sda = 1;
        }
    } else if (!s->cirrus_i2c.prev_scl && scl && s->cirrus_i2c.started) {
        if (s->cirrus_i2c.pos++ < 8) {
            if (s->cirrus_i2c.slave_read == 1) {
                s->cirrus_i2c.slave_sda = !!(s->cirrus_i2c.byte & 0x80);
                s->cirrus_i2c.byte <<= 1;
            } else {
                s->cirrus_i2c.byte <<= 1;
                s->cirrus_i2c.byte |= sda;
            }
        }
        if (s->cirrus_i2c.pos == 8) {
            switch (s->cirrus_i2c.slave_read) {
            case 2: /* address byte just received */
                s->cirrus_i2c.slave_addr = s->cirrus_i2c.byte >> 1;
                s->cirrus_i2c.slave_read = s->cirrus_i2c.byte & 1;
                if (s->cirrus_i2c.slave_addr == 0x50) {
                    s->cirrus_i2c.slave_sda = 0; /* ACK */
                    if (s->cirrus_i2c.slave_read)
                        s->cirrus_i2c.byte = cirrus_edid[s->cirrus_i2c.edid_addr];
#ifdef DEBUG_CIRRUS
                    printf("cirrus: i2c EDID slave selected for %s at offset 0x%02x\n",
                           s->cirrus_i2c.slave_read ? "read" : "write", s->cirrus_i2c.edid_addr);
#endif
                } else {
                    s->cirrus_i2c.slave_sda = 1; /* NACK: no device here */
                }
                s->cirrus_i2c.slave_read |= 0x80;
                break;
            case 0: /* write transfer byte */
                if (s->cirrus_i2c.slave_addr == 0x50 && !s->cirrus_i2c.wrote_offset) {
                    s->cirrus_i2c.edid_addr = s->cirrus_i2c.byte;
                    s->cirrus_i2c.wrote_offset = 1;
                    s->cirrus_i2c.slave_sda = 0; /* ACK */
                } else {
                    s->cirrus_i2c.slave_sda = 1; /* NACK: read-only past the offset byte */
                }
                break;
            default:
                break;
            }
        } else if (s->cirrus_i2c.pos == 9) {
            if ((s->cirrus_i2c.slave_read & 0x7f) == 1) {
                if (!sda) { /* master ACKed: advance to the next byte */
                    s->cirrus_i2c.edid_addr++;
                    s->cirrus_i2c.byte = cirrus_edid[s->cirrus_i2c.edid_addr];
                }
            } else {
                s->cirrus_i2c.slave_read &= 1;
            }
            s->cirrus_i2c.pos = 0;
        }
    } else if (s->cirrus_i2c.prev_scl && !scl && (s->cirrus_i2c.pos != 8)) {
        s->cirrus_i2c.slave_sda = 1;
    }
    s->cirrus_i2c.prev_scl = scl;
    s->cirrus_i2c.prev_sda = sda;
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
    case 0x08: /* I2C/DDC GPIO: write bit0=SCL, bit1=SDA (86Box vid_cl54xx.c:772-775) */
        cirrus_i2c_set(s, val & 0x01, val & 0x02);
        break;
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
                  * than the write side - 86Box vid_cl54xx.c:1315-1323).
                  * SDA reflects the wired-AND of our own output and the
                  * EDID slave's response, not a plain loopback of what
                  * was last written. */
        uint8_t ret = s->cirrus_sr_ext[0x08] & 0x7b;
        if (s->cirrus_i2c.prev_scl) ret |= 0x04; /* SCL */
        if (s->cirrus_i2c.prev_sda && s->cirrus_i2c.slave_sda) ret |= 0x80; /* SDA */
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
    if (index >= 0x0c && index <= 0x0f) {
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
    if (index >= 0x0c && index <= 0x0f)
        return s->cirrus_gr_ext[index - 0x0c];
    if (index == 0x3f)
        return 0x00; /* vportsync toggle, GD5446-only - fixed 0 on GD5430 (vid_cl54xx.c:1605-1609) */
    {
        int off = cirrus_gr_to_mmio_off(index);
        if (off >= 0)
            return cirrus_blt_reg_read8(s, off);
    }
    return 0xff;
}

/* Cirrus extended bank registers (GR0x09/0x0A/0x0B) split the 64K window
 * at A0000 into two independently bankable 32K halves selected by addr
 * bit 15, per 86Box's gd54xx_recalc_banking()/gd54xx_write(). */
uint32_t cirrus_banked_addr(VGAState *s, uint32_t addr)
{
    int gr0b = s->cirrus_bank_reg[2];
    int shift = (gr0b & 0x20) ? 14 : 12;
    uint32_t bank0 = s->cirrus_bank_reg[0] << shift;
    uint32_t bank1 = (gr0b & 0x01) ? (s->cirrus_bank_reg[1] << shift)
                                    : (bank0 + 0x8000);
    return (addr & 0x7fff) + (((addr >> 15) & 1) ? bank1 : bank0);
}

/* Cirrus hardware cursor overlay, drawn as a final pass over the
 * already-rendered frame (86Box vid_cl54xx.c:2075-2137,
 * gd54xx_hwcursor_draw). 2bpp-per-pixel format: byte 0 of each 8-pixel
 * group is the AND mask, byte 1 the XOR mask (offset +8 within the row
 * for the 64x64 size, +0x80 - the fixed 32x32 slot size - for 32x32),
 * comb = (XOR<<0)|(AND<<1) selects: 0=passthrough, 1=bg color,
 * 2=XOR-invert, 3=fg color. Colors come from the extended palette
 * entries 0 and 0xf (gd54xx->extpallook[0]/[0xf]). */
void cirrus_cursor_draw(VGAState *s, FBDevice *fb_dev, int i0)
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

/* CR0x1B extends the start address / line offset beyond the standard
 * VGA bit width (86Box vid_cl54xx.c, CRTC overflow extensions). Called
 * from vga_text_refresh/vga_graphic_refresh in vga.c. */
void cirrus_get_extended_addr(VGAState *s, uint32_t *start_addr, uint32_t *line_offset)
{
    *line_offset |= (s->cr[0x1b] & 0x10) << 4;
    *start_addr |= ((s->cr[0x1b] & 0x01) << 16) | ((s->cr[0x1b] & 0x0c) << 15);
}

/* CR0x1B bit1: select the display-memory wrap mask (86Box
 * vid_cl54xx.c:2056). Only bounds the start of each scanline, not
 * every per-pixel fetch within it - in practice this never differs
 * from "no mask" for any vga_mem_size we actually configure (a few
 * MB), so the approximation is harmless. */
uint32_t cirrus_get_vram_wrap_mask(VGAState *s)
{
    return (s->cr[0x1b] & 0x02) ? (uint32_t)(s->vga_ram_size - 1) : 0x3ffff;
}

/* Native Cirrus SVGA color depth: derived from the hidden DAC control
 * register (0x3c6) + SR0x07 bpp bits, not from the Bochs VBE registers
 * (86Box vid_cl54xx.c:1898-2040).
 *
 * xdiv (1 pixel per VRAM byte vs 2) is NOT simply SR07 bit0
 * (CIRRUS_SR7_BPP_SVGA): that bit only gates whether the *native* SVGA
 * path applies at all (gdcreg[5] bit6, the legacy VGA 256-color chain-4
 * mode, is always 2 output pixels per VRAM byte - see
 * gd54xx_recalctimings, the "else if (gdcreg[5] & 0x40)" branch, which
 * doesn't even look at linedbl). Only once SR07.0 is set does 86Box
 * pick between svga_render_8bpp_lowres (2x horizontal pixel doubling)
 * and *_highres (1:1) using "linedbl", an empirical heuristic comparing
 * display height to width: dispend*9/10 >= hdisp (vid_cl54xx.c:1946).
 * hdisp there is crtc[1] in raw character-clock units (NOT multiplied
 * by 8 the way our w is) - callers must pass hdisp = w/8.
 * Getting this wrong halves/doubles the effective scanline width, which
 * shows up as the image being squeezed and repeated across the screen. */
void cirrus_get_svga_depth(VGAState *s, int *bpp, int *xdiv, uint32_t *line_offset, int hdisp, int dispend)
{
    int true_svga = s->sr[7] & 0x01;
    int linedbl = dispend * 9 >= hdisp * 10;
    uint8_t ctrl = s->cirrus_dac_hidden;
    *bpp = 8;
    if (ctrl & 0x80) {
        if (ctrl & 0x40) {
            switch (ctrl & 0x0f) {
            case 0: *bpp = 15; break;
            case 1: *bpp = 16; break;
            case 5: *bpp = 24; break;
            case 8: case 9: *bpp = 8; break;
            case 0xf:
                switch (s->sr[7] & 0x0e) {
                case 0x08: *bpp = 32; break;
                case 0x04: *bpp = 24; break;
                case 0x06: case 0x02: *bpp = 16; break;
                default: *bpp = 8; break;
                }
                break;
            default: break;
            }
        } else {
            *bpp = 15;
        }
    }
    (void)linedbl;
    *xdiv = true_svga ? 1 : 2;
    if (*bpp == 32)
        *line_offset *= 2;
}

/* Handles the Cirrus-specific branches of vga_ioport_read for ports
 * that are not plain pass-through to the standard VGA registers. Sets
 * *handled=1 and returns the value if this port/state needed Cirrus
 * handling, otherwise *handled=0 and the caller falls back to the
 * standard VGA behavior. */
uint32_t cirrus_ioport_read(VGAState *s, uint32_t addr, int *handled)
{
    uint32_t val = 0;
    *handled = 1;
    switch (addr) {
    case 0x3c4:
        /* Reading the index port back while a cursor X/Y register
         * (0x10/0x11 or any of their 7 alias banks) is selected
         * and the chip is unlocked returns packed position bits
         * instead of the plain index (86Box vid_cl54xx.c:1291-
         * 1302) - an oddity of real silicon some detection code
         * checks for. */
        if (s->sr[6] == 0x12 && (s->sr_index & 0x1e) == 0x10) {
            if (s->sr_index & 1)
                val = ((s->cirrus_cursor.y & 7) << 5) | 0x11;
            else
                val = ((s->cirrus_cursor.x & 7) << 5) | 0x10;
        } else {
            *handled = 0;
        }
        break;
    case 0x3c5:
        if (s->sr_index > 7)
            val = cirrus_sr_ext_read(s, s->sr_index);
        else
            *handled = 0;
        break;
    case 0x3c6:
        if (s->cirrus_unlocked) {
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
    /* 0x3c9 is handled directly in vga_ioport_read: cirrus_dac_state
     * reset and dac_sub_index/dac_read_index advancement must always
     * run regardless of which palette (standard vs extended) is the
     * data source, which doesn't fit this handled=0/1 dispatch. */
    case 0x3ce:
        val = s->gr_index & 0x3f;
        break;
    case 0x3cf:
        val = cirrus_gr_read(s, s->gr_index);
        break;
    case 0x3b5:
    case 0x3d5:
        /* CR0x27 is the hardwired chip ID on real Cirrus silicon; the
         * in-box NT/9x drivers read it back to tell GD5430/34/36/46
         * apart. Nothing else sets this (no per-device option ROM in
         * tiny386), so without this the register reads back 0x00,
         * which isn't a valid ID and confuses the driver. */
        if (s->cr_index == 0x27) {
            val = 0xa0; /* CIRRUS_ID_CLGD5430 */
        } else if (s->cr_index == 0x28) {
            /* "Class ID" - always 0xff on real GD5430/5440 silicon
             * (86Box vid_cl54xx.c:1652-1656). */
            val = 0xff;
        } else if (s->cr_index == 0x22) {
            /* Graphics Data Latches Readback Register: byte
             * (GR4 & 3) of the latch loaded by the last VRAM read
             * (86Box vid_cl54xx.c:1639-1642). */
            val = (s->latch >> (8 * (s->gr[4] & 3))) & 0xff;
        } else if (s->cr_index == 0x24) {
            /* Attribute controller flip-flop readback (86Box
             * vid_cl54xx.c:1643-1645). */
            val = s->ar_flip_flop << 7;
        } else if (s->cr_index == 0x26) {
            /* Attribute controller index readback (86Box
             * vid_cl54xx.c:1646-1648). */
            val = s->ar_index & 0x3f;
        } else {
            *handled = 0;
        }
        break;
    default:
        *handled = 0;
        break;
    }
    return val;
}

/* Handles the Cirrus-specific branches of vga_ioport_write. Returns 1
 * if this port/state needed Cirrus handling (the caller skips its own
 * standard-VGA write), 0 otherwise. */

/* SR0x12 bit1: redirect palette index/data port (0x3c7-0x3c9) accesses
 * to the 16-entry extended palette instead of the standard 256-entry
 * one (86Box vid_cl54xx.c:895-908) - used for the hardware cursor's
 * background/foreground colors (entries 0/0xf). */
int cirrus_ext_palette_active(VGAState *s)
{
    return s->cirrus_sr_ext[0x12] & 0x02;
}

int cirrus_ioport_write(VGAState *s, uint32_t addr, uint32_t val)
{
    switch (addr) {
    case 0x3c4:
        s->sr_index = val;
        return 1;
    case 0x3c5:
        if (s->sr_index == 6) {
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
            return 1;
        } else if (s->sr_index > 7) {
            cirrus_sr_ext_write(s, s->sr_index, val);
            return 1;
        }
        return 0;
    case 0x3c6:
        /* Hidden DAC control register: 4 consecutive reads of 0x3c6
         * (with no intervening 0x3c7/0x3c8/0x3c9 access) prime the
         * state machine, then a write captures the value (86Box
         * vid_cl54xx.c:865-874). Only meaningful when unlocked, which
         * is always true on GD5430 - see cirrus_unlocked above. We
         * store the register faithfully but don't wire it to a real
         * SVGA packed-pixel color-depth renderer (out of scope, same
         * as the rest of the BPP>8 path - see vga_set_card_type). */
        if (s->cirrus_unlocked) {
            if (s->cirrus_dac_state == 4) {
                s->cirrus_dac_hidden = val;
#ifdef DEBUG_CIRRUS
                printf("cirrus: hidden DAC ctrl write val=0x%02x\n", val);
#endif
            }
            s->cirrus_dac_state = 0;
        }
        return 1;
    /* 0x3c9 is handled directly in vga_ioport_write: cirrus_dac_state
     * reset and dac_sub_index/dac_write_index advancement must always
     * run regardless of which palette (standard vs extended) is the
     * write target, which doesn't fit this handled=0/1 dispatch. See
     * cirrus_ext_palette_active(). */
    case 0x3ce:
        s->gr_index = val;
        return 1;
    case 0x3cf:
        cirrus_gr_write(s, s->gr_index, val);
        return 1;
    default:
        return 0;
    }
}

void vga_set_card_type_cirrus(VGAState *s)
{
    /* GD5430 (CR0x27=0xa0) is >= CIRRUS_ID_CLGD5429: the extended
     * register lock is hardwired open from reset on real silicon and
     * SR06 writes never change it (86Box vid_cl54xx.c:4211-4214). */
    s->cirrus_unlocked = 1;
    /* VCLK numerator/denominator reset defaults for chip ID >=
     * CIRRUS_ID_CLGD5420 (true for GD5430=0xa0), 86Box
     * vid_cl54xx.c:4180-4188. These four clock-select slots back the
     * standard 2-bit MISC-register clock select and are non-zero on
     * real silicon from power-on, not something the BIOS/driver
     * necessarily reprograms before relying on them - leaving them
     * at zero (the default cirrus_sr_ext[] state) would make any
     * mode that picks an unprogrammed clocksel slot compute a
     * bogus/zero pixel clock. */
    s->cirrus_sr_ext[0x0b] = 0x4a; s->cirrus_sr_ext[0x1b] = 0x2b;
    s->cirrus_sr_ext[0x0c] = 0x5b; s->cirrus_sr_ext[0x1c] = 0x2f;
    s->cirrus_sr_ext[0x0d] = 0x45; s->cirrus_sr_ext[0x1d] = 0x30;
    s->cirrus_sr_ext[0x0e] = 0x7e; s->cirrus_sr_ext[0x1e] = 0x33;
    /* I2C bus idles high (pulled up), matching 86Box's i2c_gpio_init(). */
    s->cirrus_i2c.prev_scl = 1;
    s->cirrus_i2c.prev_sda = 1;
    s->cirrus_i2c.slave_sda = 1;
    s->cirrus_i2c.slave_addr = 0xff;
}
