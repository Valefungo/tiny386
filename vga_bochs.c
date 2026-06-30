/*
 * Bochs VBE (VESA BIOS Extension) VGA extension: the extra banked/LFB
 * SVGA register interface implemented by Bochs/QEMU's "std" VGA card,
 * independent of the standard VGA register set handled in vga.c.
 */
#include <stdio.h>
#include <string.h>

#include "vga_internal.h"

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
