/*
 * ARM NVIDIA Tegra2 emulation.
 *
 * Copyright (c) 2014-2015 Dmitry Osipenko <digetx@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <pixman.h>

#include "tegra_common.h"

#include "sysemu/dma.h"
#include "hw/sysbus.h"

#include "../dc/registers/dc.h"
#include "gr2d.h"

#include "tegra_trace.h"

#define __ALIGN_MASK(x,mask)    (((x))&~(mask))
#define ALIGN(x,a)  __ALIGN_MASK(x,(uint32_t)(a)-1)

#define DISABLED    0
#define ENABLED     1

#define G2  0
#define SB  1

#define G2OUTPUT_MEM    0
#define G2OUTPUT_EPP    1

#define FLIP_X      0
#define FLIP_Y      1
#define TRANS_LR    2
#define TRANS_RL    3
#define ROT_90      4
#define ROT_180     5
#define ROT_270     6
#define IDENTITY    7

#define BITBLT      0
#define LINEDRAW    1
#define VCAA        2
#define RESERVED1   3

#define GXclear         0x00
#define GXand           0x88
#define GXandReverse    0x44
#define GXcopy          0xcc
#define GXandInverted   0x22
#define GXnoop          0xaa
#define GXxor           0x66
#define GXor            0xee
#define GXnor           0x11
#define GXequiv         0x99
#define GXinvert        0x55
#define GXorReverse     0xdd
#define GXcopyInverted  0x33
#define GXorInverted    0xbb
#define GXnand          0x77
#define GXset           0xff

/* FIXME: slow? TODO: color conversion */
/*static void gr2d_copy(void *src, void *dst, int src_x, int src_y,
                      int dst_x, int dst_y, int src_stride, int dst_stride,
                      int width, int height, int bytes_per_pixel,
                      bool invx, bool invy)
{
    src += src_y * src_stride;
    dst += dst_y * dst_stride;

    while (height--) {
        if (invx)
            memmove(dst + (dst_x + 1 - width) * bytes_per_pixel,
                    src + (src_x + 1 - width) * bytes_per_pixel,
                    width * bytes_per_pixel);
        else
            memcpy(dst + dst_x * bytes_per_pixel,
                   src + src_x * bytes_per_pixel,
                   width * bytes_per_pixel);

        src += invy ? -src_stride : src_stride;
        dst += invy ? -dst_stride : dst_stride;
    }
}*/

static void __process_2d(gr2d_ctx *ctx)
{
    dma_addr_t dma_dst_sz;
    dma_addr_t dma_src_sz;
    void *dst_ptr;
    void *src_ptr;

    /* TODO's */
    g_assert(ctx->g2sb_g2controlsecond.fr_mode == DISABLED);
    g_assert(!(ctx->g2sb_g2controlsecond.clipc & 2));
    g_assert(ctx->g2sb_g2controlsecond.alpsrcordst == DISABLED);
    g_assert(ctx->g2sb_g2controlsecond.bewswap == DISABLED);
    g_assert(ctx->g2sb_g2controlsecond.bebswap == DISABLED);
    g_assert(ctx->g2sb_g2controlsecond.bitswap == DISABLED);

    g_assert(ctx->g2sb_g2controlmain.faden == DISABLED);
    g_assert(ctx->g2sb_g2controlmain.alpen == DISABLED);
    g_assert(ctx->g2sb_g2controlmain.patsld == DISABLED);
    g_assert(ctx->g2sb_g2controlmain.patfl == DISABLED);

    uint32_t bpp = 8 << ctx->g2sb_g2controlmain.dstcd;
    //if (ctx->g2sb_g2controlmain.dstcd==3) bpp = 24;

    switch (ctx->g2sb_g2controlmain.cmdt) {
    case BITBLT:
        if (ctx->g2sb_g2controlmain.srcsld) {
            dma_dst_sz = ctx->g2sb_g2dstst.dsts * ctx->g2sb_g2dstsize.dstheight;

//             if (dma_dst_sz == 0)
//                 break;
            g_assert(dma_dst_sz != 0);

            g_assert(ctx->g2sb_g2controlmain.xdir == DISABLED);
            g_assert(ctx->g2sb_g2controlmain.ydir == DISABLED);
            g_assert(ctx->g2sb_g2controlmain.yflip == DISABLED);
            g_assert(ctx->g2sb_g2controlmain.xytdw == DISABLED);
            //g_assert(ctx->g2sb_g2controlmain.dstcd != RESERVED1);
            g_assert(ctx->g2sb_g2ropfade.rop == GXcopy);

            dst_ptr = dma_memory_map(&address_space_memory,
                                     ALIGN(ctx->g2sb_g2dstba.reg32, 8),
                                     &dma_dst_sz, DMA_DIRECTION_FROM_DEVICE, MEMTXATTRS_UNSPECIFIED);

            g_assert(dst_ptr != NULL);

            pixman_fill(dst_ptr, ALIGN(ctx->g2sb_g2dstst.dsts, 1) >> 2,
                        bpp,
                        ctx->g2sb_g2dstps.dstx, ctx->g2sb_g2dstps.dsty,
                        ctx->g2sb_g2dstsize.dstwidth,
                        ctx->g2sb_g2dstsize.dstheight,
                        ctx->g2sb_g2srcfgc.reg32);

            dma_memory_unmap(&address_space_memory, dst_ptr,
                             ALIGN(ctx->g2sb_g2dstba.reg32, 8),
                             DMA_DIRECTION_FROM_DEVICE, dma_dst_sz);
        } else {
            dma_dst_sz = ctx->g2sb_g2dstst.dsts * ctx->g2sb_g2dstsize.dstheight;
            dma_src_sz = ctx->g2sb_g2srcst.srcs * ctx->g2sb_g2srcsize.srcheight;

            if (dma_dst_sz == 0 || dma_src_sz == 0)
                break;
//             g_assert(dma_dst_sz != 0);
//             g_assert(dma_src_sz != 0);

//             printf("blt! src_ba=0x%08X dst_ba=0x%08X src_stride=%d dst_stride=%d "
//                    "srcx=%d srcy=%d srcwidth=%d srcheight=%d dstx=%d dsty=%d "
//                    "dstwidth=%d dstheight=%d xdir=%d ydir=%d\n",
//                    ctx->g2sb_g2srcba.reg32, ctx->g2sb_g2dstba.reg32,
//                    ctx->g2sb_g2srcst.srcs, ctx->g2sb_g2dstst.dsts,
//                    ctx->g2sb_g2srcps.srcx, ctx->g2sb_g2srcps.srcy,
//                    ctx->g2sb_g2srcsize.srcwidth, ctx->g2sb_g2srcsize.srcheight,
//                    ctx->g2sb_g2dstps.dstx, ctx->g2sb_g2dstps.dsty,
//                    ctx->g2sb_g2dstsize.dstwidth, ctx->g2sb_g2dstsize.dstheight,
//                    ctx->g2sb_g2controlmain.xdir, ctx->g2sb_g2controlmain.ydir);

            g_assert(ctx->g2sb_g2controlmain.yflip == DISABLED);
            g_assert(ctx->g2sb_g2controlmain.xytdw == DISABLED);
            //g_assert(ctx->g2sb_g2controlmain.dstcd != RESERVED1);
            g_assert(ctx->g2sb_g2ropfade.rop == GXcopy);

            /* Mono? */
            g_assert(ctx->g2sb_g2controlmain.srccd == 1);

            src_ptr = dma_memory_map(&address_space_memory,
                                     ALIGN(ctx->g2sb_g2srcba.reg32, 8),
                                     &dma_src_sz, DMA_DIRECTION_TO_DEVICE, MEMTXATTRS_UNSPECIFIED);

            dst_ptr = dma_memory_map(&address_space_memory,
                                     ALIGN(ctx->g2sb_g2dstba.reg32, 8),
                                     &dma_dst_sz, DMA_DIRECTION_FROM_DEVICE, MEMTXATTRS_UNSPECIFIED);

            g_assert(src_ptr != NULL);
            g_assert(dst_ptr != NULL);

//             g_assert(ctx->g2sb_g2srcst.srcs != (1 << ctx->g2sb_g2controlmain.dstcd) * ctx->g2sb_g2srcsize.srcwidth);
//             g_assert(ctx->g2sb_g2dstst.dsts != (1 << ctx->g2sb_g2controlmain.dstcd) * ctx->g2sb_g2dstsize.dstwidth);

            /*if (ctx->g2sb_g2controlmain.xdir || ctx->g2sb_g2controlmain.ydir) {
                gr2d_copy(src_ptr, dst_ptr,
                          ctx->g2sb_g2srcps.srcx, ctx->g2sb_g2srcps.srcy,
                          ctx->g2sb_g2dstps.dstx, ctx->g2sb_g2dstps.dsty,
                          ctx->g2sb_g2srcst.srcs, ctx->g2sb_g2dstst.dsts,
                          ctx->g2sb_g2dstsize.dstwidth,
                          ctx->g2sb_g2dstsize.dstheight,
                          bpp/8,
                          ctx->g2sb_g2controlmain.xdir,
                          ctx->g2sb_g2controlmain.ydir);
            } else*/ {
                pixman_image_t *image_src = NULL, *image_dst = NULL;

                image_src = pixman_image_create_bits_no_clear(PIXMAN_a8r8g8b8,
						              ctx->g2sb_g2srcsize.srcwidth,
						              ctx->g2sb_g2srcsize.srcheight,
						              src_ptr,
						              ctx->g2sb_g2srcst.srcs);

                image_dst = pixman_image_create_bits_no_clear(PIXMAN_a8r8g8b8,
						              !ctx->transpose ? ctx->g2sb_g2dstsize.dstwidth : ctx->g2sb_g2dstsize.dstheight,
						              !ctx->transpose ? ctx->g2sb_g2dstsize.dstheight : ctx->g2sb_g2dstsize.dstwidth,
						              dst_ptr,
						              ctx->g2sb_g2dstst.dsts);

                /*pixman_blt(src_ptr, dst_ptr,
                           ctx->g2sb_g2srcst.srcs / (bpp / 8),
                           ctx->g2sb_g2dstst.dsts / (bpp / 8),
                           bpp,
                           bpp,
                           ctx->g2sb_g2srcps.srcx, ctx->g2sb_g2srcps.srcy,
                           ctx->g2sb_g2dstps.dstx, ctx->g2sb_g2dstps.dsty,
                           ctx->g2sb_g2dstsize.dstwidth,
                           ctx->g2sb_g2dstsize.dstheight);*/

                  if (image_src && image_dst) {
                      int x = ctx->g2sb_g2dstps.dstx, y = ctx->g2sb_g2dstps.dsty;
                      int width = ctx->g2sb_g2dstsize.dstwidth, height = ctx->g2sb_g2dstsize.dstheight;

                      tegra_dc_pixman_transform_flags(image_src, ctx->g2sb_g2controlmain.xdir, ctx->g2sb_g2controlmain.ydir, ctx->transpose, &x, &y, &width, &height);

                      pixman_image_composite(PIXMAN_OP_SRC,
                                             image_src, NULL,
                                             image_dst,
                                             ctx->g2sb_g2srcps.srcx, ctx->g2sb_g2srcps.srcy,
                                             0, 0,
                                             x, y,
                                             width, height);
                  }

                  if (image_src) pixman_image_unref(image_src);
                  if (image_dst) pixman_image_unref(image_dst);
            }

            dma_memory_unmap(&address_space_memory, dst_ptr,
                             ALIGN(ctx->g2sb_g2dstba.reg32, 8),
                             DMA_DIRECTION_FROM_DEVICE, dma_dst_sz);

            dma_memory_unmap(&address_space_memory, src_ptr,
                             ALIGN(ctx->g2sb_g2srcba.reg32, 8),
                             DMA_DIRECTION_TO_DEVICE, dma_src_sz);
        }
        break;
    default:
//         g_assert_not_reached();
        break;
    }
}

void process_2d(gr2d_ctx *ctx, int sb_g2)
{
    /* TODO's */
//     g_assert(ctx->g2sb_g2cmdsel.g2output == G2OUTPUT_MEM);
//     g_assert(ctx->g2sb_g2cmdsel.vitrigger == DISABLED);
//     g_assert(ctx->g2sb_g2cmdsel.hosttrigger == DISABLED);
//     g_assert(ctx->g2sb_g2cmdsel.cbenable == DISABLED);
//
// //     g_assert(sb_g2 == G2);
//
    __process_2d(ctx);
}
