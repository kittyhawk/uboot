#include <common.h>
#include <ppc4xx.h>
#include <asm/processor.h>
#include <asm/mmu.h>
#include <asm/io.h>
#include <rtc.h>
#include <ppc450.h>
#include <command.h>
#include <asm/global_data.h>
#include <fdt.h>
#include <libfdt.h>
#include <fdt_support.h>
#include <net.h>
#include <linux/ctype.h>
#include "bgp.h"

unsigned int const COMPUTE_CARD = 34;
unsigned int const NODE_CARD = 16;
unsigned int const MID_PLANE = 2;
unsigned int const RACK_ROW = 16;
unsigned int const RACK_COL = 16;

/* The UCI is presented as the following integer vector:
 *
 * <rack-column, rack-row, mid-plane, node-card, compute-card>
 *
 * We relabel the vector as follows:
 *
 * <v, w, x, y, z>
 *
 * To obtain a monotonically-increasing map from such a vector to 
 * the integers we construct a polynomial of the form:
 *
 * j = Vv + Ww + Xx + Yy + z
 *
 * The coefficients are constructed recursively by adding 1 to
 * the maximum value of the sum of the terms to their right.
 *
 * Y = 1 + max(z)
 * X = 1 + max(z) + max(Yy)
 * W = 1 + max(z) + max(Yy) + max (Xx)
 * V = 1 + max(z) + max(Yy) + max (Xx) + max(Ww)
 *
 * This can be simplified:
 *
 * Y = 1 + 1 * max(z)
 * X = Y + Y * max(y)
 * W = X + X * max(x)
 * V = W + W * max(w)
 *
 * Note that the range 1-2 is a hole for compute-card.
 */

unsigned int bg_uci_to_uni(BGP_UCI_ComputeCard_t *uci)
{
    unsigned int j;
    unsigned int v, w, x, y, z;
    unsigned int V, W, X, Y;

    Y = 1 + 1 * (COMPUTE_CARD - 1);
    X = Y + Y * (NODE_CARD - 1);
    W = X + X * (MID_PLANE - 1);
    V = W + W * (RACK_ROW - 1);

    v = uci->RackColumn;
    w = uci->RackRow;
    x = uci->Midplane;
    y = uci->NodeCard;
    z = uci->ComputeCard >= 4 ? uci->ComputeCard - 2 : uci->ComputeCard;

    j = V*v + W*w + X*x + Y*y + z;

    return j;
}


unsigned int bg_uni_to_ipv4(unsigned int uni,
                            unsigned char *oc3, 
                            unsigned char *oc2, 
                            unsigned char *oc1, 
                            unsigned char *oc0,
                            unsigned char start_oc3,
                            unsigned char start_oc2,
                            unsigned char start_oc1,
                            unsigned char start_oc0,
                            unsigned char end_oc3,
                            unsigned char end_oc2,
                            unsigned char end_oc1,
                            unsigned char end_oc0)
{
    unsigned int start, end, ip;
    
    start = start_oc3 << 24 | start_oc2 << 16 | start_oc1 << 8 | start_oc0;
    end   = end_oc3 << 24 | end_oc2 << 16 | end_oc1 << 8 | end_oc0;

    ip = start + uni;

    if (ip > end || ip < start) return 0;
    
    *oc0 = ip & 0xff;
    *oc1 = (ip >> 8) & 0xff;
    *oc2 = (ip >> 16) & 0xff;
    *oc3 = (ip >> 24) & 0xff;
#if 0
    printf("uni=%d(0x%x) start=%d(0x%x) end=%d(0x%x) ip=%d(0x%x) oc3=%d(0x%x)"
           " oc2=%d(0x%x) oc1=%d(0x%x) oc0=%d(0x%x)\n", uni,uni,
           start, start, end, end,ip,ip, *oc3, *oc3, *oc2, *oc2, *oc1, *oc1, *oc0, *oc0);
#endif
    return 1;
}
