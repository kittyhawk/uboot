/*
 * (C) Copyright 2007
 * Volkmar Uhlig, Amos Waterland
 * IBM Research
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

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

extern inline unsigned int smp_processor_id(void)
{
    return mfspr(SPRN_PIR);
}

u32 get_processor_mhz(void)
{
    return personality->kernel.mhz;
}

int get_uci(void)
{
    return personality->kernel.uci;
}

extern void drv_bgtty_init(void);
extern void setup_mailboxes(void);
extern void mailbox_write(char *msg, unsigned len);
extern void _start_bg(int, void*);

extern void (*cpu_entry)(void);
static unsigned long *cpu_entry_unreloc = (unsigned long*)&cpu_entry;
extern void spin_app_processor(void);

static cns_descriptor_t *cns_descriptor;

static int running_on_cns(void)
{
    return 1;
}

static int valid_cns_descriptor(cns_descriptor_t *desc)
{
    // XXX: do more sanity checks!
    return (desc >= (cns_descriptor_t*)0xfff00000);
}

static int setup_global_interrupts(void)
{
    /* Bail out if personality page does not have global interrupts on.  */
    if (!(personality->kernel.node & BGP_PERS_EN_GLOBAL_INTS)) {
        return 0;
    }

    /* Enable channel zero of global interrupts.  */
    mtdcrx(BGP_DCR_GLOBAL_INT_USER_EN, BGP_DCR_GLOBAL_INT_USER_EN_CH0);

    return 0;
}

static int setup_netbus(void)
{
    int dma = personality->kernel.node & BGP_PERS_EN_DMA;
    int treeblast = personality->kernel.node & BGP_PERS_EN_TREEBLAST;
    int torus = personality->kernel.node & BGP_PERS_EN_TORUS;
    int tree = personality->kernel.node & BGP_PERS_EN_COLLECTIVE;
    u32 tokens;
    u32 enable1;
    u32 enable2;

    /* Bail out if no netbus client devices are enabled.  */
    if (!(dma || treeblast || torus || tree)) {
        return 0;
    }

    /* Decide the number of netbus tokens.  */
    tokens = BGP_DCR_NETBUS_TREE_WRITE_TOKENS(
                 BGP_DCR_NETBUS_TREE_WRITE_TOKENS_DEFAULT) |
             BGP_DCR_NETBUS_TREE_READ_TOKENS(
                 BGP_DCR_NETBUS_TREE_READ_TOKENS_DEFAULT);
    if (dma) {
        tokens |= BGP_DCR_NETBUS_TD_WRITE_TOKENS(
                      BGP_DCR_NETBUS_DMA_WRITE_TOKENS_DEFAULT) |
                  BGP_DCR_NETBUS_TD_READ_TOKENS(
                      BGP_DCR_NETBUS_DMA_READ_TOKENS_DEFAULT);
    } else {
        tokens |= BGP_DCR_NETBUS_TD_WRITE_TOKENS(
                      BGP_DCR_NETBUS_TORUS_WRITE_TOKENS_DEFAULT) |
                  BGP_DCR_NETBUS_TD_READ_TOKENS(
                      BGP_DCR_NETBUS_TORUS_READ_TOKENS_DEFAULT);
    }

    /* Build a mask that will enable torus dma and tree read/write.  */
    enable1 = BGP_DCR_NETBUS_TD_WRITE_EN |
              BGP_DCR_NETBUS_TD_READ_EN |
              BGP_DCR_NETBUS_TREE_WRITE_EN | 
              BGP_DCR_NETBUS_TREE_READ_EN;

    /* If configured add to masks the enable of tree blast.  */
    if (treeblast) {
        enable1 |= BGP_DCR_NETBUS_TREE_BLAST;
        enable2  = BGP_DCR_NETBUS_TREE_BLAST;
    }

    /* Initialize the netbus via DCR writes.  */
    mtdcrx(BGP_DCR_NETBUS_DP0_LOAD_TOKENS, tokens); /* dp0 dma - tree   */
    mtdcrx(BGP_DCR_NETBUS_DP0_ENABLE, enable1);     /* write 1s to load */
    mtdcrx(BGP_DCR_NETBUS_DP0_ENABLE, enable2);     /* toggle the load  */
    mtdcrx(BGP_DCR_NETBUS_DP1_LOAD_TOKENS, tokens); /* dp1 dma - tree   */
    mtdcrx(BGP_DCR_NETBUS_DP1_ENABLE, enable1);     /* write 1s to load */
    mtdcrx(BGP_DCR_NETBUS_DP1_ENABLE, enable2);     /* toggle the load  */

    return 0;
}

static volatile bgp_devbus_t* devbus = (bgp_devbus_t*)BGP_DEVBUS_VIRTADDR;

static int setup_devbus(void)
{
    u32 dcr;

    /* Bail out if ethernet not enabled in personality page.  */
    if (!(personality->kernel.node & BGP_PERS_EN_ETHERNET)) {
        return 0;
    }

    devbus->interrupt_enable = 0;

    devbus->l3_coherency = BGP_DEVBUS_L3CER_ETH_SELECT_ETH |
                           BGP_DEVBUS_L3CER_ETH_L3_RD_PF; 

    devbus->tomal_emac_control = BGP_DEVBUS_TECR_XMIT_BUFSZ_16K |
                                 BGP_DEVBUS_TECR_RECV_BUFSZ_10K |
                                 BGP_DEVBUS_TECR_TOMAL_PLBCLK_250MHZ;

    devbus->tomal_emac_control = devbus->tomal_emac_control    |
                                 BGP_DEVBUS_TECR_TOMAL_TX_SEL  |  
                                 BGP_DEVBUS_TECR_TOMAL_RX_SEL  |  
                                 BGP_DEVBUS_TECR_XEMAC_TX0_SEL | 
                                 BGP_DEVBUS_TECR_XEMAC_TX1_SEL | 
                                 BGP_DEVBUS_TECR_XEMAC_RX0_SEL | 
                                 BGP_DEVBUS_TECR_XEMAC_RX1_SEL | 
                                 BGP_DEVBUS_TECR_XENPAK_TX_ON;  

    devbus->xgxs_control_r0 = BGP_DEVBUS_XGCR0_DTE_PHY |
                              BGP_DEVBUS_XGCR0_PORT_ADDR(-1);

  
    dcr = mfdcrx(BGP_DCR_L30_ORDERING_CTRL);

    dcr = dcr | BN(18) | BN(19);

    mtdcrx(BGP_DCR_L30_ORDERING_CTRL, dcr);

    dcr = mfdcrx(BGP_DCR_L31_ORDERING_CTRL);

    dcr = dcr | BN(18) | BN(19);

    mtdcrx(BGP_DCR_L31_ORDERING_CTRL, dcr);

    return 0;
}

static int configure_class(u32 class, u16 specifier)
{
    u32 dcr, value, mask, prev, route;

    /* This a bug workaround, see issue #830.  */
    if (!specifier) {
        specifier = (BGP_TREE_RDR_SRCL | BGP_TREE_RDR_TGTL);
    }

    /* Bail out if specifier contains any invalid bits.  */
    if (specifier & ~BGP_TREE_RDR_ACCEPT) {
        return -1;
    }

    /* Make sure class is within allowed range.  */
    if (class >= BGP_TREE_RDR_NUM) {
        return -2;
    }

    /* Select one of the two packet classes there are per DCR.  */
    dcr = BGP_DCR_TREE_CLASS + (class >> 1);

    /* Use either the lower or upper half of the DCR.  */
    if (class & 1) {
        value = specifier;
        mask  = 0xFFFF0000;
    } else {
        value = specifier << 16;
        mask  = 0x0000FFFF;
    }

    /* Calculate the route.  */
    prev = mfdcrx(dcr);
    route = prev & mask;
    route |= value;

    /* Write the new route to the DCR.  */
    mtdcrx(dcr, route);

    return 0;
}

static int setup_collective(void)
{
    int i;
    u32 routes = 0;

    /* Bail out if personality page does not have collective on.  */
    if (!(personality->kernel.node & BGP_PERS_EN_COLLECTIVE)) {
        return 0;
    }

    /* Set node address.  */
    mtdcrx(BGP_DCR_TREE_GLOBAL_NODE_ADDR, personality->net.rank);

    /* Clear injection checksums.  */
    mtdcrx(BGP_DCR_TREE_INJ_CSHD0, 0);
    mtdcrx(BGP_DCR_TREE_INJ_CSPY0, 0);
    mtdcrx(BGP_DCR_TREE_INJ_CSHD1, 0);
    mtdcrx(BGP_DCR_TREE_INJ_CSPY1, 0);

    /* Set link idle patterns to arbitrary value.  */
    mtdcrx(BGP_DCR_TREE_CLASS_ISRA, 0xabcdabcd);
    mtdcrx(BGP_DCR_TREE_CLASS_ISRB, 0xabcdabcd);

    /* Set router timeout to maximum value.  */
    mtdcrx(BGP_DCR_TREE_ARB_RTO, 0xFFFFFFFF);

    /* If loopback enabled and SerDes is not configure loopback routes.  */
    if (personality->kernel.node & BGP_PERS_EN_LOOPBACK) {
        /* If serdes not enabled make all three channels loopback.  */
        if (!(personality->kernel.node & BGP_PERS_EN_SERDES)) {
            routes |= TREE_ARB_RCFG_LB0 | 
                      TREE_ARB_RCFG_LB1 | 
                      TREE_ARB_RCFG_LB2;
        }
    } else {
        /* Configure unused channels for loopback to avoid spurious errors.  */
        if (!(personality->kernel.node & BGP_PERS_ENABLE_TREEA))
            routes |= TREE_ARB_RCFG_LB0;
        if (!(personality->kernel.node & BGP_PERS_ENABLE_TREEB))
            routes |= TREE_ARB_RCFG_LB1;
        if (!(personality->kernel.node & BGP_PERS_ENABLE_TREEC))
            routes |= TREE_ARB_RCFG_LB2;
    }

    /* Write the loopback routes to the device control register.  */
    mtdcrx(BGP_DCR_TREE_ARB_RCFG, routes);

    /* Disable tree exceptions.  */
    mtdcrx(BGP_DCR_TREE_INJ_PIXEN, 0);
    mtdcrx(BGP_DCR_TREE_REC_PRXEN, 0);

    /* For each class configure the route passed in personality page.  */
    for (i = 0; i < BGP_TREE_RDR_NUM; i++) {
        configure_class(i, personality->net.treeroutes[i]);
    }

    /* Clear all processor interface exception flags.  */
    mfdcrx(BGP_DCR_TR_INJ_PIXF);
    mfdcrx(BGP_DCR_TR_REC_PRXF);

    /* Enable injection.  */
    mtdcrx(BGP_DCR_TR_INJ_PIXF, TREE_INJ_PIX_ENABLE);

    /* Enable all exceptions except the watermark ones.  */
    mtdcrx(BGP_DCR_TR_INJ_PIXEN, TR_INJ_PIX_ALL_BUT_WM);
    mtdcrx(BGP_DCR_TR_REC_PRXEN, TR_REC_PRX_ALL_BUT_WM);

    return 0;
}

static int setup_torus(void)
{
    /* FIXME: enable the torus.  */

    return 0;
}

static int configure_hss(void)
{
    int r = 0;
    int i = 0;
    u32 control, control0;
    u32 rx0, rx1, tx0, tx1;
    u32 reset, actual, expected;

    /* Set up bitmask.  */
    control = SERDES_HSS_CREG12_X4_HSSCALDRV(0x2) |
              SERDES_HSS_CREG12_X4_HSSDIVSEL(0x2) |
              SERDES_HSS_CREG12_X4_HSSRECCAL      |
              SERDES_HSS_CREG12_X4_HSSREFSEL      |
              SERDES_HSS_CREG12_X4_RG;

    /* Same bitmask as above but with HSSRECAL turned off.  */
    control0 = control & ~SERDES_HSS_CREG12_X4_HSSRECCAL;

    /* FIXME: temporary hack.  */
    u32 links = BGP_SERDES_TRAIN_ETH;

    if (links & BGP_SERDES_TRAIN_ETH) {
        u32 mask = SERDES_HSS_CREG12_X4_RG |
		   SERDES_HSS_CREG12_X4_HSSCALDRV(0x2) |
                   SERDES_HSS_CREG12_X4_HSSDIVSEL(0x3);
        r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG12, mask);
    } else if ((links & BGP_SERDES_TRAIN_X) && (links & BGP_SERDES_TRAIN_Z)) {
        r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG12, control0);
    }

    /* Configure X8 common control, pulsing HSSRECAL.  */
    r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG13, control0);
    r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG13, control);
    r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG13, control0);

    /* Set up bitmask for TX_REG0.  */
    tx0 = SERDES_HSS_TX_REG0_TX_AMPL(0xF)     | 
          SERDES_HSS_TX_REG0_TX_COEF(0x4)     |
          SERDES_HSS_TX_REG0_TX_SLEW(0x3)     |
          SERDES_HSS_TX_REG0_TX_JTAGAMPL(0xF) |
          SERDES_HSS_TX_REG0_TX_TS;

    /* Set up bitmask for TX_REG1.  */
    tx1 = 0;

    /* Tree A transmit.  */
    r += mtdcrx_v(BGP_DCR_SERDES_A0 + BGP_DCR_SERDES_HSS_TX_REG0_OFFSET, tx0);
    r += mtdcrx_v(BGP_DCR_SERDES_A0 + BGP_DCR_SERDES_HSS_TX_REG1_OFFSET, tx1);
    r += mtdcrx_v(BGP_DCR_SERDES_A1 + BGP_DCR_SERDES_HSS_TX_REG0_OFFSET, tx0);
    r += mtdcrx_v(BGP_DCR_SERDES_A1 + BGP_DCR_SERDES_HSS_TX_REG1_OFFSET, tx1);

    /* Tree B transmit.  */
    r += mtdcrx_v(BGP_DCR_SERDES_B0 + BGP_DCR_SERDES_HSS_TX_REG0_OFFSET, tx0);
    r += mtdcrx_v(BGP_DCR_SERDES_B0 + BGP_DCR_SERDES_HSS_TX_REG1_OFFSET, tx1);
    r += mtdcrx_v(BGP_DCR_SERDES_B1 + BGP_DCR_SERDES_HSS_TX_REG0_OFFSET, tx0);
    r += mtdcrx_v(BGP_DCR_SERDES_B1 + BGP_DCR_SERDES_HSS_TX_REG1_OFFSET, tx1);
  
    /* Tree C transmit.  */
    r += mtdcrx_v(BGP_DCR_SERDES_C0 + BGP_DCR_SERDES_HSS_TX_REG0_OFFSET, tx0);
    r += mtdcrx_v(BGP_DCR_SERDES_C0 + BGP_DCR_SERDES_HSS_TX_REG1_OFFSET, tx1);
    r += mtdcrx_v(BGP_DCR_SERDES_C1 + BGP_DCR_SERDES_HSS_TX_REG0_OFFSET, tx0);
    r += mtdcrx_v(BGP_DCR_SERDES_C1 + BGP_DCR_SERDES_HSS_TX_REG1_OFFSET, tx1);

    /* Set up bitmask for RX_REG0.  */
    rx0 = 0x03208000;

    /* Set up bitmask for RX_REG1.  */
    rx1 = 0;

    /* Tree A receive.  */
    r += mtdcrx_v(BGP_DCR_SERDES_A0 + BGP_DCR_SERDES_HSS_RX_REG0_OFFSET, rx0);
    r += mtdcrx_v(BGP_DCR_SERDES_A0 + BGP_DCR_SERDES_HSS_RX_REG1_OFFSET, rx1);
    r += mtdcrx_v(BGP_DCR_SERDES_A1 + BGP_DCR_SERDES_HSS_RX_REG0_OFFSET, rx0);
    r += mtdcrx_v(BGP_DCR_SERDES_A1 + BGP_DCR_SERDES_HSS_RX_REG1_OFFSET, rx1);
  
    /* Tree B receive.  */
    r += mtdcrx_v(BGP_DCR_SERDES_B0 + BGP_DCR_SERDES_HSS_RX_REG0_OFFSET, rx0);
    r += mtdcrx_v(BGP_DCR_SERDES_B0 + BGP_DCR_SERDES_HSS_RX_REG1_OFFSET, rx1);
    r += mtdcrx_v(BGP_DCR_SERDES_B1 + BGP_DCR_SERDES_HSS_RX_REG0_OFFSET, rx0);
    r += mtdcrx_v(BGP_DCR_SERDES_B1 + BGP_DCR_SERDES_HSS_RX_REG1_OFFSET, rx1);
  
    /* Tree C receive.  */
    r += mtdcrx_v(BGP_DCR_SERDES_C0 + BGP_DCR_SERDES_HSS_RX_REG0_OFFSET, rx0);
    r += mtdcrx_v(BGP_DCR_SERDES_C0 + BGP_DCR_SERDES_HSS_RX_REG1_OFFSET, rx1);
    r += mtdcrx_v(BGP_DCR_SERDES_C1 + BGP_DCR_SERDES_HSS_RX_REG0_OFFSET, rx0);
    r += mtdcrx_v(BGP_DCR_SERDES_C1 + BGP_DCR_SERDES_HSS_RX_REG1_OFFSET, rx1);

    /* Set up bitmask to reset HSS.  */
    reset = SERDES_HSS_CREG14_X8_HSSRESET |
            SERDES_HSS_CREG14_X4_HSSRESET |
            SERDES_HSS_CREG14_HSSRESET_CAL;

    /* Reset HSS.  */
    r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG14, reset);
    r += mtdcrx_v(BGP_DCR_SERDES_HSS_CREG14, 0x00000000);

    /* Leave now if errors were encountered.  */
    if (r) {
        printf("bailing out of HSS init because of errors\n");
        return r;
    }

    /* Set up expected bitmask for X8 core.  */ 
    expected = SERDES_HSS_SREG52_X8_HSSPLLLOCK |
               SERDES_HSS_SREG52_X8_HSSREADY;

    /* Spin until phase locked loop is achieved.  */
    do {
        /* Read the current value in the DCR.  */
        actual = mfdcrx(BGP_DCR_SERDES_HSS_SREG52);

        /* A delay is probably required here.  */
        delay(10000);

        /* Bail out if we have tried too many times.  */
        if (i >= 1000) {
            break;
        }
        i++;
    } while(actual != expected);

    return r;
}

static int train_transmitter_links(void)
{
  u32 tree_r0 = SERDES_TR_SEND_CTL_REG0_TRAINCLKS(0x40) |
                SERDES_TR_SEND_CTL_REG0_RANDOMCLKS(0x40);

  u32 tree_config_r2 = 0;
  u32 tree_run_r2 = 0;
  u32 tree_run_r3 = 0;
  u32 tree_config_r3 = SERDES_TR_SEND_CTL_REG3_SEND_NEW_PATTERN;

  /* Do SerDes scramble.  */
  tree_config_r3 |= SERDES_TR_SEND_CTL_REG3_SCRAMBLE;

  /* Training configuration for A.  */
  tree_r0 |= SERDES_TR_SEND_CTL_REG0_TRAIN_A;

  tree_config_r2 |= SERDES_TR_SEND_CTL_REG2_EB_LAT_A0(2) |
                    SERDES_TR_SEND_CTL_REG2_EB_LAT_A1(2);

  tree_run_r2 |= SERDES_TR_SEND_CTL_REG2_EB_RUN_A0 |   
                 SERDES_TR_SEND_CTL_REG2_EB_RUN_A1;

  /* Training configuration for B.  */
  tree_r0 |= SERDES_TR_SEND_CTL_REG0_TRAIN_B;

  tree_config_r2 |= SERDES_TR_SEND_CTL_REG2_EB_LAT_B0(2) | 
                    SERDES_TR_SEND_CTL_REG2_EB_LAT_B1(2);

  tree_run_r2 |= SERDES_TR_SEND_CTL_REG2_EB_RUN_B0 | 
                 SERDES_TR_SEND_CTL_REG2_EB_RUN_B1;

  /* Training configuration for C.  */
  tree_r0 |= SERDES_TR_SEND_CTL_REG0_TRAIN_C;

  tree_config_r3 |= SERDES_TR_SEND_CTL_REG3_EB_LAT_C0(2) |  
                    SERDES_TR_SEND_CTL_REG3_EB_LAT_C1(2);

  tree_run_r3 |= SERDES_TR_SEND_CTL_REG3_EB_RUN_C0 |
                 SERDES_TR_SEND_CTL_REG3_EB_RUN_C1;

  /* Write the configuration values to the SERDES Tree DCRs.  */
  mtdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG2, tree_config_r2);
  mtdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG3, tree_config_r3);
  mtdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG0, tree_r0);

  /* Add in the "run" bits.  */
  mtdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG2, (tree_config_r2 | tree_run_r2));
  mtdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG3, (tree_config_r3 | tree_run_r3));

  return 0;
}

static int train_receiver_links(void)
{
    int i, j;

    u32 tr0 = SERDES_TR_CTL_RVC_ELASTIC_BUF_LAT(2) | SERDES_TR_CTL_RVC_TRAIN;
    u32 tr1 = SERDES_TR_CTL_RVC_ELASTIC_BUF_RUN;

    /* Train tree receiver links.  */
    for (i = 0; i < 3; i += 2) {
        for (j = 0; j < 2; j++) {
            mtdcrx(BGP_DCR_SERDES_TR_CTL + i + j, tr0);
            mtdcrx(BGP_DCR_SERDES_TR_CTL + i + j, tr0 | tr1);
        }
    }

    return 0;
}

static int test_links(void)
{
    int loops = 10000;
    u32 actual, expected = 0xfc000000;

    while (loops--) {
	actual = mfdcrx(BGP_DCR_SERDES_TR_STAT_BYTE_FOUND);
	if ((actual & expected) == expected)
	    break;
    }

    printf("Active links: %x\n", (actual & expected) >> 26);
    return 0;
}

static int stop_training(void)
{
    u32 mask;

    u32 bits = SERDES_TR_SEND_CTL_REG0_TRAIN_A |
               SERDES_TR_SEND_CTL_REG0_TRAIN_B |
               SERDES_TR_SEND_CTL_REG0_TRAIN_C;

    mask = mfdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG0);
    mask &= ~bits;

    /* Clear all tree training bits on the transmitter side.  */
    mtdcrx(BGP_DCR_SERDES_TR_CTRL_SEND_REG0, mask);

    return 0;
}

static int setup_serdes(void)
{
    int r;

    r = configure_hss();
    if (r) return 1;

    r = train_transmitter_links();
    if (r) return 1;

    r = train_receiver_links();
    if (r) return 1;

    r = test_links();
    if (r) return 1;

    r = stop_training();
    if (r) return 1;

    return 0;
}

typedef union {
    unsigned gi_status_word;         /*!< Anonymous union to access as a word */
    struct {
        unsigned mode:4;          /*!< Mode 0-3 (1=sticky, 0=direct) */
        unsigned stickyInt:4;     /*!< Sticky mode interrupt 0-3 (1=active) */
        unsigned stickyActive:4;  /*!< Sticky mode in progress 0-3 (1=true) */
        unsigned stickyType:4;    /*!< Sticky mode type 0-3 (0=OR, 1=AND) */
        unsigned reserved0:1;
        unsigned accessviolation:1;/*!< Access violation (1=true) */
        unsigned reserved1:1;
        unsigned parityErr:1;     /*!< Parity error (1=true) */
        unsigned giRecvState:4;   /*!< Receive state 0-3 ("downtree" state) */
        unsigned userEnables:4;   /*!< User-level enables 0-3 (1=enabled). */
        /* Actual "uptree" state when readMode=1. */
        unsigned giSendState:4;   /*!< Send state 0-3 ("uptree" state) */
    };
} GlobInt_Status_t;

inline GlobInt_Status_t 
GlobInt_GetStatus(void)
{
    union
    {
	u32 uint32;
	GlobInt_Status_t globint;
    } status;
    status.uint32 = mfdcrx(BGP_DCR_GLOBAL_INT_STATUS);
    return status.globint;
}

inline u32
global_int_init(void)
{
    /* Enable channel zero of global interrupts.  */
    mtdcrx(BGP_DCR_GLOBAL_INT_USER_EN, BGP_DCR_GLOBAL_INT_USER_EN_CH0);

    /* Enable all channels */
    /* FIXME:  we are not actually configuring them here just using them in their last config */
    /*         probably want to fix this and configure them in the standard config */
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(0), 1); 
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(1), 1); 
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(2), 1); 
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(3), 1);

    return 1;
}

inline u32
global_int_disable_all(void)
{
    // take use out of all channels
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(0), 0);
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(1), 0);
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(2), 0);
    mtdcrx(BGP_DCR_GLOBAL_INT_ASSERT_CH(3), 0);

    // disable user access to all channels
    mtdcrx(BGP_DCR_GLOBAL_INT_USER_EN, 0);
    return 1;
}

/* We are still running out of ROM when this function is called.  */
int board_early_init_f(void)
{
    int r, cpu;

    /* Configure mailboxes for console output.  */
    setup_mailboxes();

    /* Configure global interrupt/barrier network */
    global_int_init();

    /* Bail out here if CNS is running. */
    if (running_on_cns()) {
	asm volatile ("mfspr %0,%1" 
		      : "=r"(cns_descriptor) 
		      : "i"(sprg1));

	//printf("CNS descriptor @ %p\n", cns_descriptor);

	if (valid_cns_descriptor(cns_descriptor))
	{
#if 0
	    printf("services: %p, base_vaddr=%lx, base_paddr=%lx, get_num_cores: %p, take_cpu %p\n", 
		   cns_descriptor->services, cns_descriptor->base_vaddr, 
		   cns_descriptor->base_paddr,
		   cns_descriptor->services->get_number_cores,
		   cns_descriptor->services->take_cpu);

	    printf("entry: %p\n", _start_bg);
#endif
	    /* release other CPUs when on CNS */
	    for (cpu = 1; cpu < cns_descriptor->services->get_number_cores(); cpu++) {
		r = cns_descriptor->services->take_cpu(cpu, NULL, _start_bg);
		if (r != 0)
		    printf("ERROR: failed to release CPU %d from CNS\n", cpu);
	    }
	}

        //printf("UCI #%d: deferring to CNS\n", personality->kernel.uci);
        return 0;
    }

    printf("UCI #%d: begin link training\n", personality->kernel.uci);

    /* Configure global interrupts.  */
    r = setup_global_interrupts();
    if (r) panic("panic: global interrupt initialization failed");

    /* Configure netbus.  */
    r = setup_netbus();
    if (r) panic("panic: netbus initialization failed");

    /* Configure devbus.  */
    r = setup_devbus();
    if (r) panic("panic: devbus initialization failed");

    /* Configure collective.  */
    r = setup_collective();
    if (r) panic("panic: collective initialization failed");

    /* Configure torus.  */
    r = setup_torus();
    if (r) panic("panic: torus initialization failed");

    /* Configure serdes, which constitutes link training.  */
    r = setup_serdes();
#ifndef CONFIG_BGP_KHVMM
    if (r) panic("panic: serdes initialization failed");
#endif
    return 0;
}

/* first function called after relocation */
int board_early_init_r(void)
{
    /* relocate remaining processors as well */
    *cpu_entry_unreloc = (unsigned long)spin_app_processor;
    invalidate_dcache_line(cpu_entry_unreloc);

    return 0;
}

int checkboard(void)
{
	return (0);
}

#define MB(x) ((x) * 1024 * 1024)
#define GB(x) (MB(x) * 1024)

void dump_tlb(void)
{
    int i;
    unsigned long tlb_val[3];

    for (i = 0; i < PPC4XX_TLB_SIZE; i++) {
        tlb_val[0] = mftlb1(i);
        tlb_val[1] = mftlb2(i);
        tlb_val[2] = mftlb3(i);
        if (!(tlb_val[0] & TLB_VALID))
            continue;

        printf("TLB %02d: [%08x (%2d) %08x %08x]\n", i, 
               tlb_val[0], 1 << ((tlb_val[0] & TLB_WORD0_SIZE_MASK) / 8),
               tlb_val[1], tlb_val[2]);
    } 
}

long int initdram(int board)
{
    long megabytes, bytes;

    megabytes = personality->ddr.sizemb;
#ifndef CONFIG_BGP_KHVMM
    /*
     * We limit u-boot to 2Gig, because it needs virtual address space for
     * device memory.  The true memory size is pasted into the device tree
     * in bg_fdt_setup().
     */
    if (megabytes > 2048) megabytes = 2048;
#endif
    bytes = megabytes * 1024 * 1024;

    return bytes;
}

static int add_tlb_entry(unsigned long virt_addr, unsigned long long phys_addr,
			 unsigned long size, unsigned long perms)
{
	int i;
	unsigned long val[3];

	/* First, find the index of a TLB entry not being used */
	for (i=0; i<PPC4XX_TLB_SIZE; i++) {
		val[0] = mftlb1(i);
		if ((val[0] & TLB_WORD0_V_MASK) == TLB_WORD0_V_DISABLE)
			break;
	}
	if (i >= PPC4XX_TLB_SIZE)
		return -1;

	val[0] = TLB_WORD0_EPN_ENCODE(virt_addr) | TLB_WORD0_V_ENABLE | 
		TLB_WORD0_TS_0 | size;

	val[1] = TLB_WORD1_RPN_ENCODE(phys_addr) | TLB_WORD1_ERPN_ENCODE(phys_addr >> 32ULL);
	
	val[2] =  TLB_WORD2_SR_ENABLE | TLB_WORD2_SW_ENABLE | TLB_WORD2_SX_ENABLE | 
		perms;

	sync();
	mttlb1(i, val[0]);
	mttlb2(i, val[1]);
	mttlb3(i, val[2]);
	asm("isync");

	return 0;
}


int misc_init_f (void)
{
    // add device mappings...
    
    add_tlb_entry(CFG_IRQCTRL_BASE, BGP_IRQCTRL_PHYS, SZ_4K, SA_I|SA_G|AC_R|AC_W|AC_X);
    add_tlb_entry(CFG_TREE_CHN0, BGP_TREE_CHN0_PHYS, SZ_4K, SA_I|SA_G|AC_R|AC_W|AC_X);
    add_tlb_entry(CFG_TREE_CHN1, BGP_TREE_CHN1_PHYS, SZ_4K, SA_I|SA_G|AC_R|AC_W|AC_X);

    return 0;
}


/*************************************************************************
 *  hw_watchdog_reset
 *
 *	This routine is called to reset (keep alive) the watchdog timer
 *
 ************************************************************************/
#if defined(CONFIG_HW_WATCHDOG)
void hw_watchdog_reset(void)
{

}
#endif

void board_reset(void)
{
	while(1);
}

/**********************************************************************
 * RTC dummy functions
 **********************************************************************/

static struct rtc_time time;

void rtc_get (struct rtc_time *tmp)
{
    *tmp = time;
}

/* we use rtc updates for updating the FDT which is then used by the next layer... */ 
void rtc_set (struct rtc_time *tmp) 
{
    int nodeoffset;
    time = *tmp;

    if ((nodeoffset = fdt_path_offset(fdt, "/rtc")) < 0) {
	if ((nodeoffset = fdt_add_subnode(fdt, 0, "rtc")) < 0) {
	    printf("Error creating RTC FDT entry: %s\n", fdt_strerror(nodeoffset));
	    return;
	}
    }
    
    if (fdt_setprop(fdt, nodeoffset, "second", &tmp->tm_sec, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "minute", &tmp->tm_min, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "hour", &tmp->tm_hour, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "mday", &tmp->tm_mday, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "month", &tmp->tm_mon, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "year", &tmp->tm_year, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "wday", &tmp->tm_wday, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "yday", &tmp->tm_yday, sizeof(int)) < 0 ||
	fdt_setprop(fdt, nodeoffset, "dst", &tmp->tm_isdst, sizeof(int)) < 0)
	printf("Error updating RTC values in FDT\n");
}

void rtc_reset() {}

/**********************************************************************
 * network helpers
 **********************************************************************/

unsigned int bg_uci_to_nodeid(u32 bg_uci) 
{
    return (bg_uci >> 8) & (( 1 << 19) - 1); 
}

void bg_get_eth_address(int khnic, unsigned char *eth_addr)
{
    BGP_UCI_ComputeCard_t *ccuci = (BGP_UCI_ComputeCard_t *)&(personality->kernel.uci);
    bgp_personality_ethernet_t *pers_eth = &(personality->eth);

    if (khnic == 0) {
        /* assign mac address for non kittyhawk interface */
        eth_addr[0] = pers_eth->emacid[0];
        eth_addr[1] = pers_eth->emacid[1];
        eth_addr[2] = pers_eth->emacid[2];
        eth_addr[3] = pers_eth->emacid[3];
        eth_addr[4] = pers_eth->emacid[4];
        eth_addr[5] = pers_eth->emacid[5];
    } else {
        /* we use the locally admined etherenet range Z2:XX:YY specifically we use 02:XX:YY */
        /* where XX and YY are generated from row,column, midplane, nodecard and a bit from the compute card */
        /* to distinguish each group of 16 nodes */
        eth_addr[1] = ccuci->RackRow << 4 | ccuci->RackColumn;
        eth_addr[2] = ccuci->Midplane << 5 | ccuci->NodeCard << 1;
        if (ccuci->ComputeCard >= 4) {
            eth_addr[2] |= (((ccuci->ComputeCard - 4) & 0x10) >> 4);
        } else {
            eth_addr[2] |= (ccuci->ComputeCard & 0x1);
        }

	/* mask out lower bits so that all NICs in a pset have the same prefix */
	eth_addr[2] &= ~((personality->net.psetsize / 16)- 1);

	if (khnic == 1) {
	    unsigned int nodeid = personality->net.rank;
    	    eth_addr[0] = 0x02;
	    eth_addr[3] = (nodeid >> 16) & 0xff;
	    eth_addr[4] = (nodeid >>  8) & 0xff;
	    eth_addr[5] = (nodeid >>  0) & 0xff;
	} else {
	    eth_addr[0] = 0x06;
	    eth_addr[3] = personality->net.xcoord;
	    eth_addr[4] = personality->net.ycoord;
	    eth_addr[5] = personality->net.zcoord;
	}
    }
}

/********************************************************************
 * Flattened Device tree functions
 ********************************************************************/
extern void fdt_register_mailboxes(void *fdt);

static void delete_device_node(void *fdt, char *name)
{
    int nodeoffset, err;
    nodeoffset = fdt_path_offset(fdt, name);
    if (nodeoffset < 0)
	printf("Error finding device node %s (%s)\n", 
	       name, fdt_strerror(nodeoffset));
    else if ((err = fdt_del_node(fdt, nodeoffset)) < 0)
	printf("Error deleting device node %s (%s)\n", 
	       name, fdt_strerror(err));
}


#ifndef CONFIG_BGP_KHVMM

static int
add(void *fdt, int node, char *name, void *ptr, int size)
{
    int err;

    err = fdt_setprop(fdt, node, name, ptr, size);
    if (err < 0) {
	printf("Setting property %s failed: %s\n", name, fdt_strerror(err));
	return 0;
    }

    return 1;
}

static void
add_personality(void *fdt)
{
    int top, kernel, ddr, net, eth;

#define ADD(node, name, field) \
    if (!add(fdt, node, name, &personality->field, \
				sizeof(personality->field))) return

    if (fdt_path_offset(fdt, "/personality") >= 0) {
	printf("FDT entry /personality already exists.\n");
	return;
    }

    top = fdt_add_subnode(fdt, 0, "personality");
    if (top < 0) {
	printf("Error creating /personality FDT entry: %s\n",
	       fdt_strerror(top));
	return;
    }

    ADD(top, "crc",     crc);
    ADD(top, "version", version);
    ADD(top, "size",    size);

    kernel = fdt_add_subnode(fdt, top, "kernel");
    if (kernel < 0) {
	printf("Error creating /personality/kernel FDT entry: %s\n",
	       fdt_strerror(kernel));
	return;
    }

    ADD(kernel, "uci",        kernel.uci);
    ADD(kernel, "mhz",        kernel.mhz);
    ADD(kernel, "policy",     kernel.policy);
    ADD(kernel, "process",    kernel.process);
    ADD(kernel, "trace",      kernel.trace);
    ADD(kernel, "node",       kernel.node);
    ADD(kernel, "l1",         kernel.l1);
    ADD(kernel, "l2",         kernel.l2);
    ADD(kernel, "l3",         kernel.l3);
    ADD(kernel, "l3_select",  kernel.l3_select);
    ADD(kernel, "shared_mem", kernel.shared_mem);
    ADD(kernel, "clockstop0", kernel.clockstop0);
    ADD(kernel, "clockstop1", kernel.clockstop1);

    ddr = fdt_add_subnode(fdt, top, "ddr");
    if (ddr < 0) {
	printf("Error creating /personality/ddr FDT entry: %s\n",
	       fdt_strerror(ddr));
	return;
    }

    ADD(ddr, "ddr_flags",        ddr.ddr_flags);
    ADD(ddr, "srbs0",            ddr.srbs0);
    ADD(ddr, "srbs1",            ddr.srbs1);
    ADD(ddr, "pbx0",             ddr.pbx0);
    ADD(ddr, "pbx1",             ddr.pbx1);
    ADD(ddr, "memconfig0",       ddr.memconfig0);
    ADD(ddr, "memconfig1",       ddr.memconfig1);
    ADD(ddr, "parmctl0",         ddr.parmctl0);
    ADD(ddr, "parmctl1",         ddr.parmctl1);
    ADD(ddr, "miscctl0",         ddr.miscctl0);
    ADD(ddr, "miscctl1",         ddr.miscctl1);
    ADD(ddr, "cmdbufmode0",      ddr.cmdbufmode0);
    ADD(ddr, "cmdbufmode1",      ddr.cmdbufmode1);
    ADD(ddr, "refrinterval0",    ddr.refrinterval0);
    ADD(ddr, "refrinterval1",    ddr.refrinterval1);
    ADD(ddr, "odtctl0",          ddr.odtctl0);
    ADD(ddr, "odtctl1",          ddr.odtctl1);
    ADD(ddr, "datastrobecalib0", ddr.datastrobecalib0);
    ADD(ddr, "datastrobecalib1", ddr.datastrobecalib1);
    ADD(ddr, "dqsctl",           ddr.dqsctl);
    ADD(ddr, "throttle",         ddr.throttle);
    ADD(ddr, "sizemb",           ddr.sizemb);
    ADD(ddr, "chips",            ddr.chips);
    ADD(ddr, "cas",              ddr.cas);

    net = fdt_add_subnode(fdt, top, "net");
    if (net < 0) {
	printf("Error creating /personality/net FDT entry: %s\n",
	       fdt_strerror(net));
	return;
    }

    ADD(net, "blockid",    net.blockid);
    ADD(net, "xnodes",     net.xnodes);
    ADD(net, "ynodes",     net.ynodes);
    ADD(net, "znodes",     net.znodes);
    ADD(net, "xcoord",     net.xcoord);
    ADD(net, "ycoord",     net.ycoord);
    ADD(net, "zcoord",     net.zcoord);
    ADD(net, "psetnum",    net.psetnum);
    ADD(net, "psetsize",   net.psetsize);
    ADD(net, "rankinpset", net.rankinpset);
    ADD(net, "ionodes",    net.ionodes);
    ADD(net, "rank",       net.rank);
    ADD(net, "ionoderank", net.ionoderank);
    ADD(net, "treeroutes", net.treeroutes);

    eth = fdt_add_subnode(fdt, top, "eth");
    if (eth < 0) {
	printf("Error creating /personality/eth FDT entry: %s\n",
	       fdt_strerror(eth));
	return;
    }

    ADD(eth, "mtu",         eth.mtu);
    ADD(eth, "emacid",      eth.emacid);
    ADD(eth, "ip",          eth.ip);
    ADD(eth, "netmask",     eth.netmask);
    ADD(eth, "broadcast",   eth.broadcast);
    ADD(eth, "gateway",     eth.gateway);
    ADD(eth, "nfsserver",   eth.nfsserver);
    ADD(eth, "servicenode", eth.servicenode);
    ADD(eth, "nfsexport",   eth.nfsexport);
    ADD(eth, "nfsmount",    eth.nfsmount);
    ADD(eth, "securitykey", eth.securitykey);

#undef ADD
}
#endif

void
bg_fdt_setup (void *fdt, bd_t *bd)
{
    BGP_UCI_ComputeCard_t *ccuci = (BGP_UCI_ComputeCard_t*)&personality->kernel.uci;
    int nodeoffset, err;
    u32 tmp[3];
    unsigned char mac_address[6];

#ifndef CONFIG_BGP_KHVMM
    add_personality(fdt);
#endif
    
    nodeoffset = fdt_path_offset (fdt, "/memory");
    if (nodeoffset >= 0) {
#ifndef CONFIG_BGP_KHVMM
        unsigned long long start, size;
	start = CFG_SDRAM_BASE;
	size = ((unsigned long long) personality->ddr.sizemb) * 1024 * 1024;
	if (size > 0xfffff000) size = 0xfffff000;  /* largest we can specify */

	tmp[0] = cpu_to_be32((unsigned int) (start >> 32));
	tmp[1] = cpu_to_be32((unsigned int) (start & 0xffffffff));
	tmp[2] = cpu_to_be32((unsigned int) (size & 0xffffffff));
#else
        tmp[0] = cpu_to_be32(bd->bi_memstart);
	tmp[1] = 0;
	tmp[2] = cpu_to_be32(bd->bi_memsize);
#endif
	fdt_setprop(fdt, nodeoffset, "reg", tmp, sizeof(tmp));
    }

    nodeoffset = fdt_path_offset(fdt, "/plb/opb/ethernet@720004000");
    if (nodeoffset < 0)
	printf("Error finding EMAC device node (%s)\n", fdt_strerror(nodeoffset));
    else
    {
	if (ccuci->ComputeCard >= 4) {
	    // remove ethernet node on compute nodes
	    if ((err = fdt_del_node(fdt, nodeoffset)) < 0)
		printf("Error deleting EMAC device node (%s)\n", fdt_strerror(err));
	    delete_device_node(fdt, "/plb/opb/emac-xgmii@720000000");
	    delete_device_node(fdt, "/plb/mcmal");
	} else {
            bg_get_eth_address(0, mac_address);
	    if ((err = fdt_setprop(fdt, nodeoffset, "local-mac-address", mac_address, sizeof(mac_address))) < 0)
		printf("Error updating EMAC mac address (%s)\n", fdt_strerror(err));
	}
    }

    nodeoffset = fdt_path_offset(fdt, "/plb/torus");
    if (nodeoffset < 0)
	printf("Error finding Torus device node (%s)\n", fdt_strerror(nodeoffset));
    else
    {
	// delete torus on IO nodes
	if (ccuci->ComputeCard < 4)
	    fdt_del_node(fdt, nodeoffset);
	else {
	    // set torus XYZ coordinates
	    tmp[0] = personality->net.xcoord;
	    tmp[1] = personality->net.ycoord;
	    tmp[2] = personality->net.zcoord;
	    if (fdt_setprop(fdt, nodeoffset, "coordinates", tmp, sizeof(tmp)) < 0)
		printf("FDT: Error updating torus coordinates\n");

	    // set torus size
	    tmp[0] = personality->net.xnodes;
	    tmp[1] = personality->net.ynodes;
	    tmp[2] = personality->net.znodes;
	    if (fdt_setprop(fdt, nodeoffset, "dimension", tmp, sizeof(tmp)) < 0)
		printf("FDT: Error updating torus dimension\n");
	}
    }

    /* update tree info from pers page */
    nodeoffset = fdt_path_offset(fdt, "/plb/tree");
    if (nodeoffset < 0)
        printf("Error finding Tree device node (%s)\n", fdt_strerror(nodeoffset));
    else {
        u32 tmp = personality->net.rank;
        if ((err = fdt_setprop(fdt, nodeoffset, "nodeid", &tmp, sizeof(tmp))) < 0)
            printf("Error updating tree nodeid (%s)\n", fdt_strerror(err));
        if ((err = fdt_setprop(fdt, nodeoffset, "routes", &personality->net.treeroutes,
                               sizeof(personality->net.treeroutes))) < 0)
            printf("Error updating tree routes (%s)\n", fdt_strerror(err));
    }

    // set mac address of tree/torus
    nodeoffset = fdt_path_offset(fdt, "/plb/ethernet@0");
    if (nodeoffset < 0)
	printf("Error finding tree/torus ethernet adapter\n");
    else {
	bg_get_eth_address(1, mac_address);
	if ((err = fdt_setprop(fdt, nodeoffset, "local-mac-address", mac_address, sizeof(mac_address))) < 0)
	    printf("Error updating mac address of tree/torus");
    }

    fdt_register_mailboxes(fdt);
}

/********************************************************************
 * blue gene specific environment variables
 ********************************************************************/

int is_ionode(void)
{
    bgp_personality_t *personality = (bgp_personality_t *)BGP_PERS_BASE;
    BGP_UCI_ComputeCard_t *ccuci;

    ccuci = (BGP_UCI_ComputeCard_t *)&personality->kernel.uci;

    if (ccuci->ComputeCard == 0 || ccuci->ComputeCard == 1) {
        return 1;
    }

    return 0;
}

static void bg_setenv_vars(void) 
{
    bgp_personality_t *pers = (bgp_personality_t*)BGP_PERS_BASE;
    BGP_UCI_ComputeCard_t *ccuci;
    bgp_personality_network_t *net;
    bgp_personality_ethernet_t *eth;
    char tmp[80];
    unsigned int uni, octet2 = 0, octet3 = 0, octet4 = 1;
    unsigned int total_nodes;

    net = &pers->net;
    eth = &pers->eth;

    ccuci=(BGP_UCI_ComputeCard_t *)&(pers->kernel.uci);
    sprintf(tmp, "0x%x", pers->kernel.uci);
    setenv("bgp_uci", tmp);
    

    uni = bg_uci_to_uni(ccuci);
    sprintf(tmp, "%d", uni);
    setenv("uni", tmp);
   
    sprintf(tmp, "0x%x", ccuci->Component);
    setenv("bgp_component", tmp);

    sprintf(tmp, "%d", ccuci->RackRow);
    setenv("bgp_rackrow", tmp);

    sprintf(tmp, "%d", ccuci->RackColumn);
    setenv("bgp_rackcolumn", tmp);

    sprintf(tmp, "%d", ccuci->Midplane);
    setenv("bgp_midplane", tmp);
    
    sprintf(tmp, "%d", ccuci->NodeCard);
    setenv("bgp_nodecard", tmp);

    sprintf(tmp, "%d", ccuci->ComputeCard);
    setenv("bgp_computecard", tmp);

    if (ccuci->ComputeCard == 0 || ccuci->ComputeCard == 1) {
        setenv("bgp_isio", "1");
        sprintf(tmp, "%d.%d.%d.%d",
                eth->ip.octet[12], eth->ip.octet[13], eth->ip.octet[14],
                eth->ip.octet[15]);
        setenv("bgp_ioeth_ip", tmp);
        
        sprintf(tmp, "%d.%d.%d.%d",
                eth->netmask.octet[12], eth->netmask.octet[13],
                eth->netmask.octet[14], eth->netmask.octet[15]);
        setenv("bgp_ioeth_netmask", tmp);
        
        sprintf(tmp, "%d.%d.%d.%d",
                eth->broadcast.octet[12], eth->broadcast.octet[13],
                eth->broadcast.octet[14], eth->broadcast.octet[15]);
        setenv("bgp_ioeth_broadcast", tmp);
        
        sprintf(tmp, "%d.%d.%d.%d",
                eth->gateway.octet[12],eth->gateway.octet[13],
                eth->gateway.octet[14],eth->gateway.octet[15]);
        setenv("bgp_ioeth_gateway", tmp);
        
        sprintf(tmp, "%d.%d.%d.%d",
                eth->nfsserver.octet[12],eth->nfsserver.octet[13],
                eth->nfsserver.octet[14],eth->nfsserver.octet[15]);
        setenv("bgp_ioeth_nfsserver", tmp);
        
        sprintf(tmp, "%d.%d.%d.%d",
                eth->servicenode.octet[12],eth->servicenode.octet[13],
                eth->servicenode.octet[14],eth->servicenode.octet[15]);
        setenv("bgp_ioeth_servicenode", tmp);
    } else{
        setenv("bgp_isio", "0");
    }

    sprintf(tmp, "R%1d%1d-M%1d-N%02d-J%02d", ccuci->RackRow,
            ccuci->RackColumn, ccuci->Midplane, ccuci->NodeCard,
            ccuci->ComputeCard);
    setenv("bgp_location", tmp);
    
    sprintf(tmp, "%d", net->psetnum);
    setenv("bgp_psetnum", tmp);
    
    sprintf(tmp, "%d", net->psetsize);
    setenv("bgp_psetsize", tmp);
    
    sprintf(tmp, "%d", net->rankinpset);
    setenv("bgp_rankinpset", tmp);
    
    sprintf(tmp, "%d", net->ionodes);
    setenv("bgp_ionodes", tmp);

    sprintf(tmp, "%d", net->rank);
    setenv("bgp_rank", tmp);

    sprintf(tmp, "%d", net->ionoderank);
    setenv("bgp_ionoderank", tmp);

    sprintf(tmp, "%d", net->xnodes);
    setenv("bgp_xnodes", tmp);

    sprintf(tmp, "%d", net->ynodes);
    setenv("bgp_ynodes", tmp);

    sprintf(tmp, "%d", net->znodes);
    setenv("bgp_znodes", tmp);

    total_nodes = net->xnodes * net->ynodes * net->znodes;
    sprintf(tmp, "%d", total_nodes);
    setenv("bgp_totalnodes", tmp);

    sprintf(tmp, "%d", net->xcoord);
    setenv("bgp_xcoord", tmp);

    sprintf(tmp, "%d", net->ycoord);
    setenv("bgp_ycoord", tmp);

    sprintf(tmp, "%d", net->zcoord);
    setenv("bgp_zcoord", tmp);

    strncpy(tmp, eth->nfsmount, sizeof(tmp));
    tmp[sizeof(tmp)-1]=0;
    setenv("bgp_blockid", tmp);

    sprintf(tmp, "%d", (ccuci->Midplane * 16 + ccuci->NodeCard) &
	    ~(total_nodes / 32 - 1));
    setenv("bgp_blocknum", tmp);

#if 1
    /* Lower 3 bits of row | Lower 4 bits of col | Lower 1 bit of midplane */
    octet2 |= (ccuci->Midplane & 0x1);
    octet2 |= (ccuci->RackColumn & 0xF) << 1;
    octet2 |= (ccuci->RackRow & 0x7) << 5;

    /* default address generated from bluegene node info */
    if (ccuci->ComputeCard == 0 || ccuci->ComputeCard == 1) {
        sprintf(tmp, "10.%d.%d.%d", 
	        octet2, ccuci->NodeCard, ccuci->ComputeCard+1);
    } else {
        sprintf(tmp, "10.%d.%d.%d", 
                octet2, ccuci->NodeCard, ccuci->ComputeCard);
    }
#else 
    /****************** FROM RFC 1918 **************************************
     * RFC 1918 Address Allocation for Private Internets   February 1996 * 
     * 3. Private Address Space
     *
     * The Internet Assigned Numbers Authority (IANA) has reserved the
     * following three blocks of the IP address space for private internets:
     *
     * 10.0.0.0        -   10.255.255.255  (10/8 prefix)
     * 172.16.0.0      -   172.31.255.255  (172.16/12 prefix)
     * 192.168.0.0     -   192.168.255.255 (192.168/16 prefix)
     *
     * An enterprise that decides to use IP addresses out of the address
     * space defined in this document can do so without any coordination
     * with IANA or an Internet registry. The address space can thus be used
     * by many enterprises. Addresses within this private address space will
     * only be unique within the enterprise, or the set of enterprises which
     * choose to cooperate over this space so they may communicate with each
     * other in their own private internet.
     *
     * In order to use private address space, an enterprise needs to
     * determine which hosts do not need to have network layer connectivity
     * outside the enterprise in the foreseeable future and thus could be
     * classified as private. Such hosts will use the private address space
     * defined above.  Private hosts can communicate with all other hosts
     * inside the enterprise, both public and private. However, they cannot
     * have IP connectivity to any host outside of the enterprise. While not
     * having external (outside of the enterprise) IP connectivity private
     * hosts can still have access to external services via mediating
     * gateways (e.g., application layer gateways).
     * All other hosts will be public and will use globally unique address
     * space assigned by an Internet Registry. Public hosts can communicate
     * with other hosts inside the enterprise both public and private and
     * can have IP connectivity to public hosts outside the enterprise.
     * Public hosts do not have connectivity to private hosts of other
     * enterprises.
     *
     * Moving a host from private to public or vice versa involves a change
     * of IP address, changes to the appropriate DNS entries, and changes to
     * configuration files on other hosts that reference the host by IP
     * address.
     *
     * Because private addresses have no global meaning, routing information
     * about private networks shall not be propagated on inter-enterprise
     * links, and packets with private source or destination addresses
     * should not be forwarded across such links. Routers in networks not
     * using private address space, especially those of Internet service
     * providers, are expected to be configured to reject (filter out)
     * routing information about private networks. If such a router receives
     * such information the rejection shall not be treated as a routing
     * protocol error.
     *
     * Indirect references to such addresses should be contained within the
     * enterprise. Prominent examples of such references are DNS Resource
     * Records and other information referring to internal private
     * addresses. In particular, Internet service providers should take
     * measures to prevent such leakage.
     ******************* FROM RFC 1918 **************************************/
    {
        unsigned char octet3, octet2, octet1, octet0;
        
        if (!bg_uni_to_ipv4(uni, &octet3, &octet2, &octet1, &octet0, 
                             10, 0, 0, 0, 
                             10, 255, 255, 255)) {
            printf("ERROR: exceeded ip address space\n");
            while (1);
        }
        sprintf(tmp, "%d.%d.%d.%d", octet3, octet2, octet1, octet0);
    }
#endif
    
    setenv("ipaddr", tmp);

    if (net->ionoderank >= 1024) {
        if (net->rank < 128) {
            octet3 = ccuci->NodeCard;
        } else if (net->rank < 256) {
            octet3 = ccuci->NodeCard - 1;
        } else if (net->rank < 384) {
            octet3 = ccuci->NodeCard - 2;
        } else if (net->rank < 512) {
            octet3 = ccuci->NodeCard - 3;
        } else if (net->rank < 640) {
            octet3 = ccuci->NodeCard;
        } else if (net->rank < 768) {
            octet3 = ccuci->NodeCard - 1;
        } else if (net->rank < 896) {
            octet3 = ccuci->NodeCard - 2;
        } else if (net->rank < 1024) {
            octet3 = ccuci->NodeCard - 3;
        }
    } else if (net->ionoderank >= 512) {
        if (net->rank < 128) {
            octet3 = ccuci->NodeCard;
        } else if (net->rank < 256) {
            octet3 = ccuci->NodeCard - 1;
        } else if (net->rank < 384) {
            octet3 = ccuci->NodeCard - 2;
        } else if (net->rank < 512) {
            octet3 = ccuci->NodeCard - 3;
        }
    } else if (net->ionoderank >= 256) {
        if (net->rank < 64) {
            octet3 = ccuci->NodeCard;
        } else if (net->rank < 128) {
            octet3 = ccuci->NodeCard - 1;
        } else if (net->rank < 192) {
            octet3 = ccuci->NodeCard - 2;
        } else if (net->rank < 256) {
            octet3 = ccuci->NodeCard - 3;
        }
    } else if (net->ionoderank >= 128) {
        if (net->rank < 32) {
            octet3 = ccuci->NodeCard;
        } else if (net->rank < 64) {
            octet3 = ccuci->NodeCard - 1;
        } else if (net->rank < 96) {
            octet3 = ccuci->NodeCard - 2;
        } else if (net->rank < 128) {
            octet3 = ccuci->NodeCard - 3;
        }
    } else if (net->ionoderank >= 64) {
        if (net->rank < 32) {
            octet3 = ccuci->NodeCard;
        } else if (net->rank < 64) {
            octet3 = ccuci->NodeCard - 1;
        }
    } else if (net->ionoderank >= 32) {
        octet3 = ccuci->NodeCard;
    } else if (net->ionoderank >= 16) {
        switch (ccuci->ComputeCard) {
        case 4:
        case 5:
        case 8:
        case 9:
        case 12:
        case 13:
        case 16:
        case 17:
        case 22:
        case 23:
        case 26:
        case 27:
        case 30:
        case 31:
        case 34:
        case 35:
            octet4 = 1;
            break;
        default:
            octet4 = 2;
        } 
        octet3 = ccuci->NodeCard;
    } else {
        panic("Help!");
    }

    sprintf(tmp, "10.%d.%d.%d", octet2, octet3, octet4);

    setenv("gatewayip", tmp);
    
    setenv("netmask", "255.0.0.0");

    /* we set a dummy serverip address so that tftp commands do not barf */
    /* This implies that you either set it by hand correctly later in a script*/
    /* or you name the ip address explicity in the file name */
    setenv("serverip", "0.0.0.0");

#if 0

    printf("  ip: %s\n", getenv("ipaddr"));
    printf("  gatewayip: %s\n", getenv("gatewayip"));
    printf("  netmask: %s\n", getenv("netmask"));
    printf("  serverip: %s\n", getenv("serverip"));
#endif

} 

/************************************************************************
 *
 * This is the next part if the initialization sequence: we are now 
 * running from RAM and have a "normal" C environment, i. e. global 
 * data can be written, BSS has been cleared, the stack size in not 
 * that critical any more, etc. 
 * 
 ************************************************************************ 
 */ 

int board_env_init_r(void)
{
    /* setup some stuff that will be processed byt board_init_r */
    /* setting variables early ensures that boardinfo stuff that is calculated
       and set in board_early_init_r will be done right.  Like default
       ip address */
    bg_setenv_vars();
    return 1;
}

/******************************************************************** 
 * Misc initialization 
 ********************************************************************/ 

int misc_init_r (void)
{
    drv_bgtty_init();
    return 1;
}


/******************************************************************** 
 * Flattened Device tree functions 
 ********************************************************************/ 

#if defined(CONFIG_OF_FLAT_TREE) && defined(CONFIG_OF_BOARD_SETUP)
void
ft_board_setup (void *blob, bd_t *bd)
{
}
#endif

/********************************************************************
 * Global interrupt support based on spi/cnk code
 ********************************************************************/

inline void 
GlobInt_Disarm(u32 channel)
{
	mtdcrx(BGP_DCR_GLOBAL_INT_DRIVE_CH(channel), BGP_DCR_GLOBAL_INT_DRIVE_CLEAR);
}

inline void 
GlobInt_Fire(u32 channel)
{
	mtdcrx(BGP_DCR_GLOBAL_INT_DRIVE_CH(channel), BGP_DCR_GLOBAL_INT_DRIVE_SEND);
}

inline u32 
GlobInt_IsStickyRaised(u32 channel)
{
    // NOTE: Status bits 4-7 contain the stick bits for channels 0-3, respectively.
    return (mfdcrx(BGP_DCR_GLOBAL_INT_STATUS) & BN(4+channel)) ? 1 : 0;
}

inline u32 
GlobInt_IsRaised(u32 channel)
{
    // NOTE: Status bits 20-23 contain the downtree status for channels 0-3 respectively.
    return (mfdcrx(BGP_DCR_GLOBAL_INT_STATUS) & BN(20+channel)) ? 1 : 0;
}

inline void 
GlobInt_SetARMType(u32 channel, u32 armtype)
{
	mtdcrx(BGP_DCR_GLOBAL_INT_SET_CH(channel), armtype );
}

inline u32 
GlobInt_InitBarrier(u32 channel)
{
  /* Wait for idle state. */
  while(! GlobInt_IsRaised(channel))
    {
    }
  /* Fire global interrupt to show we are entering the barrier. */
  GlobInt_Disarm(channel);
  GlobInt_SetARMType(channel, BGP_DCR_GLOBAL_INT_SET_ARM_AND);
  GlobInt_Fire(channel);
  return 0;
}

inline u32 
GlobInt_QueryDone(u32 channel)
{
  if (GlobInt_IsStickyRaised(channel))
    return 1;
  else
    return 0;
}

inline u32 
GlobInt_Barrier(u32 channel)
{
  GlobInt_InitBarrier(channel);

  for (;;)
    {
      if(GlobInt_QueryDone(channel))
	break;
    }
  return 0;
}


inline u32 
global_int_all_barrier(void)
{
    return GlobInt_Barrier(BGP_GLOBAL_INT_CHANNEL_KERNEL_ALL_NODES_BARRIER);
}

/********************************************************************
 * final initialization stuff goes here
 ********************************************************************/

static int 
setfdt(struct fdt_header *fdt_ptr)
{
    int err;

    if (fdt_ptr == NULL) return 0;

    /* Set global default fdt pointer value */
    fdt = fdt_ptr;

    err = fdt_check_header(fdt);
    if (err == 0)
        return 1;	/* valid */
    
    if (err < 0) {
        printf("libfdt: %s", fdt_strerror(err));
        /*
         * Be more informative on bad version.
         */
        if (err == -FDT_ERR_BADVERSION) {
            if (fdt_version(fdt) < FDT_FIRST_SUPPORTED_VERSION) {
                printf (" - too old, fdt $d < %d",
                        fdt_version(fdt), FDT_FIRST_SUPPORTED_VERSION);
                fdt = NULL;
            }
            if (fdt_last_comp_version(fdt) > FDT_LAST_SUPPORTED_VERSION) {
                printf (" - too new, fdt $d > %d",
                        fdt_version(fdt), FDT_LAST_SUPPORTED_VERSION);
                fdt = NULL;
            }
            return 0;
        }
        printf("\n");
        return 0;
    }
    return 1;
}

//#define STALL_SYNC_TB_BAR 1
int 
syncTimeBases(u32 *beforeu, u32 *beforel, u32 *middleu, u32 *middlel, u32 *afteru, u32 *afterl)
{
    int tbl_val, tbu_val;
    u32 bu, bl, mu, ml, au, al;

#ifdef STALL_SYNC_TB_BAR
    bgp_personality_t *pers = (bgp_personality_t*)BGP_PERS_BASE;
    BGP_UCI_ComputeCard_t *ccuci;
    bgp_personality_network_t *net;

    net = &pers->net;
    ccuci=(BGP_UCI_ComputeCard_t *)&(pers->kernel.uci);
#endif

    tbl_val=0;
    tbu_val=0;

    asm volatile ("mftbu %0":"=r" (bu):);
    asm volatile ("mftbl %0":"=r" (bl):);

#if STALL_SYNC_TB_BAR
    if (net->rank != 1) global_int_all_barrier();
#else
    global_int_all_barrier();
#endif

    asm volatile ("mftbu %0":"=r" (mu):);
    asm volatile ("mftbl %0":"=r" (ml):);

    asm volatile ("mttbl %0" :: "r" (tbl_val) );
    asm volatile ("mttbu %0" :: "r" (tbu_val) );

    asm volatile ("mftbu %0":"=r" (au):);
    asm volatile ("mftbl %0":"=r" (al):);

    *beforeu = bu;
    *beforel = bl;
    *middleu  = mu;
    *middlel  = ml;
    *afteru  = au;
    *afterl  = al;

    return 1;
}

extern void * dt_header;
DECLARE_GLOBAL_DATA_PTR;

/* Called just before monitor starts up */
int last_stage_init(void)
{
    char tmp[20];
#if 0
    printf("Doing last stage of initialization...\n");
#endif


    bg_fdt_setup(&dt_header, gd->bd);

    if (!setfdt((struct fdt_header *)&dt_header)) {
        printf("ERROR: setfdt: failed something is wrong with the"
               " default builtin device tree at 0x%x\n", 
               (struct fdt_header *)&dt_header);
    }

    sprintf(tmp, "0x%x", fdt);
    setenv("fdtdefaultaddr", tmp);

    sprintf(tmp, "%d", fdt_totalsize(fdt));
    setenv("fdttotalsize", tmp);

#ifdef CONFIG_BGP_SYNCTBASES
    {
        u32 beforeu, beforel, middleu, middlel, afteru, afterl;
        syncTimeBases(&beforeu, &beforel, &middleu, &middlel, &afteru, &afterl);
        mailbox_unsilence();
        printf("syncTimeBases: beforeu=%x:%x, middle=%x:%x after=%x:%x\n", beforeu, beforel, 
               middleu, middlel, afteru, afterl);
        mailbox_silence();
    }
#endif

    // for the moment we are disabling the use of the global barrier/interrupt network as the
    // default state
    global_int_disable_all();

    return 0;
}

void release_app_processors(void (*kernel)(void))
{
    //printf("Releasing application processors to %p\n", kernel);
    //printf("CPU entry ptr: %p %p\n", &cpu_entry, cpu_entry);

    cpu_entry = kernel;
    invalidate_dcache_line(&cpu_entry);
}

/********************************************************************
 * helper functions for lwip and lwip applications
 ********************************************************************/

char *
uboot_get_boot_load_addr(void) 
{
    char *dummy;
    char *addr = (char *)simple_strtoul(getenv("loadaddr"), &dummy, 0);
    return (addr + sizeof(struct image_header));
}

char *
uboot_get_diskimg_load_addr(void) 
{
    char *dummy;
    char *addr = (char *)simple_strtoul(getenv("ramfsaddr"), &dummy, 0);
    return (addr + sizeof(struct image_header));
}

char *
uboot_get_fdt_addr(void)
{
    char *dummy;
    char *addr = (char *)simple_strtoul(getenv("fdtbootaddr"), &dummy, 0);
    return (addr);
}

struct image_header *
uboot_mk_bgp_image_header(void  *hdr_addr, void *data, int size, int type, 
    int compression)
{
    struct image_header *hdr = (struct image_header *)hdr_addr;
    ulong checksum;
#if 0
    checksum = crc32 (0, ptr, size);
#else
    checksum = 0;
#endif
    memset(hdr, 0, sizeof(struct image_header));
    
    /* Build new header */
    hdr->ih_magic = htonl(IH_MAGIC);
    hdr->ih_time  = 0;
    hdr->ih_size  = htonl(size);
    hdr->ih_load  = 0;
    hdr->ih_ep    = 0;
    hdr->ih_dcrc  = htonl(checksum);
    hdr->ih_os    = IH_OS_LINUX;
    hdr->ih_arch  = IH_CPU_PPC;
    hdr->ih_type  = type;
    hdr->ih_comp  = compression;
    
    strncpy((char *)hdr->ih_name, "(none)", IH_NMLEN);
    
    checksum = crc32(0,(const unsigned char *)hdr,sizeof(struct image_header));
    
    hdr->ih_hcrc = htonl(checksum);
    
    return hdr;
}

static int
uboot_validate_uimg_hdr(struct image_header *hdr)
{
    struct image_header tmp;
    ulong newchecksum, oldchecksum;

    if (ntohl(hdr->ih_magic) == IH_MAGIC) {
        /* looks like we were passed a uimage */
        /* Copy header so we can blank CRC field for re-calculation */
        memmove (&tmp, (char *)hdr, sizeof(image_header_t));
        oldchecksum = ntohl(tmp.ih_hcrc);
        tmp.ih_hcrc = 0;
        newchecksum = crc32 (0, (uchar *)&tmp, sizeof(image_header_t));
        if ( newchecksum != oldchecksum) {
            printf("uboot_validate_uimg_hdr: Not a valid uimage bad header"
                   " crc %d != %d\n",
                   newchecksum, oldchecksum);
            return 0;
        }
        return 1;
    } 
    return 0;
}

static int
uboot_validate_uimg_data(struct image_header *hdr)
{
    ulong newchecksum, oldchecksum, len;
    char *s;

    s = getenv ("verify");
    if (s && (*s == 'n')) return 1;

    oldchecksum = ntohl(hdr->ih_dcrc);
    len = ntohl(hdr->ih_size);
    newchecksum = crc32(0, ((uchar *)hdr) + sizeof(image_header_t), len);

    if ( newchecksum != oldchecksum) {
        printf("uboot_validate_uimg_data: Not a valid uimage bad data"
               " crc %d != %d\n",
               newchecksum, oldchecksum);
        return 0;
    }
    return 1;
}

int 
uboot_validate_gzip_header(unsigned char *bytes)
{
    return (bytes[0] == 0x1F && bytes[1] == 0x8B && bytes[2] == 0x08);
} 

int 
uboot_validate_kernel_image(int kernlen, char *addr)
{
    struct image_header *hdr;
    unsigned char *bytes;

    if (addr) {
        hdr = (struct image_header *) addr;
    } else { 
        hdr = (struct image_header *) uboot_get_boot_load_addr();
    }
    if (kernlen) {
        if (uboot_validate_uimg_hdr(hdr)) {
            /* Validate that this is a kernel uimage */
            if (ntohl(hdr->ih_arch) != IH_CPU_PPC &&
                ntohl(hdr->ih_type) != IH_TYPE_KERNEL) {
                printf("uboot_validate_kern_image: Incorrect image"
                       " arch %d or type %d.\n",
                       hdr->ih_arch, hdr->ih_type);
		return 0;                
            }
            if (!uboot_validate_uimg_data(hdr)) {
                return 0;
            }
            return 1;
        } else {
            /* If we can verify that we got a Linux ppc gzipped kernel. If so */
            /* add a uimage header just before it  (we known there is space */
            /* since we accounted for this possiblity in 
               uboot_get_boot_load_addr) */
            if (addr == NULL) {
                bytes = (uchar *)hdr;
                if (uboot_validate_gzip_header(bytes)) {
                    printf("boot image looks like a gzip archive"
                           " creating ppc linux gzip uimage header\n");
                    uboot_mk_bgp_image_header((void *)(hdr - 1), hdr,
                                              kernlen, 
                                              IH_TYPE_KERNEL, IH_COMP_GZIP);
                    return 1;
                } else {
                    printf("uboot_validate_kern_image: Not a valid uimage :"
                           " bad magic\n");
                }
            }
        }
    }
    return 0;
}

int
uboot_validate_diskimg(int diskimglen, char *addr)
{
    struct image_header *hdr;
    unsigned char *bytes;

    if (addr) {
        hdr = (struct image_header *) addr;
    } else {
        hdr = (struct image_header *) uboot_get_diskimg_load_addr();
    }
    if (diskimglen) {
        if (uboot_validate_uimg_hdr(hdr)) {
            /* Validate that this is a ramdisk uimage */
            if (ntohl(hdr->ih_arch) != IH_CPU_PPC &&
                ntohl(hdr->ih_type) != IH_TYPE_RAMDISK) {
                printf("uboot_validate_diskimg: Incorrect image"
                       " arch %d or type %d.\n",
                       hdr->ih_arch, hdr->ih_type);
		return 0;                
            }
            if (!uboot_validate_uimg_data(hdr)) {
                return 0;
            }
            return 1;
        } else {
            /* If we can verify that we got a Linux ppc gzipped ramdisk. If so */
            /* add a uimage header just before it  (we known there is space */
            /* since we accounted for this possiblity in 
               uboot_get_boot_load_addr) */
            if (addr == NULL) {
                bytes = (uchar *)hdr;
                /* see if it is a gzip archive */
                if (uboot_validate_gzip_header(bytes)) {
                    printf("disk image looks like a gzip archive"
                           " creating ppc linux ramdisk gzip uimage header\n");
                    uboot_mk_bgp_image_header((void *)(hdr - 1), hdr,
                                              diskimglen, 
                                              IH_TYPE_RAMDISK, IH_COMP_GZIP);
                    return 1;
                } else {
                    printf("uboot_validate_disimg: Unable to identify as a"
                           "valid image.\n");
                }
            }
        }
    }
    return 0;
}

int
uboot_invoke_boot_cmd(int kernlen, 
                      int diskimglen,
                      char *cmdline, int cmdlinelen)
{
    cmd_tbl_t *cmdpt = NULL;
    int argc, i;
    char *args[4];
    char cmdstr[80];
    char loadaddrstr[80];
    char ramdiskaddrstr[80];
    char fdtaddrstr[80];
    char *kernuimagestart, *diskuimagestart, *fdtaddr;

    printf("uboot_invoke_boot_cmd: kern=%p kernlen=%p diskimg=%p "
           "diskimglen=%d cmdline=%p cmdlinelen=%d\n", 
           uboot_get_boot_load_addr(), kernlen,
           uboot_get_diskimg_load_addr(), diskimglen, 
           cmdline, cmdlinelen);    

    memset(cmdstr, 0, 80);
    strncpy(cmdstr, "bootm", 5);
    argc=0;
    args[argc] = cmdstr;

    fdtaddr = uboot_get_fdt_addr();
    kernuimagestart = uboot_get_boot_load_addr();
    /* check to see if a uimage was loaded for the kernel */
    if (!uboot_validate_kernel_image(kernlen, kernuimagestart)) {
        /* no uimage was loaded but we might have been able to create */
        /* a header if so it will preceed the image loaded */
        kernuimagestart = kernuimagestart - (sizeof(struct image_header));
        /* we force verify behaviour off as we do generate a checksum if */
        /* we have manually created a header  ... */
        setenv("verify", "no");
        if (!uboot_validate_kernel_image(kernlen, kernuimagestart)) {
            printf("ERROR: kern not valid boot image\n");
            return 0;
        }
    }
    memset(loadaddrstr, 0, 80);
    sprintf(loadaddrstr, "0x%x", kernuimagestart); 
    argc++;
    args[argc] = loadaddrstr;

    memset(ramdiskaddrstr, 0, 80);
    if (diskimglen) {
        diskuimagestart = uboot_get_diskimg_load_addr();
        /* check to see if a uimage was loaded */
        if (!uboot_validate_diskimg(diskimglen, diskuimagestart)) {
            /* no uimage was loaded but we might have been able to create */
            /* a header if so it will preceed the image loaded */
            diskuimagestart = diskuimagestart - (sizeof(struct image_header));
            /* we force verify behaviour off as we do generate a checksum if */
            /* we have manually created a header  ... */
            setenv("verify", "no");
            if (!uboot_validate_diskimg(diskimglen, diskuimagestart)) {
                printf("ERROR: kern not valid disk image\n");
                return 0;
            }
        } 
        sprintf(ramdiskaddrstr, "0x%x", diskuimagestart);
        argc++;
        args[argc] = ramdiskaddrstr;
        
    } else {
        if (fdtaddr) {
            sprintf(ramdiskaddrstr, "%s", "-");
            argc++;
            args[argc] = ramdiskaddrstr;
        }
    }
        
    if (fdtaddr) {
        sprintf(fdtaddrstr, "0x%x", fdtaddr);
        argc++;
        args[argc] = fdtaddrstr;
    }
    
    argc++;
    for (i=0; i<argc; i++) {
        printf("arg[%d]=%s\n", i, args[i]);
    }

    cmdpt = find_cmd("bootm");

    if (cmdpt) {
        cmdpt->cmd(cmdpt, 0, argc, args);
    } else {
        printf("ERROR: was not able to invoke uboot bootm command\n");
        return 0;
    }
    /* should not get here */
    printf("ERROR: yikes got here!\n");
    while (1);
    return 1;
}

void
uboot_set_boot_cmdline(char *cmdline)
{
    setenv("bootargs", cmdline);
}

char *
uboot_get_boot_cmdline(void)
{
    return getenv("bootargs");
}

unsigned long 
uboot_str_to_ip_ulong(char *s)
{
    return string_to_ip(s);
}


