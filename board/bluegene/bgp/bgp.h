/*********************************************************************
 *                
 * Copyright (C) 2007-2008, Volkmar Uhlig, IBM Corporation
 *                
 * Description:   BG-specific defines
 *                
 * All rights reserved
 *                
 ********************************************************************/
#ifndef __BOARD__BLUEGENE__BGP__BGP_H__
#define __BOARD__BLUEGENE__BGP__BGP_H__

/*
 *  Bluegene UCI defintions
 */
#define _BGP_UCI_Component_Rack              ( 0)
#define _BGP_UCI_Component_Midplane          ( 1)
#define _BGP_UCI_Component_BulkPowerSupply   ( 2)
#define _BGP_UCI_Component_PowerCable        ( 3)
#define _BGP_UCI_Component_PowerModule       ( 4)
#define _BGP_UCI_Component_ClockCard         ( 5)
#define _BGP_UCI_Component_FanAssembly       ( 6)
#define _BGP_UCI_Component_Fan               ( 7)
#define _BGP_UCI_Component_ServiceCard       ( 8)
#define _BGP_UCI_Component_LinkCard          ( 9)
#define _BGP_UCI_Component_LinkChip          (10)
#define _BGP_UCI_Component_LinkPort          (11)  // Identifies 1 end of a LinkCable
#define _BGP_UCI_Component_NodeCard          (12)
#define _BGP_UCI_Component_ComputeCard       (13)
#define _BGP_UCI_Component_IOCard            (14)
#define _BGP_UCI_Component_DDRChip           (15)
#define _BGP_UCI_Component_ENetConnector     (16)

typedef struct _BGP_UCI_ComputeCard_t
{                           // "Rxy-Mm-Nnn-Jxx": R<RackRow><RackColumn>-M<Midplane>-N<NodeCard>-J<ComputeCard>
    unsigned int Component   :  5;  // when _BGP_UCI_Component_ComputeCard
    unsigned int RackRow     :  4;  // 0..F
    unsigned int RackColumn  :  4;  // 0..F
    unsigned int Midplane    :  1;  // 0=Bottom, 1=Top
    unsigned int NodeCard    :  4;  // 00..15: 00=BF, 07=TF, 08=BR, 15=TR)
    unsigned int ComputeCard :  6;  // 04..35 (00-01 IOCard, 02-03 Reserved, 04-35 ComputeCard)
    unsigned int _zero       :  8;  // zero's
} BGP_UCI_ComputeCard_t;

/*
 * tree specific defines
 */
#define BGP_TREE_BASE		(0x610000000ULL)
#define BGP_TREE_OFFSET		(0x001000000ULL)

#define TREE_FIFO_SIZE		8
#define TREE_PAYLOAD		256
#define TREE_DCR_BASE		(0xc00)
#define TREE_DCR(x)		(TREE_DCR_BASE + (x))

#define BGP_DCR_TR_GLOB_NADDR   TREE_DCR(0x41)
#define BGP_DCR_TR_GLOB_VCFG0   TREE_DCR(0x42)
#define BGP_DCR_TR_GLOB_VCFG1   TREE_DCR(0x43)
#define BGP_DCR_TR_REC_PRXF     TREE_DCR(0x44)
#define BGP_DCR_TR_REC_PRXEN    TREE_DCR(0x45)
#define BGP_DCR_TR_INJ_PIXF     TREE_DCR(0x48)
#define BGP_DCR_TR_INJ_PIXEN    TREE_DCR(0x49)

// tree ifc memory offsets
#define BGP_TRx_DI		(0x00)
#define BGP_TRx_HI		(0x10)
#define BGP_TRx_DR		(0x20)
#define BGP_TRx_HR		(0x30)
#define BGP_TRx_Sx		(0x40)

#define TR_IRQ_PENDING_MASK(idx) (1U << (1 - idx))
#define TR_GLOB_VCFG_RWM(x)	 ((x) << (31-23))
#define TR_REC_PRX_WM0           (1 << 1)
#define TR_REC_PRX_WM1           (1 << 0)
#define TREE_INJ_PIX_ENABLE      BN(31)

#define TR_INJ_PIX_APAR0  BN( 6)  /* P0 address parity error             */
#define TR_INJ_PIX_APAR1  BN( 7)  /* P1 address parity error             */
#define TR_INJ_PIX_ALIGN0 BN( 8)  /* P0 address alignment error          */
#define TR_INJ_PIX_ALIGN1 BN( 9)  /* P1 address alignment error          */
#define TR_INJ_PIX_ADDR0  BN(10)  /* P0 bad (unrecognized) address error */
#define TR_INJ_PIX_ADDR1  BN(11)  /* P1 bad (unrecognized) address error */
#define TR_INJ_PIX_DPAR0  BN(12)  /* P0 data parity error                */
#define TR_INJ_PIX_DPAR1  BN(13)  /* P1 data parity error                */
#define TR_INJ_PIX_COLL   BN(14)  /* FIFO write collision error          */
#define TR_INJ_PIX_UE     BN(15)  /* Uncorrectable SRAM ECC error        */
#define TR_INJ_PIX_PFO0   BN(25)  /* VC0 payload FIFO overflow error     */
#define TR_INJ_PIX_PFO1   BN(26)  /* VC1 payload FIFO overflow error     */
#define TR_INJ_PIX_HFO0   BN(27)  /* VC0 header FIFO overflow error      */
#define TR_INJ_PIX_HFO1   BN(28)  /* VC1 header FIFO overflow error      */

#define TR_REC_PRX_APAR0  BN( 8)  /* P0 address parity error             */
#define TR_REC_PRX_APAR1  BN( 9)  /* P1 address parity error             */
#define TR_REC_PRX_ALIGN0 BN(10)  /* P0 address alignment error          */
#define TR_REC_PRX_ALIGN1 BN(11)  /* P1 address alignment error          */
#define TR_REC_PRX_ADDR0  BN(12)  /* P0 bad (unrecognized) address error */
#define TR_REC_PRX_ADDR1  BN(13)  /* P1 bad (unrecognized) address error */
#define TR_REC_PRX_COLL   BN(14)  /* FIFO read collision error           */
#define TR_REC_PRX_UE     BN(15)  /* Uncorrectable SRAM ECC error        */
#define TR_REC_PRX_PFU0   BN(26)  /* VC0 payload FIFO under-run error    */
#define TR_REC_PRX_PFU1   BN(27)  /* VC1 payload FIFO under-run error    */
#define TR_REC_PRX_HFU0   BN(28)  /* VC0 header FIFO under-run error     */
#define TR_REC_PRX_HFU1   BN(29)  /* VC1 header FIFO under-run error     */

#define TR_INJ_PIX_ALL_BUT_WM (TR_INJ_PIX_APAR0  | TR_INJ_PIX_APAR1  | \
                               TR_INJ_PIX_ALIGN0 | TR_INJ_PIX_ALIGN1 | \
                               TR_INJ_PIX_ADDR0  | TR_INJ_PIX_ADDR1  | \
                               TR_INJ_PIX_DPAR0  | TR_INJ_PIX_DPAR1  | \
                               TR_INJ_PIX_COLL   | TR_INJ_PIX_UE     | \
                               TR_INJ_PIX_PFO0   | TR_INJ_PIX_PFO1   | \
                               TR_INJ_PIX_HFO0   | TR_INJ_PIX_HFO1)

#define TR_REC_PRX_ALL_BUT_WM (TR_REC_PRX_APAR0  | TR_REC_PRX_APAR1  | \
                               TR_REC_PRX_ALIGN0 | TR_REC_PRX_ALIGN1 | \
                               TR_REC_PRX_ADDR0  | TR_REC_PRX_ADDR1  | \
                               TR_REC_PRX_COLL   | TR_REC_PRX_UE     | \
                               TR_REC_PRX_PFU0   | TR_REC_PRX_PFU1   | \
                               TR_REC_PRX_HFU0   | TR_REC_PRX_HFU1)

/*
 * physical memory layout
 */
#define BGP_IRQCTRL_PHYS	0x730000000ULL
#define BGP_TREE_CHN0_PHYS	(BGP_TREE_BASE + 0)
#define BGP_TREE_CHN1_PHYS	(BGP_TREE_BASE + BGP_TREE_OFFSET)
#define BGP_TREE_RDR_NUM        (16)
#define BGP_TREE_RDR_SRCL       (0x0002)
#define BGP_TREE_RDR_TGTL       (0x0001)

/* Mask of all valid source and target bits.  */
#define BGP_TREE_RDR_ACCEPT     (0x7703)

#define BGP_PERS_BASE		0xfffff800U

#define BN(b)   ((1<<(31-(b))))
#define B2(b,x) (((x)&0x3)<<(31-(b)))
#define B3(b,x) (((x)&0x7)<<(31-(b)))
#define B4(b,x) (((x)&0xF)<<(31-(b)))
#define B5(b,x) (((x)&0x1F)<<(31-(b)))
#define B8(b,x) (((x)&0xFF)<<(31-(b)))

/* Personality constants.  */
#define BGP_PERS_EN_GLOBAL_INTS (1<<(31-5))
#define BGP_PERS_EN_DMA         (1<<(31-14))
#define BGP_PERS_EN_TREEBLAST   (1<<(31-25))
#define BGP_PERS_EN_TORUS       (1<<(31-7))
#define BGP_PERS_EN_COLLECTIVE  (1<<(31-6))
#define BGP_PERS_EN_ETHERNET    BN(18)
#define BGP_PERS_EN_LOOPBACK    BN(4)
#define BGP_PERS_EN_SERDES      BN(15)
#define BGP_PERS_ENABLE_TREEA   BN(11)
#define BGP_PERS_ENABLE_TREEB   BN(12)
#define BGP_PERS_ENABLE_TREEC   BN(13)

/* Global interrupts offsets.  */
#define BGP_DCR_GLOBAL_INT             (0x660)
#define BGP_DCR_GLOBAL_INT_STATUS         (BGP_DCR_GLOBAL_INT + 0x000) 
#define BGP_DCR_GLOBAL_INT_USER_EN     (BGP_DCR_GLOBAL_INT + 0x006)
#define BGP_DCR_GLOBAL_INT_USER_EN_CH0 (1<<(31-28))

#define BGP_DCR_GLOBAL_INT_ASSERT_CH(ch)      (BGP_DCR_GLOBAL_INT + (ch))    
#define BGP_DCR_GLOBAL_INT_ASSERT_CH0         (BGP_DCR_GLOBAL_INT + 0x000)  
#define BGP_DCR_GLOBAL_INT_ASSERT_CH1         (BGP_DCR_GLOBAL_INT + 0x001)  
#define BGP_DCR_GLOBAL_INT_ASSERT_CH2         (BGP_DCR_GLOBAL_INT + 0x002)   

#define _BGP_DCR_GLOBINT_ASSERT_CH3         (_BGP_DCR_GLOBINT + 0x003)   // Write-Only

#define BGP_DCR_GLOBAL_INT_DRIVE_CH(ch)  (BGP_DCR_GLOBAL_INT_DRIVE_CH0 + (ch)) 
#define BGP_DCR_GLOBAL_INT_DRIVE_CH0     (BGP_DCR_GLOBAL_INT + 0x008)   
#define BGP_DCR_GLOBAL_INT_DRIVE_CH1     (BGP_DCR_GLOBAL_INT + 0x009) 
#define BGP_DCR_GLOBAL_INT_DRIVE_CH2     (BGP_DCR_GLOBAL_INT + 0x00A) 
#define BGP_DCR_GLOBAL_INT_DRIVE_CH3     (BGP_DCR_GLOBAL_INT + 0x00B)  
#define BGP_DCR_GLOBAL_INT_DRIVE_SEND    (1) 
#define BGP_DCR_GLOBAL_INT_DRIVE_CLEAR   (0)
#define BGP_DCR_GLOBAL_INT_SET_CH(ch)    (BGP_DCR_GLOBAL_INT_SET_CH0 + (ch)) 
#define BGP_DCR_GLOBAL_INT_SET_CH0       (BGP_DCR_GLOBAL_INT + 0x00C)
#define BGP_DCR_GLOBAL_INT_SET_CH1       (BGP_DCR_GLOBAL_INT + 0x00D)
#define BGP_DCR_GLOBAL_INT_SET_CH2       (BGP_DCR_GLOBAL_INT + 0x00E)
#define BGP_DCR_GLOBAL_INT_SET_CH3       (BGP_DCR_GLOBAL_INT + 0x00F) 
#define BGP_DCR_GLOBAL_INT_SET_ARM_OR    (0)
#define BGP_DCR_GLOBAL_INT_SET_ARM_AND   (1) 
#define BGP_GLOBAL_INT_CHANNEL_APP_BARRIER                  (0) 
#define BGP_GLOBAL_INT_CHANNEL_KERNEL_THROTTLE              (1) 
#define BGP_GLOBAL_INT_CHANNEL_KERNEL_COMPUTE_NODES_BARRIER (2) 
#define BGP_GLOBAL_INT_CHANNEL_KERNEL_ALL_NODES_BARRIER     (3) 

/* Netbus offsets.  */
#define BGP_DCR_NETBUS_TREE_BLAST    (1<<(31-27))
#define BGP_DCR_NETBUS_TD_WRITE_EN   (1<<(31-28))
#define BGP_DCR_NETBUS_TD_READ_EN    (1<<(31-29))
#define BGP_DCR_NETBUS_TREE_WRITE_EN (1<<(31-30))
#define BGP_DCR_NETBUS_TREE_READ_EN  (1<<(31-31))

/* Default number of tokens when in DMA mode.  */
#define BGP_DCR_NETBUS_DMA_WRITE_TOKENS_DEFAULT 8
#define BGP_DCR_NETBUS_DMA_READ_TOKENS_DEFAULT  8
/* Default number of tokens when in Torus mode (reset default).  */
#define BGP_DCR_NETBUS_TORUS_WRITE_TOKENS_DEFAULT 3
#define BGP_DCR_NETBUS_TORUS_READ_TOKENS_DEFAULT  3
/* Default number of tokens for Tree (reset default).  */
#define BGP_DCR_NETBUS_TREE_WRITE_TOKENS_DEFAULT 3
#define BGP_DCR_NETBUS_TREE_READ_TOKENS_DEFAULT  3

#define BGP_DCR_NETBUS_TD_WRITE_TOKENS(x)   B4(19,(x))
#define BGP_DCR_NETBUS_TD_READ_TOKENS(x)    B4(23,(x))
#define BGP_DCR_NETBUS_TREE_WRITE_TOKENS(x) B4(27,(x))
#define BGP_DCR_NETBUS_TREE_READ_TOKENS(x)  B4(31,(x))

#define BGP_DCR_NETBUS (0x630)

#define BGP_DCR_NETBUS_DP0_LOAD_TOKENS (BGP_DCR_NETBUS + 0x00)
#define BGP_DCR_NETBUS_DP0_ENABLE      (BGP_DCR_NETBUS + 0x01)
#define BGP_DCR_NETBUS_DP1_LOAD_TOKENS (BGP_DCR_NETBUS + 0x08)
#define BGP_DCR_NETBUS_DP1_ENABLE      (BGP_DCR_NETBUS + 0x09)

#define BGP_DCR_TREE                  (0xc00)
#define BGP_DCR_TREE_GLOBAL           (BGP_DCR_TREE + 0x40)
#define BGP_DCR_TREE_GLOBAL_NODE_ADDR (BGP_DCR_TREE_GLOBAL + 0x01)

#define BGP_DCR_TREE_INJ       (BGP_DCR_TREE + 0x48)
#define BGP_DCR_TREE_INJ_CSPY0 (BGP_DCR_TREE_INJ + 0x04)
#define BGP_DCR_TREE_INJ_CSHD0 (BGP_DCR_TREE_INJ + 0x05)
#define BGP_DCR_TREE_INJ_CSPY1 (BGP_DCR_TREE_INJ + 0x06)
#define BGP_DCR_TREE_INJ_CSHD1 (BGP_DCR_TREE_INJ + 0x07)
#define BGP_DCR_TREE_INJ_PIXEN (BGP_DCR_TREE_INJ + 0x01)

#define BGP_DCR_TREE_REC       (BGP_DCR_TREE + 0x44)
#define BGP_DCR_TREE_REC_PRXEN (BGP_DCR_TREE_REC + 0x01)

#define BGP_DCR_TREE_CLASS      (BGP_DCR_TREE + 0x00)
#define BGP_DCR_TREE_CLASS_ISRA (BGP_DCR_TREE_CLASS + 0x08)
#define BGP_DCR_TREE_CLASS_ISRB (BGP_DCR_TREE_CLASS + 0x09)

#define BGP_DCR_TREE_ARB      (BGP_DCR_TREE + 0x10)
#define BGP_DCR_TREE_ARB_RTO  (BGP_DCR_TREE_ARB + 0x01)
#define BGP_DCR_TREE_ARB_RCFG (BGP_DCR_TREE_ARB + 0x00)

#define TREE_ARB_RCFG_LB2 BN(25)
#define TREE_ARB_RCFG_LB1 BN(26)
#define TREE_ARB_RCFG_LB0 BN(27)

#define SERDES_HSS_CREG12_X4_HSSCALDRV(x) B3( 2,x)
#define SERDES_HSS_CREG12_X4_HSSDIVSEL(x) B2( 4,x)

#define SERDES_HSS_CREG12_X4_HSSRECCAL BN(8)
#define SERDES_HSS_CREG12_X4_HSSREFSEL BN(9)
#define SERDES_HSS_CREG12_X4_RG        BN(14)

#define SERDES_HSS_CREG14_X8_HSSRESET  BN(0)
#define SERDES_HSS_CREG14_X4_HSSRESET  BN(1)
#define SERDES_HSS_CREG14_HSSRESET_CAL BN(2)

#define SERDES_HSS_TX_REG0_TX_AMPL(x)     B4(3,x)
#define SERDES_HSS_TX_REG0_TX_COEF(x)     B4(7,x)
#define SERDES_HSS_TX_REG0_TX_SLEW(x)     B2(9,x)
#define SERDES_HSS_TX_REG0_TX_JTAGAMPL(x) B4(13,x)
#define SERDES_HSS_TX_REG0_TX_TS          BN(19)

#define BGP_DCR_SERDES_HSS_SREG52 (BGP_DCR_SERDES_HSS + 0x34)

#define SERDES_HSS_SREG52_X8_HSSPLLLOCK BN(8)
#define SERDES_HSS_SREG52_X8_HSSREADY   BN(9)
#define SERDES_HSS_SREG52_X8_CALCOMP    BN(16)

#define SERDES_TS_SEND_CTL_REG0_SD_TS_TRAINCLKS(x)  B8(7,x)
#define SERDES_TS_SEND_CTL_REG0_SD_TS_RAMDOMCLKS(x) B8(15,x)

#define SERDES_TR_SEND_CTL_REG0_TRAINCLKS(x)  B8(7,x)
#define SERDES_TR_SEND_CTL_REG0_RANDOMCLKS(x) B8(15,x)
#define SERDES_TR_SEND_CTL_REG0_TRAIN_A       BN(24)
#define SERDES_TR_SEND_CTL_REG0_TRAIN_B       BN(25)
#define SERDES_TR_SEND_CTL_REG0_TRAIN_C       BN(26)

#define SERDES_TR_SEND_CTL_REG2_EB_LAT_A0(x) B3(3,x)
#define SERDES_TR_SEND_CTL_REG2_EB_LAT_A1(x) B3(11,x)
#define SERDES_TR_SEND_CTL_REG2_EB_RUN_A0    BN(0)
#define SERDES_TR_SEND_CTL_REG2_EB_RUN_A1    BN(8)

#define SERDES_TR_SEND_CTL_REG2_EB_LAT_B0(x) B3(19,x)
#define SERDES_TR_SEND_CTL_REG2_EB_LAT_B1(x) B3(27,x)
#define SERDES_TR_SEND_CTL_REG2_EB_RUN_B0    BN(16)
#define SERDES_TR_SEND_CTL_REG2_EB_RUN_B1    BN(24)

#define SERDES_TR_SEND_CTL_REG3_EB_LAT_C0(x) B3(3,x)
#define SERDES_TR_SEND_CTL_REG3_EB_LAT_C1(x) B3(11,x)
#define SERDES_TR_SEND_CTL_REG3_EB_RUN_C0    BN(0)
#define SERDES_TR_SEND_CTL_REG3_EB_RUN_C1    BN(8)

#define SERDES_TR_SEND_CTL_REG3_SEND_NEW_PATTERN BN(16)
#define SERDES_TR_SEND_CTL_REG3_SCRAMBLE         BN(17)

#define BGP_DCR_SERDES (0x200)

#define BGP_DCR_SERDES_TR_CTRL (BGP_DCR_SERDES + 0x01A0)

#define BGP_DCR_SERDES_TR_STAT_BYTE_FOUND (BGP_DCR_SERDES_TR_CTRL + 0x0d)
#define BGP_DCR_SERDES_TR_CTRL_SEND_REG0 (BGP_DCR_SERDES_TR_CTRL + 0x10)
#define BGP_DCR_SERDES_TR_CTRL_SEND_REG2 (BGP_DCR_SERDES_TR_CTRL + 0x12)
#define BGP_DCR_SERDES_TR_CTRL_SEND_REG3 (BGP_DCR_SERDES_TR_CTRL + 0x13)

#define BGP_DCR_SERDES_HSS (BGP_DCR_SERDES + 0x01C0)

#define BGP_DCR_SERDES_HSS_CREG12 (BGP_DCR_SERDES_HSS + 0x0C)
#define BGP_DCR_SERDES_HSS_CREG13 (BGP_DCR_SERDES_HSS + 0x0D)
#define BGP_DCR_SERDES_HSS_CREG14 (BGP_DCR_SERDES_HSS + 0x0E)

#define BGP_DCR_SERDES_A0 (BGP_DCR_SERDES + 0x00C0)
#define BGP_DCR_SERDES_A1 (BGP_DCR_SERDES + 0x00E0)
#define BGP_DCR_SERDES_B0 (BGP_DCR_SERDES + 0x0100)
#define BGP_DCR_SERDES_B1 (BGP_DCR_SERDES + 0x0120)
#define BGP_DCR_SERDES_C0 (BGP_DCR_SERDES + 0x0140)
#define BGP_DCR_SERDES_C1 (BGP_DCR_SERDES + 0x0160)

#define BGP_DCR_SERDES_HSS_TX_REG0_OFFSET (0x002)
#define BGP_DCR_SERDES_HSS_TX_REG1_OFFSET (0x003)
#define BGP_DCR_SERDES_HSS_RX_REG0_OFFSET (0x000)
#define BGP_DCR_SERDES_HSS_RX_REG1_OFFSET (0x001)

#if 0
#define BGP_SerDes_Train_Eth _BN(19)
#define BGP_SerDes_Train_XP  _BN(20)
#define BGP_SerDes_Train_XM  _BN(21)
#define BGP_SerDes_Train_YP  _BN(22)
#define BGP_SerDes_Train_YM  _BN(23)
#define BGP_SerDes_Train_ZP  _BN(24)
#define BGP_SerDes_Train_ZM  _BN(25)
#define BGP_SerDes_Train_A0  _BN(26)
#define BGP_SerDes_Train_A1  _BN(27)
#define BGP_SerDes_Train_B0  _BN(28)
#define BGP_SerDes_Train_B1  _BN(29)
#define BGP_SerDes_Train_C0  _BN(30)
#define BGP_SerDes_Train_C1  _BN(31)

#define BGP_SerDes_Train_X  (BGP_SerDes_Train_XP | BGP_SerDes_Train_XM)
#define BGP_SerDes_Train_Y  (BGP_SerDes_Train_YP | BGP_SerDes_Train_YM)
#define BGP_SerDes_Train_Z  (BGP_SerDes_Train_ZP | BGP_SerDes_Train_ZM)

#define BGP_SerDes_Train_A  (BGP_SerDes_Train_A0 | BGP_SerDes_Train_A1)
#define BGP_SerDes_Train_B  (BGP_SerDes_Train_B0 | BGP_SerDes_Train_B1)
#define BGP_SerDes_Train_C  (BGP_SerDes_Train_C0 | BGP_SerDes_Train_C1)
#endif

#define BGP_DCR_SERDES_TR_CTL (BGP_DCR_SERDES + 0x01A0)

#define SERDES_TR_CTL_RVC_ELASTIC_BUF_LAT(x) B3(23,x)
#define SERDES_TR_CTL_RVC_ELASTIC_BUF_RUN   BN(20)

#define SERDES_TS_CTL_RVC_TRAIN BN(17)
#define SERDES_TR_CTL_RVC_TRAIN BN(17)

#define BGP_MEM_DEVBUS_TECR_OFFSET     (0x08)
#define BGP_DEVBUS_TECR_XMIT_BUFSZ(x)  B4( 3,x)
#define BGP_DEVBUS_TECR_XMIT_BUFSZ_4K  BGP_DEVBUS_TECR_XMIT_BUFSZ(0)
#define BGP_DEVBUS_TECR_XMIT_BUFSZ_16K BGP_DEVBUS_TECR_XMIT_BUFSZ(3)
#define BGP_DEVBUS_TECR_RECV_BUFSZ(x)  B4(11,x)
#define BGP_DEVBUS_TECR_RECV_BUFSZ_10K BGP_DEVBUS_TECR_RECV_BUFSZ(4)

#define BGP_DEVBUS_TECR_TOMAL_PLBCLK(x)     B2(15,x)
#define BGP_DEVBUS_TECR_TOMAL_PLBCLK_250MHZ BGP_DEVBUS_TECR_TOMAL_PLBCLK(0)

#define BGP_DEVBUS_VIRTADDR (0xFFFD9000)

#define BGP_DEVBUS_L3CER_ETH_SELECT_ETH BN(14) /* 0=TorusDMA, 1=Ethernet */
#define BGP_DEVBUS_L3CER_ETH_L3_RD_PF   BN(16) /* L3 Read Prefetch Enable */

#define BGP_DEVBUS_TECR_TOMAL_TX_SEL BN(12)
#define BGP_DEVBUS_TECR_TOMAL_RX_SEL BN(13)

#define BGP_DEVBUS_TECR_XEMAC_TX0_SEL BN(20)
#define BGP_DEVBUS_TECR_XEMAC_TX1_SEL BN(21)
#define BGP_DEVBUS_TECR_XEMAC_RX0_SEL BN(22)
#define BGP_DEVBUS_TECR_XEMAC_RX1_SEL BN(23)
#define BGP_DEVBUS_TECR_XENPAK_TX_ON  BN(18)

#define BGP_DEVBUS_XGCR0_DTE_PHY      BN(22)
#define BGP_DEVBUS_XGCR0_PORT_ADDR(x) B5(20,x)

#define BGP_DCR_L30 (0x500) /* 0x500-0x53f: L3-Cache */
#define BGP_DCR_L31 (0x540) /* 0x540-0x57f: L3-Cache 1 */

#define BGP_DCR_L3_ORDERING_CTRL_OFFSET (0x0A)
#define BGP_DCR_L30_ORDERING_CTRL (BGP_DCR_L30+BGP_DCR_L3_ORDERING_CTRL_OFFSET)
#define BGP_DCR_L31_ORDERING_CTRL (BGP_DCR_L31+BGP_DCR_L3_ORDERING_CTRL_OFFSET)

#define BGP_SERDES_TRAIN_ETH BN(19)
#define BGP_SERDES_TRAIN_XP  BN(20)
#define BGP_SERDES_TRAIN_XM  BN(21)
#define BGP_SERDES_TRAIN_YP  BN(22)
#define BGP_SERDES_TRAIN_YM  BN(23)
#define BGP_SERDES_TRAIN_ZP  BN(24)
#define BGP_SERDES_TRAIN_ZM  BN(25)
#define BGP_SERDES_TRAIN_A0  BN(26)
#define BGP_SERDES_TRAIN_A1  BN(27)
#define BGP_SERDES_TRAIN_B0  BN(28)
#define BGP_SERDES_TRAIN_B1  BN(29)
#define BGP_SERDES_TRAIN_C0  BN(30)
#define BGP_SERDES_TRAIN_C1  BN(31)

#define BGP_SERDES_TRAIN_X  (BGP_SERDES_TRAIN_XP | BGP_SERDES_TRAIN_XM)
#define BGP_SERDES_TRAIN_Y  (BGP_SERDES_TRAIN_YP | BGP_SERDES_TRAIN_YM)
#define BGP_SERDES_TRAIN_Z  (BGP_SERDES_TRAIN_ZP | BGP_SERDES_TRAIN_ZM)

#define BGP_SERDES_TRAIN_A  (BGP_SERDES_TRAIN_A0 | BGP_SERDES_TRAIN_A1)
#define BGP_SERDES_TRAIN_B  (BGP_SERDES_TRAIN_B0 | BGP_SERDES_TRAIN_B1)
#define BGP_SERDES_TRAIN_C  (BGP_SERDES_TRAIN_C0 | BGP_SERDES_TRAIN_C1)

typedef struct {
    u32 interrupt_enable;
    u32 l3_coherency;
    u32 tomal_emac_control;
    u32 xgxs_control_r0;
    u32 xgxs_control_r1;
    u32 xgxs_control_r3;
    u32 xgxs_status;
    u32 dma_arbiter_control_status;
} bgp_devbus_t;

typedef struct {
    u32 uci;
    u32 mhz;
    u32 policy;
    u32 process;
    u32 trace;
    u32 node;
    u32 l1;
    u32 l2;
    u32 l3;
    u32 l3_select;
    u32 shared_mem;
    u32 clockstop0;
    u32 clockstop1;
} bgp_personality_kernel_t;

typedef struct {
    u32 ddr_flags;
    u32 srbs0;
    u32 srbs1;
    u32 pbx0;
    u32 pbx1;
    u32 memconfig0;
    u32 memconfig1;
    u32 parmctl0;
    u32 parmctl1;
    u32 miscctl0;
    u32 miscctl1;
    u32 cmdbufmode0;     
    u32 cmdbufmode1;     
    u32 refrinterval0;   
    u32 refrinterval1;   
    u32 odtctl0;         
    u32 odtctl1;         
    u32 datastrobecalib0;
    u32 datastrobecalib1;
    u32 dqsctl;  
    u32 throttle;
    u16 sizemb;
    u8  chips;
    u8  cas;
} bgp_personality_ddr_t;

typedef struct {
    u32 blockid;
    u8  xnodes;
    u8  ynodes;
    u8  znodes;
    u8  xcoord;
    u8  ycoord;
    u8  zcoord;
    u16 psetnum;
    u32 psetsize;
    u32 rankinpset;
    u32 ionodes;
    u32 rank;   
    u32 ionoderank; 
    u16 treeroutes[16];
} bgp_personality_network_t;

typedef struct {
    u8 octet[16];
} bgp_ipaddr_t;

typedef struct {
    u16          mtu;
    u8           emacid[6];
    bgp_ipaddr_t ip;
    bgp_ipaddr_t netmask;
    bgp_ipaddr_t broadcast;
    bgp_ipaddr_t gateway;  
    bgp_ipaddr_t nfsserver;  
    bgp_ipaddr_t servicenode;
    char         nfsexport[32];
    char         nfsmount[32];
    u8           securitykey[32];
} bgp_personality_ethernet_t;

typedef struct
{
    u16 crc;
    u8  version;
    u8  size;
    bgp_personality_kernel_t   kernel;
    bgp_personality_ddr_t      ddr;
    bgp_personality_network_t  net;
    bgp_personality_ethernet_t eth;
    u32 pad[2];
} bgp_personality_t;

static bgp_personality_t * const 
    personality = (bgp_personality_t *)BGP_PERS_BASE;


/* CNS support */
typedef struct {
    unsigned int (*is_ionode)(void);
    unsigned int (*get_personality_size)(void);
    void* (*get_personality_data)(void);
    unsigned int (*get_number_cores)(void);
    int (*take_cpu)(unsigned int cpu, void *arg, void (*entry)(int, void*));
    /* rest omitted */
} cns_service_t;

typedef struct {
    cns_service_t *services;
    unsigned long base_vaddr;
    unsigned long size;
    unsigned long base_paddr;
    unsigned long base_paddr_erpn;
} cns_descriptor_t;


void bg_get_eth_address(int khnic, unsigned char *eth_addr);
unsigned int bg_uci_to_uni(BGP_UCI_ComputeCard_t *uci);
unsigned int bg_uni_to_ipv4(unsigned int uni, 
                       unsigned char *oc3, unsigned char *oc2, unsigned char *oc1, unsigned char *oc0,
                       unsigned char start_oc3, unsigned char start_oc2, unsigned char start_oc1, unsigned char start_oc0,
                       unsigned char end_oc3, unsigned char end_oc2, unsigned char end_oc1, unsigned char end_oc0);
void mailbox_silence(void);
void mailbox_unsilence(void);
int  is_mailbox_silent(void);

int write_ras_event(unsigned facility, unsigned unit, unsigned short errcode,
                    unsigned numdetails, unsigned details[], int blocking);

int is_ionode(void);

#endif /* !__BOARD__BLUEGENE__BGP__BGP_H__ */
