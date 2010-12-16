/*********************************************************************
 *                
 * Copyright (C) 2007-2008, 2010, Volkmar Uhlig, IBM Corporation
 *                
 * Description:   Ethernet driver for BG networks
 *                
 * All rights reserved
 *                
 ********************************************************************/

#include <config.h>
#include <common.h>
#include <malloc.h>
#include <net.h>
#include <devices.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <commproc.h>
#include <ppc4xx.h>
#include <ppc4xx_enet.h>
#include <ppc450.h>

#include "bgp.h"

extern void mailbox_write(char *, unsigned);

#define mb_printf(format...)			\
do {						\
    char tmp[1024];				\
    sprintf(tmp, format);			\
    mailbox_write(tmp, strlen(tmp));		\
} while(0)
    


//#define VERBOSE

/* hardware header */
union bgtree_header {
	unsigned int raw;
	struct {
		unsigned int pclass	: 4;
		unsigned int p2p	: 1;
		unsigned int irq	: 1;
		unsigned vector		: 24;
		unsigned int csum_mode	: 2;
	} p2p;
	struct {
		unsigned int pclass	: 4;
		unsigned int p2p	: 1;
		unsigned int irq	: 1;
		unsigned int op		: 3;
		unsigned int opsize	: 7;
		unsigned int tag	: 14;
		unsigned int csum_mode	: 2;
	} bcast;
} __attribute__((packed));


union bgtree_status {
	unsigned int raw;
	struct {
		unsigned int inj_pkt	: 4;
		unsigned int inj_qwords	: 4;
	        unsigned int __res0	: 4;      
		unsigned int inj_hdr	: 4;
		unsigned int rcv_pkt	: 4;
		unsigned int rcv_qwords : 4;
		unsigned int __res1	: 3;
		unsigned int irq	: 1;
		unsigned int rcv_hdr	: 4;
	} x;
} __attribute__((packed));


/* link layer */
struct bglink_hdr
{
    unsigned int dst_key; 
    unsigned int src_key; 
    unsigned short conn_id; 
    unsigned char this_pkt; 
    unsigned char total_pkt;
    unsigned short lnk_proto;  // 1 eth, 2 con, 3...
    union {
	unsigned short optional; // for encapsulated protocol use
	struct {
	    u16 option	 : 4;
	    u16 pad_head : 4;
	    u16 pad_tail : 8;
	} opt_eth;
    };
} __attribute__((packed));


struct bg_tree {
    unsigned int curr_conn;
    unsigned int nodeid;
    unsigned int net_active;
    unsigned long version;
};

struct bg_tree_eth {
    unsigned int network_id;
    unsigned int eth_bridge_vector;
    unsigned int route;
};

// VC0 and VC1 Watermark

#define TREE_IRQMASK_INJ	(0)
#define TREE_IRQMASK_REC	(3)
#define TREE_ETH		1
#define TREE_TTY		16

#define TREE_FRAGPAYLOAD	(TREE_PAYLOAD - sizeof(struct bglink_hdr))

#define MAX_FRAGMENTS		(16)
/*
 * Putting a large buffer in BSS makes uboot too big.  See the ASSERT in
 * u-boot.lds.  For now we're getting away with it because BSS is not
 * actually zeroed.
#define PACKET_BUF_SIZE		128
 */
#ifndef CONFIG_BGP_KHVMM
#define PACKET_BUF_SIZE		2048
#else
#define PACKET_BUF_SIZE		64
#endif 

typedef struct {
    struct bglink_hdr lnk;
    unsigned char data[MAX_FRAGMENTS * TREE_FRAGPAYLOAD];
    unsigned long version; // version counter to maintain ordering
} __attribute__((packed)) tree_packet_t;

static tree_packet_t packet_buffer[PACKET_BUF_SIZE] __attribute__((aligned(16)));
static int (*early_packet_handler)(tree_packet_t *packet);

// VU: Can't pre-initialize functions since pointers would not be
// relocated
static struct eth_device tree_dev = { .name = "tree_dev" };
static struct bg_tree tree;
static struct bg_tree_eth bg_tree_eth0;
static void flush_pending_packets(void);
static void bg_tree_receive(int chnidx);
static int bg_tree_early_packet_drop(struct bglink_hdr *lnkhdr);


static void bg_tree_drain(int chnidx)
{
    static char drain[16] __attribute__((aligned(16)));
    void *ioaddr =  (void*)((chnidx == 0) ? CFG_TREE_CHN0 : CFG_TREE_CHN1);
    union bgtree_status status;

    status.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_Sx));

    while (status.raw & 0xffff) {
	if (status.x.rcv_hdr)
	    in_be32((unsigned*)(ioaddr + BGP_TRx_HR));
	if (status.x.rcv_pkt || status.x.rcv_qwords)
	    in_be128(&drain, (void*)ioaddr + BGP_TRx_DR);
	status.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_Sx));

	printf("drain status: %x\n", status.raw);
    }
}

static void bg_tree_enable_irqs(struct eth_device *dev)
{
    // set watermarks
    mtdcrx( BGP_DCR_TR_GLOB_VCFG0, TR_GLOB_VCFG_RWM(0));
    mtdcrx( BGP_DCR_TR_GLOB_VCFG1, TR_GLOB_VCFG_RWM(0));

    // enable interrupts
    mtdcrx( BGP_DCR_TR_INJ_PIXEN, TREE_IRQMASK_INJ );
    mtdcrx( BGP_DCR_TR_REC_PRXEN, TREE_IRQMASK_REC );

    // clear exception flags
    mfdcrx( BGP_DCR_TR_INJ_PIXF );
    mfdcrx( BGP_DCR_TR_REC_PRXF );
}

/* This function is not called by anybody.  */
#if 0
static void bg_tree_disable_irqs(struct eth_device *dev)
{
    mtdcrx( BGP_DCR_TR_INJ_PIXEN, 0 );
    mtdcrx( BGP_DCR_TR_REC_PRXEN, 0 );
}
#endif

static void bg_tree_transmit(union bgtree_header dest, struct bglink_hdr *lnkhdr, 
			    int chnidx, void *data, int len)
{
    unsigned idx, fragidx;
    union bgtree_status status;
    void *payloadptr = data;
    void *ioaddr =  (void*)((chnidx == 0) ? CFG_TREE_CHN0 : CFG_TREE_CHN1);
    int flags;

    enable_fpu();

    // prepare link header
    lnkhdr->conn_id = tree.curr_conn++;
    lnkhdr->total_pkt = ((len - 1) / TREE_FRAGPAYLOAD) + 1;
    lnkhdr->this_pkt = 0;

#ifdef VERBOSE
    mb_printf("transmit: len=%d, dest=%lx, [key dest=%lx, src=%lx], total=%d\n", 
	      len, dest.raw, lnkhdr->dst_key, lnkhdr->src_key, lnkhdr->total_pkt);
#endif

    // iterate over fragments and send them out...
    for (fragidx = 0; fragidx < lnkhdr->total_pkt; fragidx++)
    {
	// check that we can insert a packet and poll for a bit if
	// fifo is full
	status.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_Sx));

	if (status.x.inj_hdr >= TREE_FIFO_SIZE)
	{
	    int max_poll = (850 * 1000) * 200;
	    do {
		status.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_Sx));
		if (max_poll-- == 0) {
		    mb_printf("tree: send fifo full, dropping packets (status=%08x)\n", status.raw);
		    goto out;
		}
	    } while (status.x.inj_hdr >= TREE_FIFO_SIZE);
	}

	/* protect FPU registers */
	flags = disable_interrupts();

	// write destination header
	out_be32((unsigned*)(ioaddr + BGP_TRx_HI), dest.raw);

	// update fragment index
	lnkhdr->this_pkt = fragidx;

	// send link header (16 bytes)
	out_be128((void*)(ioaddr + BGP_TRx_DI), lnkhdr);

	// send payload
	for (idx = sizeof(*lnkhdr); len > 0 && idx < TREE_PAYLOAD;
	     idx += 16, payloadptr += 16, len -= 16)
	    out_be128((void*)(ioaddr + BGP_TRx_DI), payloadptr);

	// pad last packet with zeros
	if (idx < TREE_PAYLOAD)
	    outs_zero128((void*)(ioaddr + BGP_TRx_DI), (TREE_PAYLOAD - idx) / 16);

	if (flags)
	    enable_interrupts();
    }

 out:;
}

static void bg_tree_receive(int chnidx)
{
    static struct bglink_hdr lnkhdr __attribute__((aligned(16)));
    static unsigned char drain[16] __attribute__((aligned(16)));

    void *ioaddr =  (void*)((chnidx == 0) ? CFG_TREE_CHN0 : CFG_TREE_CHN1);
    void *payloadptr;
    unsigned i;
    union bgtree_header dst;
    union bgtree_status status;
#ifdef VERBOSE
    char tmp[1024];
#endif

    enable_fpu();

    status.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_Sx));

    while ( status.x.rcv_hdr )
    {
	tree_packet_t *packet = NULL;
	unsigned drop = 0;

	// packet destination
	dst.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_HR));

	// read the first 16 bytes from the payload
	in_be128(&lnkhdr, (void*)ioaddr + BGP_TRx_DR);

#ifdef VERBOSE
	if (dst.raw != 0xf0000000) {
            sprintf(tmp,  "bgnet: nodeid=%x stat=%x, dst=%x, hdr: conn=%x, this_pkt=%x, tot_pkt=%x, dst=%x, src=%x]\n", 
                    tree.nodeid, status.raw, dst.raw, lnkhdr.conn_id, lnkhdr.this_pkt, lnkhdr.total_pkt,
                    lnkhdr.dst_key, lnkhdr.src_key);
            mailbox_write(tmp, strlen(tmp));
        }
#endif

	if (bg_tree_early_packet_drop(&lnkhdr)) {
	    drop = 1;
	    goto rcv_pkt;
	}

	// early check if packet is valid
	if (lnkhdr.total_pkt < 1 || lnkhdr.this_pkt >= lnkhdr.total_pkt) {
	    mb_printf("bgnet: invalid packet tot: %d, this: %d\n", 
		      lnkhdr.total_pkt, lnkhdr.this_pkt);
	    drop = 1;
	    goto rcv_pkt;
	}

	// are there other fragment already?
	for (i = 0; i < PACKET_BUF_SIZE; i++)
	    if (packet_buffer[i].lnk.total_pkt &&
		packet_buffer[i].lnk.src_key == lnkhdr.src_key &&
		packet_buffer[i].lnk.dst_key == lnkhdr.dst_key &&
		packet_buffer[i].lnk.conn_id == lnkhdr.conn_id)
	    {
		packet = &packet_buffer[i];
		break;
	    }

	if (!packet) {
	    for (i = 0; i < PACKET_BUF_SIZE; i++)
		if (packet_buffer[i].lnk.total_pkt == 0) {
		    packet = &packet_buffer[i];
		    packet->version = tree.version++;
		    break;
		}

	    // we don't fuzz too much here: if the queues overrun we
	    // just throw away everything pending and wait for re-transmit
	    if (!packet) {
#if 0
		mb_printf("bgtree: no free receive descriptors (last: %lx, src=%x, dst=%x)\n",
			  dst.raw, lnkhdr.src_key, lnkhdr.dst_key);
#endif
		flush_pending_packets();
		packet = &packet_buffer[0];
	    }

	    packet->lnk = lnkhdr;
	    packet->lnk.this_pkt = 0; // reset counter
	} 
	else {
	    // perform some sanity checks
	    if ( (packet->lnk.total_pkt != lnkhdr.total_pkt) || 
		 (packet->lnk.this_pkt > lnkhdr.total_pkt) )
	    {
		drop = 1;
#ifdef VERBOSE
		mb_printf("bgtree: packet link header differs from previous packet(s); drop\n");
		mb_printf("tot [%d, %d], this %d\n", packet->lnk.total_pkt, lnkhdr.total_pkt, packet->lnk.this_pkt);
#endif
	    }
	}

    rcv_pkt:
	if (drop)
	{
#ifdef VERBOSE
	    // drain the packet buffer
	    mb_printf("bgtree drained and dropped: stat=%x, dst=%x, hdr: conn=%x, this_pkt=%x, tot_pkt=%x, dst=%x, src=%x]\n", 
		      status.raw, dst.raw, lnkhdr.conn_id, lnkhdr.this_pkt, lnkhdr.total_pkt,
		      lnkhdr.dst_key, lnkhdr.src_key);
#endif
	    for (i = 16; i < TREE_PAYLOAD; i += 16)
		in_be128(drain, (void*)ioaddr + BGP_TRx_DR);
	}
	else {
	    // receive packet into skb
	    payloadptr = &packet->data[lnkhdr.this_pkt * TREE_FRAGPAYLOAD];

	    // read remaining payload
	    ins_be128(payloadptr, (void*)ioaddr + BGP_TRx_DR, 
		      (TREE_PAYLOAD - sizeof(struct bglink_hdr)) / 16);

	    //fskb->frag_timeout = jiffies + FRAGMENT_TIMEOUT;
	    packet->lnk.this_pkt++;

	    if (early_packet_handler && (packet->lnk.this_pkt == packet->lnk.total_pkt)) {
		if (early_packet_handler(packet))
		    packet->lnk.total_pkt = 0;
	    }
	}

	// pick up packets that arrived meanwhile...
	if ( (status.x.rcv_hdr--) == 0)
	    status.raw = in_be32((unsigned*)(ioaddr + BGP_TRx_Sx));
    }

}

int bg_tree_interrupt (struct eth_device *dev)
{
    unsigned pend_irq;

    // read & clear interrupts
    pend_irq = mfdcrx(BGP_DCR_TR_REC_PRXF);

    if (pend_irq & ~(TR_REC_PRX_WM0 | TR_REC_PRX_WM1)) {
	printf("TREE: error receiving packets (%x)\n", pend_irq);
	hang();
    }

    if (pend_irq & TR_REC_PRX_WM0)
	bg_tree_receive(0);
    if (pend_irq & TR_REC_PRX_WM1)
	bg_tree_receive(1);

    return -1;
}

/*
 * ethernet over tree
 */
extern inline int is_eth_broadcast(unsigned char *addr)
{
    return (addr[0] & addr[1] & addr[2] & 
	    addr[3] & addr[4] & addr[5]) == 0xff;
}

extern inline unsigned int eth_to_key(unsigned char *addr)
{
    unsigned int key;
    if (is_eth_broadcast(addr))
	key = ~0U;
    else
	key = (addr[3] << 16) | (addr[4] << 8) | (addr[5] << 0);
    return key;
}


static int bg_tree_eth_send (struct eth_device *dev, volatile void *ptr, int len)
{
    static struct bglink_hdr snd_lnkhdr __attribute__((aligned(16))); // static for correct alignment
    union bgtree_header dest;
    Ethernet_t *hdr = (Ethernet_t*)ptr;
    int pad_head = (unsigned long)ptr & 0xf;
    struct bg_tree_eth *tree_eth = (struct bg_tree_eth *)dev->priv;

    memset(&snd_lnkhdr, 0, sizeof(snd_lnkhdr));

#ifdef VERBOSE
    printf("BG Tree send src=%02x:%02x:%02x:%02x:%02x:%02x, dest=%02x:%02x:%02x:%02x:%02x:%02x\n",
	   hdr->et_src[0], hdr->et_src[1], hdr->et_src[2], hdr->et_src[3], hdr->et_src[4], hdr->et_src[5],
	   hdr->et_dest[0], hdr->et_dest[1], hdr->et_dest[2], hdr->et_dest[3], hdr->et_dest[4], hdr->et_dest[5]);
#endif
    dest.raw = 0;
    dest.p2p.pclass = tree_eth->route;
    if (tree_eth->eth_bridge_vector == -1) {
        if (is_eth_broadcast(hdr->et_dest))
            dest.bcast.tag = 0;
        else {
            dest.p2p.vector = *(unsigned int*)(&hdr->et_dest[2]);
            dest.p2p.p2p = 1;
        }
    } else {
        dest.p2p.vector = tree_eth->eth_bridge_vector;
        dest.p2p.p2p = 1;        
    }

    snd_lnkhdr.dst_key = tree_eth->network_id;   
    snd_lnkhdr.src_key = tree.nodeid;
    snd_lnkhdr.lnk_proto = TREE_ETH; // ethernet

    /* head padding is safe since we only mask out the lowest most
     * bits and therefore always stay in the same page.  We transmit
     * garbage, though */
    snd_lnkhdr.opt_eth.pad_head = pad_head;
    snd_lnkhdr.opt_eth.pad_tail = (240 - ((len + pad_head) % 240)) % 240;
 
    bg_tree_transmit(dest, &snd_lnkhdr, 0, (void*)ptr - pad_head, len + pad_head);

    return len;
}

#if 0
static int bg_tree_eth_update_from_env(struct eth_device *dev)
{
    char *s;
    struct bg_tree_eth *tree_eth = (struct bg_tree_eth *)dev->priv;

    if ((s = getenv("bg_eth0_netid")) != NULL) {
	tree_eth->network_id = simple_strtoul(s, NULL, 10);
    } else {
        /* This is so that behaviour is not totally stupid when bg_eth0_netid is not found */
        tree_eth->network_id = 1;
        setenv("bg_eth0_netid", "1");
    }
    return 1;
}
#endif

static int bg_tree_eth_init (struct eth_device *dev, bd_t * bis)
{
//    struct bg_tree_eth *tree_eth = (struct bg_tree_eth *)dev->priv;
    int flags;
//    char eth_addr_str[64];
//    bgp_personality_t *pers = (bgp_personality_t*)BGP_PERS_BASE;
//    bgp_personality_network_t *net = &pers->net;

    //printf("BG Tree init\n");


#if 0
    if ( ! bg_tree_eth_update_from_env(dev) ) return 0;
    bg_netid_to_eth_address(tree_eth->network_id, dev->enetaddr);
    sprintf (eth_addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
             dev->enetaddr[0], dev->enetaddr[1], dev->enetaddr[2],
             dev->enetaddr[3], dev->enetaddr[4], dev->enetaddr[5]);
    setenv ("ethaddr", eth_addr_str);

 
    if (tree_eth->network_id == 1) {
        tree_eth->eth_bridge_vector = net->ionoderank;
        tree_eth->route = 0;
    } else {
        tree_eth->eth_bridge_vector = -1;
        tree_eth->route = 15;
    }
#endif

    flags = disable_interrupts();

    bg_tree_drain(0);
    bg_tree_drain(1);
    flush_pending_packets();

    tree.net_active = 1;
	
    if (flags)
	enable_interrupts();
    
    return 1;
}

static void bg_tree_eth_halt (struct eth_device *dev)
{
    //printf("BG Tree halt\n");
    tree.net_active = 0;
}

static int bg_tree_eth_recv (struct eth_device *dev)
{
    struct bg_tree_eth *tree_eth = (struct bg_tree_eth *)dev->priv;
    int flags, i;
    flags = disable_interrupts();
//    printf("BG Tree receive\n");

    for (i = 0; i < PACKET_BUF_SIZE; i++)
	if (packet_buffer[i].lnk.total_pkt != 0 &&
	    packet_buffer[i].lnk.total_pkt == packet_buffer[i].lnk.this_pkt &&
            packet_buffer[i].lnk.lnk_proto == TREE_ETH &&
            packet_buffer[i].lnk.dst_key == tree_eth->network_id)
            
	{
	    uchar *data = packet_buffer[i].data + packet_buffer[i].lnk.opt_eth.pad_head;
	    int len = packet_buffer[i].lnk.total_pkt * 240 - 
		packet_buffer[i].lnk.opt_eth.pad_head - 
		packet_buffer[i].lnk.opt_eth.pad_tail;
#ifdef VERBOSE
	    Ethernet_t *hdr = (Ethernet_t*)data;
	    if (!is_eth_broadcast(hdr->et_dest)) {
		printf("eth: received packet of %d bytes\n", len);
		printf("     src=%02x:%02x:%02x:%02x:%02x:%02x, dest=%02x:%02x:%02x:%02x:%02x:%02x\n",
		       hdr->et_src[0], hdr->et_src[1], hdr->et_src[2], hdr->et_src[3], hdr->et_src[4], hdr->et_src[5],
		       hdr->et_dest[0], hdr->et_dest[1], hdr->et_dest[2], hdr->et_dest[3], hdr->et_dest[4], hdr->et_dest[5]);
	    }
#endif
	    // hand to packet filter...
	    if (len > 0)
		NetReceive(data, len);
	    packet_buffer[i].lnk.total_pkt = 0;
	}

    if (flags)
	enable_interrupts();
    return 0;
}

int ppc_4xx_eth_initialize (bd_t * bis)
{
    bgp_personality_t *pers = (bgp_personality_t*)BGP_PERS_BASE;
    bgp_personality_network_t *net = &pers->net;

    char eth_addr_str[64];

    tree.nodeid = pers->net.rank;

    tree_dev.init = bg_tree_eth_init;
    tree_dev.halt = bg_tree_eth_halt;
    tree_dev.send = bg_tree_eth_send;
    tree_dev.recv = bg_tree_eth_recv;

    bg_tree_eth0.network_id = 1;
    bg_tree_eth0.route = 0;
    bg_tree_eth0.eth_bridge_vector = net->ionoderank;

    tree_dev.priv = (void *) &bg_tree_eth0;
    
    bg_get_eth_address(1, tree_dev.enetaddr);

    /* update the environment with the correct ethernet address */
    sprintf (eth_addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
             tree_dev.enetaddr[0], tree_dev.enetaddr[1], tree_dev.enetaddr[2],
             tree_dev.enetaddr[3], tree_dev.enetaddr[4], tree_dev.enetaddr[5]);
    setenv ("ethaddr", eth_addr_str);

    irq_install_handler (181, (interrupt_handler_t *)bg_tree_interrupt, &tree_dev);
    bg_tree_enable_irqs(&tree_dev);

#if defined(CONFIG_NET_MULTI)
    eth_register (&tree_dev);
#endif
#ifdef VERBOSE
    printf("ppc_4xx_eth_initialize: ethaddr %02X:%02X:%02X:%02X:%02X:%02X "
           "netid=%d route=%d eth-bridge-vector=%d ",
           tree_dev.enetaddr[0], tree_dev.enetaddr[1], tree_dev.enetaddr[2],
           tree_dev.enetaddr[3], tree_dev.enetaddr[4], tree_dev.enetaddr[5],
           bg_tree_eth0.network_id, bg_tree_eth0.route, bg_tree_eth0.eth_bridge_vector);
#endif
    return 0;
}


/**********************************************************************
 * Tree-based console
 **********************************************************************/

#define TTY_BUF_SIZE	240
static char tty_out_buffer[TTY_BUF_SIZE];
static int tty_out_len = 0;
static int tty_in_len = 0;
static unsigned char *tty_in_ptr = NULL;
static int tty_in_packet;

static int tty_sendid;
static int tty_rcvid;
static int tty_dest;

static void bgtty_update_env(void)
{
    char *s;
    tty_sendid = 0;
    tty_rcvid = 0;
    tty_dest = -1;

    if ((s = getenv("bgtty_sendid")) != NULL)
	tty_sendid = simple_strtoul(s, NULL, 10);

    if ((s = getenv("bgtty_rcvid")) != NULL)
	tty_rcvid = simple_strtoul(s, NULL, 10);

    if ((s = getenv("bgtty_dest")) != NULL && s[0] != 'b')
	tty_dest = simple_strtoul(s, NULL, 10);
}

static void bgtty_flush_buffer(void)
{
    if (tty_out_len) {
	static struct bglink_hdr lnkhdr __attribute__((aligned(16)));
	union bgtree_header dest;

	bgtty_update_env();

	lnkhdr.dst_key = tty_sendid;
	lnkhdr.src_key = tree.nodeid;
	lnkhdr.lnk_proto = TREE_TTY;
	lnkhdr.optional = tty_out_len;

	dest.raw = 0;		// default: broadcast
	dest.p2p.pclass = 15;	// route 15

	if (tty_dest != -1) {
	    dest.p2p.vector = tty_dest;
	    dest.p2p.p2p = 1;
	}

	bg_tree_transmit(dest, &lnkhdr, 0, tty_out_buffer, tty_out_len);
	tty_out_len = 0;
    }
}

static void bgtty_putc(char c)
{
    tty_out_buffer[tty_out_len++] = c;

    if (tty_out_len >= TTY_BUF_SIZE || c == '\n')
	bgtty_flush_buffer();
}

static void bgtty_puts(const char *str)
{
    while(*str != 0)
	bgtty_putc(*str++);
}

static int bgtty_tstc(void)
{
    int i, flags;

    if (!tty_in_len)
    {
	bgtty_update_env();

	flags = disable_interrupts();

	for (i = 0; i < PACKET_BUF_SIZE; i++)
	    if (packet_buffer[i].lnk.total_pkt != 0 &&
		packet_buffer[i].lnk.total_pkt == packet_buffer[i].lnk.this_pkt &&
		packet_buffer[i].lnk.lnk_proto == TREE_TTY)
	    {
#if 0
		printf("found tty packet (src=%d, dst=%d; filter: snd=%d rcv=%d)\n", 
		       packet_buffer[i].lnk.src_key, packet_buffer[i].lnk.dst_key, 
		       tty_sendid, tty_rcvid);
#endif

		// drop packets if they are from ourselves or don't match the rcvid
		if (packet_buffer[i].lnk.src_key == tree.nodeid ||
		    packet_buffer[i].lnk.dst_key != tty_rcvid) {
		    packet_buffer[i].lnk.total_pkt = 0;
		    continue;
		}

		// found a packet... is it oldest? (assume no wrap)
		// if (tty_in_len && (tree->version - packet_buffer[i].version < tree->version - packet_buffer[tty_in_packet].version))
		if (tty_in_len && packet_buffer[i].version > packet_buffer[tty_in_packet].version)
		    continue;

		tty_in_packet = i;
		tty_in_ptr = packet_buffer[i].data;
		tty_in_len = packet_buffer[i].lnk.optional;
	    }

	if (flags)
	    enable_interrupts();
    }

    return tty_in_len;
}

static int bgtty_getc(void)
{
    char c;
    while (!bgtty_tstc())
    { /* wait */ }
    c = *tty_in_ptr++;
    if (--tty_in_len == 0)
	packet_buffer[tty_in_packet].lnk.total_pkt = 0;
    return c;
}

int drv_bgtty_init(void)
{
    device_t dev;
    int rc;
  
    memset (&dev, 0, sizeof (dev));

    strcpy (dev.name, "bgtty");
    dev.flags = DEV_FLAGS_OUTPUT | DEV_FLAGS_INPUT | DEV_FLAGS_SYSTEM;
    dev.putc = bgtty_putc;
    dev.puts = bgtty_puts;
    dev.getc = bgtty_getc;
    dev.tstc = bgtty_tstc;
    
    rc = device_register (&dev);
    
    return (rc == 0) ? 1 : rc;
}

/* cut-through receive function */
static volatile void *early_tty_addr;
static volatile int early_tty_len;

static int early_tty_packet_handler(tree_packet_t *packet)
{
    int len;
    int rc = 1;

    if (packet->lnk.lnk_proto != TREE_TTY)
	return 0;

    if (packet->lnk.src_key != tree.nodeid &&
	packet->lnk.dst_key == tty_rcvid)
    {
	len = packet->lnk.optional;
	if (len > early_tty_len) {
	    len = early_tty_len;

	    tty_in_packet = ((void*)packet - (void*)packet_buffer) / sizeof(tree_packet_t);
	    tty_in_ptr = &packet->data[len];
	    tty_in_len = packet->lnk.optional - len;
	    rc = 0;
	}

	memcpy((void*)early_tty_addr, packet->data, len);
	early_tty_addr += len;
	early_tty_len -= len;

	if (early_tty_len <= 0)
	    early_packet_handler = NULL;
    }
    return rc;
}

void bgtty_direct_receive(void *addr, unsigned count)
{
    early_tty_addr = addr;
    early_tty_len = count;
    early_packet_handler = &early_tty_packet_handler;

    asm("":::"memory");

    enable_interrupts();

    while(early_tty_len) { /* wait */ }
}

static void flush_pending_packets(void)
{
    int i;
    for (i = 0; i < PACKET_BUF_SIZE; i++)
	packet_buffer[i].lnk.total_pkt = 0;
    
    /* clear tty in buffer */
    tty_in_len = 0;
}


/* early packet filter to minimize pressure on buffer space */
static int bg_tree_early_packet_drop(struct bglink_hdr *lnkhdr)
{
    // drop packets from self
    if (lnkhdr->src_key == tree.nodeid)
	return 1;

    // drop eth packets early if nic is inactive
    if ((lnkhdr->lnk_proto == TREE_ETH) && !tree.net_active)
	return 1;

#if 0
    if ((lnkhdr->lnk_proto == TREE_TTY) && lnkhdr->dst_key != tty_rcvid)
	return 1;
#endif

    return 0;
}

