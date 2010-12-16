/*********************************************************************
 *                
 * Copyright (C) 2007, 2010, Volkmar Uhlig, IBM Corporation
 *                
 * Description:   Mailbox driver
 *                
 * All rights reserved
 *                
 ********************************************************************/

#include <config.h>
#include <common.h>
#include <asm/processor.h>
#include <asm/io.h>
#include <ppc4xx.h>
#include <ppc450.h>

#include <fdt.h>
#include <libfdt.h>


/*
 * mailbox access
 */

struct bgp_mailbox_desc {
	u16 offset;	// offset from SRAM base
	u16 size;	// size including header, 0=not present
} __attribute__((packed));

struct bgp_mailbox {
	volatile u16 command;	// comand; upper bit=ack
	u16 len;		// length (does not include header)
	u16 result;		// return code from reader
	u16 crc;		// 0=no CRC
	char data[0];
} __attribute__((packed));

struct ras_event {
    uint32_t uci;
    uint8_t  facility;
    uint8_t  unit;
    uint8_t  errcode;
    uint8_t  detailwords;
    uint64_t timestamp;
    uint32_t ecid[4];
    union {
        uint32_t words[32];
        char text[128];
    } __attribute__((packed)) details;
} __attribute__((packed));

#define BGP_MAX_MAILBOX			(4)
#define BGP_MAILBOX_DESC_OFFSET		(0x7fd0)

#define BGP_DCR_TEST(x)			(0x400 + (x))
#define BGP_DCR_GLOB_ATT_WRITE_SET	BGP_DCR_TEST(0x17)
#define BGP_DCR_GLOB_ATT_WRITE_CLEAR	BGP_DCR_TEST(0x18)
#define BGP_DCR_TEST_STATUS6		BGP_DCR_TEST(0x3a)
#define   BGP_TEST_STATUS6_IO_CHIP	(0x80000000U >> 3)

#define BGP_ALERT_OUT(core)	        (0x80000000U >> (24 + core))
#define BGP_ALERT_IN(core)	        (0x80000000U >> (28 + core))

#define BGP_JMB_CMD2HOST_PRINT          (0x0002)

#define BGP_JMB_CMD2HOST_RAS 0x0004

#define ECID0 BGP_DCR_TEST(0x00D)
#define ECID1 BGP_DCR_TEST(0x00E)
#define ECID2 BGP_DCR_TEST(0x00F)
#define ECID3 BGP_DCR_TEST(0x010)

int get_uci(void);

const void* sram_mapping = (void*)CFG_MAILBOX_BASE;

/* VU HACK: move mb structures into data section so that the late BSS
 * zeroing doesn't wipe the gathered information... */
static struct {
	struct bgp_mailbox *mb;
	unsigned size;
} mb_in[BGP_MAX_MAILBOX] __attribute__((section(".data"))),
  mb_out[BGP_MAX_MAILBOX] __attribute__((section(".data")));

#ifdef CONFIG_OF_LIBFDT
void fdt_register_mailboxes(void *fdt)
{
	int nodeoffset, jtagnode, i; 
	if (!mb_out[0].mb)
		return;
	
	/* update FDT with console0 */
	jtagnode = fdt_path_offset(fdt, "/jtag");
	
	if (jtagnode < 0) {
		jtagnode = fdt_add_subnode(fdt, 0, "jtag");
		if (jtagnode < 0) {
			printf("libfdt: %s\n", fdt_strerror(jtagnode));
			return;
		}
	}
	
	for (i = 0; i < BGP_MAX_MAILBOX; i++)
	{
		char buf[128];
		struct { 
			unsigned long long addr; 
			u32 size; 
		} __attribute__((packed)) desc = {
			(unsigned long)mb_out[i].mb, 
			mb_out[i].size };
		u32 val[2];

		if (!mb_out[i].mb)
			continue;
		
		sprintf(buf, "console%d", i);

		nodeoffset = fdt_add_subnode(fdt, jtagnode, buf);
		if (nodeoffset < 0)
			continue;
		
		fdt_setprop(fdt, nodeoffset, "reg", &desc, sizeof(desc));

		val[0] = BGP_DCR_GLOB_ATT_WRITE_SET;
		val[1] = BGP_DCR_GLOB_ATT_WRITE_CLEAR;
		fdt_setprop(fdt, nodeoffset, "dcr-reg", &val, sizeof(val));

		val[0] = BGP_ALERT_OUT(i);
		val[1] = BGP_ALERT_OUT(i);
		fdt_setprop(fdt, nodeoffset, "dcr-mask", &val, sizeof(val));
	}
}
#endif


void setup_mailboxes(void)
{
    int i;
    struct bgp_mailbox_desc* mb_desc = 
	(struct bgp_mailbox_desc*)(sram_mapping + BGP_MAILBOX_DESC_OFFSET);

    for (i = 0; i < BGP_MAX_MAILBOX; i++)
    {
	if ( mb_desc->size ) {
	    mb_in[i].mb = (struct bgp_mailbox*)(sram_mapping + mb_desc->offset);
	    mb_in[i].size = mb_desc->size - sizeof(struct bgp_mailbox);
	}
	mb_desc++;
	
	if ( mb_desc->size ) {
	    mb_out[i].mb = (struct bgp_mailbox*)(sram_mapping + mb_desc->offset);
	    mb_out[i].size = mb_desc->size - sizeof(struct bgp_mailbox);
	}
	mb_desc++;
    }

}

void mailbox_write(void *data, unsigned len, u16 command)
{
    const int cpu = 0;
    struct bgp_mailbox *mbox = mb_out[cpu].mb;
    void *line;
    if (!mbox)
	return;
    
    if (len > mb_out[cpu].size)
	len = mb_out[cpu].size;

    memcpy(&mbox->data, data, len);
    mbox->len = len;
    mbox->command = command;
    sync();
    mtdcrx(BGP_DCR_GLOB_ATT_WRITE_SET, BGP_ALERT_OUT(cpu));

    // wait for completion
    line = (void *)&(mbox->command);
    do { 
        invalidate_dcache_line(line);
    } while(!(mbox->command & 0x8000));
}

int serial_init(void)
{
    return 0;
}

// no baudrate to be set
void serial_setbrg(void) 
{ 
}

#define MB_BUF_SIZE	128
static char mailbox_buffer[MB_BUF_SIZE];
static int buflen = 0;
#ifndef CONFIG_BGP_KHVMM
static int mailbox_silent = 1;
#else
static int mailbox_silent = 0;
#endif

int is_mailbox_silent(void) 
{
    return mailbox_silent;
}

void mailbox_silence(void) 
{
    mailbox_silent = 1;
}

void mailbox_unsilence(void) 
{
    mailbox_silent = 0;
}

void mailbox_flush_buffer(void)
{
    if (buflen) {
	if (!mailbox_silent) {
            mailbox_write(mailbox_buffer, buflen, BGP_JMB_CMD2HOST_PRINT);
        }
	buflen = 0;
    }
}

void serial_putc (const char c)
{
    if (c < 32 && c != '\n' && c != '\r') return;

    mailbox_buffer[buflen] = c;
    buflen++;

    if (buflen >= MB_BUF_SIZE || c == '\n')
	mailbox_flush_buffer();
}

void serial_puts (const char *str)
{
    while(*str != 0)
	serial_putc(*str++);
}

int serial_getc (void)
{
    return 0;
}

int serial_tstc(void)
{
    return 0;
}

int write_ras_event(unsigned facility, unsigned unit, unsigned short errcode,
                    unsigned numdetails, unsigned details[], int blocking)
{
    struct ras_event event;
    int i;
    unsigned int len;

    event.uci       = get_uci();
    event.facility  = facility;
    event.unit      = unit;
    event.errcode   = errcode;
    event.timestamp = 0;
    event.ecid[0]   = mfdcrx(ECID0);
    event.ecid[1]   = mfdcrx(ECID1);
    event.ecid[2]   = mfdcrx(ECID2);
    event.ecid[3]   = mfdcrx(ECID3);

    event.detailwords = numdetails;
    for (i = 0; i < numdetails; i++)
        event.details.words[i] = details[i];

    len = sizeof(event) - sizeof(event.details) + numdetails * sizeof(uint32_t);

    mailbox_write(&event, len, BGP_JMB_CMD2HOST_RAS);

    return 0;
}
