/*
 * (C) Copyright 2007-2008
 * Volkmar Uhlig, Jonathan Appavoo, Amos Waterland
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
#include <command.h>
#include <console.h>
#include <fdt.h>
#include <libfdt.h>
#include <fdt_support.h>
#include <net.h>
#include "bgp.h"

extern void bgtty_direct_receive(void *addr, unsigned count);

int do_mem_mrw ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong	addr, count;
        char    writeval;

	if (argc < 3) {
		printf ("Usage:\n%s\n", cmdtp->usage);
		return 1;
	}

	/* Address is specified since argc > 1
	*/
	addr = simple_strtoul(argv[1], NULL, 16);

	/* Get Count of bytes to read and write */
        count = simple_strtoul(argv[2], NULL, 10);

        printf("Reading %d bytes from stdin to store at 0x%x ...",
               count,addr);

	/* cut-through for tree */
	if (strcmp(stdio_devices[stdin]->name, "bgtty") == 0) {
	    disable_interrupts();

	    /* drain pending console packets */
	    while (tstc() > 0 && count) {
		*((char*)addr) = getc();
		addr++;
		count--;
	    }
	    if (count)
		bgtty_direct_receive((void*)addr, count);

	    enable_interrupts();
	} else {
	    while (count-- > 0) {
		writeval = getc();
		*((char *)addr) = writeval;
		addr++;
	    }
	}
        printf("Done.\n");
	return 0;
}


U_BOOT_CMD(
        mrw,   3,   1, do_mem_mrw,
        "mrw     - memory raw write (read input from stdin directy to memory)\n",
        "address count\n"
);       

int do_silence ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    mailbox_silence();
    return 0;
}

int do_unsilence ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    mailbox_unsilence();
    return 0;
}

int do_is_silent ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[]) 
{
    return !is_mailbox_silent();
}

U_BOOT_CMD(
        silence,   1,   1, do_silence,
        "silence     - squash console mailbox writes\n",
        "no arguments\n"
);       

U_BOOT_CMD(
        unsilence,   1,   1, do_unsilence,
        "unsilence     - enable console mailbox writes\n",
        "no arguments\n"
);       

U_BOOT_CMD(
        issilent,   1,   1, do_is_silent,
        "issilent     - return 0 if silent and 1 if not silent\n",
        "no arguments\n"
);       

int do_eth_mketh ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    ulong netid, ifindex, mtu = 0;
    int nodeoffset, err;
    int tmp;
    unsigned char mac_address[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    char string[80];
    char treephandle[80];
    int treephandlelen;
    char torusphandle[80];
    int torusphandlelen = 0;
    void *data;
    int vnic = 0;
    
    if (argc < 3) {
        printf("Usage:\n%s\n", cmdtp->usage);
        return 1;
    }

    netid = simple_strtoul(argv[1], NULL, 10);
    ifindex = simple_strtoul(argv[2], NULL, 10);
    if (argc >= 3)
	mtu = simple_strtoul(argv[3], NULL, 10); 
    if (argc >= 4 && netid >= 2)
        vnic = !strncmp(argv[4], "vnic", 4);

    /* we rserve network id 0 for kittyhawk use               */
    /*                   id 1 for the external network        */ 
    /*                   id 2 for the internal public network */

    if (netid == 0) {
        printf("ERROR: netid=%d is reserved\n", netid);
        return 1;
    }

    sprintf(string, "/plb/ethernet@%d", ifindex);

    if (strlen(string) >= 80) {
        printf("ERROR: yikes!!!\n");
        return 1;
    }

    nodeoffset = fdt_path_offset(fdt, string);
    
    if (nodeoffset >= 0) {
        if ( (err = fdt_del_node(fdt, nodeoffset)) < 0) {
            printf("ERROR: deleting device node %s (%s)\n",
                   string, fdt_strerror(err));
            return 1;
        }
    }

    nodeoffset = fdt_path_offset(fdt, "/plb/tree");
    if (nodeoffset < 0) {
        printf("ERROR: could not find /plb/tree\n");
        return 1;
    }
    
    data = fdt_getprop(fdt, nodeoffset, "linux,phandle", &treephandlelen);
    if (data == NULL || treephandlelen >= 80 ) {
        printf("ERROR: unable to get value of /plb/tree/linux,phandle\n");
        return -1;
    }
    memcpy(treephandle, data, treephandlelen);

    if (netid > 1) {
	nodeoffset = fdt_path_offset(fdt, "/plb/torus");
	if (nodeoffset < 0) {
	    printf("ERROR: could not find /plb/torus (%s)\n", fdt_strerror(nodeoffset));
	    return 1;
	}
	data = fdt_getprop(fdt, nodeoffset, "linux,phandle", &torusphandlelen);
	if (data == NULL || torusphandlelen >= 80) {
	    printf("ERROR: unable to get value of /plb/torus/linux,phandle\n");
	    return 1;
	}
	memcpy(torusphandle, data, torusphandlelen);
    }

    /* start creating... */
    nodeoffset = fdt_path_offset(fdt, "/plb");
    if (nodeoffset < 0) {
        printf("ERROR: could not find /plb\n");
        return 1;
    }
    
    sprintf(string, "ethernet@%d", ifindex);
    nodeoffset = fdt_add_subnode(fdt, nodeoffset, string);
    if (nodeoffset < 0) {
        printf("ERROR: unable to create %s under /plb\n", string);
        return 1;
    }

    if (netid == 1) {
        char *str=getenv("bgp_ionoderank");
        if (str==NULL) { 
            printf("ERROR: could not determine ionode's rank\n");
            return -1;
        }
        tmp = simple_strtoul(str, NULL, 10);
        if ((err = fdt_setprop(fdt, nodeoffset, "bridge-vector", &tmp, sizeof(tmp))) < 0) {
            printf("ERROR: could not set network-id\n");
            return 1;
        }
    }

    if (!vnic)
    {
        if ((err = fdt_setprop(fdt, nodeoffset, "network-id", &netid, sizeof(netid))) < 0) {
            printf("ERROR: could not set network-id\n");
            return 1;
        }

        tmp = 0x1;
        if ((err = fdt_setprop(fdt, nodeoffset, "link-protocol", &tmp, sizeof(tmp))) < 0) {
            printf("ERROR: could not set link-protocol\n");
            return 1;
        }

        tmp = (netid == 1) ? 1 : 0;
        if ((err = fdt_setprop(fdt, nodeoffset, "tree-channel", &tmp, sizeof(tmp))) < 0) {
            printf("ERROR: could not set tree-channel\n");
            return 1;
        }

        tmp = (netid == 1) ? 0x0 : 0xf;
        if ((err = fdt_setprop(fdt, nodeoffset, "tree-route", &tmp, sizeof(tmp))) < 0) {
            printf("ERROR: could not set tree-route\n");
            return 1;
        }

        if ((err = fdt_setprop(fdt, nodeoffset, "tree-device", treephandle, treephandlelen)) < 0) {
            printf("ERROR: could not set tree-device\n");
            return 1;
        }

        if (netid > 1) {
            if ((err = fdt_setprop(fdt, nodeoffset, "torus-device", torusphandle, torusphandlelen)) < 0) {
                printf("ERROR: could not set torus-device (%s)\n", fdt_strerror(err));
                return 1;
            }
        }
        
        if ((err = fdt_setprop(fdt, nodeoffset, "compatible", "ibm,kittyhawk", 14)) < 0) {
            printf("ERROR: could not set compatible\n");
            return 1;
        }

    }
    else 
    {
        ulong interrupts = 0x110;
        ulong reg[3] = { 0x00000007, 0x40000000, 0x00001000 };
        int *bicref = 0;
        int bic;
        int biclen;
            
        if ((err = fdt_setprop(fdt, nodeoffset, "compatible", "ns83820", 7)) < 0) {
            printf("ERROR: could not set compatible\n");
            return 1;
        }
        
        if ((err = fdt_setprop(fdt, nodeoffset, "interrupts", &interrupts, sizeof(interrupts))) < 0) {
            printf("ERROR: could not set interrupts\n");
            return 1;
        }
        
  
        bicref = fdt_getprop(fdt, fdt_path_offset(fdt, "/plb/torus"), "interrupt-parent", &biclen);
        if (bicref == NULL || biclen != 4)
        {
            printf("ERROR: could not retrieve torus BIC reference\n");
            return 1;
        }
        bic = *bicref;
        
        if ((err = fdt_setprop(fdt, nodeoffset, "interrupt-parent", &bic, biclen)) < 0) {
            printf("ERROR: could not set BIC reference \n");
            return 1;
        }
        
        /* Remove TORUS entry */
        if ((err = fdt_del_node(fdt, fdt_path_offset(fdt, "/plb/torus"))) < 0){
            printf("ERROR: could not delete torus entry\n");
            return 1;
        }

        
        if ((err = fdt_setprop(fdt, nodeoffset, "reg", &reg, sizeof(reg))) < 0) {
            printf("ERROR: could not set regs\n");
            return 1;
        }




    }

    if ((err = fdt_setprop(fdt, nodeoffset, "device_type", "network", 8)) < 0) {
        printf("ERROR: could not set device_type\n");
        return 1;
    }

    if (mtu && (err = fdt_setprop(fdt, nodeoffset, "frame-size", &mtu, sizeof(mtu))) < 0) {
	printf("ERROR: could not set frame size\n");
	return 1;
    }

    if ((err = fdt_setprop(fdt, nodeoffset, "linux,network-index", &ifindex, sizeof(ifindex))) < 0) {
        printf("ERROR: could not set linux,network-index\n");
        return 1;
    }
    
    bg_get_eth_address(netid, mac_address);

    if ((err = fdt_setprop(fdt, nodeoffset, "local-mac-address", mac_address, sizeof(mac_address))) < 0) {
        printf("ERROR: could not set local-mac-address\n");
        return 1;
    }
    return 0;
}

U_BOOT_CMD(
        mketh,   5,   1, do_eth_mketh,
        "mketh     - create a kittyhawk private ethernet fdt entry.\n",
        "netid index [mtu] [vnic]\n"
);       


int do_uni2ip ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    char *var;
    unsigned int uni;
    unsigned int num_vms, vm_nr;
    char *startip, *endip;
    unsigned long sip, eip;
    unsigned char oc3, oc2, oc1, oc0;
    unsigned char soc3, soc2, soc1, soc0;
    unsigned char eoc3, eoc2, eoc1, eoc0;
    char ip[24];
    int rc;

    if (argc != 7) {
        printf("Usage:\n%s\n", cmdtp->usage);
        printf("actual argc %d\n",argc);
        return 1;
    }

    var = argv[1];
    uni = simple_strtoul(argv[2], NULL, 10);
    startip = argv[3];
    endip = argv[4];

	vm_nr = simple_strtoul(argv[5], NULL, 10);
	num_vms = simple_strtoul(argv[6], NULL, 10);
	uni = uni * num_vms + vm_nr;

    sip = string_to_ip(startip);
    eip = string_to_ip(endip);
    soc0 = sip & 0xff; soc1 = (sip >> 8) & 0xff; soc2 = (sip >> 16) & 0xff; soc3 = (sip >> 24) & 0xff;
    eoc0 = eip & 0xff; eoc1 = (eip >> 8) & 0xff; eoc2 = (eip >> 16) & 0xff; eoc3 = (eip >> 24) & 0xff;

    rc = bg_uni_to_ipv4(uni, 
                        &oc3, &oc2, &oc1, &oc0, 
                        soc3, soc2, soc1, soc0,
                        eoc3, eoc2, eoc1, eoc0);
    if (rc==0) {
        printf("ERROR: uni2ip unable to map uni %d between %s and %s\n", uni, startip, endip);
        return 1;
    }
    sprintf(ip, "%u.%u.%u.%u", oc3, oc2, oc1, oc0);
    setenv(var, ip);
    return 0;
}

U_BOOT_CMD(
        uni2ip,   7,   1, do_uni2ip,
        "uni2ip  - sets var to an ipv4 address generated from uni between startip and endip \n",
        "var uni startip endip\n"
);       

int do_handshake( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
    if (is_ionode()) {
        write_ras_event(0x01, 0x1E, 0x00, 0, 0, 0);
    }

    return 0;
}

U_BOOT_CMD(
        handshake, 1, 1, do_handshake,
        "handshake     - switch control system state from booting to initialized\n",
        "no arguments\n"
);
