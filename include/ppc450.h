/*********************************************************************
 *                
 * Copyright (C) 2007-2008, Volkmar Uhlig, Amos Waterland, IBM Corporation
 *                
 * Description:   PPC 450 support functions
 *                
 * All rights reserved
 *                
 ********************************************************************/
#ifndef __PPC450_H__
#define __PPC450_H__

#ifndef __ASSEMBLY__

asm(".macro lfpdx   frt, idx, reg; .long ((31<<26)|((\\frt)<<21)|(\\idx<<16)|(\\reg<<11)|(462<<1)); .endm");
asm(".macro lfpdux  frt, idx, reg; .long ((31<<26)|((\\frt)<<21)|(\\idx<<16)|(\\reg<<11)|(494<<1)); .endm");
asm(".macro stfpdx  frt, idx, reg; .long ((31<<26)|((\\frt)<<21)|(\\idx<<16)|(\\reg<<11)|(974<<1)); .endm");
asm(".macro stfpdux frt, idx, reg; .long ((31<<26)|((\\frt)<<21)|(\\idx<<16)|(\\reg<<11)|(1006<<1)); .endm");

extern inline void fpu_memcpy_16(void *dst, void *src)
{
    asm volatile("lfpdx 0,0,%0\n"
		 "stfpdx 0,0,%1\n"
		 :
		 : "b"(src), "b"(dst)
		 : "fr0", "memory");
}

extern inline void delay(u32 pclocks)
{
   while(pclocks--)
       asm volatile("nop");
}

extern inline void mtdcrx(unsigned int dcrn, unsigned int value)
{
    asm volatile("mtdcrx %0,%1": :"r" (dcrn), "r" (value) : "memory");
}

extern inline unsigned int mfdcrx(unsigned int dcrn)
{
    unsigned int value;
    asm volatile ("mfdcrx %0,%1": "=r" (value) : "r" (dcrn) : "memory");
    return value;
}

#define isync(void) do { asm volatile ("isync" : : : "memory"); } while(0)

extern inline int mtdcrx_v(unsigned int dcrn, unsigned int value)
{
    u32 actual;

    mtdcrx(dcrn, value);
    isync();
    actual = mfdcrx(dcrn);

    return actual != value;
}

extern inline void invalidate_dcache_line(void* ptr)
{
    asm volatile ("dcbi  0,%0" : : "r" (ptr) : "memory");
}

extern inline void out_be128(void *port, void *ptrval)
{
    static u32 tmp[4] __attribute__((aligned(16)));

    if ((u32)ptrval & 0xf)
    {
        //printk("out from unaligned address %p\n", ptrval);
        memcpy(tmp, ptrval, 16);
        ptrval = tmp;
    }

    fpu_memcpy_16(port, ptrval);
}

extern inline void outs_be128(void *src, void *port, unsigned num)
{
    static u32 tmp[4] __attribute__((aligned(16)));
	
    if ((u32)src & 0xf)
	// unaligned destination
	while(num--) {
	    memcpy(tmp, src, 16);
	    fpu_memcpy_16(port, tmp);
	    src += 16;
	}
    else
	while(num--) {
	    fpu_memcpy_16(port, src);
	    src += 16;
	}
}

extern inline void outs_zero128(void *port, unsigned num)
{
    static u32 zero[4] __attribute__((aligned(16))) = {0, };
    while (num--)
	out_be128(port, zero);
}

/*
 * in string operation similar to x86: reads block of data from port
 * into memory
 */
extern inline void ins_be128(void *dest, void *port, unsigned num)
{
    static u32 tmp[4] __attribute__((aligned(16)));

    if ((u32)dest & 0xf)
    {
	// unaligned destination
	while(num--) {
	    fpu_memcpy_16(tmp, port);
	    memcpy(dest, tmp, 16);
	    dest += 16;
	}
    }
    else
    {
	while(num--) {
	    fpu_memcpy_16(dest, port);
	    dest += 16;
	}
    }
}

extern inline void in_be128(void *dest, void *port)
{
    static char tmp[16] __attribute__((aligned(16)));
    void *ptr = dest;

    if ((u32)dest & 0xf)
	ptr = tmp;

    fpu_memcpy_16(ptr, port);

    if ((u32)dest & 0xf)
	memcpy(dest, tmp, 16);
}

extern inline void enable_fpu(void)
{
    unsigned dummy;
    asm volatile("mfmsr %0\n"
		 "ori %0,%0,%1\n"
		 "mtmsr %0\n"
		 : "=r"(dummy)
		 : "i"(MSR_FP));
}


#endif /* __ASSEMBLY__ */

#endif /* !__PPC450_H__ */
