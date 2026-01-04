// Test of parallel AD9226 ADC using Raspberry Pi SMI (Secondary Memory Interface)
// For detailed description, see https://iosoft.blog
//
// Copyright (c) 2020 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.06 JPB 16/7/20 Tidied up for Github

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "rpi_dma_utils.h"
#include "rpi_smi_defs.h"

// Set zero for single value, non-zero for block read
#define USE_DMA         1
// Use test pin in place of GPIO mode setting (to check timing)
//#define USE_TEST_PIN    0

//define SMI timings for USB-LS
#if PHYS_REG_BASE==PI_4_REG_BASE        // Timings for RPi v4 (1.5 GHz)
#define SMI_TIMING       10, 15, 30, 15    // 400 ns cycle time
#else                                   // Timings for RPi v0-3 (1 GHz)
#define SMI_TIMING       10, 17, 33, 17   // 670 ns cycle time (roughly 1.49MHz)
#endif

#define SMI_NUM_BITS SMI_8_BITS

// Number of raw bytes per ADC sample
#define SAMPLE_SIZE     1

// Number of samples to be captured, and number to be discarded
#define NSAMPLES        500

// DMA request threshold
#define REQUEST_THRESH  4

// SMI register names for diagnostic print
char *smi_regstrs[] = {
    "CS","LEN","A","D","DSR0","DSW0","DSR1","DSW1",
    "DSR2","DSW2","DSR3","DSW3","DMC","DCS","DCA","DCD",""
};

// SMI CS register field names for diagnostic print
#define STRS(x)     STRS_(x) ","
#define STRS_(...)  #__VA_ARGS__
char *smi_cs_regstrs = STRS(SMI_CS_FIELDS);

// Structures for mapped I/O devices, and non-volatile memory
extern MEM_MAP gpio_regs, dma_regs;
MEM_MAP vc_mem, clk_regs, smi_regs;

// Pointers to SMI registers
volatile SMI_CS_REG  *smi_cs;
volatile SMI_L_REG   *smi_l;
volatile SMI_A_REG   *smi_a;
volatile SMI_D_REG   *smi_d;
volatile SMI_DMC_REG *smi_dmc;
volatile SMI_DSR_REG *smi_dsr;
volatile SMI_DSW_REG *smi_dsw;
volatile SMI_DCS_REG *smi_dcs;
volatile SMI_DCA_REG *smi_dca;
volatile SMI_DCD_REG *smi_dcd;

// Buffer for captured samples
uint8_t sample_data[NSAMPLES];

// Non-volatile memory size
#define VC_MEM_SIZE(nsamp) (PAGE_SIZE + ((nsamp)+4)*SAMPLE_SIZE)

void map_devices(void);
void fail(char *s);
void terminate(int sig);
void start_smi(MEM_MAP *mp);
uint32_t *adc_dma_start(MEM_MAP *mp, int nsamp);
int adc_dma_end(void *buff, uint8_t *data, int nsamp);
void init_smi(int width, int ns, int setup, int hold, int strobe);
void disp_reg_fields(char *regstrs, char *name, uint32_t val);
void dma_wait(int chan);

int main(int argc, char *argv[])
{
    void *rxbuff;
    int i;

    signal(SIGINT, terminate);
    map_devices();
    init_smi(SMI_NUM_BITS, SMI_TIMING);
    map_uncached_mem(&vc_mem, VC_MEM_SIZE(NSAMPLES));
    rxbuff = adc_dma_start(&vc_mem, NSAMPLES);
    start_smi(&vc_mem); //actually start the SMI transfer
    while (dma_active(DMA_CHAN_A)) ;
    adc_dma_end(rxbuff, sample_data, NSAMPLES);
    for (i=0; i<NSAMPLES; i++)
        printf("Sample Num %d: %d\n", i, sample_data[i]);
    terminate(0);
    return(0);
}

// Start SMI DMA transfers
void start_smi(MEM_MAP *mp)
{
    DMA_CB *cbs=mp->virt;

    start_dma(mp, DMA_CHAN_A, &cbs[0], 0);
    smi_cs->start = 1;
}

// Start DMA for SMI ADC, return Rx data buffer
uint32_t *adc_dma_start(MEM_MAP *mp, int nsamp)
{
    DMA_CB *cbs=mp->virt;

    uint32_t *data=(uint32_t *)(cbs+4), *rxdata=data+0x20;
    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    smi_cs->pxldat = 1;
    smi_l->len = nsamp * sizeof(uint8_t);
    smi_cs->write = 0;
    enable_dma(DMA_CHAN_A);
    cbs[0].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_DEST_INC;
    cbs[0].tfr_len = nsamp * sizeof(uint8_t);
    cbs[0].srce_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[0].dest_ad = MEM_BUS_ADDR(mp, rxdata);
    return(rxdata);
}

// ADC DMA is complete, get data
int adc_dma_end(void *buff, uint8_t *data, int nsamp)
{
    uint8_t *bp = (uint8_t *)buff;
    int i;

    for (i=0; i<nsamp; i++)
    {
        *data++ = bp[i];
    }
    return(nsamp);
}

// Display bit values in register
void disp_reg_fields(char *regstrs, char *name, uint32_t val)
{
    char *p=regstrs, *q, *r=regstrs;
    uint32_t nbits, v;

    printf("%s %08X", name, val);
    while ((q = strchr(p, ':')) != 0)
    {
        p = q + 1;
        nbits = 0;
        while (*p>='0' && *p<='9')
            nbits = nbits * 10 + *p++ - '0';
        v = val & ((1 << nbits) - 1);
        val >>= nbits;
        if (v && *r!='_')
            printf(" %.*s=%X", q-r, r, v);
        while (*p==',' || *p==' ')
            p = r = p + 1;
    }
    printf("\n");
}



// Map GPIO, DMA and SMI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&smi_regs, (void *)SMI_BASE, PAGE_SIZE);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Free memory segments and exit
void terminate(int sig)
{
    printf("Closing\n");
    if (smi_regs.virt)
        *REG32(smi_regs, SMI_CS) = 0;
    stop_dma(DMA_CHAN_A);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&smi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    exit(0);
}

// Initialise SMI, given data width, time step, and setup/hold/strobe counts
// Step value is in nanoseconds: even numbers, 2 to 30
void init_smi(int width, int ns, int setup, int strobe, int hold)
{
    int divi = ns / 2;

    smi_cs  = (SMI_CS_REG *) REG32(smi_regs, SMI_CS);
    smi_l   = (SMI_L_REG *)  REG32(smi_regs, SMI_L);
    smi_a   = (SMI_A_REG *)  REG32(smi_regs, SMI_A);
    smi_d   = (SMI_D_REG *)  REG32(smi_regs, SMI_D);
    smi_dmc = (SMI_DMC_REG *)REG32(smi_regs, SMI_DMC);
    smi_dsr = (SMI_DSR_REG *)REG32(smi_regs, SMI_DSR0);
    smi_dsw = (SMI_DSW_REG *)REG32(smi_regs, SMI_DSW0);
    smi_dcs = (SMI_DCS_REG *)REG32(smi_regs, SMI_DCS);
    smi_dca = (SMI_DCA_REG *)REG32(smi_regs, SMI_DCA);
    smi_dcd = (SMI_DCD_REG *)REG32(smi_regs, SMI_DCD);
    smi_cs->value = smi_l->value = smi_a->value = 0;
    smi_dsr->value = smi_dsw->value = smi_dcs->value = smi_dca->value = 0;
    if (*REG32(clk_regs, CLK_SMI_DIV) != divi << 12)
    {
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | (1 << 5);
        usleep(10);
        while (*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) ;
        usleep(10);
        *REG32(clk_regs, CLK_SMI_DIV) = CLK_PASSWD | (divi << 12);
        usleep(10);
        *REG32(clk_regs, CLK_SMI_CTL) = CLK_PASSWD | 6 | (1 << 4);
        usleep(10);
        while ((*REG32(clk_regs, CLK_SMI_CTL) & (1 << 7)) == 0) ;
        usleep(100);
    }
    if (smi_cs->seterr)
        smi_cs->seterr = 1;
    smi_dsr->rsetup = smi_dsw->wsetup = setup;
    smi_dsr->rstrobe = smi_dsw->wstrobe = strobe;
    smi_dsr->rhold = smi_dsw->whold = hold;
    smi_dmc->panicr = smi_dmc->panicw = 8;
    smi_dmc->reqr = smi_dmc->reqw = REQUEST_THRESH;
    smi_dsr->rwidth = smi_dsw->wwidth = width;
}

// EOF
