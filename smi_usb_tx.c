// Raspberry Pi WS2812 LED driver using SMI
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
// v0.01 JPB 16/7/20 Adapted from rpi_smi_adc_test v0.06
// v0.02 JPB 15/9/20 Addded RGB to GRB conversion
// v0.03 JPB 15/9/20 Added red-green flashing
// v0.04 JPB 16/9/20 Added test mode
// v0.05 JPB 19/9/20 Changed test mode colours
// v0.06 JPB 20/9/20 Outlined command-line data input
// v0.07 JPB 25/9/20 Command-line data input if not in test mode
// v0.08 JPB 26/9/20 Changed from 4 to 3 pulses per LED bit
//                   Added 4-bit zero preamble
//                   Added raw Tx data test
// v0.09 JPB 27/9/20 Added 16-channel option
// v0.10 JPB 28/9/20 Corrected Pi Zero caching problem
// v0.11 JPB 29/9/20 Added enable_dma before transfer (in case still active)
//                   Corrected DMA nsamp value (was byte count)
// v0.12 JPB 26/5/21 Corrected transfer length for 16-bit mode

#include <stdio.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include "rpi_dma_utils.h"
#include "rpi_smi_defs.h"

#if PHYS_REG_BASE==PI_4_REG_BASE        // Timings for RPi v4 (1.5 GHz)
#define SMI_TIMING       10, 15, 30, 15    // 400 ns cycle time
#else                                   // Timings for RPi v0-3 (1 GHz)
#define SMI_TIMING       10, 17, 33, 17   // 670 ns cycle time (roughly 1.49MHz)
#endif

#define TX_TEST         0   // If non-zero, use dummy Tx data
#define LED_D0_PIN      8   // GPIO pin for D0 output
#define REQUEST_THRESH  2   // DMA request threshold
#define DMA_CHAN        10  // DMA channel to use

// User defined paramaters

#define ARRAY_SIZE     100     //Size of data array
#define SMI_DATA_WIDTH 8       //8-bit or 16-bit SMI transfers

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

uint8_t tx_data[ARRAY_SIZE];

#define VC_MEM_SIZE         (PAGE_SIZE + ARRAY_SIZE * sizeof(uint8_t))

uint8_t *txdata;                       // Pointer to uncached Tx data buffer

void map_devices(void);
void fail(char *s);
void terminate(int sig);
void init_smi(int width, int ns, int setup, int hold, int strobe);
void setup_smi_dma(MEM_MAP *mp, int nsamp);
void start_smi(MEM_MAP *mp);

// Values for different USB Low-Speed states
// 0  - D+ Lo D- Lo : Single Ended 0 : 
// 1  - D+ Lo D- Hi : Differential 0 : Low Speed J State
// 16 - D+ Hi D- Lo : Differential 1 : Low Speed K State
// 17 - D+ Hi D- Hi : Single Ended 1 : 
//
// KJKJKJKK
// 16,1,16,1,16,1,16,16

int main(int argc, char *argv[])
{
    signal(SIGINT, terminate);

    for(int i=0; i < ARRAY_SIZE; i++) {
	tx_data[i] = i%2 ? 17 : 0; 
	//tx_data[i] = 16; 
    }

    map_devices();
    init_smi(SMI_DATA_WIDTH, SMI_TIMING);
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);
    setup_smi_dma(&vc_mem, ARRAY_SIZE);
    memcpy(txdata, tx_data, ARRAY_SIZE * sizeof(uint8_t));
    enable_dma(DMA_CHAN);
    start_smi(&vc_mem);
    usleep(10);
    while (dma_active(DMA_CHAN))
        usleep(10);
    terminate(0);
    return(0);
}

uint8_t reverse_bits (uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; // Step 1: Swap nibbles
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2; // Step 2: Swap pairs within nibbles
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1; // Step 3: Swap adjacent bits
    return b;
}
/*
void crc5 () {

}

void crc16 () {

}

void encode_nrzi() {
    // Prefix Sync and Append EOP
}

uint32_t create_token_packet (uint8_t pid, uint8_t addr, uint8_t endpt) {
    // Token Packet Structure
    // |SYNC|PID|ADDR|ENDP|CRC5|EOP|
    
    uint8_t pid_mask=15, addr_mask=127, endpt_mask=15, crc;
    uint32_t packet;
    
    pid = reverse_bits(pid & pid_mask);
    pid = (pid | (pid >> 4 ^ pid_mask));

    addr = reverse_bits(addr) >> 1;

    endpt = reverse_bits(endpt & endpt_mask) >> 4;

    packet = (addr << 4) + endpt;

    crc = reverse_bits(crc5(packet)) >> 3;

    packet = (pid << 16) + (packet << 5) + crc;

    return packet;
}
*/

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
    stop_dma(DMA_CHAN);
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

// Set up SMI transfers using DMA
void setup_smi_dma(MEM_MAP *mp, int nsamp)
{
    DMA_CB *cbs=mp->virt;

    txdata = (uint8_t *)(cbs + 1); //Data starts after control blocks
    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    smi_cs->pxldat = 1;
    smi_l->len = nsamp * sizeof(uint8_t);
    smi_cs->write = 1;
    enable_dma(DMA_CHAN);
    cbs[0].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC | DMA_WAIT_RESP;
    cbs[0].tfr_len = nsamp * sizeof(uint8_t);
    cbs[0].srce_ad = MEM_BUS_ADDR(mp, txdata);
    cbs[0].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
}

// Start SMI DMA transfers
void start_smi(MEM_MAP *mp)
{
    DMA_CB *cbs=mp->virt;

    start_dma(mp, DMA_CHAN, &cbs[0], 0);
    smi_cs->start = 1;
}

// EOF
