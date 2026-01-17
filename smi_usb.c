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

// SMI_TIMING timebase, wsetup wstrobe, whold, rsetup, rhold, rstrobe

#if PHYS_REG_BASE==PI_4_REG_BASE                // Timings for RPi v4 (1.5 GHz)
#define SMI_TIMING        1, 250, 500, 250, 31, 63, 31   // 1000 (roughly 1.5MHz)
#else                                           // Timings for RPi v0-3 (1 GHz)
#define SMI_TIMING        28, 8, 8, 8, 2, 2, 2   // 670 (roughly 1.49MHz)
#endif

#define REQUEST_THRESH  2   // DMA request threshold
#define DMA_CHAN        10  // DMA channel to use
			    //
// Number of samples to be captured
#define NSAMPLES        1024

// Bus state definitions for USB
#define JSTATE 0x01
#define KSTATE 0x10
#define SEZERO 0x00
#define SEONE  0x11

// PID Definitions for USB LS
// all PID are pre reversed and have reversed compliment appended
// Token
#define OUT     0x87 //0x01
#define IN      0x96 //0x09
#define SOF     0xA5 //0x05
#define SETUP   0xB4 //0x0D
// Data
#define DATA0   0xC3 //0x03
#define DATA1   0xD2 //0x0B
#define DATA2   0xE1 //0x07
#define MDATA   0xF0 //0x0F
// Handshake
#define ACK     0x4B //0x02
#define NAK     0x5A //0x0A
#define STALL   0x78 //0x0E
#define NYET    0x69 //0x06
// Special  
#define PRE     0x3C //0x0C
#define ERR     0x3C //0x0C
#define SPLIT   0x1E //0x08
#define PING    0x2D //0x04


// User defined paramaters

#define ARRAY_SIZE     256     //Size of data array
#define SMI_DATA_WIDTH 8       //8-bit or 16-bit SMI transfers

uint8_t syncData[8] = {16,1,16,1,16,1,16,16}, eopData[4] = {0,0,1,1};//{0,0,1,1};
uint8_t packetData[8] = {0x00,0x05,0x0C,0x00,0x00,0x00,0x00,0x00};

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

uint8_t tx_data[ARRAY_SIZE], packetBuffer[ARRAY_SIZE];
uint8_t sample_data[NSAMPLES];

#define VC_MEM_SIZE         (PAGE_SIZE + ARRAY_SIZE * sizeof(uint8_t))

void *txdata;                       // Pointer to uncached Tx data buffer
void *rxbuff;

void map_devices(void);
void fail(char *s);
void terminate(int sig);
void init_smi(int width, int ns, int wsetup, int whold, int wstrobe, int rsetup, int rhold, int rstrobe);
void start_smi(MEM_MAP *mp);
int adc_dma_end(void *buff, uint8_t *data, int nsamp);
uint8_t reverse_bits(uint8_t input);
void swap_bytes(void *data, int len);
uint8_t crc5(unsigned short);
uint16_t crc16(uint8_t *data, int len);
uint8_t enc_nrzi(uint8_t *outbuffer, uint8_t *inbuffer, uint32_t len, uint32_t offset);
uint8_t create_token_packet (uint8_t pid, uint8_t addr, uint8_t endpt, uint8_t *outbuffer);
uint8_t create_data_packet(uint8_t pid, uint8_t *data, uint8_t len, uint8_t *outbuffer);
// create handshake packet. (hack: pass PID into nrzi encode with length of one)
uint32_t *setup_smi_dma(MEM_MAP *mp, int nsamp, int mode);
void dump_buffer(uint8_t *buffer, int len);
void mode_word(uint32_t *wp, int n, uint32_t mode);
void reset_bus();

// USB GPIO PIN MAPPINGS
// (D- 8 (SMI0), D+ 12 (SMI4))

// Values for different USB Low-Speed states
// 0  - D+ Lo D- Lo : Single Ended 0 : 
// 1  - D+ Lo D- Hi : Differential 0 : Low Speed J State: Low  Speed Idle
// 16 - D+ Hi D- Lo : Differential 1 : Low Speed K State: Full Speed Idle
// 17 - D+ Hi D- Hi : Single Ended 1 : 
//
// KJKJKJKK
// 16,1,16,1,16,1,16,16

int main(int argc, char *argv[]) {
    
    int len = 0, lenTwo = 0, waitTime = 0;
    signal(SIGINT, terminate);
    /*
    for(int i=0; i < ARRAY_SIZE; i++) {
	    tx_data[i] = 0;
    }
    */
    // testing values of (IN, 24, 1) align with the Ben Eater's "How Does a USB Keyboard Work" (13, 0, 0)
    map_devices();
    init_smi(SMI_DATA_WIDTH, SMI_TIMING); //setup SMI registers for DMA TX.
    map_uncached_mem(&vc_mem, VC_MEM_SIZE); //map GPIO, DMA, and SMI registers into virtual mem (user space)
    reset_bus(); //Trigger USB bus reset (required for connected devices to respond to commands)
    len = create_token_packet(SETUP,0,0, packetBuffer);
    len = enc_nrzi(tx_data, packetBuffer, len, 0);
    lenTwo = create_data_packet(DATA0, packetData, 8, packetBuffer);
    len = enc_nrzi(tx_data, packetBuffer, lenTwo, len);
    dump_buffer(tx_data, len);
    printf("\nLen: %d\n", len);
    swap_bytes(tx_data, len); //fixes byte ordering issue
    txdata = setup_smi_dma(&vc_mem, len, 1); //setup DMA for SMI DMA TX.
    memcpy((uint8_t*)txdata, tx_data, len); //copy data from prep buffer to DMA transmission buffer.
    start_smi(&vc_mem); //actually start the SMI transfer
    while (!smi_cs->done){waitTime++;};
    smi_cs->write = 0;
    smi_cs->start = 1;
    smi_l->len = 1024 * sizeof(uint8_t);
    printf("Wait Time: %d\n", waitTime);
    len = adc_dma_end(txdata, sample_data, NSAMPLES);
    dump_buffer(sample_data, len);
    printf("\n\n\n");
    terminate(0);
    return(0);
}
/*

IMPORTANT: try and speed this up by using DMA to write directly to SMI cs register to initialize and set modes

*/
// Set up SMI transfers using DMA
// potential garbage collection issues with "dmabuffer"
uint32_t *setup_smi_dma(MEM_MAP *mp, int nsamp, int mode) {
    DMA_CB *cbs=mp->virt;
    uint32_t *data=(uint32_t *)(cbs+6), *modes=data+0x10;
    uint32_t *dmabuffer=data+0x40, i;
    // Get current mode register values, first two indexes are reserved for next step
    for (i=0; i<2; i++)
        modes[i] = modes[i+2] = *REG32(gpio_regs, GPIO_MODE0 + i*4);

    mode_word(&modes[0], 8, GPIO_ALT1);// GPIO 8: Ctrl register 0 position 8
    mode_word(&modes[1], 2, GPIO_ALT1);// GPIO 12: Ctrl register 1 position 2  
    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    smi_cs->pxldat = 1;
    smi_l->len = nsamp * sizeof(uint8_t);
    smi_cs->write = mode;
    enable_dma(DMA_CHAN);
    // Set GPIO 8 and 12 to SMI mode (ALT1)
    cbs[0].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[0].tfr_len = 8; // Spans both MODE0 and MODE1 registers
    cbs[0].srce_ad = MEM_BUS_ADDR(mp, modes);
    cbs[0].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0);
    cbs[0].next_cb = MEM_BUS_ADDR(mp, &cbs[1]);

    cbs[1].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC | DMA_WAIT_RESP;
    cbs[1].tfr_len = nsamp * sizeof(uint8_t);
    cbs[1].srce_ad = MEM_BUS_ADDR(mp, dmabuffer);
    cbs[1].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[1].next_cb = MEM_BUS_ADDR(mp, &cbs[2]);

    cbs[2].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_DEST_INC;
    cbs[2].tfr_len = 1024 * sizeof(uint8_t);
    cbs[2].srce_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[2].dest_ad = MEM_BUS_ADDR(mp, dmabuffer);
    cbs[2].next_cb = MEM_BUS_ADDR(mp, &cbs[3]);

    // Set GPIO 8 and 12 to whatever mode they were in before (must be input to work properly)
    cbs[3].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC;
    cbs[3].tfr_len = 8; // Spans both MODE0 and MODE1 registers
    cbs[3].srce_ad = MEM_BUS_ADDR(mp, &modes[2]);
    cbs[3].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0);

    return(dmabuffer);
}

// Start SMI DMA transfers
void start_smi(MEM_MAP *mp) {
    DMA_CB *cbs=mp->virt;

    start_dma(mp, DMA_CHAN, &cbs[0], 0);
    smi_cs->start = 1;
}

// ADC DMA is complete, get data
int adc_dma_end(void *buff, uint8_t *data, int nsamp) {
    uint8_t *bp = (uint8_t *)buff;
    int packetSize=0, i=0;
    while(bp[i]==JSTATE) {
        i++;
    }
    do {
        if((bp[i]==bp[i+2])||(bp[i]==bp[i+3])) {
            *data = bp[i];
        }
        i+=4;
        packetSize++;
    } while(*data++!=0);
    return(packetSize);
}

uint8_t reverse_bits (uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; // Step 1: Swap nibbles
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2; // Step 2: Swap pairs within nibbles
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1; // Step 3: Swap adjacent bits
    return b;
}

// Swap adjacent bytes in transmit data
void swap_bytes(void *data, int len) {
    uint16_t *wp = (uint16_t *)data;
    len = (len + 1) / 2;
    while (len-- > 0)
    {
        *wp = __builtin_bswap16(*wp);
        wp++;
    }
}

uint8_t crc5(unsigned short input) {
    unsigned char res = 0x1f;
    unsigned char b;
    int i;
    for (i = 0;  i < 11;  ++i) {
        b = (input ^ res) & 1;
        input >>= 1;
        if (b) {
            res = (res >> 1) ^ 0x14;        /* 10100 */
        } else {
            res = (res >> 1);
        }
    }
    return res ^ 0x1f;
}

uint16_t crc16(uint8_t *data, int len) {
    uint16_t crc = 0xFFFF;
    uint8_t b, byte, i;
    while (len--) {
        byte = *data++;
        for(i=0; i<8; i++) {
            b = (byte ^ crc) & 1;
            byte >>= 1;
            crc >>= 1;
            if (b) {
                crc ^=0xA001;
            }
        }
    }
    return crc ^ 0xFFFF;
}


void dump_buffer(uint8_t *buffer, int len) {
    for(int i = 0; i < len; i++) {
        switch(buffer[i]) {
            case 0:
                printf("0");
                break;
            case JSTATE:
                printf("J");
                break;
            case KSTATE:
                printf("K");
                break;
            case 17:
                printf("1");
                break;
            default:
            printf("Undefined Val: %d\n", buffer[i]);
        }
    }
}

// Get GPIO mode value into 32-bit word
void mode_word(uint32_t *wp, int n, uint32_t mode) {
    uint32_t mask = 7 << (n * 3);
    *wp = (*wp & ~mask) | (mode << (n * 3));
}

uint8_t enc_nrzi(uint8_t *outbuffer, uint8_t *inbuffer, uint32_t len, uint32_t offset) {
    uint8_t i = 0, j = 0, seqOnes = 0, txSize = 12; // 8 bits for sync and 3 for EOP
    uint8_t byteMask = 0x80;
    if(offset != 0) {
        outbuffer += offset; // Skip over offset, allows for queuing multiple packets.
        txSize += offset;
    }
    memcpy(outbuffer, syncData, 8); // Populate SYNC at start of buffer.
    outbuffer += 8; // Skips over SYNC above
    for(i=0; i<len; i++) {
        byteMask = 0x80;
        for(j=0; j<8; j++) {
            if(seqOnes > 5) {
                *outbuffer = (*(outbuffer - 1) ^ 0x11);
	            seqOnes = 0;
	            outbuffer++;
	            txSize++;
	        }
            if(inbuffer[i] & byteMask) { // NRZI represents 1 as no transition
                *outbuffer = *(outbuffer - 1);
	            seqOnes++;
	        } else { // NRZI represents 0 as a transition
	            *outbuffer = (*(outbuffer - 1) ^ 0x11);
	            seqOnes = 0;
	        }
	        outbuffer++;
	        txSize++;
	        byteMask = byteMask >> 1;
        }
    }
    // Populate EOP at end of buffer
    memcpy(outbuffer, eopData, 4);
    outbuffer+=3; //above memcpy uses 3 bytes past buffer, this accounts for that

    // pads the buffer to keep the DMA busy, DMA is used to set pin mode 
    // and if the buffer is not padded it will change the mode before the
    // important data is done being output
    /*
    for(i=0; i<12; i++) {
        outbuffer++;
        txSize++;
 	    *outbuffer = JSTATE;
    }
    */
    if(txSize%2) { //ensures an even number of bytes in TX buffer (for purposes of byte swap)
        outbuffer++;
        txSize++;
        *outbuffer = JSTATE;
    }
    return txSize; //return size of total packet in bytes(including sync and eop)
}

uint8_t create_data_packet(uint8_t pid, uint8_t *data, uint8_t len, uint8_t *outbuffer) {
    uint8_t i;
    uint16_t crc;
    
    *outbuffer = pid;
    crc = crc16(data, len);

    for(i=0; i<len; i++) {
        outbuffer++;
        *outbuffer = reverse_bits(data[i]);
    }
    for(i=0; i<2; i++) {
        outbuffer++;
        *outbuffer = reverse_bits(crc & 0x00FF);
        crc >>= 8;
    }
    return len+3;
}
uint8_t create_token_packet (uint8_t pid, uint8_t addr, uint8_t endpt, uint8_t *outbuffer) {
    // Token Packet Structure
    // |SYNC|PID|ADDR|ENDP|CRC5|EOP|

    uint32_t packet, crc, i=0;

    crc = (reverse_bits(crc5((endpt << 7) | addr))) >> 3;
    addr = reverse_bits(addr) >> 1;
    endpt = reverse_bits(endpt) >> 4;

    packet = ((addr << 4) + endpt);
    packet = (((pid << 16) + (packet << 5)) + crc);
    /*
    for(i=0; i<3; i++) {
        outbuffer[2-i] = packet & 0xFF;
        packet = packet >> 8;
    }
    */

    i=3;
    while(i--) {
        outbuffer[i] = packet & 0xFF;
        packet = packet >> 8; 
    }

    return 3;
}

// Map GPIO, DMA and SMI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void) {
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&smi_regs, (void *)SMI_BASE, PAGE_SIZE);
}

// Catastrophic failure in initial setup
void fail(char *s) {
    printf(s);
    terminate(0);
}

// Free memory segments and exit
void terminate(int sig) {
    printf("Closing\n");
    gpio_mode(8, GPIO_IN);
    gpio_mode(12, GPIO_IN);
    gpio_pull(8, 0);
    gpio_pull(12, 0);  
    if (smi_regs.virt)
        *REG32(smi_regs, SMI_CS) = 0;
    stop_dma(DMA_CHAN);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&smi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    exit(0);
}

void reset_bus() {
    gpio_mode(8, GPIO_OUT); // Set pins to outputs
    gpio_mode(12, GPIO_OUT);
    gpio_out(8, 0); // Set pins to LOW state
    gpio_out(12, 0);
    usleep(13000); // Hold bus low for 13ms (USB spec states 10-20ms for reset)
    gpio_mode(8, GPIO_IN); // Sets pins to inputs
    gpio_mode(12, GPIO_IN);
    gpio_pull(8, 0); // Enables weak pulldown resistors
    gpio_pull(12, 0);
    usleep(10000); // Allow up to 10ms for attached devices to "recover" (also in USB spec)
}

// Initialise SMI, given data width, time step, and setup/hold/strobe counts
// Step value is in nanoseconds: even numbers, 2 to 30
void init_smi(int width, int ns, int wsetup, int wstrobe, int whold, int rsetup, int rstrobe, int rhold) {
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
    smi_dsr->rsetup = rsetup;
    smi_dsw->wsetup = wsetup;
    smi_dsr->rstrobe = rstrobe;
    smi_dsw->wstrobe = wstrobe;
    smi_dsr->rhold = rhold;
    smi_dsw->whold = whold;
    smi_dmc->panicr = smi_dmc->panicw = 8;
    smi_dmc->reqr = smi_dmc->reqw = REQUEST_THRESH;
    smi_dsr->rwidth = smi_dsw->wwidth = width;
}

// EOF
