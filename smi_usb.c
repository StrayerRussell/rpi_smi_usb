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
#include <stdbool.h>
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
#define SMI_TIMING        28, 8, 8, 8, 2, 2, 2   // 1.49Mhz TX 6Mhz RX
#endif

#define REQUEST_THRESH  2   // DMA request threshold
#define DMA_CHAN        10  // DMA channel to use
			    //
// Number of samples to be captured
#define NSAMPLES    128

// Bus state definitions for USB
#define JSTATE 0x01
#define KSTATE 0x10
#define SEZERO 0x00
#define SEONE  0x11

// PID Definitions for USB LS
// all PID are pre reversed and have reversed compliment appended
// Token
#define OUT      0x87 //0x01
#define IN       0x96 //0x09
#define SOF      0xA5 //0x05
#define SETUP    0xB4 //0x0D
// Data
#define DATA0    0xC3 //0x03
#define DATA1    0xD2 //0x0B
#define DATA2    0xE1 //0x07
#define MDATA    0xF0 //0x0F
// Handshake
#define ACK      0x4B //0x02
#define NAK      0x5A //0x0A
#define STALL    0x78 //0x0E
#define NYET     0x69 //0x06
// Special  
#define PRE      0x3C //0x0C
#define ERR      0x3C //0x0C
#define SPLIT    0x1E //0x08
#define PING     0x2D //0x04
// Parse Assistance
#define PARSEERR 0x00


// User defined paramaters

#define ARRAY_SIZE     256     //Size of data array
#define SMI_DATA_WIDTH 8       //8-bit or 16-bit SMI transfers

static uint8_t syncData[8] = {16,1,16,1,16,1,16,16}, eopData[4] = {0,0,1,1};
static uint8_t ackData[32] = {1,16,1,16,1,16,16,16,1,1,1,16,16,1,16,16,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1}; //Pads to 32 samples
static uint8_t setAddrData[8] = {0x00,0x05,0x0C,0x00,0x00,0x00,0x00,0x00};
static uint8_t reqConfData[8] = {0x80,0x06,0x00,0x01,0x00,0x00,0x12,0x00};

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

uint8_t prepBuff[ARRAY_SIZE], packetBuff[ARRAY_SIZE];
uint8_t rxBuff[1024];

#define VC_MEM_SIZE         (PAGE_SIZE + ARRAY_SIZE * sizeof(uint8_t))

void *smiBuff;                       // Pointer to uncached Tx data buffer
uint32_t *txLen, *rxLen;

void map_devices(void);
void fail(char *s);
void terminate(int sig);
void init_smi(int width, int ns, int wsetup, int whold, int wstrobe, int rsetup, int rhold, int rstrobe);
void run_smi(MEM_MAP *mp, uint16_t txCnt, uint16_t rxCnt, bool hostAck);
int parse_rx_data(void *buff, uint8_t *data, int nsamp);
uint8_t reverse_bits(uint8_t input);
void swap_bytes(void *data, int len);
uint8_t crc5(unsigned short);
uint16_t crc16(uint8_t *data, int len);
uint8_t enc_nrzi(uint8_t *outbuffer, uint8_t *inbuffer, int len, int offset);
uint8_t create_token_packet (uint8_t *outbuffer, uint8_t pid, uint8_t addr, uint8_t endpt);
uint8_t create_data_packet(uint8_t *outbuffer, uint8_t pid, uint8_t *data, int len);
// create handshake packet. (hack: pass PID into nrzi encode with length of one)
uint32_t *setup_smi_dma(MEM_MAP *mp, uint32_t **txSizePtr, uint32_t **rxSizePtr);
void dump_buffer(uint8_t *buffer, int len);
void mode_word(uint32_t *wp, int n, uint32_t mode);
int run_transcieve_cycle(void *outbuffer, uint8_t *prepbuffer, uint8_t *inbuffer, int outLen, int inLen, bool sendAck);
void reset_bus();
uint8_t findMode(uint8_t *buff, int size);

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
    
    uint32_t len = 0, lenTwo = 0;
    signal(SIGINT, terminate);
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE); //map GPIO, DMA, and SMI registers into virtual mem (user space)
    init_smi(SMI_DATA_WIDTH, SMI_TIMING); //configure SMI registers with proper settings
    smiBuff=setup_smi_dma(&vc_mem, &txLen, &rxLen); //configure DMA control blocks

    reset_bus(); //Trigger USB bus reset (required for connected devices to respond to commands)

    // Set Device Address
    printf("\n--Set Device Address--\n");
    len = create_token_packet(packetBuff, SETUP, 0x00, 0x00);
    len = enc_nrzi(prepBuff, packetBuff, len, 0);
    lenTwo = create_data_packet(packetBuff, DATA0, setAddrData, 8);
    len = enc_nrzi(prepBuff, packetBuff, lenTwo, len);
    len = run_transcieve_cycle(smiBuff, prepBuff, rxBuff, len, NSAMPLES, false);

    // Check Device Address
    printf("\n--Check Device Address--\n");
    len = create_token_packet(packetBuff, IN, 0x00, 0x00);
    len = enc_nrzi(prepBuff, packetBuff, len, 0);
    len = run_transcieve_cycle(smiBuff, prepBuff, rxBuff, len, 180, true);

    // Set Device Configuration
    printf("\n--Request Device Descriptor--\n");
    len = create_token_packet(packetBuff, SETUP, 0x0C, 0x00);
    len = enc_nrzi(prepBuff, packetBuff, len, 0);
    lenTwo = create_data_packet(packetBuff, DATA0, reqConfData, 8);
    len = enc_nrzi(prepBuff, packetBuff, lenTwo, len);
    len = run_transcieve_cycle(smiBuff, prepBuff, rxBuff, len, NSAMPLES, false);

    // Check Device Address
    printf("\n--Get Desc Chunk One--\n");
    len = create_token_packet(packetBuff, IN, 0x0C, 0x00);
    len = enc_nrzi(prepBuff, packetBuff, len, 0);
    len = run_transcieve_cycle(smiBuff, prepBuff, rxBuff, len, 420, true);
    
    // Check Device Address Cont
    printf("\n--Get Desc Chunk Two--\n");
    len = create_token_packet(packetBuff, IN, 0x0C, 0x00);
    len = enc_nrzi(prepBuff, packetBuff, len, 0);
    len = run_transcieve_cycle(smiBuff, prepBuff, rxBuff, len, 420, true);

    // Check Device Address Cont
    printf("\n--Get Desc Chunk Three--\n");
    len = create_token_packet(packetBuff, IN, 0x0C, 0x00);
    len = enc_nrzi(prepBuff, packetBuff, len, 0);
    len = run_transcieve_cycle(smiBuff, prepBuff, rxBuff, len, 420, true);

    terminate(0);
    return(0);
}


int run_transcieve_cycle(void *trxbuffer, uint8_t *outbuffer, uint8_t *inbuffer, int outLen, int inLen, bool sendAck) {
    //dump_buffer(outbuffer, outLen);
    swap_bytes(outbuffer, outLen); // Fixes byte ordering issue
    memcpy((uint8_t*)trxbuffer, outbuffer, outLen); // Copy data from prep buffer to DMA transmission buffer.
    run_smi(&vc_mem, outLen, inLen, sendAck); // Actually start the SMI transfer
    dump_buffer(trxbuffer, inLen);
    printf("<--------------------------------------->\n");
    inLen = parse_rx_data(trxbuffer, inbuffer, inLen); // Crop RX Buffer down to actual packet content.
    //dump_buffer(inbuffer, inLen);
    printf("\n");
    return inLen;
}

// Set up SMI transfers using DMA
// potential garbage collection issues with "dmabuffer"
uint32_t *setup_smi_dma(MEM_MAP *mp, uint32_t **txSizePtr, uint32_t **rxSizePtr) {
    DMA_CB *cbs=mp->virt;
    uint32_t *data=(uint32_t *)(cbs+15), *modes=data+0x10;
    uint32_t *smiWriteVal=data+0x40, *smiReadVal=data+0x44, *smiTxLen=data+0x48, *smiRxLen=data+0x4C;
    uint32_t *ackbuffer=data+0x50, *dmabuffer=data+0x64, i;
    // Get current mode register values, first two indexes are reserved for next step
    for (i=0; i<2; i++)
        modes[i] = modes[i+2] = *REG32(gpio_regs, GPIO_MODE0 + i*4);

    memcpy((uint8_t*)ackbuffer, ackData, 32); //populate ackbuffer with precomputed ACK packet

    //dump_buffer(ackData, 32);
    //printf("\n");
    //dump_buffer((uint8_t*)ackbuffer, 32);
    //printf("\n");
    
    *txSizePtr = smiTxLen;
    *rxSizePtr = smiRxLen;

    *smiTxLen = 32;
    //printf("SMi TX Len: %d\n", *smiTxLen);

    mode_word(&modes[0], 8, GPIO_ALT1);// GPIO 8: Ctrl register 0 position 8
    mode_word(&modes[1], 2, GPIO_ALT1);// GPIO 12: Ctrl register 1 position 2  

    smi_dmc->dmaen = 1;
    smi_cs->enable = 1;
    smi_cs->clear = 1;
    smi_cs->pxldat = 1;
    smi_cs->write = 0;
    *smiReadVal = smi_cs->value | 0x08;
    smi_cs->write = 1;
    *smiWriteVal = smi_cs->value | 0x08;

    printf("readVal: %x writeVal: %x\n", *smiReadVal, *smiWriteVal);
 
    enable_dma(DMA_CHAN);
    // Set GPIO 8 and 12 to SMI mode (ALT1)
    cbs[11].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[11].tfr_len = 8; // Spans both MODE0 and MODE1 registers
    cbs[11].srce_ad = MEM_BUS_ADDR(mp, modes);
    cbs[11].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0);
    cbs[11].next_cb = MEM_BUS_ADDR(mp, &cbs[10]);

    // Set SMI to write mode and set start bit
    cbs[10].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[10].tfr_len = 4;
    cbs[10].srce_ad = MEM_BUS_ADDR(mp, smiWriteVal);
    cbs[10].dest_ad = REG_BUS_ADDR(smi_regs, SMI_CS);
    cbs[10].next_cb = MEM_BUS_ADDR(mp, &cbs[9]);

    // Output data in dmabuffer
    cbs[9].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC | DMA_WAIT_RESP;
    cbs[9].srce_ad = MEM_BUS_ADDR(mp, dmabuffer);
    cbs[9].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[9].next_cb = MEM_BUS_ADDR(mp, &cbs[8]);

    // Delay to buy SMI TX time to complete
    cbs[8].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[8].tfr_len = 200;
    cbs[8].srce_ad = MEM_BUS_ADDR(mp, dmabuffer);
    cbs[8].dest_ad = MEM_BUS_ADDR(mp, dmabuffer);
    cbs[8].next_cb = MEM_BUS_ADDR(mp, &cbs[7]);

    // Update SMI transaction len for RX step
    cbs[7].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[7].tfr_len = 4;
    cbs[7].srce_ad = MEM_BUS_ADDR(mp, smiRxLen);
    cbs[7].dest_ad = REG_BUS_ADDR(smi_regs, SMI_L);
    cbs[7].next_cb = MEM_BUS_ADDR(mp, &cbs[6]);

    // Set SMI to read mode and set start bit
    cbs[6].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[6].tfr_len = 4;
    cbs[6].srce_ad = MEM_BUS_ADDR(mp, smiReadVal);
    cbs[6].dest_ad = REG_BUS_ADDR(smi_regs, SMI_CS);
    cbs[6].next_cb = MEM_BUS_ADDR(mp, &cbs[5]);

    // Read data into dmabuffer
    cbs[5].ti = DMA_SRCE_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_DEST_INC;
    cbs[5].srce_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[5].dest_ad = MEM_BUS_ADDR(mp, dmabuffer);
    cbs[5].next_cb = MEM_BUS_ADDR(mp, &cbs[4]);

    // Insert Delay Here

    // Update SMI transaction len for ACK TX step
    cbs[4].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[4].tfr_len = 4;
    cbs[4].srce_ad = MEM_BUS_ADDR(mp, smiTxLen);
    cbs[4].dest_ad = REG_BUS_ADDR(smi_regs, SMI_L);
    cbs[4].next_cb = MEM_BUS_ADDR(mp, &cbs[3]);

    // Set SMI to write mode and set start bit
    cbs[3].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[3].tfr_len = 4;
    cbs[3].srce_ad = MEM_BUS_ADDR(mp, smiWriteVal);
    cbs[3].dest_ad = REG_BUS_ADDR(smi_regs, SMI_CS);
    cbs[3].next_cb = MEM_BUS_ADDR(mp, &cbs[2]);

    // Change RX Len to signal completion of read.
    cbs[2].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC | DMA_WAIT_RESP;
    cbs[2].tfr_len = 4;
    cbs[2].srce_ad = MEM_BUS_ADDR(mp, smiTxLen);
    cbs[2].dest_ad = MEM_BUS_ADDR(mp, smiRxLen);
    cbs[2].next_cb = MEM_BUS_ADDR(mp, &cbs[0]);

    // Transmit ACK : OPTIONAL
    cbs[1].ti = DMA_DEST_DREQ | (DMA_SMI_DREQ << 16) | DMA_CB_SRCE_INC | DMA_WAIT_RESP;
    cbs[1].tfr_len = 32 * sizeof(uint8_t); // Len of precal ACK packet
    cbs[1].srce_ad = MEM_BUS_ADDR(mp, ackbuffer);
    cbs[1].dest_ad = REG_BUS_ADDR(smi_regs, SMI_D);
    cbs[1].next_cb = MEM_BUS_ADDR(mp, &cbs[0]);

    // Set GPIO 8 and 12 to whatever mode they were in before (must be input to work properly)
    cbs[0].ti = DMA_CB_SRCE_INC | DMA_CB_DEST_INC;
    cbs[0].tfr_len = 8; // Spans both MODE0 and MODE1 registers
    cbs[0].srce_ad = MEM_BUS_ADDR(mp, &modes[2]);
    cbs[0].dest_ad = REG_BUS_ADDR(gpio_regs, GPIO_MODE0);

    return(dmabuffer);
}

// Start SMI DMA transfers
void run_smi(MEM_MAP *mp, uint16_t txCnt, uint16_t rxCnt, bool hostAck) {
    DMA_CB *cbs=mp->virt;

    uint32_t *data=(uint32_t *)(cbs+15), *smiRxLen=data+0x4C;
    cbs[2].next_cb = MEM_BUS_ADDR(mp, &cbs[hostAck]); // Bypasses DMA CBs 1-4 if no host ack required
    cbs[9].tfr_len = smi_l->len = txCnt * sizeof(uint8_t);
    cbs[5].tfr_len = *smiRxLen = rxCnt * sizeof(uint8_t);

    start_dma(mp, DMA_CHAN, &cbs[11], 0); // Enables DMA

    while (dma_active(DMA_CHAN)){
        usleep(100);
    }
}

// ADC DMA is complete, get data
int parse_rx_data(void *buff, uint8_t *data, int nsamp) {
    uint8_t *bp = (uint8_t *)buff, *tempBuff, crc=0, crcMask=0x80;
    int packetSize=0, parsedSize=0, seqOnes=0, i=0;
    //int jCnt=0, kCnt=0, zroCnt=0, oneCnt=0, i=0, maxCnt=0, maxVal=0;
    while(bp[i]==JSTATE) {
        i++;
    }
    printf("Rx Presamp Cnt: %d\n", i);
    bp+=i;
    do {
        data[packetSize] = findMode(bp, 4);
        bp+=4;
    } while(data[packetSize++]!=0);
    printf("Shabingus\n");
    packetSize-=2;
    for(i=7; i<15; i++) {
        if(data[i] == data[i+1]) {
            printf("1");
            crc = crc ^ crcMask;
        } else { 
            printf("0");
        }
        crcMask >>= 1;
    }
    printf("\n");
    switch(crc) {
        case ACK:
            printf("ACK: ");
            break;
        case DATA0:
        case DATA1:
            printf("DATA: ");
            break;
        default:
            printf("UNKNOWN PID: \n");
            break;
    }
    for(i=i; i<packetSize; i++) {
        if(seqOnes > 5) {
            i++;
            seqOnes=0;
        }
        if(data[i] == data[i+1]) {
            printf("1");
            seqOnes++;
        } else { 
            printf("0");
            seqOnes=0;
        }
        parsedSize++;
    }
    printf("\nParsed Size: %d\n", parsedSize);
    return(packetSize);
}

uint8_t findMode(uint8_t *buff, int size) {
    uint8_t maxVal=0, maxCnt=0, cnt, i, j;
    for(i=0; i<size; ++i) {
        cnt=0;
        for(j=0; j<size; ++j) {
            if(buff[j] == buff[i])
                ++cnt;
        }
        if(cnt > maxCnt) {
            maxCnt = cnt;
            maxVal = buff[i];
        }
    }
    return maxVal;
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
            res = (res >> 1) ^ 0x14; // 10100
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
    printf("\n");
}

// Get GPIO mode value into 32-bit word
void mode_word(uint32_t *wp, int n, uint32_t mode) {
    uint32_t mask = 7 << (n * 3);
    *wp = (*wp & ~mask) | (mode << (n * 3));
}

uint8_t enc_nrzi(uint8_t *outbuffer, uint8_t *inbuffer, int len, int offset) {
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
    
    /*
    for(i=0; i<8; i++) {
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

uint8_t create_data_packet(uint8_t *outbuffer, uint8_t pid, uint8_t *data, int len) {
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
uint8_t create_token_packet (uint8_t *outbuffer, uint8_t pid, uint8_t addr, uint8_t endpt) {
    // Token Packet Structure
    // |SYNC|PID|ADDR|ENDP|CRC5|EOP|

    uint32_t packet, crc, i=0;

    crc = (reverse_bits(crc5((endpt << 7) | addr))) >> 3;
    addr = reverse_bits(addr) >> 1;
    endpt = reverse_bits(endpt) >> 4;

    packet = ((addr << 4) + endpt);
    packet = (((pid << 16) + (packet << 5)) + crc);
    
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
