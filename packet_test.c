#include <stdio.h>
#include <string.h>
#include <stdint.h>

uint8_t syncData[8] = {16,1,16,1,16,1,16,16}, eopData[3] = {0,0,1}, tx_data[40];

//void *txdata;                       // Pointer to uncached Tx data buffer

uint8_t reverse_bits (uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; // Step 1: Swap nibbles
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2; // Step 2: Swap pairs within nibbles
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1; // Step 3: Swap adjacent bits
    return b;
}

uint8_t crc5(unsigned short input)
{
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

uint8_t enc_token_nrzi(uint32_t token, uint8_t *buffer) {
    uint8_t i = 0, seqOnes = 0, txSize = 11; // 8 bits for sync and 3 for EOP
    uint32_t tokenMask = 8388608; //all zeroes except for a 1 in the 24th bit.
    // Prefix Sync and Append EOP
    // Remember potential for quadruple timing workaround
    
    // Populate sync signals at start of buffer
    memcpy(buffer, syncData, 8);
    buffer += 8;
    // Primary contents
    for(i=0; i<24; i++) {
        if(seqOnes > 5) {
            *buffer = (*(buffer - 1) ^ 0x11);
	    seqOnes = 0;
	    buffer++;
	    txSize++;
	}
        if(token & tokenMask) {
            *buffer = *(buffer - 1);
	    seqOnes++;
	} else {
	    *buffer = (*(buffer - 1) ^ 0x11);
	    seqOnes = 0;
	}
	buffer++;
	txSize++;
	tokenMask = tokenMask >> 1;
    }
    // Populate EOP at end of buffer
    memcpy(buffer, eopData, 3);
    return txSize; //return size of total packet in bytes(including sync and eop)
}

uint32_t create_token_packet (uint8_t pid, uint8_t addr, uint8_t endpt) {
    // Token Packet Structure
    // |SYNC|PID|ADDR|ENDP|CRC5|EOP|

    uint8_t pid_mask=15, addr_mask=127, endpt_mask=15;
    uint32_t packet, crc;

    crc = (reverse_bits(crc5((endpt << 7) | addr))) >> 3;

    printf("Crc: %d\n",crc);

    pid = reverse_bits(pid & pid_mask);
    pid = (pid | (pid >> 4 ^ pid_mask));

    addr = reverse_bits(addr) >> 1;

    endpt = reverse_bits(endpt & endpt_mask) >> 4;

    packet = (addr << 4) | endpt;

    packet = (pid << 16) | (packet << 5) + crc;

    return packet;
}

int main() {
	uint8_t len, i;
	//printf("%d\n",create_token_packet(9,24,1));
	len = enc_token_nrzi(create_token_packet(9,24,1), tx_data);
	for(i=0; i<len; i++) {
	    switch(tx_data[i]) {
	        case 0:
		    printf("0");
		    break;
                case 1:
		    printf("J");
		    break;
		case 16:
		    printf("K");
		    break;
		default:
		    printf("Undefined\n");
	    }
            //printf("%d\n", tx_data[i]);
	}
	printf("Len: %d\n", len);
	//uint8_t endpt = 1, addr = 24;
	//printf("crc: %x\n",crc5((endpt << 7) | addr));
	return 0;
}
