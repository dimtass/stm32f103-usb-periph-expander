/*
 * comm_protocol.c
 *
 *  Created on: 21 May 2018
 *      Author: dimtass
 */
#include "comm_protocol.h"




int comm_validate(const uint8_t * packet, uint16_t len)
{
	int ret = -COMM_RET_ERR_INV_LENGTH;
	int i, tmp_i;
	uint16_t * p;
	struct comm_header * p_header = NULL;

//	for (i=0; i<len; i++) {
//		TRACE(("%02X,", 0xFF & packet[i]));
//	}

	TRACEL(TRACE_LEVEL_COMM, ("\n[comm_parse] length %d : %u\n", len, sizeof(*p_header)));
	// At least the struct com_header length is needed
	if (len >= sizeof(*p_header)) {
		// Scan for the preamble in the packet
		// use tmp_i as bool
		tmp_i = 0;			// default not found
		for (i=0; i<len-1; i++) {
			p = (uint16_t*) &packet[i];
			if (*p == COMM_PREAMBLE) {
				tmp_i = 1;	// preamble found
				break;
			}
		}

		// If preamble is not found, exit.
		if (!tmp_i) {
			TRACEL(TRACE_LEVEL_COMM, ("[comm_parse] Invalid preamble\n"));
			ret = -COMM_RET_ERR_INV_PREAMBLE;
			goto __err_exit;
		}

		// Grab the packet from the header
		p_header = (struct comm_header*) &packet[i];

		// Check the remaining length
		TRACEL(TRACE_LEVEL_COMM, ("[comm_parse] real lengths %u : %u\n", len-i, p_header->length));
		if ((len - i) >= p_header->length) {
			// Check for CRC
			uint16_t inc_crc = p_header->crc; // get CRC from the incoming data
			p_header->crc = 0;	// reset the crc for next calculation
			uint16_t crc = crc16((const uint8_t*) p_header, p_header->length, COMM_CRC_POLY);	// calculate incoming crc

			TRACEL(TRACE_LEVEL_COMM, ("[comm_parse] %d:%d\n", inc_crc, crc));
			if (crc == inc_crc) {
				// valid preamble, length and checksum
				ret = i;	// return the index where the data are located
			}
			else {
				TRACEL(TRACE_LEVEL_COMM, ("[comm_parse] crc_calc: 0x%04X, crc_in: 0x%04X\n", crc, inc_crc));
				ret = -COMM_RET_ERR_INV_CRC16;
				goto __err_exit;
			}
		}
	}

__err_exit:
	return ret;
}
