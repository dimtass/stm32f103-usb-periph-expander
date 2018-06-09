/*
 * crc16.h
 *
 *  Created on: 21 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef CRC16_H_
#define CRC16_H_

static inline uint16_t crc16(const uint8_t* data, size_t length, uint16_t poly)
{
    uint8_t x;
    uint16_t crc = poly;

    while (length--)
    {
        x = crc >> 8 ^ *data++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t) (x << 12)) ^ ((uint16_t) (x << 5)) ^ ((uint16_t) x);
    }
    return crc;
}

#endif /* CRC16_H_ */
