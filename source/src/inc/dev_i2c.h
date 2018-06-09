/*
 * dev_i2c.h
 *
 *  Created on: 14 May 2018
 *      Author: dimtass
 */

#ifndef DEV_I2C_H_
#define DEV_I2C_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

struct dev_i2c_module {

};

struct dev_i2c {
	I2C_InitTypeDef * i2c;

};

#endif /* DEV_I2C_H_ */
