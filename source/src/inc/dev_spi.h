/*
 * dev_spi.h
 *
 *  Created on: 14 May 2018
 *      Author: Dimitris Tassopoulos
 */

#ifndef DEV_SPI_H_
#define DEV_SPI_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "platform_config.h"
#include "list.h"

enum en_spi_mode {
	DEV_SPI1_GPIOA,
	DEV_SPI1_GPIOB,
	DEV_SPI2
};

#define DECLARE_SPI_CHANNEL(NAME, OWNER, CHANNEL) \
	struct dev_spi NAME = { \
		.owner = OWNER, \
		.channel = CHANNEL, \
	}

struct dev_spi {
	struct dev_spi_module * owner;
	uint8_t 		channel;
	/* all below are filled in the module */
	SPI_TypeDef * 	spi;
	GPIO_TypeDef *  port;
	uint16_t		miso;
	uint16_t		mosi;
	uint16_t		sck;
	uint16_t		nss;
	DMA_Channel_TypeDef * dma_tx_ch;
	uint32_t		dma_tx_flags;
	DMA_Channel_TypeDef * dma_rx_ch;
	uint32_t		dma_rx_flags;
	struct list_head list;
};

#define DECLARE_MODULE_SPI(NAME, MODE) \
	struct dev_spi_module NAME = { \
		.mode = MODE, \
	}

struct dev_spi_module {
	enum en_spi_mode mode;
	struct list_head list;
};

void dev_spi_init(struct dev_spi_module * dev);

void* dev_spi_probe(struct dev_spi * spi);

void dev_spi_remove(struct dev_spi * spi);

int dev_spi_send(struct dev_spi * spi, uint8_t * data, size_t data_len);

int dev_spi_recv(struct dev_spi * spi, uint8_t * data, size_t data_len);

uint16_t dev_spi_send_recv(struct dev_spi * spi, uint8_t * data_out, uint16_t data_out_len, uint8_t * data_in, uint16_t data_in_len);

#endif /* DEV_SPI_H_ */
