/*
 * dev_spi.c
 *
 *  Created on: 14 May 2018
 *      Author: Dimitris Tassopoulos
 */

#include "dev_spi.h"

#define SPI_MASTER_DR_Base           0x4001300C

LIST_HEAD(spi_list);

#define SPI_EXISTS(SPI, ITTERATOR) (SPI->channel == ITTERATOR)


void dev_spi_init(struct dev_spi_module * dev)
{
	INIT_LIST_HEAD(&spi_list);
}

static inline struct dev_spi * dev_spi_find_channel(uint8_t ch)
{
	if (!list_empty(&spi_list)) {
		struct dev_spi * spi_it = NULL;
		list_for_each_entry(spi_it, &spi_list, list) {
			if SPI_EXISTS(spi_it, ch) {
				/* found */
				return(spi_it);
			}
		}
	}
	return NULL;
}


void* dev_spi_probe(struct dev_spi * spi)
{
	if (!spi || !spi->owner) return NULL;
	if (dev_spi_find_channel(spi->channel)) return NULL;

	struct dev_spi_module * dev = spi->owner;
	struct dev_spi * new_spi = (struct dev_spi *) malloc(sizeof(struct dev_spi));
	memcpy(new_spi, spi, sizeof(struct dev_spi_module));
	if (dev->mode == DEV_SPI1_GPIOA) {
		new_spi->spi = SPI1;
		new_spi->port = GPIOA;
		new_spi->nss = GPIO_Pin_4;
		new_spi->sck = GPIO_Pin_5;
		new_spi->miso = GPIO_Pin_6;
		new_spi->mosi = GPIO_Pin_7;
		new_spi->dma_rx_ch = DMA1_Channel2;
		new_spi->dma_rx_flags = DMA1_FLAG_TC2;
		new_spi->dma_tx_ch = DMA1_Channel3;
		new_spi->dma_tx_flags = DMA1_FLAG_TC3;
		/* RCC configuration */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
	}
	else if (dev->mode == DEV_SPI1_GPIOB) {
		new_spi->spi = SPI1;
		new_spi->port = GPIOB;
		new_spi->sck = GPIO_Pin_3;
		new_spi->miso = GPIO_Pin_4;
		new_spi->mosi = GPIO_Pin_5;
		new_spi->dma_rx_ch = DMA1_Channel2;
		new_spi->dma_rx_flags = DMA1_FLAG_TC2;
		new_spi->dma_tx_ch = DMA1_Channel3;
		new_spi->dma_tx_flags = DMA1_FLAG_TC3;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
	}
	else if (dev->mode == DEV_SPI2) {
		new_spi->spi = SPI2;
		new_spi->port = GPIOB;
		new_spi->sck = GPIO_Pin_13;
		new_spi->miso = GPIO_Pin_14;
		new_spi->mosi = GPIO_Pin_15;
		new_spi->dma_rx_ch = DMA1_Channel4;
		new_spi->dma_rx_flags = DMA1_FLAG_TC4;
		new_spi->dma_tx_ch = DMA1_Channel5;
		new_spi->dma_tx_flags = DMA1_FLAG_TC5;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	}

	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = new_spi->sck | new_spi->mosi;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(new_spi->port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = new_spi->miso;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(new_spi->port, &GPIO_InitStructure);

    /* SPI configuration */
    SPI_InitTypeDef   SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 10;
	SPI_CalculateCRC(new_spi->spi, DISABLE);
    SPI_Init(new_spi->spi, &SPI_InitStructure);

    SPI_Cmd(new_spi->spi, ENABLE);

    /* DMA configuration */
    DMA_InitTypeDef	DMA_InitStructure;
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Initialize DMA struct */
	DMA_StructInit(&DMA_InitStructure);

	/* SPI_MASTER_Rx_DMA_Channel configuration */
	DMA_DeInit(new_spi->dma_rx_ch);
	/* SPI_MASTER_Tx_DMA_Channel configuration */
	DMA_DeInit(new_spi->dma_tx_ch);

	/* Init default DMA struct */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SPI_MASTER_DR_Base;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	new_spi->spi->DR;
	new_spi->spi->CR1 |= ((uint16_t)0x0040);

	/* init dev head list */
	INIT_LIST_HEAD(&spi_list);
	/* Add to led_list */
	list_add(&new_spi->list, &spi_list);
	TRACEL(TRACE_LEVEL_ADC, ("SPI:add->%d\n", new_spi->channel));

	return new_spi;
}


void dev_spi_remove(struct dev_spi * spi)
{
	if (!spi || !spi->owner) return;
	if (dev_spi_find_channel(spi->channel)) return;
	list_del(&spi->list);
	free(spi);
}


static inline void dev_spi_start(struct dev_spi * spi)
{
	spi->port->ODR &= ~spi->nss;
}


static inline void dev_spi_stop(struct dev_spi * spi)
{
	spi->port->ODR |= spi->nss;
}


int dev_spi_send(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	if (!spi || !spi->owner) return -1;
	struct dev_spi * dev = dev_spi_find_channel(spi->channel);

	dev_spi_start(dev);
	dev->spi->DR;

	/* Configure Tx DMA */
    DMA_InitTypeDef	DMA_InitStructure;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(data);
	DMA_InitStructure.DMA_BufferSize = data_len;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Enable DMA channels */
	dev->dma_tx_ch->CCR |= DMA_CCR1_EN;
	/* Enable SPI_MASTER DMA Tx request */
	dev->spi->CR2 |= SPI_I2S_DMAReq_Tx;
	/* Enable SPI_MASTER */
	dev->spi->CR1 |= ((uint16_t)0x0040);

	/* Wait DMA to finish */
	while (DMA_GetFlagStatus(dev->dma_tx_flags)==RESET);
	while (SPI_I2S_GetFlagStatus(dev->spi, SPI_I2S_FLAG_TXE) == RESET);


	/* Clear interrupt flag */
    DMA1->IFCR = dev->dma_tx_flags;

    /* Disable DMA */
	DMA_DeInit(dev->dma_tx_ch);

	/* Disable SPI Tx DMA request */
	dev->spi->CR2 &= (uint16_t)~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

	/* Empty the I2C register */
	dev->spi->DR;
	dev_spi_stop(dev);
	return data_len;
}


int dev_spi_recv(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	uint16_t ret = 0;
	if (!spi || !spi->owner) return -1;

	struct dev_spi * dev = dev_spi_find_channel(spi->channel);
	if (!dev) return -1;

	dev_spi_start(dev);

	dev->spi->DR;
	/* Configure Tx DMA */
    DMA_InitTypeDef	DMA_InitStructure;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)NULL;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(dev->dma_tx_ch, &DMA_InitStructure);

	/* Configure Rx DMA */
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(data);
	DMA_InitStructure.DMA_BufferSize = data_len;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(dev->dma_rx_ch, &DMA_InitStructure);

	/* Enable the DMA channel */
	dev->dma_rx_ch->CCR |= DMA_CCR1_EN;
	dev->dma_tx_ch->CCR |= DMA_CCR1_EN;

	/* Enable the SPI Rx/Tx DMA request */
	dev->spi->CR2 |= SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx;

	// SPI_Cmd(SPI_MASTER, ENABLE);
	dev->spi->CR1 |= ((uint16_t)0x0040);

	while (DMA_GetFlagStatus(dev->dma_rx_flags)==RESET);
	while (DMA_GetFlagStatus(dev->dma_tx_flags)==RESET);

    DMA1->IFCR = DMA1_FLAG_TC3 | DMA1_FLAG_TC2;

    dev->dma_rx_ch->CCR &= (uint16_t)(~DMA_CCR1_EN);
    dev->dma_tx_ch->CCR &= (uint16_t)(~DMA_CCR1_EN);

	dev->spi->CR2 &= (uint16_t)~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

	/* Empty the I2C register */
	SPI1->DR;

	return (ret);

}



uint16_t dev_spi_send_recv(struct dev_spi * spi, uint8_t * data_out, uint16_t data_out_len, uint8_t * data_in, uint16_t data_in_len)
{
	uint16_t ret = 0;
	if (!spi || !spi->owner) return -1;

	struct dev_spi * dev = dev_spi_find_channel(spi->channel);
	if (!dev) return -1;

	dev_spi_start(dev);

	dev->spi->DR;
	/* Configure Tx DMA */
    DMA_InitTypeDef	DMA_InitStructure;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(data_out);
	DMA_InitStructure.DMA_BufferSize = data_out_len;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(dev->dma_tx_ch, &DMA_InitStructure);

	/* Configure Rx DMA */
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(data_in);
	DMA_InitStructure.DMA_BufferSize = data_in_len;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_Init(dev->dma_rx_ch, &DMA_InitStructure);

	/* Enable the DMA channel */
	DMA_Cmd(dev->dma_rx_ch, ENABLE);
	DMA_Cmd(dev->dma_tx_ch, ENABLE);

	/* Enable the SPI Rx/Tx DMA request */
	SPI_I2S_DMACmd(dev->spi, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	SPI_Cmd(dev->spi, ENABLE);

	while (DMA_GetFlagStatus(dev->dma_rx_flags)==RESET);
	while (DMA_GetFlagStatus(dev->dma_tx_flags)==RESET);

	DMA1->IFCR = dev->dma_rx_flags | dev->dma_tx_flags;

    dev->dma_rx_ch->CCR &= (uint16_t)(~DMA_CCR1_EN);
    dev->dma_tx_ch->CCR &= (uint16_t)(~DMA_CCR1_EN);

	dev->spi->CR2 &= (uint16_t)~(SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx);

	/* Empty the I2C register */
	dev->spi->DR;

	return (ret);
}
