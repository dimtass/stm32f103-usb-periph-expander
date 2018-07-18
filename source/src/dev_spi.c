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
		new_spi->dma_rx_iqrn = DMA1_Channel2_IRQn;
		new_spi->dma_tx_ch = DMA1_Channel3;
		new_spi->dma_tx_flags = DMA1_FLAG_TC3;
		new_spi->dma_tx_iqrn = DMA1_Channel3_IRQn;
		/* RCC configuration */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);
	}
	else if (dev->mode == DEV_SPI1_GPIOB) {
		new_spi->spi = SPI1;
		new_spi->port = GPIOB;
		new_spi->nss = GPIO_Pin_2;
		new_spi->sck = GPIO_Pin_3;
		new_spi->miso = GPIO_Pin_4;
		new_spi->mosi = GPIO_Pin_5;
		new_spi->dma_rx_ch = DMA1_Channel2;
		new_spi->dma_rx_flags = DMA1_FLAG_TC2;
		new_spi->dma_rx_iqrn = DMA1_Channel2_IRQn;
		new_spi->dma_tx_ch = DMA1_Channel3;
		new_spi->dma_tx_flags = DMA1_FLAG_TC3;
		new_spi->dma_tx_iqrn = DMA1_Channel3_IRQn;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);
	}
	else if (dev->mode == DEV_SPI2) {
		new_spi->spi = SPI2;
		new_spi->port = GPIOB;
		new_spi->nss = GPIO_Pin_12;
		new_spi->sck = GPIO_Pin_13;
		new_spi->miso = GPIO_Pin_14;
		new_spi->mosi = GPIO_Pin_15;
		new_spi->dma_rx_ch = DMA1_Channel4;
		new_spi->dma_rx_flags = DMA1_FLAG_TC4;
		new_spi->dma_rx_iqrn = DMA1_Channel4_IRQn;
		new_spi->dma_tx_ch = DMA1_Channel5;
		new_spi->dma_tx_flags = DMA1_FLAG_TC5;
		new_spi->dma_tx_iqrn = DMA1_Channel4_IRQn;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	}

	/* GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = new_spi->sck | new_spi->mosi;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(new_spi->port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = spi->nss;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(spi->port, &GPIO_InitStructure);
	spi->port->ODR |= spi->nss;

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
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	/* Initialize DMA struct */
	DMA_StructInit(&new_spi->dma_struct);

	/* SPI_MASTER_Rx_DMA_Channel configuration */
	DMA_DeInit(new_spi->dma_rx_ch);
	/* SPI_MASTER_Tx_DMA_Channel configuration */
	DMA_DeInit(new_spi->dma_tx_ch);

	/* Init default DMA struct */
	new_spi->dma_struct.DMA_PeripheralBaseAddr = (uint32_t) &spi->spi->DR;
	new_spi->dma_struct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	new_spi->dma_struct.DMA_MemoryInc = DMA_MemoryInc_Enable;
	new_spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	new_spi->dma_struct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	new_spi->dma_struct.DMA_Mode = DMA_Mode_Normal;
	new_spi->dma_struct.DMA_Priority = DMA_Priority_High;
	new_spi->dma_struct.DMA_M2M = DMA_M2M_Disable;

    NVIC_EnableIRQ(new_spi->dma_rx_iqrn);
    DMA_ITConfig(new_spi->dma_rx_ch, DMA_IT_TC, ENABLE);
    NVIC_EnableIRQ(new_spi->dma_tx_iqrn);
    DMA_ITConfig(new_spi->dma_tx_ch, DMA_IT_TC, ENABLE);

	/* Enable Tx DMA DMA Tx request */
	SPI_I2S_DMACmd(new_spi->spi, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	return new_spi;

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


inline void dev_spi_start(struct dev_spi * spi)
{
	spi->port->ODR &= ~spi->nss;
}

inline void dev_spi_stop(struct dev_spi * spi)
{
	spi->port->ODR |= spi->nss;
}

static inline void dev_spi_tx(struct dev_spi * spi)
{
	DMA_Init(spi->dma_tx_ch, &spi->dma_struct);
	/* Enable DMA channels */
	DMA_Cmd(spi->dma_tx_ch, ENABLE);
}

static inline void dev_spi_rx(struct dev_spi * spi)
{
	DMA_Init(spi->dma_rx_ch, &spi->dma_struct);
	/* Enable DMA channels */
	DMA_Cmd(spi->dma_rx_ch, ENABLE);
}

inline void dev_spi_wait(struct dev_spi * spi)
{
	/* Wait DMA to finish */
	while(SPI_I2S_GetFlagStatus(spi->spi, SPI_I2S_FLAG_BSY) == SET);
//    /* Disable DMA */
	DMA_DeInit(spi->dma_tx_ch);
	DMA_DeInit(spi->dma_rx_ch);
}

inline void dev_spi_send8(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	/* Configure Tx DMA */
	spi->dma_struct.DMA_MemoryBaseAddr = (uint32_t)(data);
	spi->dma_struct.DMA_BufferSize = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Normal;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralDST;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	dev_spi_tx(spi);
}

inline void dev_spi_sendCircular8(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	/* Configure Tx DMA */
	spi->dma_struct.DMA_MemoryBaseAddr = (uint32_t)(data);
	spi->dma_struct.DMA_BufferSize = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Circular;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralDST;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	dev_spi_tx(spi);
}


inline void dev_spi_recv8(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	/* Configure Rx DMA */
	spi->dma_struct.DMA_MemoryBaseAddr = (uint32_t)(data);
	spi->dma_struct.DMA_BufferSize = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Normal;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	dev_spi_rx(spi);
}

inline void dev_spi_recvCircular8(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	/* Configure Tx DMA */
	spi->dma_struct.DMA_MemoryBaseAddr = (uint32_t)(data);
	spi->dma_struct.DMA_BufferSize = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Circular;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	dev_spi_rx(spi);
}

inline void dev_spi_sendCircular16(struct dev_spi * spi, uint16_t *data, size_t data_len)
{
    spi->dma_struct.DMA_MemoryBaseAddr = (u32) data;
    spi->dma_struct.DMA_BufferSize     = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Circular;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Disable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralDST;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

    dev_spi_tx(spi);
}

inline void dev_spi_send16(struct dev_spi * spi, uint16_t *data, size_t data_len)
{
    spi->dma_struct.DMA_MemoryBaseAddr = (u32) data;
    spi->dma_struct.DMA_BufferSize     = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Normal;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralDST;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

    dev_spi_tx(spi);
}

inline void dev_spi_recv16(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	/* Configure Rx DMA */
	spi->dma_struct.DMA_MemoryBaseAddr = (uint32_t)(data);
	spi->dma_struct.DMA_BufferSize = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Normal;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

	dev_spi_rx(spi);
}

inline void dev_spi_recvCircular16(struct dev_spi * spi, uint8_t * data, size_t data_len)
{
	/* Configure Rx DMA */
	spi->dma_struct.DMA_MemoryBaseAddr = (uint32_t)(data);
	spi->dma_struct.DMA_BufferSize = data_len;

    spi->dma_struct.DMA_Mode               = DMA_Mode_Circular;
    spi->dma_struct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    spi->dma_struct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    spi->dma_struct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    spi->dma_struct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;

	dev_spi_rx(spi);
}


void dev_spi_set8(struct dev_spi * spi)
{
    spi->spi->CR1 &= ~SPI_CR1_SPE; // DISABLE SPI
    spi->spi->CR1 &= ~SPI_CR1_DFF; // SPI 8
    spi->spi->CR1 |= SPI_CR1_SPE;  // ENABLE SPI
}

void dev_spi_set16(struct dev_spi * spi)
{
    spi->spi->CR1 &= ~SPI_CR1_SPE; // DISABLE SPI
    spi->spi->CR1 |= SPI_CR1_DFF;  // SPI 16
    spi->spi->CR1 |= SPI_CR1_SPE;  // ENABLE SPI
}

void DMA1_Channel2_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC2) == SET) {
        DMA_Cmd(DMA1_Channel2, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC2);
    }
}

void DMA1_Channel3_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC3) == SET) {
        DMA_Cmd(DMA1_Channel3, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC3);
    }
}

void DMA1_Channel4_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC4) == SET) {
        DMA_Cmd(DMA1_Channel4, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC4);
    }
}

void DMA1_Channel5_IRQHandler(void) {
    if (DMA_GetITStatus(DMA1_IT_TC5) == SET) {
        DMA_Cmd(DMA1_Channel5, DISABLE);
        DMA_ClearITPendingBit(DMA1_IT_TC5);
    }
}