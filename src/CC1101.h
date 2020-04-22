
#ifndef _CC1101_H
#define _CC1101_H
// RF settings for CC1101
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "queue.h"
//#include "cc1101_conf.h"

#define SRES 	0x30 // Reset chip
#define SFSTXON 0x31 // Enable and calibrate frequency synthesizer (for quick RX / TX turnaround)
#define SXOFF 	0x32 // Turn off crystal oscillator.
#define SCAL	0x33 // Calibrate frequency synthesizer and turn it off.
#define SRX 	0x34 // Enable RX.
#define STX 	0x35 // In IDLE state: Enable TX. In RX & CCA: Only go to TX if channel is clear.
#define SIDLE 	0x36 // Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
#define SWOR 	0x38 // Start automatic RX polling sequence (Wake-on-Radio)
#define SPWD 	0x39 // Enter power down mode when CSn goes high.
#define SFRX 	0x3A // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define SFTX 	0x3B // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define SWORRST 0x3C // Reset real time clock to Event1 value
#define SNOP 	0x3D // No operation. May be used to get access to the chip status byte.

#define FIFO 	0x3F
#define PATAB 	0x3E
#define READ 	0x80
#define WRITE 	0x00
#define BURST 	0x40
#define SINGLE 	0x00


#define IDLE_MODE 			0x0 // IDLE state
#define RX_MODE 			0x1 // Receive mode
#define TX_MODE 			0x2 // Transmit mode
#define FSTXON_MODE 		0x3 // Fast TX ready
#define CALIBRATE_MODE 		0x4 // Frequency synthesizer calibration is running
#define SETTLING_MODE 		0x5 // PLL is settling
#define RXFIFO_OVERFLOW 	0x6 // RX FIFO has overflowed
#define TXFIFO_UNDERFLOW	0x7 // TX FIFO has underflowed

#define CC_READY   0
#define CC_UNREADY 1

#define PORT_NORMAL 0
#define PORT_REMAP  1

#define RX_EVENT  1
#define TX_EVENT  1
#define QUEUE_SIZE 	1
#define PACK_SIZE 	6
#define PACK_ADD	7
#define PACK_RX_COUNT PACK_SIZE + PACK_ADD
#define PACK_TX_COUNT PACK_SIZE + PACK_ADD - 2
/*
 * @info
 * refer to how calc bitbang address
 *
 * (uint32_t*)(PERIPH_BB_BASE + ((GPIO_X_BASE-PERIPH_BASE+ REG_OFFSET )  * 32) + ( BIT * 4));
 *
 */

#define FREERTOS
typedef uint32_t btype_t;

void ARadioTask (void* pvParameters);
void ARadioTaskS (void* pvParameters);

// Rf settings for CC1101
// communicatin format packet
struct pack
{
	uint8_t bLeng=(PACK_SIZE+PACK_ADD)-3;
	uint8_t addrdst;
	uint8_t addrsrc;
	uint8_t data[PACK_SIZE];
	uint8_t rssi;
	uint8_t crc8d;
	uint8_t rssi_r;
	uint8_t lqi;
};
/*
 * status of chip
 */
 struct  cc1101Status
{
	uint8_t rdy;
	uint8_t state;
	uint8_t fifo_rx_av;
	uint8_t fifo_tx_av;
};

 /*
  * rtos task creation parameters
  */
 struct xTaskParam
{
	uint32_t pTaskSerial;// base addres of used port
	uint8_t xTaskPortH;
	uint32_t* pRxEvent;//bit bang input pin of event
	uint32_t* pTxcEvent;
#ifdef FREERTOS
	QueueHandle_t xCommRX;
	QueueHandle_t xCommTX;
	QueueHandle_t xErr;
#else
	uint32_t *xCommRX;
	uint32_t *xCommTX;
	uint32_t *xErr;
#endif

};

  class cc11xx_class
  {
  private:
	  pack *rxp= new pack;
	  pack *txp= new pack;
	  cc1101Status* cStatus= new cc1101Status;
	  SPI_TypeDef *SP;
	  uint32_t *NSS_set; //address for only set or reset
	  uint32_t *NSS_get; //adress for read nss
	  uint32_t *NSS_reset;
	  uint32_t *rxEvent;
	  uint32_t *txcEvent;
	  uint32_t *MISO_lv; // can only read via BB
	  btype_t selectChip(void);// return 1 if success bit set
	  btype_t deselectChip(void);// return 1 if success bit set
	  btype_t getMISO(void); // return MISO level
	  uint8_t xPortHW; // this byte define hw of port



  public:

	#ifdef FREERTOS
	  QueueHandle_t pRX;
	  QueueHandle_t pTX;
	  QueueHandle_t pErr;
	#else
	  uint32_t *pRX;
	  uint32_t *pTX;
	  uint32_t *pErr;
	#endif
	  cc11xx_class(xTaskParam * pPortParam, uint8_t set_len, uint8_t *rfSettings);
	  btype_t sendByte(uint8_t address, uint8_t  cmd);
	  uint8_t readByte(uint8_t address);
	  btype_t sendBurst(uint8_t sAddress, uint8_t  cmdCount, uint8_t* cmds);
	  btype_t readBurst(uint8_t sAddress, uint8_t  cmdCount, uint8_t* cmds);
	  btype_t chekStatus(void);
	  btype_t txPack(void);
	  btype_t rxPack(void);
	  pack* getRxPack(void);
	  btype_t sendSTB(uint8_t stb);
	  uint8_t crc8(uint8_t *pcBlock, uint8_t len);
	  btype_t rxEventHook(void);
#ifdef FREERTOS
	  btype_t txEventHook(void);

#else
	  btype_t txEventHook(pack* TXPACK);

#endif

  };
#endif
