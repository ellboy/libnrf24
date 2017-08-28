/*
    PLAY Embedded - Copyright (C) 2006..2015 Rocco Marco Guglielmi

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
    Special thanks to Giovanni Di Sirio for teachings, his moral support and
    friendship. Note that some or every piece of this file could be part of
    the ChibiOS project that is intellectual property of Giovanni Di Sirio.
    Please refer to ChibiOS/RT license before use this file.
	
	For suggestion or Bug report - guglielmir@playembedded.org
 */

/**
 * @file    rf.c
 * @brief   RF complex driver code.
 *
 * @addtogroup RF
 * @{
 */

#include "rf.h"

#include "debug.h"

#if USERLIB_USE_RF || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define  NRF24L01_MAX_ADD_LENGHT                 ((uint8_t)  5)
#define  NRF24L01_MAX_PL_LENGHT                  ((uint8_t) 32)
#define  NRF24L01_MAX_PPP                        ((uint8_t)  5)

#define ACTIVATE                                 0x73


/**
 * @brief   Resets all Status flags.
 *
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 *
 * @return              the status register value
 */
#define  nrf24l01Reset(spip) {                                               \
                                                                             \
  nrf24l01WriteRegister(spip, NRF24L01_AD_STATUS,                            \
                                NRF24L01_DI_STATUS_MAX_RT |                  \
                                NRF24L01_DI_STATUS_RX_DR |                   \
                                NRF24L01_DI_STATUS_TX_DS);                   \
}

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   RFD1 driver identifier.
 */
RFDriver RFD1;

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

static uint32_t cbcounter;
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Gets the status register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 *
 * @return              the status register value
 */
static uint8_t nrf24l01GetStatus(SPIDriver *spip) {
  uint8_t txbuf = NRF24L01_CMD_NOP;
  uint8_t status;
  spiSelect(spip);
  spiExchange(spip, 1, &txbuf, &status);
  spiUnselect(spip);
  return status;
}

/**
 * @brief   Reads a generic register value.
 *
 * @note    Cannot be used to set addresses
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       register number
 * @param[out] pvalue   pointer to a data buffer
 *
 * @return              the status register value
 */
static uint8_t nrf24l01ReadRegister(SPIDriver *spip, uint8_t reg,
                                       uint8_t* pvalue) {
  uint8_t txbuf = (NRF24L01_CMD_READ | reg);
  uint8_t status = 0xFF;
  spiSelect(spip);
  spiExchange(spip, 1, &txbuf, &status);
  spiReceive(spip, 1, pvalue);
  spiUnselect(spip);
  return status;
}

static uint8_t nrf24l01ReadRegisterRange(SPIDriver *spip, uint8_t reg,
                                       uint8_t* pvalue, uint8_t range) {
  uint8_t txbuf = (NRF24L01_CMD_READ | reg);
  uint8_t status = 0xFF;
  spiSelect(spip);
  spiExchange(spip, 1, &txbuf, &status);
  spiReceive(spip, range, pvalue);
  spiUnselect(spip);
  return status;
}

/**
 * @brief   Writes a generic register value.
 *
 * @note    Cannot be used to set addresses
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       register number
 * @param[in] value     data value
 *
 * @return              the status register value
 */
static uint8_t nrf24l01WriteRegister(SPIDriver *spip, uint8_t reg,
                                        uint8_t value) {

  uint8_t txbuf[2] = {(NRF24L01_CMD_WRITE | reg), value};
  uint8_t rxbuf[2] = {0xFF, 0xFF};
  switch (reg) {

    default:
      /* Reserved register must not be written, according to the datasheet
       * this could permanently damage the device.
       */
      osalDbgAssert(FALSE, "lg3d20WriteRegister(), reserved register");
    case NRF24L01_AD_OBSERVE_TX:
    case NRF24L01_AD_CD:
    case NRF24L01_AD_RX_ADDR_P0:
    case NRF24L01_AD_RX_ADDR_P1:
    case NRF24L01_AD_RX_ADDR_P2:
    case NRF24L01_AD_RX_ADDR_P3:
    case NRF24L01_AD_RX_ADDR_P4:
    case NRF24L01_AD_RX_ADDR_P5:
    case NRF24L01_AD_TX_ADDR:
    /* Read only or addresses registers cannot be written,
     * the command is ignored.
     */
      return 0;
    case NRF24L01_AD_CONFIG:
    case NRF24L01_AD_EN_AA:
    case NRF24L01_AD_EN_RXADDR:
    case NRF24L01_AD_SETUP_AW:
    case NRF24L01_AD_SETUP_RETR:
    case NRF24L01_AD_RF_CH:
    case NRF24L01_AD_RF_SETUP:
    case NRF24L01_AD_STATUS:
    case NRF24L01_AD_RX_PW_P0:
    case NRF24L01_AD_RX_PW_P1:
    case NRF24L01_AD_RX_PW_P2:
    case NRF24L01_AD_RX_PW_P3:
    case NRF24L01_AD_RX_PW_P4:
    case NRF24L01_AD_RX_PW_P5:
    case NRF24L01_AD_FIFO_STATUS:
    case NRF24L01_AD_DYNPD:
    case NRF24L01_AD_FEATURE:
      spiSelect(spip);
      spiExchange(spip, 2, txbuf, rxbuf);
      spiUnselect(spip);
      return rxbuf[0];
  }
}

/**
 * @brief   Writes an address.
 *
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       register number
 * @param[in] pvalue    pointer to address value
 * @param[in] addlen    address len
 *
 * @return              the status register value
 */
static uint8_t nrf24l01WriteAddress(SPIDriver *spip, uint8_t reg,
                                       uint8_t *pvalue, uint8_t addlen) {

  uint8_t txbuf[NRF24L01_MAX_ADD_LENGHT + 1];
  uint8_t rxbuf[NRF24L01_MAX_ADD_LENGHT + 1];
  unsigned i;

  if(addlen > NRF24L01_MAX_ADD_LENGHT) {
    osalDbgAssert(FALSE, "nrf24l01WriteAddress(), wrong address length");
    return 0;
  }
  txbuf[0] = (NRF24L01_CMD_WRITE | reg);
  rxbuf[0] = 0xFF;
  for(i = 1; i <= addlen; i++) {
    txbuf[i] = *(pvalue + (i - 1));
    rxbuf[i] = 0xFF;
  }
  switch (reg) {

    default:
      /* Reserved register must not be written, according to the datasheet
       * this could permanently damage the device.
       */
      osalDbgAssert(FALSE, "nrf24l01WriteAddress(), reserved register");
    case NRF24L01_AD_OBSERVE_TX:
    case NRF24L01_AD_CD:
    case NRF24L01_AD_CONFIG:
    case NRF24L01_AD_EN_AA:
    case NRF24L01_AD_EN_RXADDR:
    case NRF24L01_AD_SETUP_AW:
    case NRF24L01_AD_SETUP_RETR:
    case NRF24L01_AD_RF_CH:
    case NRF24L01_AD_RF_SETUP:
    case NRF24L01_AD_STATUS:
    case NRF24L01_AD_RX_PW_P0:
    case NRF24L01_AD_RX_PW_P1:
    case NRF24L01_AD_RX_PW_P2:
    case NRF24L01_AD_RX_PW_P3:
    case NRF24L01_AD_RX_PW_P4:
    case NRF24L01_AD_RX_PW_P5:
    case NRF24L01_AD_FIFO_STATUS:
    case NRF24L01_AD_DYNPD:
    case NRF24L01_AD_FEATURE:
    /* Not address registers cannot be written, the command is ignored.*/
      return 0;
    case NRF24L01_AD_RX_ADDR_P0:
    case NRF24L01_AD_RX_ADDR_P1:
    case NRF24L01_AD_RX_ADDR_P2:
    case NRF24L01_AD_RX_ADDR_P3:
    case NRF24L01_AD_RX_ADDR_P4:
    case NRF24L01_AD_RX_ADDR_P5:
    case NRF24L01_AD_TX_ADDR:
      spiSelect(spip);
      spiExchange(spip, addlen + 1, txbuf, rxbuf);
      spiUnselect(spip);
      return rxbuf[0];
  }
}

/**
 * @brief   Reads RX payload from FIFO.
 *
 * @note    Payload is deleted from FIFO after it is read. Used in RX mode.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] paylen    payload length
 * @param[in] rxbuf     pointer to a buffer
 *
 * @return              the status register value
 */
static uint8_t nrf24l01GetRxPl(SPIDriver *spip, uint8_t paylen, uint8_t* rxbuf) {

  uint8_t txbuf = NRF24L01_CMD_R_RX_PAYLOAD;
  uint8_t status;
  if(paylen > NRF24L01_MAX_PL_LENGHT) {
    return 0;
  }
  spiSelect(spip);
  spiExchange(spip, 1, &txbuf, &status);
  spiReceive(spip, paylen, rxbuf);
  spiUnselect(spip);
  return status;
}

/**
 * @brief   Writes TX payload on FIFO.
 *
 * @note    Used in TX mode.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] paylen    payload length
 * @param[in] rxbuf     pointer to a buffer
 *
 * @return              the status register value
 */
static uint8_t nrf24l01WriteTxPl(SPIDriver *spip, uint8_t paylen,
                                    uint8_t* txbuf) {

  uint8_t cmd = NRF24L01_CMD_W_TX_PAYLOAD;
  uint8_t status;
  if(paylen > NRF24L01_MAX_PL_LENGHT) {
    return 0;
  }
  spiSelect(spip);
  spiExchange(spip, 1, &cmd, &status);
  spiSend(spip, paylen, txbuf);
  spiUnselect(spip);
  return status;
}


/**
 * @brief   Flush TX FIFO.
 *
 * @note    Used in TX mode.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 *
 * @return              the status register value
 */
static uint8_t nrf24l01FlushTx(SPIDriver *spip) {

  uint8_t txbuf = NRF24L01_CMD_FLUSH_TX;
  uint8_t status;
  spiSelect(spip);
  spiExchange(spip, 1, &txbuf, &status);
  spiUnselect(spip);
  return status;
}

/**
 * @brief   Flush RX FIFO.
 *
 * @note    Used in RX mode. Should not be executed during transmission of
            acknowledge, that is, acknowledge package will not be completed.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 *
 * @return              the status register value
 */
static uint8_t nrf24l01FlushRx(SPIDriver *spip) {

  uint8_t txbuf = NRF24L01_CMD_FLUSH_RX;
  uint8_t status;
  spiSelect(spip);
  spiExchange(spip, 1, &txbuf, &status);
  spiUnselect(spip);
  return status;
}

/**
 * @brief   Activates the following features:
 *          R_RX_PL_WID        -> (In order to enable DPL the EN_DPL bit in the
 *                                 FEATURE register must be set)
 *          W_ACK_PAYLOAD      -> (In order to enable PL with ACK the EN_ACK_PAY
 *                                 bit in the FEATURE register must be set)
 *          W_TX_PAYLOAD_NOACK -> (In order to send a PL without ACK
 *                                 the EN_DYN_ACK it in the FEATURE register
 *                                 must be set)
 *
 * @note    A new ACTIVATE command with the same data deactivates them again.
 *          This is executable in power down or stand by modes only.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 *
 * @return              the status register value
 */
static uint8_t nrf24l01Activate(SPIDriver *spip) {

  uint8_t txbuf[2] = {NRF24L01_CMD_FLUSH_RX, ACTIVATE};
  uint8_t rxbuf[2];
  spiSelect(spip);
  spiExchange(spip, 2, txbuf, rxbuf);
  spiUnselect(spip);
  return rxbuf[0];
}

/**
 * @brief   Reads RX payload lenght for the top R_RX_PAYLOAD
 *          in the RX FIFO when Dynamic Payload Length is activated.
 *
 * @note    R_RX_PL_WID must be set and activated.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] ppaylen   pointer to the payload length variable
 *
 * @return              the status register value
 */
static uint8_t nrf24l01ReadRxPlWid(SPIDriver *spip, uint8_t *ppaylen) {

  uint8_t txbuf[2] = {NRF24L01_CMD_R_RX_PL_WID, 0xFF};
  uint8_t rxbuf[2];
  spiSelect(spip);
  spiExchange(spip, 2, txbuf, rxbuf);
  spiUnselect(spip);
  *ppaylen = rxbuf[1];
  return rxbuf[0];
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   RF Complex Driver initialization.
 *
 * @init
 */
void rfInit(void) {

  rfObjectInit(&RFD1);
}

/**
 * @brief   Initializes an instance.
 *
 * @param[out] rfp         pointer to the @p RFDriver object
 *
 * @init
 */
void rfObjectInit(RFDriver *rfp){

  rfp->state  = RF_STOP;
  rfp->config = NULL;
  osalEventObjectInit(&rfp->irq_event);
#if RF_USE_MUTUAL_EXCLUSION == TRUE
  osalMutexObjectInit(&rfp->mutex);
#endif
}

/**
 * @brief   Configures and activates RF Complex Driver peripheral.
 *
 * @param[in] rfp   pointer to the @p RFDriver object
 * @param[in] config    pointer to the @p RFConfig object
 *
 * @api
 */
void rfStart(RFDriver *rfp, RFConfig *config) {

  osalDbgCheck((rfp != NULL) && (config != NULL));

  osalDbgAssert((rfp->state == RF_STOP) || (rfp->state == RF_READY),
              "rfStart(), invalid state");
  rfp->config = config;

  //To do check why osal has not this call
  chEvtRegister(&rfp->irq_event, &rfp->el, 0);
  extStart(rfp->config->extp, rfp->config->extcfg);
  //extChannelEnable(&EXTD1, INT0);
  //extChannelEnable(rfp->config->extp, 1);
  spiStart(rfp->config->spip, rfp->config->spicfg);

  nrf24l01Reset(rfp->config->spip);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_CONFIG,
		                rfp->config->en_crc | NRF24L01_DI_CONFIG_PWR_UP);
  osalThreadSleepMilliseconds(2);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_EN_AA,
                        rfp->config->en_auto_ack);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_EN_RXADDR,
                        NRF24L01_DI_EN_RXADDR);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_RF_CH,
                        rfp->config->channel_freq);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_SETUP_RETR,
                        rfp->config->auto_retr_count |
                        rfp->config->auto_retr_delay);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_SETUP_AW,
                        rfp->config->address_width);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_RF_SETUP,
                        rfp->config->data_rate |
                        rfp->config->out_pwr |
                        rfp->config->lna);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_FEATURE,
                        rfp->config->en_dyn_ack | rfp->config->en_ack_pay | rfp->config->en_dpl);
  nrf24l01Activate(rfp->config->spip);
  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_DYNPD,
                        NRF24L01_DI_DYNPD); // dynamic payload on all pipes

  nrf24l01FlushRx(rfp->config->spip);
  nrf24l01FlushTx(rfp->config->spip);

  palClearPad(rfp->config->ceport, rfp->config->cepad);
  rfp->state = RF_READY;

  printDetails(rfp);

}

/**
 * @brief   Deactivates the RF Complex Driver peripheral.
 *
 * @param[in] rfp      pointer to the @p RFDriver object
 *
 * @api
 */
void rfStop(RFDriver *rfp) {

  osalDbgCheck(rfp != NULL);

  osalDbgAssert((rfp->state == RF_STOP) || (rfp->state == RF_READY),
              "rfStop(), invalid state");
  if (rfp->state == RF_READY) {
      nrf24l01WriteRegister(rfp->config->spip,
                            NRF24L01_AD_CONFIG, 0);
      spiStop(rfp->config->spip);
      extStop(rfp->config->extp);
      chEvtUnregister(&rfp->irq_event, &rfp->el);
  }
  rfp->state = RF_STOP;
}

/**
 * @brief   Checks if there are empty spaces in the TX FIFO.
 *
 * @param[in] rfp        Pointer to the @p RFDriver object
 * @return               The operation result.
 * @retval TRUE          There is an empty space.
 * @retval FALSE         No empty space available.
 * @api
 */
bool rfTxIsEmpty(RFDriver *rfp) {

  uint8_t fifo_status;
  nrf24l01ReadRegister(rfp->config->spip,
                       NRF24L01_AD_FIFO_STATUS, &fifo_status);
  return(!(fifo_status & NRF24L01_DI_FIFO_STATUS_TX_FULL));
}

/**
 * @brief   Checks if there are packets in the RX FIFO.
 *
 * @param[in] rfp        Pointer to the @p RFDriver object
 * @return               The operation result.
 * @retval TRUE          There is a packet.
 * @retval FALSE         RX FIFO is empty.
 * @api
 */
bool rfRxIsNonEmpty(RFDriver *rfp) {

  uint8_t fifo_status;
    nrf24l01ReadRegister(rfp->config->spip,
                         NRF24L01_AD_FIFO_STATUS, &fifo_status);
    return (!(fifo_status & NRF24L01_DI_FIFO_STATUS_RX_EMPTY));
  return FALSE;
}

/**
 * @brief   Receives n Rx frames from the RF Complex Driver peripheral.
 *
 * @param[in] rfp        Pointer to the @p RFDriver object
 * @param[in] n          RFTxFrame array length
 * @param[in] rxbuff     Pointer to an array of @p RFRxFrame
 * @param[in] time       The number of ticks before the operation timeouts,
 *                       the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 * @return               The operation result.
 * @retval RF_OK         The operation succeeds.
 * @retval RF_ERROR      Error during the transmission.
 * @api
 */
rf_msg_t rfReceiveFrame(RFDriver *rfp, uint32_t n, RFRxFrame *rxbuff) {

  uint8_t status;
  uint32_t cnt;

  osalDbgCheck((rfp != NULL) && (rxbuff != NULL) && (n > 0));

  osalDbgAssert((rfp->state == RF_READY),
              "rfReceive(), invalid state");

  nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_CONFIG,
                        NRF24L01_DI_CONFIG_PWR_UP |
                        rfp->config->en_crc |
                        NRF24L01_DI_CONFIG_PRIM_RX);

//  nrf24l01WriteAddress(rfp->config->spip, NRF24L01_AD_TX_ADDR, TODO Why tX addr?
//                       rxbuff->rx_address, RF_ADDLEN);
  nrf24l01WriteAddress(rfp->config->spip,
                       NRF24L01_AD_RX_ADDR_P1,
                       rxbuff->rx_address, RF_ADDLEN);
  nrf24l01FlushRx(rfp->config->spip);
  nrf24l01Reset(rfp->config->spip);
  cnt = 0;
  //print_status(nrf24l01GetStatus(rfp->config->spip));
  //printDetails(rfp);
  rfp->state = RF_RX;
  palSetPad(rfp->config->ceport, rfp->config->cepad);
  //printDetails(rfp);
  while(cnt < n) {
    if(chEvtWaitOneTimeout(ALL_EVENTS, rfp->config->time_out) == 0) {
        rfp->state = RF_READY;
        palClearPad(rfp->config->ceport, rfp->config->cepad);
        //print_dbg("timeout\r\n");
        return RF_TIMEOUT;
    }
    //print_dbg("irq\r\n");
    status = nrf24l01GetStatus(rfp->config->spip);
    if (((status & NRF24L01_DI_STATUS_RX_DR) ||
        (status & NRF24L01_DI_STATUS_TX_DS)) && (rfRxIsNonEmpty(rfp))) {

      nrf24l01ReadRxPlWid(rfp->config->spip, &(rxbuff + cnt)->rx_paylen);
      osalDbgCheck((rxbuff + cnt)->rx_paylen <= RF_PAYLEN);
      nrf24l01GetRxPl(rfp->config->spip, (rxbuff + cnt)->rx_paylen,
                      (rxbuff + cnt)->rx_payload);
      cnt++;
      nrf24l01Reset(rfp->config->spip);
      continue;
    }
    else {
      nrf24l01Reset(rfp->config->spip);
      rfp->state = RF_READY;
      palClearPad(rfp->config->ceport, rfp->config->cepad);
      return RF_ERROR;
    }
    palClearPad(rfp->config->ceport, rfp->config->cepad);
  }
  rfp->state = RF_READY;
  palClearPad(rfp->config->ceport, rfp->config->cepad);
  return RF_OK;
}

/**
 * @brief   Transmits n Tx frames on the RF Complex Driver peripheral.
 *
 * @param[in] rfp        Pointer to the @p RFDriver object
 * @param[in] n          RFTxFrame array length
 * @param[in] txbuff     Pointer to an array of @p RFTxFrame
 * @param[in] time       The number of ticks before the operation timeouts,
 *                       the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *                      .
 *
 * @return               The operation result.
 * @retval RF_OK         The operation succeeds.
 * @retval RF_ERROR      Error during the transmission.
 * @api
 */
rf_msg_t rfTransmitFrame(RFDriver *rfp, uint32_t n, RFTxFrame *txbuff, RFAckFrame *ackbuff) {
	uint8_t status;
	uint32_t cnt;
	bool flag;
	rf_msg_t ret = RF_OK;
	uint8_t buffer;

	osalDbgCheck((rfp != NULL) && (txbuff != NULL));

	osalDbgAssert((rfp->state == RF_READY), "rfTransmit(), invalid state");
	nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_CONFIG,
	NRF24L01_DI_CONFIG_PWR_UP | rfp->config->en_crc);

	nrf24l01WriteAddress(rfp->config->spip, NRF24L01_AD_TX_ADDR,
			txbuff->tx_address, RF_ADDLEN);
	nrf24l01WriteAddress(rfp->config->spip, NRF24L01_AD_RX_ADDR_P0,
			txbuff->tx_address, RF_ADDLEN);

	nrf24l01Reset(rfp->config->spip);

	cnt = 0;
	flag = TRUE;
	rfp->state = RF_TX;

	//print_dbg("\r\nrfTransmit\r\n");
	while (cnt < n) {
		if (rfTxIsEmpty(rfp) && flag) {
			osalDbgCheck((txbuff + cnt)->tx_paylen <= RF_PAYLEN);
			nrf24l01WriteTxPl(rfp->config->spip, (txbuff + cnt)->tx_paylen,
					(txbuff + cnt)->tx_payload);
			palSetPad(rfp->config->ceport, rfp->config->cepad);
			osalThreadSleepMilliseconds(1);
			palClearPad(rfp->config->ceport, rfp->config->cepad);
			flag = FALSE;
		}

		if (chEvtWaitOneTimeout(ALL_EVENTS, rfp->config->time_out) == 0) {
			ret = RF_TIMEOUT;
			break;
		}

		status = nrf24l01GetStatus(rfp->config->spip);

		// Data Send TX FIFO interrupt
		if (status & NRF24L01_DI_STATUS_TX_DS) {
			nrf24l01Reset(rfp->config->spip);
			flag = TRUE;
			cnt++;

			if (rfp->config->en_ack_pay == NRF24L01_ACK_PAY_enabled) {
				if (rfRxIsNonEmpty(rfp)) {
					nrf24l01ReadRegister(rfp->config->spip, NRF24L01_AD_OBSERVE_TX, &buffer);
					(ackbuff + cnt)->arc_cnt = buffer & NRF24L01_DI_ARC_CNT;
					nrf24l01ReadRxPlWid(rfp->config->spip, &(ackbuff + cnt)->ack_paylen);

					osalDbgCheck((ackbuff + cnt)->ack_paylen <= RF_PAYLEN);
					nrf24l01GetRxPl(rfp->config->spip, (ackbuff + cnt)->ack_paylen,
							(ackbuff + cnt)->ack_payload);
					print_dbg("ack len = %d, arc = %d\r\n", (ackbuff + cnt)->ack_paylen, (ackbuff + cnt)->arc_cnt);
				}
			}

			continue;
		}

		palClearPad(rfp->config->ceport, rfp->config->cepad);

		if (status & NRF24L01_DI_STATUS_MAX_RT) {
			nrf24l01ReadRegister(rfp->config->spip, NRF24L01_AD_OBSERVE_TX, &buffer);
			(ackbuff + cnt)->arc_cnt = buffer & NRF24L01_DI_ARC_CNT;
			print_dbg("arc = %d\r\n", (ackbuff + cnt)->arc_cnt);
			nrf24l01Reset(rfp->config->spip);
			nrf24l01FlushTx(rfp->config->spip);
			ret = RF_ERROR;
			break;
		}

	}
	rfp->state = RF_READY;

	palClearPad(rfp->config->ceport, rfp->config->cepad);

	nrf24l01WriteRegister(rfp->config->spip, NRF24L01_AD_STATUS,
			NRF24L01_DI_CONFIG_MASK_MAX_RT | NRF24L01_DI_CONFIG_MASK_TX_DS
					| NRF24L01_DI_CONFIG_MASK_RX_DR);

	return ret;
}

/**
 * @brief   Receives a string from an addressed channel.
 *
 * @param[in] rfp        Pointer to the @p RFDriver object
 * @param[in] sp         String pointer. The associated buffer should be long at
 *                       least RF_PAYLEN + 1
 * @param[in] addp       Channel address as string
 * @param[in] time       The number of ticks before the operation timeouts,
 *                       the following special values are allowed:
 *                      - @a TIME_IMMEDIATE immediate timeout.
 *                      - @a TIME_INFINITE no timeout.
 *
 * @return               The operation result.
 * @retval RF_OK         The operation succeeds.
 * @retval RF_ERROR      Error during the transmission.
 * @api
 */
uint8_t rfReceiveData(RFDriver *rfp, uint8_t* buffer, uint8_t buffer_size, uint8_t* addp) {
  RFRxFrame _rxframe;
  _rxframe.rx_paylen = 0;
  rf_msg_t msg;
  uint8_t i;
  osalDbgCheck((rfp != NULL) && (buffer != NULL) && (addp != NULL) && (buffer_size >= RF_PAYLEN));

  osalDbgAssert((rfp->state == RF_READY),
              "rfReceive(), invalid state");

  for (i = 0; i < (uint8_t)rfp->config->address_width + 2; i++) {
    _rxframe.rx_address[i] = *addp;
    addp++;
  }

	msg = rfReceiveFrame(rfp, 1, &_rxframe);
	if (msg == RF_OK) {
		//print_dbg("paylen: %d\r\n", _rxframe.rx_paylen);
		for (i = 0; i < _rxframe.rx_paylen; i++) {
			*buffer = _rxframe.rx_payload[i];
			buffer++;
		}

	} else {
		//print_dbg("NOK: %d\r\n", msg);
	}

	return _rxframe.rx_paylen;
}

/**
 * @brief   Transmits data from an addressed channel.
 *
 * @param[in] rfp        Pointer to the @p RFDriver object
 * @param[in] buffer     Send buffer. The size of the buffer cannot be longer than
 *                       RF_PAYLEN
 * @param[in] buffer_size Send buffer size. The size of the buffer cannot be longer than
 *                       RF_PAYLEN
 * @param[in] addp       Channel address as string
 *
 * @return               The operation result.
 * @retval RF_OK         The operation succeeds.
 * @retval RF_ERROR      Error during the transmission.
 * @api
 */
rf_msg_t rfTransmitData(RFDriver *rfp, uint8_t* buffer, uint8_t buffer_size, uint8_t* addp){

  RFTxFrame _txframe;
  RFAckFrame _ackframe;
  rf_msg_t msg;
  uint8_t i;

  osalDbgCheck((rfp != NULL) && (buffer != NULL) && (addp != NULL));
  osalDbgAssert((rfp->state == RF_READY), "rfReceive(), invalid state");

  for (i = 0; i < (uint8_t)rfp->config->address_width + 2; i++) {
    _txframe.tx_address[i] = *addp ;
    addp++;
  }

  if (buffer_size > RF_PAYLEN)
	  buffer_size = RF_PAYLEN;

  _txframe.tx_paylen = buffer_size;

  for(i = 0; i < buffer_size; i++){
    _txframe.tx_payload[i] = *buffer ;
    buffer++;
  }
  msg = rfTransmitFrame(rfp, 1, &_txframe, &_ackframe);
  return msg;
}

/**
 * @brief   This is the callback used by EXT on interrupt request.
 *
 * @notapi
 */
void rfExtCallBack(EXTDriver *extp, expchannel_t channel) {

  (void) extp;
  //palToggleLine(LINE_LED_GREEN);
  osalSysLockFromISR();
  cbcounter++;

  if(channel == RFD1.config->irqpad) {
    osalEventBroadcastFlagsI(&RFD1.irq_event, RF_GENERIC_IRQ);
  }
  osalSysUnlockFromISR();
}

#if (RF_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   Gains exclusive access to the RF.
 * @details This function tries to gain ownership to the RF, if the bus
 *          is already being used then the invoking thread is queued.
 * @pre     In order to use this function the option @p RF_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] rfp      pointer to the @p RFDriver object
 *
 * @api
 */
void rfAcquireBus(RFDriver *rfp) {

  osalDbgCheck(rfp != NULL);

  osalMutexLock(&rfp->mutex);
}

/**
 * @brief   Releases exclusive access to the RF.
 * @pre     In order to use this function the option @p RF_USE_MUTUAL_EXCLUSION
 *          must be enabled.
 *
 * @param[in] rfp      pointer to the @p RFDriver object
 *
 * @api
 */
void rfReleaseBus(RFDriver *rfp) {

  osalDbgCheck(rfp != NULL);

  osalMutexUnlock(&rfp->mutex);
}
#endif /* RF_USE_MUTUAL_EXCLUSION == TRUE */

NRF24L01_ADR_t getDataRate( RFDriver *rfp )
{
  //rf24_datarate_e result ;

  uint8_t buffer;
  nrf24l01ReadRegister(rfp->config->spip, NRF24L01_AD_RF_SETUP, &buffer);
  return buffer & NRF24L01_DI_RF_SETUP_RF_DR;

}

char *DataRateToString(NRF24L01_ADR_t dr) {
	if(dr == NRF24L01_ADR_1Mbps) {
		return "1MBPS";
	} else if (dr == NRF24L01_ADR_2Mbps) {
		return "2MBS";
	} else {
		return "UNKNOWN";
	}
}

NRF24L01_CRC_t getCrcLength(RFDriver *rfp) {
	//rf24_datarate_e result ;
	NRF24L01_CRC_t crc = NRF24L01_CRC_disabled;

	uint8_t config;
	uint8_t aa;
	nrf24l01ReadRegister(rfp->config->spip, NRF24L01_AD_RF_SETUP, &config);
	config = config & (NRF24L01_DI_CONFIG_CRCO | NRF24L01_DI_CONFIG_EN_CRC);
	nrf24l01ReadRegister(rfp->config->spip, NRF24L01_AD_EN_AA, &aa);

	if(config & NRF24L01_DI_CONFIG_EN_CRC || aa){
		if(config & NRF24L01_DI_CONFIG_CRCO) {
			return NRF24L01_CRC_16bit;
		} else {
			return NRF24L01_CRC_8bit;
		}
	}

	return crc;
}

char *crcLengthToString(NRF24L01_CRC_t crc) {
	if (crc == NRF24L01_CRC_disabled) {
		return "Disabled";
	} else if (crc == NRF24L01_CRC_16bit) {
		return "16 bits";
	} else {
		return "8 bits";
	}
}


void print_status(uint8_t status)
{
  print_dbg("STATUS\t\t = 0x%02x RX_DR=%x TX_DS=%x MAX_RT=%x RX_P_NO=%x TX_FULL=%x\r\n",
           status,
           (status & NRF24L01_DI_STATUS_RX_DR)?1:0,
           (status & NRF24L01_DI_STATUS_TX_DS)?1:0,
           (status & NRF24L01_DI_STATUS_MAX_RT)?1:0,
           ((status >> 1) & 0x07),
           (status & NRF24L01_DI_STATUS_TX_FULL)?1:0
          );
}

void print_fifo_status(uint8_t status)
  {
    print_dbg("FIFO STATUS\t\t = 0x%02x TX_REUSE=%x TX_FULL=%x TX_EMPTY=%x RX_FULL=%x RX_EMPTY=%x\r\n",
             status,
             (status & NRF24L01_DI_FIFO_STATUS_TX_REUSE)?1:0,
             (status & NRF24L01_DI_FIFO_STATUS_TX_FULL)?1:0,
             (status & NRF24L01_DI_FIFO_STATUS_TX_EMPTY)?1:0,
             (status & NRF24L01_DI_FIFO_STATUS_RX_FULL)?1:0,
             (status & NRF24L01_DI_FIFO_STATUS_RX_EMPTY)?1:0
            );
  }

void print_address_register(RFDriver *rfp, const char* name, uint8_t reg, uint8_t qty)
{
    print_dbg("%s\t =",name);

  while (qty--)
  {
	  // TODO buffer size
    uint8_t buffer[5];
    nrf24l01ReadRegisterRange(rfp->config->spip, reg++,buffer, rfp->config->address_width);

    print_dbg(" 0x");
    uint8_t* bufptr = buffer + rfp->config->address_width;
    while( --bufptr >= buffer )
      print_dbg("%02x",*bufptr);

    //print_dbg("(%s)", buffer);

  }

  print_dbg("\r\n");
}

void print_byte_register(RFDriver *rfp, const char* name, uint8_t reg, uint8_t qty)
{
	print_dbg("%s(0x%02x)\t =", name, reg);

	uint8_t buffer;
  while (qty--) {

	      nrf24l01ReadRegister(rfp->config->spip, reg++,&buffer);
	      print_dbg(" 0x%02x", buffer);

  }
  print_dbg("\r\n");
}


void printDetails(RFDriver *rfp)
{
	uint8_t buffer;

  print_status(nrf24l01GetStatus(rfp->config->spip));

  print_address_register(rfp, "RX_ADDR_P0-1",NRF24L01_AD_RX_ADDR_P0,2);
  print_byte_register(rfp, "RX_ADDR_P2-5",NRF24L01_AD_RX_ADDR_P2,4);
  print_address_register(rfp, "TX_ADDR\t",NRF24L01_AD_TX_ADDR, 1);
//
//  print_byte_register(PSTR("RX_PW_P0-6"),RX_PW_P0,6);
  print_byte_register(rfp, "EN_AA\t",NRF24L01_AD_EN_AA, 1);
  print_byte_register(rfp, "EN_RXADDR" ,NRF24L01_AD_EN_RXADDR, 1);
//  print_byte_register(rfp, "RF_CH\t",NRF24L01_AD_RF_CH, 1);
//  print_byte_register(rfp, "RF_SETUP",NRF24L01_AD_RF_SETUP, 1);
  print_byte_register(rfp, "CONFIG\t",NRF24L01_AD_CONFIG, 1);
  print_byte_register(rfp, "DYNPD/FEATURE",NRF24L01_AD_DYNPD,2);

  nrf24l01ReadRegister(rfp->config->spip, NRF24L01_AD_FIFO_STATUS,&buffer);
  print_fifo_status(buffer);

  print_dbg("Data Rate\t = %s\r\n", DataRateToString(getDataRate(rfp)));
//  printf_P(PSTR("Model\t\t = " PRIPSTR "\r\n"),pgm_read_word(&rf24_model_e_str_P[isPVariant()]));
  print_dbg("CRC Length\t = %s\r\n", crcLengthToString(getCrcLength(rfp)));
//  printf_P(PSTR("PA Power\t = " PRIPSTR "\r\n"),  pgm_read_word(&rf24_pa_dbm_e_str_P[getPALevel()]));

}

#endif /* USERLIB_USE_RF */

/** @} */
