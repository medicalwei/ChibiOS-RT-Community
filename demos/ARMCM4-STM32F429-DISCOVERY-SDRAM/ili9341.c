/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

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

/**
 * @file    ili9341.c
 * @brief   ILI9341 TFT LCD diaplay controller driver.
 * @note    Does not support multiple calling threads natively.
 */

#include "ch.h"
#include "hal.h"
#include "ili9341.h"

/**
 * @addtogroup ili9341
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

void ili9341ObjectInit(ILI9341Driver *driverp) {

  chDbgCheck((driverp != NULL), "ili9341ObjectInit");

  driverp->state = ILI9341_UNINIT;
  driverp->config = NULL;
}

void ili9341Init(ILI9341Driver *driverp) {

  chDbgCheck((driverp != NULL), "ili9341Init");
  chDbgCheck((driverp->state == ILI9341_UNINIT), "ili9341Init");

  driverp->state = ILI9341_STOP;
}

void ili9341Start(ILI9341Driver *driverp, const ILI9341Config *configp) {

  chDbgCheck((driverp != NULL), "ili9341Start");
  chDbgCheck((driverp->state == ILI9341_STOP), "ili9341Start");
  chDbgCheck((configp != NULL), "ili9341Start");
  chDbgCheck((configp->spi != NULL), "ili9341Start");

  spiSelectI(configp->spi);
  spiUnselectI(configp->spi);
  driverp->config = configp;
  driverp->state = ILI9341_READY;
}

void ili9341Stop(ILI9341Driver *driverp) {

  chDbgCheck((driverp != NULL), "ili9341Stop");
  chDbgCheck((driverp->state == ILI9341_READY), "ili9341Stop");

  driverp->state = ILI9341_STOP;
}

#if ILI9341_IM == ILI9341_IM_4LSI_1 /* 4-wire, half-duplex */

void ili9341Select(ILI9341Driver *driverp) {

  chDbgCheck((driverp != NULL), "ili9341Select");
  chDbgCheck((driverp->state == ILI9341_READY), "ili9341Select");

  driverp->state = ILI9341_ACTIVE;
  spiSelect(driverp->config->spi);
}

void ili9341Unselect(ILI9341Driver *driverp) {

  chDbgCheck((driverp != NULL), "ili9341Unselect");
  chDbgCheck((driverp->state == ILI9341_ACTIVE), "ili9341Unselect");

  spiUnselect(driverp->config->spi);
  driverp->state = ILI9341_READY;
}

void ili9341WriteCommand(ILI9341Driver *driverp, uint8_t cmd) {

  chDbgCheck((driverp != NULL), "ili9341WriteCommand");
  chDbgCheck((driverp->state == ILI9341_ACTIVE), "ili9341WriteCommand");

  driverp->value = cmd;
  palClearPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* !Cmd */
  spiSend(driverp->config->spi, 1, &driverp->value);
}

void ili9341WriteByte(ILI9341Driver *driverp, uint8_t value) {

  chDbgCheck((driverp != NULL), "ili9341WriteByte");
  chDbgCheck((driverp->state == ILI9341_ACTIVE), "ili9341WriteByte");

  driverp->value = value;
  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad); /* Data */
  spiSend(driverp->config->spi, 1, &driverp->value);
}

uint8_t ili9341ReadByte(ILI9341Driver *driverp) {

  chDbgAssert(FALSE, "ili9341ReadByte()", "should not be used");

  chDbgCheck((driverp != NULL), "ili9341ReadByte");
  chDbgCheck((driverp->state == ILI9341_ACTIVE), "ili9341ReadByte");

  spiReceive(driverp->config->spi, 1, &driverp->value);
  return driverp->value;
}

void ili9341WriteChunk(ILI9341Driver *driverp, const uint8_t chunk[],
                       size_t length) {

  chDbgCheck((driverp != NULL), "ili9341WriteChunk");
  chDbgCheck((chunk != NULL), "ili9341WriteChunk");
  chDbgCheck((driverp->state == ILI9341_ACTIVE), "ili9341WriteChunk");

  if (length == 0)
    return;

  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad);
  spiSend(driverp->config->spi, length, chunk);
}

void ili9341ReadChunk(ILI9341Driver *driverp, uint8_t chunk[],
                      size_t length) {

  chDbgCheck((driverp != NULL), "ili9341ReadChunk");
  chDbgCheck((chunk != NULL), "ili9341ReadChunk");
  chDbgCheck((driverp->state == ILI9341_ACTIVE), "ili9341ReadChunk");

  if (length == 0)
    return;

  palSetPad(driverp->config->dcx_port, driverp->config->dcx_pad);
  spiReceive(driverp->config->spi, length, chunk);
}

#else /* ILI9341_IM == * */
#error "Only ILI9341_IM_4LSI_1 interface mode is supported"
#endif /* ILI9341_IM == * */

/** @} */
