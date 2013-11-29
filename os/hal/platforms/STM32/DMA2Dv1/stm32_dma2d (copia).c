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
 * @file    stm32_dma2d.c
 * @brief   DMA2D/Chrom-ART driver.
 */

#include "ch.h"
#include "hal.h"
#include "stm32_dma2d.h"

#if STM32_DMA2D_USE_DMA2D || defined(__DOXYGEN__)

/* Ignore annoying warning messages for actually safe code.*/
#if __GNUC__
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif

/**
 * @addtogroup dma2d
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief DMA2DD1 driver identifier.*/
DMA2DDriver DMA2DD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Bits per pixel lookup table.
 */
static const uint8_t dma2d_bpp[11] = {
  32, /* DMA2D_FMT_ARGB8888 */
  24, /* DMA2D_FMT_RGB888 */
  16, /* DMA2D_FMT_RGB565 */
  16, /* DMA2D_FMT_ARGB1555 */
  16, /* DMA2D_FMT_ARGB4444 */
   8, /* DMA2D_FMT_L8 */
   8, /* DMA2D_FMT_AL44 */
  16, /* DMA2D_FMT_AL88 */
   4, /* DMA2D_FMT_L4 */
   8, /* DMA2D_FMT_A8 */
   4 /* DMA2D_FMT_A4 */
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   DMA2D global interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(DMA2D_IRQHandler) {

  DMA2DDriver *const dma2dp = &DMA2DD1;

  CH_IRQ_PROLOGUE();

  /* Handle Configuration Error ISR.*/
  if ((DMA2D->ISR & DMA2D_ISR_CEIF) && (DMA2D->CR & DMA2D_CR_CEIE)) {
    chDbgAssert(dma2dp->config->cfgerr_isr != NULL,
                "DMA2D_IRQHandler(), #1", "invalid state");
    dma2dp->config->cfgerr_isr(dma2dp);
    DMA2D->IFCR |= DMA2D_IFSR_CCEIF;
  }

  /* Handle CLUT (Palette) Transfer Complete ISR.*/
  if ((DMA2D->ISR & DMA2D_ISR_CTCIF) && (DMA2D->CR & DMA2D_CR_CTCIE)) {
    chDbgAssert(dma2dp->config->paltrfdone_isr != NULL,
                "DMA2D_IRQHandler(), #2", "invalid state");
    dma2dp->config->paltrfdone_isr(dma2dp);
    DMA2D->IFCR |= DMA2D_IFSR_CCTCIF;
  }

  /* Handle CLUT (Palette) Access Error ISR.*/
  if ((DMA2D->ISR & DMA2D_ISR_CAEIF) && (DMA2D->CR & DMA2D_CR_CAEIE)) {
    chDbgAssert(dma2dp->config->palacserr_isr != NULL,
                "DMA2D_IRQHandler(), #3", "invalid state");
    dma2dp->config->palacserr_isr(dma2dp);
    DMA2D->IFCR |= DMA2D_IFSR_CCAEIF;
  }

  /* Handle Transfer Watermark ISR.*/
  if ((DMA2D->ISR & DMA2D_ISR_TWIF) && (DMA2D->CR & DMA2D_CR_TWIE)) {
    chDbgAssert(dma2dp->config->trfwmark_isr != NULL,
                "DMA2D_IRQHandler(), #4", "invalid state");
    dma2dp->config->trfwmark_isr(dma2dp);
    DMA2D->IFCR |= DMA2D_IFSR_CTWIF;
  }

  /* Handle Transfer Complete ISR.*/
  if ((DMA2D->ISR & DMA2D_ISR_TCIF) && (DMA2D->CR & DMA2D_CR_TCIE)) {
    chDbgAssert(dma2dp->config->trfdone_isr != NULL,
                "DMA2D_IRQHandler(), #5", "invalid state");
    dma2dp->config->trfdone_isr(dma2dp);
    DMA2D->IFCR |= DMA2D_IFSR_CTCIF;
  }

  /* Handle Transfer Error ISR.*/
  if ((DMA2D->ISR & DMA2D_ISR_TEIF) && (DMA2D->CR & DMA2D_CR_TEIE)) {
    chDbgAssert(dma2dp->config->trferr_isr != NULL,
                "DMA2D_IRQHandler(), #6", "invalid state");
    dma2dp->config->trferr_isr(dma2dp);
    DMA2D->IFCR |= DMA2D_IFSR_CTEIF;
  }

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   DMA2D Driver initialization.
 * @details Initializes the DMA2D subsystem and chosen drivers. Should be
 *          called at board initialization.
 *
 * @init
 */
void dma2dInit(void) {

  /* Reset the DMA2D hardware module.*/
  rccResetDMA2D();

  /* Enable the DMA2D clock.*/
  rccEnableDMA2D(FALSE);

  /* Driver struct initialization.*/
  dma2dObjectInit(&DMA2DD1);
  DMA2DD1.state = DMA2D_STOP;
}

/**
 * @brief   Initializes the standard part of a @p DMA2DDriver structure.
 *
 * @param[out] dma2dp   pointer to the @p DMA2DDriver object
 *
 * @init
 */
void dma2dObjectInit(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dObjectInit");

  dma2dp->state = DMA2D_UNINIT;
  dma2dp->config = NULL;
  chMtxInit(&dma2dp->lock);
}

/**
 * @brief   Get the driver state.
 *
 * @param[in] dma2dp    pointer to the @p DMA2DDriver object
 *
 * @retun               driver state
 *
 * @iclass
 */
dma2d_state_t dma2dGetStateI(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dGetStateI");
  chDbgCheckClassI();

  return dma2dp->state;
}

/**
 * @brief   Get the driver state.
 *
 * @param[in] dma2dp    pointer to the @p DMA2DDriver object
 *
 * @retun               driver state
 *
 * @api
 */
dma2d_state_t dma2dGetState(DMA2DDriver *dma2dp) {

  dma2d_state_t state;
  chSysLock();
  state = dma2dGetStateI(dma2dp);
  chSysUnlock();
  return state;
}

/**
 * @brief   Configures and activates the DMA2D peripheral.
 *
 * @param[in] dma2dp    pointer to the @p DMA2DDriver object
 * @param[in] confip    pointer to the @p DMA2DConfig object
 *
 * @api
 */
void dma2dStart(DMA2DDriver *dma2dp, const DMA2DConfig *configp) {

  chSysLock();

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dStart");
  chDbgCheck(configp != NULL, "dma2dStart");
  chDbgAssert(dma2dp->state == DMA2D_STOP,
              "dma2dStart(), #1", "invalid state");

  dma2dp->config = configp;

  /* Turn off the controller and its interrupts.*/
  DMA2D->CR = 0;

  // TODO

  dma2dp->state = DMA2D_READY;
  chSysUnlock();
}

/**
 * @brief   Deactivates the DMA2D peripheral.
 *
 * @param[in] dma2dp    pointer to the @p DMA2DDriver object
 *
 * @api
 */
void dma2dStop(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dStop");
  chDbgAssert(dma2dp->state == DMA2D_READY,
              "dma2dStop(), #1", "invalid state");

  dma2dp->state = DMA2D_STOP;
}

#if DMA2D_NEED_MULTITHREADING

void dma2dAcquireS(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dAcquireS");
  chDbgCheckClassS();

  chMtxLockS(&dma2dp->lock);
}

void dma2dAcquire(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dAcquire");

  chSysLock();
  dma2dAcquireS(dma2dp);
  chSysUnlock();
}

void dma2dReleaseS(DMA2DDriver *dma2dp) {

  Mutex *lockp;

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dReleaseS");
  chDbgCheckClassS();
  (void)dma2dp;
  (void)lockp;

  lockp = chMtxUnlockS();
  chDbgCheck(lockp == &dma2dp->lock, "dma2dReleaseS");
}

void dma2dRelease(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dRelease");

  chSysLock();
  dma2dReleaseS(dma2dp);
  chSysUnlock();
}

#endif

uint16_t dma2dGetWatermarkPos(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dGetWatermarkPos");
  (void)dma2dp;

  return (uint16_t)(DMA2D->LWR & DMA2D_LWR_LW);
}

void dma2dSetWatermarkPos(DMA2DDriver *dma2dp, uint16_t line) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dSetWatermarkPos");
  (void)dma2dp;

  DMA2D->LWR = ((uint32_t)line & DMA2D_LWR_LW);
}

bool_t dma2dIsWatermarkEnabled(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dIsWatermarkEnabled");
  (void)dma2dp;

  return (DMA2D->CR & DMA2D_CR_TWIE) != 0;
}

void dma2dEnableWatermark(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dEnableWatermark");
  (void)dma2dp;

  DMA2D->CR |= DMA2D_CR_TWIE;
}

void dma2dDisableWatermark(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dDisableWatermark");
  (void)dma2dp;

  DMA2D->CR &= ~DMA2D_CR_TWIE;
}

uint32_t dma2dGetDeadTime(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dGetDeadTime");
  (void)dma2dp;

  return (DMA2D->AMTCR & DMA2D_AMTCR_DT) >> 8;
}

void dma2dSetDeadTime(DMA2DDriver *dma2dp, uint32_t cycles) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dSetDeadTime");
  chDbgAssert(cycles <= DMA2D_MAX_DEADTIME_CYCLES,
              "dma2dSetDeadTime(), #1", "outside range");
  (void)dma2dp;

  DMA2D->AMTCR = (DMA2D->AMTCR & ~DMA2D_AMTCR_DT) |
                 ((cycles << 8) & DMA2D_AMTCR_DT);
}

bool_t dma2dIsDeadTimeEnabled(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dIsDeadTimeEnabled");
  (void)dma2dp;

  return (DMA2D->AMTCR & DMA2D_AMTCR_EN) != 0;
}

void dma2dEnableDeadTime(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dEnableDeadTime");
  (void)dma2dp;

  DMA2D->AMTCR |= DMA2D_AMTCR_EN;
}

void dma2dDisableDeadTime(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dDisableDeadTime");
  (void)dma2dp;

  DMA2D->AMTCR &= ~DMA2D_AMTCR_EN;
}

void dma2dGetSize(DMA2DDriver *dma2dp, uint16_t *widthp, uint16_t *heightp) {

  uint32_t r;

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dGetSize");
  chDbgCheck(widthp != NULL, "dma2dGetSize");
  chDbgCheck(heightp != NULL, "dma2dGetSize");
  (void)dma2dp;

  r = DMA2D->NLR;
  *widthp  = (r & DMA2D_NLR_PL) >> 16;
  *heightp = (r & DMA2D_NLR_NL) >>  0;
}

void dma2dSetSize(DMA2DDriver *dma2dp, uint16_t width, uint16_t height) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dSetSize");
  chDbgAssert(width <= DMA2D_MAX_WIDTH,
              "dma2dSetSize(), #1", "outside range");
  chDbgAssert(height <= DMA2D_MAX_HEIGHT,
              "dma2dSetSize(), #2", "outside range");
  (void)dma2dp;

  DMA2D->NLR = (((uint32_t)width  << 16) & DMA2D_NLR_PL) |
               (((uint32_t)height <<  0) & DMA2D_NLR_NL);
}

void dma2dJobStart(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dJobStart");
  (void)dma2dp;

  DMA2D->CR |= DMA2D_CR_START;
}

void dma2dJobWaitCompletion(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dJobWaitCompletion");
  (void)dma2dp;

  while (DMA2D->CR & DMA2D_CR_START)
    ;
}

void dma2dJobSuspend(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dJobSuspend");
  (void)dma2dp;

  DMA2D->CR |= DMA2D_CR_SUSP;
}

void dma2dJobResume(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dJobResume");
  (void)dma2dp;

  DMA2D->CR &= ~DMA2D_CR_SUSP;
}

void dma2dJobAbort(DMA2DDriver *dma2dp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dJobAbort");
  (void)dma2dp;

  DMA2D->CR |= DMA2D_CR_ABORT;
}

void *dma2dLayerGetAddress(DMA2DDriver *dma2dp, dma2d_layerid_t layer) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetAddress");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    return (void *)DMA2D->BGMAR;
  case DMA2D_FG:
    return (void *)DMA2D->FGMAR;
  case DMA2D_OUT:
    return (void *)DMA2D->OMAR;
  default:
    chDbgAssert(FALSE, "dma2dLayerGetAddress(), #1", "invalid layer");
    return NULL;
  }
}

void dma2dLayerSetAddress(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                          void *bufferp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetAddress");
  chDbgCheck(dma2dIsAligned(bufferp, dma2dLayerGetPixelFormat(dma2dp, layer)),
             "dma2dLayerSetAddress");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGMAR = (uint32_t)bufferp;
    return;
  case DMA2D_FG:
    DMA2D->FGMAR = (uint32_t)bufferp;
    return;
  case DMA2D_OUT:
    DMA2D->OMAR = (uint32_t)bufferp;
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetAddress(), #1", "invalid layer");
    return;
  }
}

size_t dma2dLayerGetOffset(DMA2DDriver *dma2dp, dma2d_layerid_t layer) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetOffset");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    return (size_t)(DMA2D->BGOR & DMA2D_BGOR_LO);
  case DMA2D_FG:
    return (size_t)(DMA2D->FGOR & DMA2D_FGOR_LO);
  case DMA2D_OUT:
    return (size_t)(DMA2D->OOR & DMA2D_OOR_LO);
  default:
    chDbgAssert(FALSE, "dma2dLayerGetOffset(), #1", "invalid layer");
    return 0;
  }
}

void dma2dLayerSetOffset(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                         size_t offset) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetOffset");
  chDbgAssert(offset <= DMA2D_MAX_OFFSET,
              "dma2dLayerSetOffset(), #1", "outside range");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGOR = (uint32_t)offset & DMA2D_BGOR_LO;
    return;
  case DMA2D_FG:
    DMA2D->FGOR = (uint32_t)offset & DMA2D_FGOR_LO;
    return;
  case DMA2D_OUT:
    DMA2D->OOR = (uint32_t)offset & DMA2D_OOR_LO;
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetOffset(), #1", "invalid layer");
    return;
  }
}

uint8_t dma2dLayerGetConstantAlpha(DMA2DDriver *dma2dp,
                                   dma2d_layerid_t layer) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetConstantAlpha");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    return (uint8_t)((DMA2D->BGPFCCR & DMA2D_BGPFCCR_ALPHA) >> 24);
  case DMA2D_FG:
    return (uint8_t)((DMA2D->FGPFCCR & DMA2D_FGPFCCR_ALPHA) >> 24);
  default:
    chDbgAssert(FALSE, "dma2dLayerGetConstantAlpha(), #1", "invalid layer");
    return 0;
  }
}

void dma2dLayerSetConstantAlpha(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                                uint8_t a) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetConstantAlpha");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGPFCCR = (DMA2D->BGPFCCR & ~DMA2D_BGPFCCR_ALPHA) |
                     (((uint32_t)a << 24) & DMA2D_BGPFCCR_ALPHA);
    return;
  case DMA2D_FG:
    DMA2D->FGPFCCR = (DMA2D->FGPFCCR & ~DMA2D_FGPFCCR_ALPHA) |
                     (((uint32_t)a << 24) & DMA2D_FGPFCCR_ALPHA);
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetConstantAlpha(), #1", "invalid layer");
    return;
  }
}

dma2d_amode_t dma2dLayerGetAlphaMode(DMA2DDriver *dma2dp,
                                     dma2d_layerid_t layer) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetAlphaMode");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    return (dma2d_amode_t)(DMA2D->BGPFCCR & DMA2D_BGPFCCR_AM);
  case DMA2D_FG:
    return (dma2d_amode_t)(DMA2D->FGPFCCR & DMA2D_FGPFCCR_AM);
  default:
    chDbgAssert(FALSE, "dma2dLayerGetAlphaMode(), #1", "invalid layer");
    return 0;
  }
}

void dma2dLayerSetAlphaMode(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                            dma2d_amode_t mode) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetAlphaMode");
  chDbgAssert((mode & ~DMA2D_BGPFCCR_AM) == 0,
             "dma2dLayerSetAlphaMode(), #1", "outside range");
  chDbgAssert((mode & DMA2D_BGPFCCR_AM) != DMA2D_BGPFCCR_AM,
             "dma2dLayerSetAlphaMode(), #2", "outside range");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGPFCCR = (DMA2D->BGPFCCR & ~DMA2D_BGPFCCR_AM) |
                     ((uint32_t)mode & DMA2D_BGPFCCR_AM);
    return;
  case DMA2D_FG:
    DMA2D->FGPFCCR = (DMA2D->FGPFCCR & ~DMA2D_FGPFCCR_AM) |
                     ((uint32_t)mode & DMA2D_FGPFCCR_AM);
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetConstantAlpha(), #1", "invalid layer");
    return;
  }
}

dma2d_pixfmt_t dma2dLayerGetPixelFormat(DMA2DDriver *dma2dp,
                                        dma2d_layerid_t layer) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetPixelFormat");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    return (dma2d_pixfmt_t)(DMA2D->BGPFCCR & DMA2D_BGPFCCR_CM);
  case DMA2D_FG:
    return (dma2d_pixfmt_t)(DMA2D->FGPFCCR & DMA2D_FGPFCCR_CM);
  case DMA2D_OUT:
    return (dma2d_pixfmt_t)(DMA2D->OPFCCR & DMA2D_OPFCCR_CM);
  default:
    chDbgAssert(FALSE, "dma2dLayerGetPixelFormat(), #1", "invalid layer");
    return 0;
  }
}

void dma2dLayerSetPixelFormat(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                              dma2d_pixfmt_t fmt) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetPixelFormat");
  chDbgAssert(fmt <= DMA2D_MAX_PIXFMT_ID,
              "dma2dLayerSetPixelFormat(), #1", "outside range");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGPFCCR = (DMA2D->BGPFCCR & ~DMA2D_BGPFCCR_CM) |
                     ((uint32_t)fmt & DMA2D_BGPFCCR_CM);
    return;
  case DMA2D_FG:
    DMA2D->FGPFCCR = (DMA2D->FGPFCCR & ~DMA2D_FGPFCCR_CM) |
                     ((uint32_t)fmt & DMA2D_FGPFCCR_CM);
    return;
  case DMA2D_OUT:
    chDbgAssert(fmt <= DMA2D_MAX_OUTPIXFMT_ID,
                "dma2dLayerSetPixelFormat(), #2", "outside range");
    DMA2D->OPFCCR = (DMA2D->OPFCCR & ~DMA2D_OPFCCR_CM) |
                    ((uint32_t)fmt & DMA2D_OPFCCR_CM);
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetPixelFormat(), #3", "invalid layer");
    return;
  }
}

dma2d_color_t dma2dLayerGetDefaultColor(DMA2DDriver *dma2dp,
                                        dma2d_layerid_t layer) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetDeafultColor");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    return (dma2d_color_t)(DMA2D->BGCOLR & 0x00FFFFFF);
  case DMA2D_FG:
    return (dma2d_color_t)(DMA2D->FGCOLR & 0x00FFFFFF);
  case DMA2D_OUT:
    return (dma2d_color_t)(DMA2D->OCOLR & 0x00FFFFFF);
  default:
    chDbgAssert(FALSE, "dma2dLayerGetDeafultColor(), #1", "invalid layer");
    return 0;
  }
}

void dma2dLayerSetDefaultColor(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                               dma2d_color_t c) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetDefaultColor");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGCOLR = (uint32_t)c & 0x00FFFFFF;
    return;
  case DMA2D_FG:
    DMA2D->FGCOLR = (uint32_t)c & 0x00FFFFFF;
    return;
  case DMA2D_OUT:
    DMA2D->OCOLR = (uint32_t)c & 0x00FFFFFF;
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetDeafultColor(), #1", "invalid layer");
    return;
  }
}

void dma2dLayerGetPalette(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                          dma2d_palcfg_t *palettep) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGetPalette");
  chDbgCheck(palettep != NULL, "dma2dLayerGetPalette");
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    palettep->colorsp = (const void *)DMA2D->BGCLUT;
    palettep->length = (uint16_t)((DMA2D->BGPFCCR & DMA2D_BGPFCCR_CS) >> 8) + 1;
    palettep->fmt = (dma2d_pixfmt_t)((DMA2D->BGPFCCR & DMA2D_BGPFCCR_CCM) >> 4);
    return;
  case DMA2D_FG:
    palettep->colorsp = (const void *)DMA2D->FGCLUT;
    palettep->length = (uint16_t)((DMA2D->FGPFCCR & DMA2D_FGPFCCR_CS) >> 8) + 1;
    palettep->fmt = (dma2d_pixfmt_t)((DMA2D->FGPFCCR & DMA2D_FGPFCCR_CCM) >> 4);
    return;
  default:
    chDbgAssert(FALSE, "dma2dLayerGetPalette(), #1", "invalid layer");
    return;
  }
}

void dma2dLayerSetPalette(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                          const dma2d_palcfg_t *palettep) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSetPalette");
  chDbgCheck(palettep != NULL, "dma2dLayerSetPalette");
  chDbgCheck(palettep->colorsp != NULL, "dma2dLayerSetPalette");
  chDbgAssert(palettep->length > 0,
              "dma2dLayerSetPalette(), #1", "outside range");
  chDbgAssert(palettep->length <= DMA2D_MAX_PALETTE_LENGTH,
              "dma2dLayerSetPalette(), #2", "outside range");
  chDbgAssert((palettep->fmt == DMA2D_FMT_ARGB8888) ||
              (palettep->fmt == DMA2D_FMT_RGB888),
              "dma2dLayerSetPalette(), #3", "invalid format")
  (void)dma2dp;

  switch (layer) {
  case DMA2D_BG:
    DMA2D->BGCMAR = (uint32_t)palettep->colorsp;
    DMA2D->BGPFCCR = (DMA2D->BGPFCCR & ~DMA2D_BGPFCCR_CS) |
                     ((((uint32_t)palettep->length - 1) << 8) &
                      DMA2D_BGPFCCR_CS) | ((uint32_t)palettep->fmt << 4);
    DMA2D->BGPFCCR |= DMA2D_BGPFCCR_START;
    while (DMA2D->BGPFCCR & DMA2D_BGPFCCR_START)
      ;
    break;
  case DMA2D_FG:
    DMA2D->FGCMAR = (uint32_t)palettep->colorsp;
    DMA2D->FGPFCCR = (DMA2D->FGPFCCR & ~DMA2D_FGPFCCR_CS) |
                     ((((uint32_t)palettep->length - 1) << 8) &
                      DMA2D_FGPFCCR_CS) | ((uint32_t)palettep->fmt << 4);
    DMA2D->FGPFCCR |= DMA2D_FGPFCCR_START;
    while (DMA2D->FGPFCCR & DMA2D_FGPFCCR_START)
      ;
    break;
  default:
    chDbgAssert(FALSE, "dma2dLayerSetPalette(), #1", "invalid layer");
    return;
  }
}

void dma2dLayerGet(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                   dma2d_laycfg_t *cfgp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerGet");
  chDbgCheck(cfgp != NULL, "dma2dLayerGet");

  cfgp->bufferp = dma2dLayerGetAddress(dma2dp, layer);
  cfgp->offset = dma2dLayerGetOffset(dma2dp, layer);
  cfgp->fmt = dma2dLayerGetPixelFormat(dma2dp, layer);
  cfgp->def_color = dma2dLayerGetDefaultColor(dma2dp, layer);
  cfgp->const_alpha = dma2dLayerGetConstantAlpha(dma2dp, layer);
  if (cfgp->palettep != NULL)
    dma2dLayerGetPalette(dma2dp, layer, (dma2d_palcfg_t *)cfgp->palettep);
}

void dma2dLayerSet(DMA2DDriver *dma2dp, dma2d_layerid_t layer,
                   const dma2d_laycfg_t *cfgp) {

  chDbgCheck(dma2dp == &DMA2DD1, "dma2dLayerSet");
  chDbgCheck(cfgp != NULL, "dma2dLayerSet");

  dma2dLayerSetAddress(dma2dp, layer, cfgp->bufferp);
  dma2dLayerSetOffset(dma2dp, layer, cfgp->offset);
  dma2dLayerSetPixelFormat(dma2dp, layer, cfgp->fmt);
  dma2dLayerSetDefaultColor(dma2dp, layer, cfgp->def_color);
  dma2dLayerSetConstantAlpha(dma2dp, layer, cfgp->const_alpha);
  if (cfgp->palettep != NULL)
    dma2dLayerSetPalette(dma2dp, layer, cfgp->palettep);
}

bool_t dma2dIsAligned(const void *bufferp, dma2d_pixfmt_t fmt) {

  switch (fmt) {
  case DMA2D_FMT_ARGB8888:
  case DMA2D_FMT_RGB888:
    return ((uintptr_t)bufferp & 3) == 0;
  case DMA2D_FMT_RGB565:
  case DMA2D_FMT_ARGB1555:
  case DMA2D_FMT_ARGB4444:
  case DMA2D_FMT_AL88:
    return ((uintptr_t)bufferp & 1) == 0;
  case DMA2D_FMT_L8:
  case DMA2D_FMT_AL44:
  case DMA2D_FMT_L4:
  case DMA2D_FMT_A8:
  case DMA2D_FMT_A4:
    return TRUE;
  default:
    chDbgAssert(FALSE, "dma2dIsAligned(), #1", "invalid format");
    return FALSE;
  }
}

/**
 * @brief   Compute bits per pixel.
 * @details Computes the bits per pixel for the specified pixel format.
 *
 * @param[in] fmt       pixel format
 *
 * @retuen              bits per pixel
 *
 * @api
 */
size_t dma2dBitsPerPixel(dma2d_pixfmt_t fmt) {

  chDbgAssert(fmt < DMA2D_MAX_PIXFMT_ID,
              "dma2dBitsPerPixel(), #1", "invalid format");

  return (size_t)dma2d_bpp[(unsigned)fmt];
}

#if DMA2D_NEED_CONVERSIONS || defined(__DOXYGEN__)

/**
 * @brief   Convert from ARGB-8888.
 * @details Converts an ARGB-8888 color to the specified pixel format.
 *
 * @param[in] c         color, ARGB-8888
 * @param[in] fmt       target pixel format
 *
 * @return              raw color value for the target pixel format, left
 *                      padded with zeros.
 *
 * @api
 */
dma2d_color_t dma2dFromARGB8888(dma2d_color_t c, dma2d_pixfmt_t fmt) {

  switch (fmt) {
  case DMA2D_FMT_ARGB8888: {
    return c;
  }
  case DMA2D_FMT_RGB888: {
    return c & 0x00FFFFFF;
  }
  case DMA2D_FMT_RGB565: {
    return ((c & 0x000000F8) >> ( 8 -  5)) |
           ((c & 0x0000FC00) >> (16 - 11)) |
           ((c & 0x00F80000) >> (24 - 16));
  }
  case DMA2D_FMT_ARGB1555: {
    return ((c & 0x000000F8) >> ( 8 -  5)) |
           ((c & 0x0000F800) >> (16 - 10)) |
           ((c & 0x00F80000) >> (24 - 15)) |
           ((c & 0x80000000) >> (32 - 16));
  }
  case DMA2D_FMT_ARGB4444: {
    return ((c & 0x000000F0) >> ( 8 -  4)) |
           ((c & 0x0000F000) >> (16 -  8)) |
           ((c & 0x00F00000) >> (24 - 12)) |
           ((c & 0xF0000000) >> (32 - 16));
  }
  case DMA2D_FMT_L8: {
    return c & 0x000000FF;
  }
  case DMA2D_FMT_AL44: {
    return ((c & 0x000000F0) >> ( 8 - 4)) |
           ((c & 0xF0000000) >> (32 - 8));
  }
  case DMA2D_FMT_AL88: {
    return ((c & 0x000000FF) >> ( 8 -  8)) |
           ((c & 0xFF000000) >> (32 - 16));
  }
  // TODO
  default:
    chDbgPanic("invalid format");
    return 0;
  }
}

/**
 * @brief   Convert to ARGB-8888.
 * @details Converts color of the specified pixel format to an ARGB-8888 color.
 *
 * @param[in] c         color for the source pixel format, left padded with
 *                      zeros.
 * @param[in] fmt       source pixel format
 *
 * @return              color in ARGB-8888 format
 *
 * @api
 */
dma2d_color_t dma2dToARGB8888(dma2d_color_t c, dma2d_pixfmt_t fmt) {

  switch (fmt) {
  case DMA2D_FMT_ARGB8888: {
    return c;
  }
  case DMA2D_FMT_RGB888: {
    return (c & 0x00FFFFFF) | 0xFF000000;
  }
  case DMA2D_FMT_RGB565: {
    register dma2d_color_t output = 0xFF000000;
    if (c & 0x001F) output |= ((c & 0x001F) << ( 8 -  5)) | 0x00000007;
    if (c & 0x07E0) output |= ((c & 0x07E0) << (16 - 11)) | 0x00000300;
    if (c & 0xF800) output |= ((c & 0xF800) << (24 - 16)) | 0x00070000;
    return output;
  }
  case DMA2D_FMT_ARGB1555: {
    register dma2d_color_t output = 0x00000000;
    if (c & 0x001F) output |= ((c & 0x001F) << ( 8 -  5)) | 0x00000007;
    if (c & 0x03E0) output |= ((c & 0x03E0) << (16 - 10)) | 0x00000700;
    if (c & 0x7C00) output |= ((c & 0x7C00) << (24 - 15)) | 0x00070000;
    if (c & 0x8000) output |= 0xFF000000;
    return output;
  }
  case DMA2D_FMT_ARGB4444: {
    register dma2d_color_t output = 0x00000000;
    if (c & 0x000F) output |= ((c & 0x000F) << ( 8 -  4)) | 0x0000000F;
    if (c & 0x00F0) output |= ((c & 0x00F0) << (16 -  8)) | 0x00000F00;
    if (c & 0x0F00) output |= ((c & 0x0F00) << (24 - 12)) | 0x000F0000;
    if (c & 0xF000) output |= ((c & 0xF000) << (32 - 16)) | 0x0F000000;
    return output;
  }
  case DMA2D_FMT_L8: {
    return (c & 0xFF) | 0xFF000000;
  }
  case DMA2D_FMT_AL44: {
    register dma2d_color_t output = 0x00000000;
    if (c & 0x0F) output |= ((c & 0x0F) << ( 8 - 4)) | 0x0000000F;
    if (c & 0xF0) output |= ((c & 0xF0) << (32 - 8)) | 0x0F000000;
    return output;
  }
  case DMA2D_FMT_AL88: {
    return ((c & 0x00FF) << ( 8 -  8)) |
           ((c & 0xFF00) << (32 - 16));
  }
  // TODO
  default:
    chDbgAssert(FALSE, "(), #1", "invalid format");
    return 0;
  }
}

#endif /* DMA2D_NEED_CONVERSIONS */

/** @} */

#endif /* STM32_DMA2D_USE_DMA2D */
