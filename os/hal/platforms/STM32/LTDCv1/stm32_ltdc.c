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
 * @file    stm32_ltdc.c
 * @brief   LCD-TFT Controller Driver.
 */

#include "ch.h"
#include "hal.h"
#include "stm32_ltdc.h"

#if STM32_LTDC_USE_LTDC || defined(__DOXYGEN__)

/* Ignore annoying warning messages for actually safe code.*/
#if __GNUC__
#pragma GCC diagnostic ignored "-Wtype-limits"
#endif

/**
 * @addtogroup ltdc
 * @{
 */

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief LTDC1 driver identifier.*/
LTDCDriver LTDCD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Bits per pixel lookup table.
 */
static const uint8_t ltdc_bpp[8] = {
  32, /* LTDC_FMT_ARGB8888 */
  24, /* LTDC_FMT_RGB888 */
  16, /* LTDC_FMT_RGB565 */
  16, /* LTDC_FMT_ARGB1555 */
  16, /* LTDC_FMT_ARGB4444 */
   8, /* LTDC_FMT_L8 */
   8, /* LTDC_FMT_AL44 */
  16  /* LTDC_FMT_AL88 */
};

/**
 * @brief   Invalid frame.
 */
static const ltdc_frame_t ltdc_invalid_frame = {
  NULL,
  1,
  1,
  1,
  LTDC_FMT_L8
};

/**
 * @brief   Invalid window.
 * @details Pixel size, located at the origin of the screen.
 */
static const ltdc_window_t ltdc_invalid_window = {
  0,
  1,
  0,
  1
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static LTDC_Layer_TypeDef *get_layer(ltdc_layerid_t layer) {

  chDbgAssert(layer == LTDC_L1 || layer == LTDC_L2,
              "get_layer(), #1", "outside range");

#ifdef STM32F429_439xx
  return (LTDC_Layer_TypeDef *)(((uint32_t)layer << 7) + LTDC_Layer1_BASE);
#else
  return (layer == LTDC_L1) ? LTDC_Layer1 : LTDC_Layer2;
#endif
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   LTDC event interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(LTDC_EV_IRQHandler) {

  static LTDCDriver *const ltdcp = &LTDCD1;

  CH_IRQ_PROLOGUE();

  /* Handle Line Interrupt ISR.*/
  if ((LTDC->ISR & LTDC_ISR_LIF) &&
      (LTDC->IER & LTDC_IER_LIE)) {
    chDbgAssert(ltdcp->config->line_isrcb != NULL,
                "LTDC_EV_IRQHandler(), #1", "invalid state");
    ltdcp->config->line_isrcb(ltdcp);
  }

  /* Handle Register Reload ISR.*/
  if ((LTDC->ISR & LTDC_ISR_RRIF) &&
      (LTDC->IER & LTDC_IER_RRIE)) {
    chDbgAssert(ltdcp->config->rr_isrcb != NULL,
                "LTDC_EV_IRQHandler(), #2", "invalid state");
    ltdcp->config->rr_isrcb(ltdcp);
  }

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   LTDC error interrupt handler.
 *
 * @isr
 */
CH_IRQ_HANDLER(LTDC_ER_IRQHandler) {

  static LTDCDriver *const ltdcp = &LTDCD1;

  CH_IRQ_PROLOGUE();

  /* Handle FIFO Underrun ISR.*/
  if ((LTDC->ISR & LTDC_ISR_FUIF) &&
      (LTDC->IER & LTDC_IER_FUIE)) {
    chDbgAssert(ltdcp->config->fuerr_isrcb != NULL,
                "LTDC_ER_IRQHandler(), #1", "invalid state");
    ltdcp->config->fuerr_isrcb(ltdcp);
  }

  /* Handle Transfer Error ISR.*/
  if ((LTDC->ISR & LTDC_ISR_TERRIF) &&
      (LTDC->IER & LTDC_IER_TERRIE)) {
    chDbgAssert(ltdcp->config->terr_isrcb != NULL,
                "LTDC_ER_IRQHandler(), #2", "invalid state");
    ltdcp->config->terr_isrcb(ltdcp);
  }

  CH_IRQ_EPILOGUE();
}

/**
 * @brief   LTDC Driver initialization.
 * @details Initializes the LTDC subsystem and chosen drivers. Should be
 *          called at board initialization.
 *
 * @init
 */
void ltdcInit() {

  /* Reset the LTDC hardware module.*/
  rccDisableLTDC(FALSE);
  rccResetLTDC();

  /* Enable the LTDC clock.*/
  RCC->DCKCFGR = (RCC->DCKCFGR & ~RCC_DCKCFGR_PLLSAIDIVR) | (2 << 16); /* /8 */
  rccEnableLTDC(FALSE);

  /* Driver struct initialization.*/
  ltdcObjectInit(&LTDCD1);
  LTDCD1.state = LTDC_STOP;
}

/**
 * @brief   Initializes the standard part of a @p LTDCDriver structure.
 *
 * @param[out] ltdcp    pointer to the @p LTDCDriver object
 *
 * @init
 */
void ltdcObjectInit(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcObjectInit");

  ltdcp->state = LTDC_UNINIT;
  ltdcp->config = NULL;
  ltdcp->active_window = ltdc_invalid_window;
}

/**
 * @brief   Get the driver state.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @retun               driver state
 *
 * @iclass
 */
ltdc_state_t ltdcGetStateI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcGetStateI");
  chDbgCheckClassI();

  return ltdcp->state;
}

/**
 * @brief   Get the driver state.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @retun               driver state
 *
 * @api
 */
ltdc_state_t ltdcGetState(LTDCDriver *ltdcp) {

  ltdc_state_t state;
  chSysLock();
  state = ltdcGetStateI(ltdcp);
  chSysUnlock();
  return state;
}

/**
 * @brief   Configures and activates the LTDC peripheral.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] confip    pointer to the @p LTDCConfig object
 *
 * @api
 */
void ltdcStart(LTDCDriver *ltdcp, const LTDCConfig *configp) {

  uint32_t hacc, vacc, flags;
  ltdc_layerid_t l;

  chSysLock();

  chDbgCheck(ltdcp != NULL, "ltdcStart");
  chDbgCheck(configp != NULL, "ltdcStart");
  chDbgAssert(ltdcp->state == LTDC_STOP, "ltdcStart(), #1", "invalid state");

  ltdcp->config = configp;

  /* Turn off the controller and its interrupts.*/
  LTDC->GCR = 0;
  LTDC->IER = 0;
  ltdcReloadAndWaitI(ltdcp, TRUE);

  /* Set synchronization params.*/
  chDbgAssert(configp->hsync_width >= LTDC_MIN_HSYNC_WIDTH,
              "ltdcStart(), #11", "outside range");
  chDbgAssert(configp->hsync_width <= LTDC_MAX_HSYNC_WIDTH,
              "ltdcStart(), #12", "outside range");
  chDbgAssert(configp->vsync_height >= LTDC_MIN_VSYNC_HEIGHT,
              "ltdcStart(), #13", "outside range");
  chDbgAssert(configp->vsync_height <= LTDC_MAX_VSYNC_HEIGHT,
              "ltdcStart(), #14", "outside range");

  hacc = configp->hsync_width - 1;
  vacc = configp->vsync_height - 1;

  LTDC->SSCR = ((hacc << 16) & LTDC_SSCR_HSW) |
               ((vacc <<  0) & LTDC_SSCR_VSH);

  /* Set accumulated back porch params.*/
  chDbgAssert(configp->hbp_width >= LTDC_MIN_HBP_WIDTH,
              "ltdcStart(), #21", "outside range");
  chDbgAssert(configp->hbp_width <= LTDC_MAX_HBP_WIDTH,
              "ltdcStart(), #22", "outside range");
  chDbgAssert(configp->vbp_height >= LTDC_MIN_VBP_HEIGHT,
              "ltdcStart(), #23", "outside range");
  chDbgAssert(configp->vbp_height <= LTDC_MAX_VBP_HEIGHT,
              "ltdcStart(), #24", "outside range");

  hacc += configp->hbp_width;
  vacc += configp->vbp_height;

  chDbgAssert(hacc + 1 >= LTDC_MIN_ACC_HBP_WIDTH,
              "ltdcStart(), #31", "outside range");
  chDbgAssert(hacc + 1 <= LTDC_MAX_ACC_HBP_WIDTH,
              "ltdcStart(), #32", "outside range");
  chDbgAssert(vacc + 1 >= LTDC_MIN_ACC_VBP_HEIGHT,
              "ltdcStart(), #33", "outside range");
  chDbgAssert(vacc + 1 <= LTDC_MAX_ACC_VBP_HEIGHT,
              "ltdcStart(), #34", "outside range");

  LTDC->BPCR = ((hacc << 16) & LTDC_BPCR_AHBP) |
               ((vacc <<  0) & LTDC_BPCR_AVBP);

  ltdcp->active_window.hstart = hacc + 1;
  ltdcp->active_window.vstart = vacc + 1;

  /* Set accumulated active params.*/
  chDbgAssert(configp->screen_width >= LTDC_MIN_SCREEN_WIDTH,
              "ltdcStart(), #41", "outside range");
  chDbgAssert(configp->screen_width <= LTDC_MAX_SCREEN_WIDTH,
              "ltdcStart(), #42", "outside range");
  chDbgAssert(configp->screen_height >= LTDC_MIN_SCREEN_HEIGHT,
              "ltdcStart(), #43", "outside range");
  chDbgAssert(configp->screen_height <= LTDC_MAX_SCREEN_HEIGHT,
              "ltdcStart(), #44", "outside range");

  hacc += configp->screen_width;
  vacc += configp->screen_height;

  chDbgAssert(hacc + 1 >= LTDC_MIN_ACC_ACTIVE_WIDTH,
              "ltdcStart(), #51", "outside range");
  chDbgAssert(hacc + 1 <= LTDC_MAX_ACC_ACTIVE_WIDTH,
              "ltdcStart(), #52", "outside range");
  chDbgAssert(vacc + 1 >= LTDC_MIN_ACC_ACTIVE_HEIGHT,
              "ltdcStart(), #53", "outside range");
  chDbgAssert(vacc + 1 <= LTDC_MAX_ACC_ACTIVE_HEIGHT,
              "ltdcStart(), #54", "outside range");

  LTDC->AWCR = ((hacc << 16) & LTDC_AWCR_AAW) |
               ((vacc <<  0) & LTDC_AWCR_AAH);

  ltdcp->active_window.hstop = hacc;
  ltdcp->active_window.vstop = vacc;

  /* Set accumulated total params.*/
  chDbgAssert(configp->hfp_width >= LTDC_MIN_HFP_WIDTH,
              "ltdcStart(), #61", "outside range");
  chDbgAssert(configp->hfp_width <= LTDC_MAX_HFP_WIDTH,
              "ltdcStart(), #62", "outside range");
  chDbgAssert(configp->vfp_height >= LTDC_MIN_VFP_HEIGHT,
              "ltdcStart(), #63", "outside range");
  chDbgAssert(configp->vfp_height <= LTDC_MAX_VFP_HEIGHT,
              "ltdcStart(), #64", "outside range");

  hacc += configp->hfp_width;
  vacc += configp->vfp_height;

  chDbgAssert(hacc + 1 >= LTDC_MIN_ACC_TOTAL_WIDTH,
              "ltdcStart(), #71", "outside range");
  chDbgAssert(hacc + 1 <= LTDC_MAX_ACC_TOTAL_WIDTH,
              "ltdcStart(), #72", "outside range");
  chDbgAssert(vacc + 1 >= LTDC_MIN_ACC_TOTAL_HEIGHT,
              "ltdcStart(), #73", "outside range");
  chDbgAssert(vacc + 1 <= LTDC_MAX_ACC_TOTAL_HEIGHT,
              "ltdcStart(), #74", "outside range");

  LTDC->TWCR = ((hacc << 16) & LTDC_TWCR_TOTALW) |
               ((vacc <<  0) & LTDC_TWCR_TOTALH);

  /* Set signal polarities and other flags.*/
  ltdcSetEnableFlagsI(ltdcp, configp->flags & ~LTDC_EF_ENABLE);

  /* Color settings.*/
  ltdcSetBackgroundColorI(ltdcp, configp->bg_color);

  /* Load layer configurations.*/
  for (l = LTDC_L1; l <= LTDC_L2; ++l) {
    const ltdc_laycfg_t *const lcfgp = configp->laycfgs[l];
    if (lcfgp != NULL) {
      ltdcLayerSetEnableFlagsI(ltdcp, l, lcfgp->flags & ~LTDC_LEF_ENABLE);
      ltdcLayerSetFrameI(ltdcp, l, lcfgp->frame);
      ltdcLayerSetWindowI(ltdcp, l, lcfgp->window);
      ltdcLayerSetDefaultColorI(ltdcp, l, lcfgp->def_color);
      ltdcLayerSetKeyingColorI(ltdcp, l, lcfgp->key_color);
      ltdcLayerSetConstantAlphaI(ltdcp, l, lcfgp->const_alpha);
      ltdcLayerSetBlendingFactorsI(ltdcp, l, lcfgp->blending);

      if (lcfgp->pal_length > 0)
        ltdcLayerSetPaletteI(ltdcp, l, lcfgp->pal_colors, lcfgp->pal_length);
      else { /* Default grayscale palette.*/
        unsigned i;
        for (i = 0; i < 256; ++i)
          ltdcLayerSetPaletteColorI(ltdcp, l, (uint8_t)i,
                                    ltdcMakeARGB8888(0xFF, i, i, i));
      }
    } else {
      ltdcLayerSetEnableFlagsI(ltdcp, l, 0);
      ltdcLayerSetFrameI(ltdcp, l, &ltdc_invalid_frame);
      ltdcLayerSetWindowI(ltdcp, l, &ltdc_invalid_window);
      ltdcLayerSetDefaultColorI(ltdcp, l, LTDC_COLOR_BLACK);
      ltdcLayerSetKeyingColorI(ltdcp, l, LTDC_COLOR_BLACK);
      ltdcLayerSetConstantAlphaI(ltdcp, l, 0x00);
      ltdcLayerSetBlendingFactorsI(ltdcp, l, LTDC_BLEND_FIX1_FIX2);
    }
  }

  /* Enable only the assigned interrupt service routines.*/
  nvicEnableVector(STM32_LTDC_EV_NUMBER,
                   CORTEX_PRIORITY_MASK(STM32_LTDC_EV_IRQ_PRIORITY));
  nvicEnableVector(STM32_LTDC_ER_NUMBER,
                   CORTEX_PRIORITY_MASK(STM32_LTDC_ER_IRQ_PRIORITY));

  flags = 0;
  if (configp->line_isrcb != NULL)
    flags |= LTDC_IER_LIE;
  if (configp->rr_isrcb != NULL)
    flags |= LTDC_IER_RRIE;
  if (configp->fuerr_isrcb != NULL)
    flags |= LTDC_IER_FUIE;
  if (configp->terr_isrcb != NULL)
    flags |= LTDC_IER_TERRIE;
  LTDC->IER = flags;

  /* Apply settings.*/
  ltdcReloadAndWaitI(ltdcp, TRUE);

  /* Turn on the controller.*/
  for (l = LTDC_L1; l <= LTDC_L2; ++l)
    if (configp->laycfgs[l]->flags & LTDC_LEF_ENABLE)
      ltdcLayerEnableI(ltdcp, l);
  LTDC->GCR |= LTDC_GCR_LTDCEN;
  ltdcReloadAndWaitI(ltdcp, TRUE);

  ltdcp->state = LTDC_READY;
  chSysUnlock();
}

/**
 * @brief   Deactivates the LTDC peripheral.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcStop(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcStop");

  chSysLock();
  chDbgAssert(ltdcp->state == LTDC_READY, "ltdcStop(), #1", "invalid state");

  /* Turn off the controller and its interrupts.*/
  LTDC->GCR &= ~LTDC_GCR_LTDCEN;
  LTDC->IER = 0;
  ltdcReloadAndWait(ltdcp, TRUE);

  ltdcp->state = LTDC_STOP;
  chSysUnlock();
}

/**
 * @brief   Get enabled flags.
 * @details Returns all the flags of the <tt>LTDC_EF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              enabled flags
 *
 * @iclass
 */
ltdc_flags_t ltdcGetEnableFlagsI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcGetEnableFlagsI");
  chDbgCheckClassI();
  (void)ltdcp;

  return LTDC->GCR & LTDC_EF_MASK;
}

/**
 * @brief   Set enabled flags.
 * @details Sets all the flags of the <tt>LTDC_EF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] flags     enabled flags
 *
 * @iclass
 */
void ltdcSetEnableFlagsI(LTDCDriver *ltdcp, ltdc_flags_t flags) {

  chDbgCheck(ltdcp != NULL, "ltdcSetEnableFlagsI");
  chDbgCheckClassI();
  (void)ltdcp;

  LTDC->GCR = flags & LTDC_EF_MASK;
}

/**
 * @brief   Reloading shadow registers.
 * @details Tells whether the LTDC is reloading shadow registers.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return  reloading
 *
 * @iclass
 */
bool_t ltdcIsReloadingI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcIsReloadingI");
  chDbgCheckClassI();
  (void)ltdcp;

  return (LTDC->SRCR & (LTDC_SRCR_IMR | LTDC_SRCR_VBR)) != 0;
}

/**
 * @brief   Reload shadow registers.
 * @details Reloads LTDC shadow registers, upon vsync or immediately.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] immediate reload immediately, not upon vsync
 *
 * @iclass
 */
void ltdcReloadI(LTDCDriver *ltdcp, bool_t immediate) {

  chDbgCheck(ltdcp != NULL, "ltdcReloadI");
  chDbgCheckClassI();
  (void)ltdcp;

  if (immediate)
    LTDC->SRCR |= LTDC_SRCR_IMR;
  else
    LTDC->SRCR |= LTDC_SRCR_VBR;
}

/**
 * @brief   Wait shadow registers reloading.
 * @details Waits while reloading LTDC shadow registers.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @iclass
 */
void ltdcWaitReloadI(LTDCDriver *ltdcp) {

  while (ltdcIsReloadingI(ltdcp))
    ;
}

/**
 * @brief   Dithering enabled.
 * @details Tells whether the dithering is enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              enabled
 *
 * @api
 */
bool_t ltdcIsDitheringEnabledI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcIsDitheringEnabledI");
  chDbgCheckClassI();
  (void)ltdcp;

  return (LTDC->GCR & LTDC_GCR_DTEN) != 0;
}

/**
 * @brief   Enable dithering.
 * @details Enables dithering capabilities for pixel formats with less than
 *          8 bits per channel.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcEnableDitheringI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcEnableDitheringI");
  chDbgCheckClassI();
  (void)ltdcp;

  LTDC->GCR |= LTDC_GCR_DTEN;
}

/**
 * @brief   Disable dithering.
 * @details Disables dithering capabilities for pixel formats with less than
 *          8 bits per channel.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcDisableDitheringI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcDisableDitheringI");
  chDbgCheckClassI();
  (void)ltdcp;

  LTDC->GCR &= ~LTDC_GCR_DTEN;
}

/**
 * @brief   Get background color.
 * @details Gets the background color.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              background color, RGB-888
 *
 * @iclass
 */
ltdc_color_t ltdcGetBackgroundColorI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcGetBackgroundColorI");
  chDbgCheckClassI();
  (void)ltdcp;

  return (ltdc_color_t)(LTDC->BCCR & 0x00FFFFFF);
}

/**
 * @brief   Set background color.
 * @details Sets the background color.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] c         background color, RGB-888
 *
 * @iclass
 */
void ltdcSetBackgroundColorI(LTDCDriver *ltdcp, ltdc_color_t c) {

  chDbgCheck(ltdcp != NULL, "ltdcSetBackgroundColorI");
  chDbgCheckClassI();
  (void)ltdcp;

  LTDC->BCCR = c & 0x00FFFFFF;
}

/**
 * @brief   Get line interrupt position.
 * @details Gets the line interrupt position.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              line interrupt position
 *
 * @iclass
 */
uint16_t ltdcGetLineInterruptPosI(LTDCDriver *ltdcp) {

  chDbgCheck(ltdcp != NULL, "ltdcGetLineInterruptPosI");
  chDbgCheckClassI();
  (void)ltdcp;

  return (uint16_t)(LTDC->LIPCR & LTDC_LIPCR_LIPOS);
}

/**
 * @brief   Set line interrupt position.
 * @details Sets the line interrupt position.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] pos       line interrupt position
 *
 * @iclass
 */
void ltdcSetLineInterruptPosI(LTDCDriver *ltdcp, uint16_t pos) {

  chDbgCheck(ltdcp != NULL, "ltdcSetLineInterruptPosI");
  chDbgCheckClassI();
  (void)ltdcp;

  LTDC->LIPCR = (uint32_t)pos & LTDC_LIPCR_LIPOS;
}

/**
 * @brief   Get current position.
 * @details Gets the current position.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[out] xp       pointer to the output horizontal coordinate
 * @param[out] yp       pointer to the output vertical coordinate
 *
 * @iclass
 */
void ltdcGetCurrentPosI(LTDCDriver *ltdcp, uint16_t *xp, uint16_t *yp) {

  chDbgCheck(ltdcp != NULL, "ltdcGetCurrentPosI");
  chDbgCheckClassI();
  (void)ltdcp;

  const uint32_t r = LTDC->CPSR;
  *xp = (uint16_t)((r & LTDC_CPSR_CXPOS) >> 16);
  *yp = (uint16_t)((r & LTDC_CPSR_CYPOS) >>  0);
}

/**
 * @brief   Get layer enabled flags.
 * @details Returns all the flags of the <tt>LTDC_LEF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              enabled flags
 *
 * @iclass
 */
ltdc_flags_t ltdcLayerGetEnableFlagsI(LTDCDriver *ltdcp,
                                      ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetEnableFlagsI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetEnableFlagsI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return get_layer(layer)->CR & LTDC_LEF_MASK;
}

/**
 * @brief   Set layer enabled flags.
 * @details Sets all the flags of the <tt>LTDC_LEF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] flags     enabled flags
 *
 * @iclass
 */
void ltdcLayerSetEnableFlagsI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              ltdc_flags_t flags) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetEnableFlagsI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetEnableFlagsI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR = flags & LTDC_LEF_MASK;
}

/**
 * @brief   Layer enabled.
 * @details Tells whether a layer is enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              enabled
 *
 * @iclass
 */
bool_t ltdcLayerIsEnabledI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerIsEnabledI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerIsEnabledI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (get_layer(layer)->CR & ~LTDC_LxCR_LEN) != 0;
}

/**
 * @brief   Layer enable.
 * @details Enables a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerEnableI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerEnableI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerEnableI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR |= LTDC_LxCR_LEN;
}

/**
 * @brief   Layer disable.
 * @details Disables a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerDisableI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerDisableI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerDisableI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR &= ~LTDC_LxCR_LEN;
}

/**
 * @brief   Layer palette enabled.
 * @details Tells whether a layer palette (color lookup table) is enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              enabled
 *
 * @iclass
 */
bool_t ltdcLayerIsPaletteEnabledI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerIsPaletteEnabledI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerIsPaletteEnabledI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (get_layer(layer)->CR & ~LTDC_LxCR_CLUTEN) != 0;
}

/**
 * @brief   Enable layer palette.
 * @details Enables the palette (color lookup table) of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerEnablePaletteI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerEnablePaletteI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerEnablePaletteI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR |= LTDC_LxCR_CLUTEN;
}

/**
 * @brief   Disable layer palette.
 * @details Disables the palette (color lookup table) of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerDisablePaletteI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerDisablePaletteI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerDisablePaletteI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR &= ~LTDC_LxCR_CLUTEN;
}

/**
 * @brief   Set palette color.
 * @details Sets the color of a palette (color lookup table) slot.
 * @note    Palette colors should be changed only at vsync, or while the LTDC
 *          is disabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] slot      palette slot
 * @param[in] c         color, RGB-888
 *
 * @iclass
 */
void ltdcLayerSetPaletteColorI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               uint8_t slot, ltdc_color_t c) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetPaletteColorI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetPaletteColorI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CLUTWR = ((uint32_t)slot << 24) | (c & 0x00FFFFFF);
}

/**
 * @brief   Set layer palette.
 * @details Sets the entire palette color (color lookup table) slot.
 * @note    Palette colors should be changed only at vsync, or while the LTDC
 *          is disabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] colors    array of palette colors, RGB-888
 * @param[in] length    number of palette colors
 *
 * @iclass
 */
void ltdcLayerSetPaletteI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                          const ltdc_color_t colors[], uint16_t length) {

  LTDC_Layer_TypeDef *layerp;
  uint16_t i;

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetPaletteI");
  chDbgCheck((colors == NULL) == (length == 0), "ltdcLayerSetPaletteI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetPaletteI(), #1", "invalid layer");
  chDbgAssert(length <= 256,
              "ltdcLayerSetPaletteI(), #2", "outside range");
  chDbgCheckClassI();
  (void)ltdcp;

  layerp = get_layer(layer);
  for (i = 0; i < length; ++i)
    layerp->CLUTWR = ((uint32_t)i << 24) | (colors[i] & 0x00FFFFFF);
}

/**
 * @brief   Get layer pixel format.
 * @details Gets the pixel format of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              pixel format
 *
 * @iclass
 */
ltdc_pixfmt_t ltdcLayerGetPixelFormatI(LTDCDriver *ltdcp,
                                       ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetPixelFormatI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetPixelFormatI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (ltdc_pixfmt_t)(get_layer(layer)->PFCR & LTDC_LxPFCR_PF);
}

/**
 * @brief   Set layer pixel format.
 * @details Sets the pixel format of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] fmt       pixel format
 *
 * @iclass
 */
void ltdcLayerSetPixelFormatI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              ltdc_pixfmt_t fmt) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetPixelFormatI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetPixelFormatI(), #1", "invalid layer");
  chDbgAssert(fmt >= LTDC_MIN_PIXFMT_ID,
              "ltdcLayerSetPixelFormatI(), #2", "outside range");
  chDbgAssert(fmt <= LTDC_MAX_PIXFMT_ID,
              "ltdcLayerSetPixelFormatI(), #3", "outside range");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->PFCR = (uint32_t)fmt & LTDC_LxPFCR_PF;
}

/**
 * @brief   Layer color keying enabled.
 * @details Tells whether a layer has color keying enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              enabled
 *
 * @iclass
 */
bool_t ltdcLayerIsKeyingEnabledI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerIsKeyingEnabledI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerIsKeyingEnabledI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (get_layer(layer)->CR & ~LTDC_LxCR_COLKEN) != 0;
}

/**
 * @brief   Enable layer color keying.
 * @details Enables color keying capabilities of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerEnableKeyingI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerEnableKeyingI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerEnableKeyingI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR |= LTDC_LxCR_COLKEN;
}

/**
 * @brief   Disable layer color keying.
 * @details Disables color keying capabilities of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerDisableKeyingI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerDisableKeyingI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerDisableKeyingI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CR &= ~LTDC_LxCR_COLKEN;
}

/**
 * @brief   Get layer color key.
 * @details Gets the color key of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              color key, RGB-888
 *
 * @iclass
 */
ltdc_color_t ltdcLayerGetKeyingColorI(LTDCDriver *ltdcp,
                                      ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetKeyingColorI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetKeyingColorI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (ltdc_color_t)(get_layer(layer)->CKCR & 0x00FFFFFF);
}

/**
 * @brief   Set layer color key.
 * @details Sets the color key of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] c         color key, RGB-888
 *
 * @iclass
 */
void ltdcLayerSetKeyingColorI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              ltdc_color_t c) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetKeyingColorI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetKeyingColorI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CKCR = (uint32_t)c & 0x00FFFFFF;
}

/**
 * @brief   Get layer constant alpha.
 * @details Gets the constant alpha component of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              constant alpha component, A-8
 *
 * @iclass
 */
uint8_t ltdcLayerGetConstantAlphaI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetConstantAlphaI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetConstantAlphaI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (uint8_t)(get_layer(layer)->CACR & LTDC_LxCACR_CONSTA);
}

/**
 * @brief   Set layer constant alpha.
 * @details Sets the constant alpha component of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] a         constant alpha component, A-8
 *
 * @iclass
 */
void ltdcLayerSetConstantAlphaI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                uint8_t a) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetConstantAlphaI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetConstantAlphaI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CACR = (uint32_t)a & LTDC_LxCACR_CONSTA;
}

/**
 * @brief   Get layer default color.
 * @details Gets the default color of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              default color, ARGB-8888
 *
 * @iclass
 */
ltdc_color_t ltdcLayerGetDefaultColorI(LTDCDriver *ltdcp,
                                       ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetDefaultColorI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetDefaultColorI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (ltdc_color_t)get_layer(layer)->DCCR;
}

/**
 * @brief   Set layer default color.
 * @details Sets the default color of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] c         default color, ARGB-8888
 *
 * @iclass
 */
void ltdcLayerSetDefaultColorI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               ltdc_color_t c) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetDefaultColorI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetDefaultColorI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->DCCR = (uint32_t)c;
}

/**
 * @brief   Get layer blending factors.
 * @details Gets the blending factors of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              blending factors
 *
 * @iclass
 */
ltdc_blendf_t ltdcLayerGetBlendingFactorsI(LTDCDriver *ltdcp,
                                           ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetBlendingFactorsI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetBlendingFactorsI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (ltdc_blendf_t)(get_layer(layer)->BFCR &
                         (LTDC_LxBFCR_BF1 | LTDC_LxBFCR_BF2));
}

/**
 * @brief   Set layer blending factors.
 * @details Sets the blending factors of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] factors   blending factors
 *
 * @iclass
 */
void ltdcLayerSetBlendingFactorsI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                  ltdc_blendf_t bf) {

  chDbgCheck(ltdcp != NULL, "ltdcSetBlendingFactorsI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetBlendingFactorsI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->BFCR = (uint32_t)bf & (LTDC_LxBFCR_BF1 | LTDC_LxBFCR_BF2);
}

/**
 * @brief   Get layer window specs.
 * @details Gets the window specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] winp     pointer to the output window specifications
 *
 * @iclass
 */
void ltdcLayerGetWindowI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                         ltdc_window_t *winp) {

  LTDC_Layer_TypeDef *layerp;

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetWindowI");
  chDbgCheck(winp != NULL, "ltdcLayerGetWindowI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetWindowI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  layerp = get_layer(layer);
  winp->hstart = (uint16_t)((layerp->WHPCR & LTDC_LxWHPCR_WHSTPOS) >>  0);
  winp->hstop  = (uint16_t)((layerp->WHPCR & LTDC_LxWHPCR_WHSPPOS) >> 16);
  winp->vstart = (uint16_t)((layerp->WVPCR & LTDC_LxWVPCR_WVSTPOS) >>  0);
  winp->vstop  = (uint16_t)((layerp->WVPCR & LTDC_LxWVPCR_WVSPPOS) >> 16);
}

/**
 * @brief   Set layer window specs.
 * @details Sets the window specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] winp     pointer to the window specifications
 *
 * @iclass
 */
void ltdcLayerSetWindowI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                         const ltdc_window_t *winp) {

  LTDC_Layer_TypeDef *layerp;
  uint32_t start, stop;

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetWindowI");
  chDbgCheck(winp != NULL, "ltdcLayerSetWindowI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetWindowI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  chDbgAssert(winp->hstop < ltdcp->config->screen_width,
              "ltdcLayerSetWindowI(), #11", "outside range");
  chDbgAssert(winp->vstop < ltdcp->config->screen_height,
              "ltdcLayerSetWindowI(), #12", "outside range");

  layerp = get_layer(layer);

  /* Horizontal boundaries.*/
  start = (uint32_t)winp->hstart + ltdcp->active_window.hstart;
  stop  = (uint32_t)winp->hstop  + ltdcp->active_window.hstart;

  chDbgAssert(start >= ltdcp->active_window.hstart,
              "ltdcLayerSetWindowI(), #21", "outside range");
  chDbgAssert(stop <= ltdcp->active_window.hstop,
              "ltdcLayerSetWindowI(), #22", "outside range");

  layerp->WHPCR = ((start <<  0) & LTDC_LxWHPCR_WHSTPOS) |
                  ((stop  << 16) & LTDC_LxWHPCR_WHSPPOS);

  /* Vertical boundaries.*/
  start = (uint32_t)winp->vstart + ltdcp->active_window.vstart;
  stop  = (uint32_t)winp->vstop  + ltdcp->active_window.vstart;

  chDbgAssert(start >= ltdcp->active_window.vstart,
              "ltdcLayerSetWindowI(), #31", "outside range");
  chDbgAssert(stop <= ltdcp->active_window.vstop,
              "ltdcLayerSetWindowI(), #32", "outside range");

  layerp->WVPCR = ((start <<  0) & LTDC_LxWVPCR_WVSTPOS) |
                  ((stop  << 16) & LTDC_LxWVPCR_WVSPPOS);
}

/**
 * @brief   Set layer window as invalid.
 * @details Sets the window specifications of a layer so that the window is
 *          pixel sized at the screen origin.
 * @note    Useful before reconfiguring the frame specifications of the layer,
 *          to avoid errors.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @iclass
 */
void ltdcLayerSetInvalidWindowI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  (void)ltdcp;
  ltdcLayerSetWindowI(ltdcp, layer, &ltdc_invalid_window);
}

/**
 * @brief   Get layer frame buffer specs.
 * @details Gets the frame buffer specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] framep   pointer to the output frame buffer specifications
 *
 * @iclass
 */
void ltdcLayerGetFrameI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                        ltdc_frame_t *framep) {

  LTDC_Layer_TypeDef *layerp;

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetFrameI");
  chDbgCheck(framep != NULL, "ltdcLayerGetFrameI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetFrameI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  layerp = get_layer(layer);
  framep->bufferp = (void *)(layerp->CFBAR & LTDC_LxCFBAR_CFBADD);
  framep->pitch   = (size_t)((layerp->CFBLR & LTDC_LxCFBLR_CFBP) >> 16);
  framep->width   = (uint16_t)(((layerp->CFBLR & LTDC_LxCFBLR_CFBLL) - 3) /
                    ltdcBytesPerPixel(ltdcLayerGetPixelFormatI(ltdcp, layer)));
  framep->height  = (uint16_t)(layerp->CFBLNR & LTDC_LxCFBLNR_CFBLNBR);
}

/**
 * @brief   Set layer frame buffer specs.
 * @details Sets the frame buffer specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] framep   pointer to the frame buffer specifications
 *
 * @iclass
 */
void ltdcLayerSetFrameI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                        const ltdc_frame_t *framep) {

  LTDC_Layer_TypeDef *layerp;
  size_t linesize;

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetFrameI");
  chDbgCheck(framep != NULL, "ltdcLayerSetFrameI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetFrameI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  ltdcLayerSetPixelFormatI(ltdcp, layer, framep->fmt);

  layerp = get_layer(layer);
  linesize = ltdcBytesPerPixel(framep->fmt) * framep->width;

  chDbgAssert(framep->width  <= ltdcp->config->screen_width,
              "ltdcLayerSetFrameI(), #11", "outside range");
  chDbgAssert(framep->height <= ltdcp->config->screen_height,
              "ltdcLayerSetFrameI(), #12", "outside range");
  chDbgAssert(linesize >= LTDC_MIN_FRAME_WIDTH_BYTES,
              "ltdcLayerSetFrameI(), #13", "outside range");
  chDbgAssert(linesize <= LTDC_MAX_FRAME_WIDTH_BYTES,
              "ltdcLayerSetFrameI(), #14", "outside range");
  chDbgAssert(framep->height >= LTDC_MIN_FRAME_HEIGHT_LINES,
              "ltdcLayerSetFrameI(), #15", "outside range");
  chDbgAssert(framep->height <= LTDC_MAX_FRAME_HEIGHT_LINES,
              "ltdcLayerSetFrameI(), #16", "outside range");
  chDbgAssert(framep->pitch  >= linesize,
              "ltdcLayerSetFrameI(), #17", "outside range");

  layerp->CFBAR  = (uint32_t)framep->bufferp & LTDC_LxCFBAR_CFBADD;
  layerp->CFBLR  = ((((uint32_t)framep->pitch << 16) & LTDC_LxCFBLR_CFBP) |
                    ((linesize + 3) & LTDC_LxCFBLR_CFBLL));
  layerp->CFBLNR = (uint32_t)framep->height & LTDC_LxCFBLNR_CFBLNBR;
}

/**
 * @brief   Get layer frame buffer address.
 * @details Gets the frame buffer address of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              frame buffer address
 *
 * @iclass
 */
void *ltdcLayerGetFrameAddressI(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerGetFrameAddressI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerGetFrameAddressI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  return (void *)(get_layer(layer)->CFBAR & LTDC_LxCFBAR_CFBADD);
}

/**
 * @brief   Set layer frame buffer address.
 * @details Sets the frame buffer address of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] bufferp   frame buffer address
 *
 * @iclass
 */
void ltdcLayerSetFrameAddressI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               void *bufferp) {

  chDbgCheck(ltdcp != NULL, "ltdcLayerSetFrameAddressI");
  chDbgCheck(bufferp != NULL, "ltdcLayerSetFrameAddressI");
  chDbgAssert((layer & ~1) == 0,
              "ltdcLayerSetFrameAddressI(), #1", "invalid layer");
  chDbgCheckClassI();
  (void)ltdcp;

  get_layer(layer)->CFBAR = (uint32_t)bufferp & LTDC_LxCFBAR_CFBADD;
}

/**
 * @brief   Get enabled flags.
 * @details Returns all the flags of the <tt>LTDC_EF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              enabled flags
 *
 * @api
 */
ltdc_flags_t ltdcGetEnableFlags(LTDCDriver *ltdcp) {

  ltdc_flags_t flags;
  chSysLock();
  flags = ltdcGetEnableFlagsI(ltdcp);
  chSysUnlock();
  return flags;
}

/**
 * @brief   Set enabled flags.
 * @details Sets all the flags of the <tt>LTDC_EF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] flags     enabled flags
 *
 * @api
 */
void ltdcSetEnableFlags(LTDCDriver *ltdcp, ltdc_flags_t flags) {

  chSysLock();
  ltdcSetEnableFlagsI(ltdcp, flags);
  chSysUnlock();
}

/**
 * @brief   Reloading shadow registers.
 * @details Tells whether the LTDC is reloading shadow registers.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              reloading
 *
 * @api
 */
bool_t ltdcIsReloading(LTDCDriver *ltdcp) {

  bool_t reloading;
  chSysLock();
  reloading = ltdcIsReloadingI(ltdcp);
  chSysUnlock();
  return reloading;
}

/**
 * @brief   Reload shadow registers.
 * @details Reloads LTDC shadow registers, upon vsync or immediately.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] immediate reload immediately, not upon vsync
 *
 * @api
 */
void ltdcReload(LTDCDriver *ltdcp, bool_t immediate) {

  chSysLock();
  ltdcReloadI(ltdcp, immediate);
  chSysUnlock();
}

/**
 * @brief   Wait shadow registers reloading.
 * @details Waits while reloading LTDC shadow registers.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcWaitReload(LTDCDriver *ltdcp) {

  while (ltdcIsReloading(ltdcp))
    ;
}

/**
 * @brief   Dithering enabled.
 * @details Tells whether the dithering is enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              enabled
 *
 * @api
 */
bool_t ltdcIsDitheringEnabled(LTDCDriver *ltdcp) {

  bool_t enabled;
  chSysLock();
  enabled = ltdcIsDitheringEnabledI(ltdcp);
  chSysUnlock();
  return enabled;
}

/**
 * @brief   Enable dithering.
 * @details Enables dithering capabilities for pixel formats with less than
 *          8 bits per channel.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcEnableDithering(LTDCDriver *ltdcp) {

  chSysLock();
  ltdcEnableDitheringI(ltdcp);
  chSysUnlock();
}

/**
 * @brief   Disable dithering.
 * @details Disables dithering capabilities for pixel formats with less than
 *          8 bits per channel.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcDisableDithering(LTDCDriver *ltdcp) {

  chSysLock();
  ltdcDisableDitheringI(ltdcp);
  chSysUnlock();
}

/**
 * @brief   Get background color.
 * @details Gets the background color.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              background color, RGB-888
 *
 * @api
 */
ltdc_color_t ltdcGetBackgroundColor(LTDCDriver *ltdcp) {

  ltdc_color_t c;
  chSysLock();
  c = ltdcGetBackgroundColorI(ltdcp);
  chSysUnlock();
  return c;
}

/**
 * @brief   Set background color.
 * @details Sets the background color.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] c         background color, RGB-888
 *
 * @api
 */
void ltdcSetBackgroundColor(LTDCDriver *ltdcp, ltdc_color_t c) {

  chSysLock();
  ltdcSetBackgroundColorI(ltdcp, c);
  chSysUnlock();
}

/**
 * @brief   Get line interrupt position.
 * @details Gets the line interrupt position.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              line interrupt position
 *
 * @api
 */
uint16_t ltdcGetLineInterruptPos(LTDCDriver *ltdcp) {

  uint16_t pos;
  chSysLock();
  pos = ltdcGetLineInterruptPosI(ltdcp);
  chSysUnlock();
  return pos;
}

/**
 * @brief   Set line interrupt position.
 * @details Sets the line interrupt position.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @api
 */
void ltdcSetLineInterruptPos(LTDCDriver *ltdcp, uint16_t pos) {

  chSysLock();
  ltdcSetLineInterruptPosI(ltdcp, pos);
  chSysUnlock();
}

/**
 * @brief   Get current position.
 * @details Gets the current position.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[out] xp       pointer to the destination horizontal coordinate
 * @param[out] yp       pointer to the destination vertical coordinate
 *
 * @api
 */
void ltdcGetCurrentPos(LTDCDriver *ltdcp, uint16_t *xp, uint16_t *yp) {

  chSysLock();
  ltdcGetCurrentPosI(ltdcp, xp, yp);
  chSysUnlock();
}

/**
 * @brief   Get layer enabled flags.
 * @details Returns all the flags of the <tt>LTDC_LEF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 *
 * @return              enabled flags
 *
 * @api
 */
ltdc_flags_t ltdcLayerGetEnableFlags(LTDCDriver *ltdcp,
                                     ltdc_layerid_t layer) {

  ltdc_flags_t flags;
  chSysLock();
  flags = ltdcLayerGetEnableFlagsI(ltdcp, layer);
  chSysUnlock();
  return flags;
}

/**
 * @brief   Set layer enabled flags.
 * @details Sets all the flags of the <tt>LTDC_LEF_*</tt> group at once.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] flags     enabled flags
 *
 * @api
 */
void ltdcLayerSetEnableFlags(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              ltdc_flags_t flags) {

  chSysLock();
  ltdcLayerSetEnableFlagsI(ltdcp, layer, flags);
  chSysUnlock();
}

/**
 * @brief   Layer enabled.
 * @details Tells whether a layer is enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              enabled
 *
 * @api
 */
bool_t ltdcLayerIsEnabled(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  bool_t enabled;
  chSysLock();
  enabled = ltdcLayerIsEnabledI(ltdcp, layer);
  chSysUnlock();
  return enabled;
}

/**
 * @brief   Layer enable.
 * @details Enables a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerEnable(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerEnableI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Layer disable.
 * @details Disables a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerDisable(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerDisableI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Layer palette enabled.
 * @details Tells whether a layer palette (color lookup table) is enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              enabled
 *
 * @api
 */
bool_t ltdcLayerIsPaletteEnabled(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  bool_t enabled;
  chSysLock();
  enabled = ltdcLayerIsPaletteEnabledI(ltdcp, layer);
  chSysUnlock();
  return enabled;
}

/**
 * @brief   Enable layer palette.
 * @details Enables the palette (color lookup table) of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerEnablePalette(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerEnablePaletteI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Disable layer palette.
 * @details Disables the palette (color lookup table) of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerDisablePalette(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerDisablePaletteI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Set layer palette color.
 * @details Sets the color of a palette (color lookup table) slot to the
 *          specified layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] slot      palette slot
 * @param[in] c         color, RGB-888
 *
 * @api
 */
void ltdcLayerSetPaletteColor(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              uint8_t slot, ltdc_color_t c) {

  chSysLock();
  ltdcLayerSetPaletteColorI(ltdcp, layer, slot, c);
  chSysUnlock();
}

/**
 * @brief   Set layer palette.
 * @details Sets the entire palette color (color lookup table) slot.
 * @note    Palette colors should be changed only at vsync, or while the LTDC
 *          is disabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] colors    array of palette colors, RGB-888
 * @param[in] length    number of palette colors
 *
 * @api
 */
void ltdcLayerSetPalette(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                         const ltdc_color_t colors[], uint16_t length) {

  chSysLock();
  ltdcLayerSetPaletteI(ltdcp, layer, colors, length);
  chSysUnlock();
}

/**
 * @brief   Get layer pixel format.
 * @details Gets the pixel format of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              pixel format
 *
 * @api
 */
ltdc_pixfmt_t ltdcLayerGetPixelFormat(LTDCDriver *ltdcp,
                                      ltdc_layerid_t layer) {

  ltdc_pixfmt_t fmt;
  chSysLock();
  fmt = ltdcLayerGetPixelFormatI(ltdcp, layer);
  chSysUnlock();
  return fmt;
}

/**
 * @brief   Set layer pixel format.
 * @details Sets the pixel format of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] fmt       pixel format
 *
 * @api
 */
void ltdcLayerSetPixelFormat(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                             ltdc_pixfmt_t fmt) {

  chSysLock();
  ltdcLayerSetPixelFormatI(ltdcp, layer, fmt);
  chSysUnlock();
}

/**
 * @brief   Layer color keying enabled.
 * @details Tells whether a layer has color keying enabled.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              enabled
 *
 * @api
 */
bool_t ltdcLayerIsKeyingEnabled(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  bool_t enabled;
  chSysLock();
  enabled = ltdcLayerIsKeyingEnabledI(ltdcp, layer);
  chSysUnlock();
  return enabled;
}

/**
 * @brief   Enable layer color keying.
 * @details Enables color keying capabilities of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerEnableKeying(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerEnableKeyingI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Disable layer color keying.
 * @details Disables color keying capabilities of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerDisableKeying(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerDisableKeyingI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Get layer color key.
 * @details Gets the color key of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              color key, RGB-888
 *
 * @api
 */
ltdc_color_t ltdcLayerGetKeyingColor(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  ltdc_color_t c;
  chSysLock();
  c = ltdcLayerGetKeyingColorI(ltdcp, layer);
  chSysUnlock();
  return c;
}

/**
 * @brief   Set layer color key.
 * @details Sets the color key of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] c         color key, RGB-888
 *
 * @api
 */
void ltdcLayerSetKeyingColor(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                             ltdc_color_t c) {

  chSysLock();
  ltdcLayerSetKeyingColorI(ltdcp, layer, c);
  chSysUnlock();
}

/**
 * @brief   Get layer constant alpha.
 * @details Gets the constant alpha component of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              constant alpha component, A-8
 *
 * @api
 */
uint8_t ltdcLayerGetConstantAlpha(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  uint8_t a;
  chSysLock();
  a = ltdcLayerGetConstantAlphaI(ltdcp, layer);
  chSysUnlock();
  return a;
}

/**
 * @brief   Set layer constant alpha.
 * @details Sets the constant alpha component of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] a         constant alpha component, A-8
 *
 * @api
 */
void ltdcLayerSetConstantAlpha(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               uint8_t a) {

  chSysLock();
  ltdcLayerSetConstantAlphaI(ltdcp, layer, a);
  chSysUnlock();
}

/**
 * @brief   Get layer default color.
 * @details Gets the default color of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              default color, RGB-888
 *
 * @api
 */
ltdc_color_t ltdcLayerGetDefaultColor(LTDCDriver *ltdcp,
                                      ltdc_layerid_t layer) {

  ltdc_color_t c;
  chSysLock();
  c = ltdcLayerGetDefaultColorI(ltdcp, layer);
  chSysUnlock();
  return c;
}

/**
 * @brief   Set layer default color.
 * @details Sets the default color of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] c         default color, RGB-888
 *
 * @api
 */
void ltdcLayerSetDefaultColor(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              ltdc_color_t c) {

  chSysLock();
  ltdcLayerSetDefaultColorI(ltdcp, layer, c);
  chSysUnlock();
}

/**
 * @brief   Get layer blending factors.
 * @details Gets the blending factors of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              blending factors
 *
 * @api
 */
ltdc_blendf_t ltdcLayerGetBlendingFactors(LTDCDriver *ltdcp,
                                          ltdc_layerid_t layer) {

  ltdc_blendf_t factors;
  chSysLock();
  factors = ltdcLayerGetBlendingFactorsI(ltdcp, layer);
  chSysUnlock();
  return factors;
}

/**
 * @brief   Set layer blending factors.
 * @details Sets the blending factors of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] factors   blending factors
 *
 * @api
 */
void ltdcLayerSetBlendingFactors(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                 ltdc_blendf_t bf) {

  chSysLock();
  ltdcLayerSetBlendingFactorsI(ltdcp, layer, bf);
  chSysUnlock();
}

/**
 * @brief   Get layer window specs.
 * @details Gets the window specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] winp     pointer to the output window specifications
 *
 * @api
 */
void ltdcLayerGetWindow(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                        ltdc_window_t *winp) {

  chSysLock();
  ltdcLayerGetWindowI(ltdcp, layer, winp);
  chSysUnlock();
}

/**
 * @brief   Set layer window specs.
 * @details Sets the window specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] winp     pointer to the window specifications
 *
 * @api
 */
void ltdcLayerSetWindow(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                        const ltdc_window_t *winp) {

  chSysLock();
  ltdcLayerSetWindowI(ltdcp, layer, winp);
  chSysUnlock();
}

/**
 * @brief   Set layer window as invalid.
 * @details Sets the window specifications of a layer so that the window is
 *          pixel sized at the screen origin.
 * @note    Useful before reconfiguring the frame specifications of the layer,
 *          to avoid errors.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @api
 */
void ltdcLayerSetInvalidWindow(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  chSysLock();
  ltdcLayerSetInvalidWindowI(ltdcp, layer);
  chSysUnlock();
}

/**
 * @brief   Get layer frame buffer specs.
 * @details Gets the frame buffer specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] framep   pointer to the output frame buffer specifications
 *
 * @api
 */
void ltdcLayerGetFrame(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                       ltdc_frame_t *framep) {

  chSysLock();
  ltdcLayerGetFrameI(ltdcp, layer, framep);
  chSysUnlock();
}

/**
 * @brief   Set layer frame buffer specs.
 * @details Sets the frame buffer specifications of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[out] framep   pointer to the frame buffer specifications
 *
 * @api
 */
void ltdcLayerSetFrame(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                       const ltdc_frame_t *framep) {

  chSysLock();
  ltdcLayerSetFrameI(ltdcp, layer, framep);
  chSysUnlock();
}

/**
 * @brief   Get layer frame buffer address.
 * @details Gets the frame buffer address of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 *
 * @return              frame buffer address
 *
 * @api
 */
void *ltdcLayerGetFrameAddress(LTDCDriver *ltdcp, ltdc_layerid_t layer) {

  void *bufferp;
  chSysLock();
  bufferp = ltdcLayerGetFrameAddressI(ltdcp, layer);
  chSysUnlock();
  return bufferp;
}

/**
 * @brief   Set layer frame buffer address.
 * @details Sets the frame buffer address of a layer.
 *
 * @param[in] ltdcp     pointer to the @p LTDCDriver object
 * @param[in] layer     layer identifier
 * @param[in] bufferp   frame buffer address
 *
 * @api
 */
void ltdcLayerSetFrameAddress(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                              void *bufferp) {

  chSysLock();
  ltdcLayerSetFrameAddressI(ltdcp, layer, bufferp);
  chSysUnlock();
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
size_t ltdcBitsPerPixel(ltdc_pixfmt_t fmt) {

  chDbgAssert(fmt < 8, "ltdcBitsPerPixel(), #1", "invalid format");

  return (size_t)ltdc_bpp[(unsigned)fmt];
}

#if LTDC_NEED_CONVERSIONS || defined(__DOXYGEN__)

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
ltdc_color_t ltdcFromARGB8888(ltdc_color_t c, ltdc_pixfmt_t fmt) {

  switch (fmt) {
  case LTDC_FMT_ARGB8888: {
    return c;
  }
  case LTDC_FMT_RGB888: {
    return c & 0x00FFFFFF;
  }
  case LTDC_FMT_RGB565: {
    return ((c & 0x000000F8) >> ( 8 -  5)) |
           ((c & 0x0000FC00) >> (16 - 11)) |
           ((c & 0x00F80000) >> (24 - 16));
  }
  case LTDC_FMT_ARGB1555: {
    return ((c & 0x000000F8) >> ( 8 -  5)) |
           ((c & 0x0000F800) >> (16 - 10)) |
           ((c & 0x00F80000) >> (24 - 15)) |
           ((c & 0x80000000) >> (32 - 16));
  }
  case LTDC_FMT_ARGB4444: {
    return ((c & 0x000000F0) >> ( 8 -  4)) |
           ((c & 0x0000F000) >> (16 -  8)) |
           ((c & 0x00F00000) >> (24 - 12)) |
           ((c & 0xF0000000) >> (32 - 16));
  }
  case LTDC_FMT_L8: {
    return c & 0x000000FF;
  }
  case LTDC_FMT_AL44: {
    return ((c & 0x000000F0) >> ( 8 - 4)) |
           ((c & 0xF0000000) >> (32 - 8));
  }
  case LTDC_FMT_AL88: {
    return ((c & 0x000000FF) >> ( 8 -  8)) |
           ((c & 0xFF000000) >> (32 - 16));
  }
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
ltdc_color_t ltdcToARGB8888(ltdc_color_t c, ltdc_pixfmt_t fmt) {

  switch (fmt) {
  case LTDC_FMT_ARGB8888: {
    return c;
  }
  case LTDC_FMT_RGB888: {
    return (c & 0x00FFFFFF) | 0xFF000000;
  }
  case LTDC_FMT_RGB565: {
    register ltdc_color_t output = 0xFF000000;
    if (c & 0x001F) output |= ((c & 0x001F) << ( 8 -  5)) | 0x00000007;
    if (c & 0x07E0) output |= ((c & 0x07E0) << (16 - 11)) | 0x00000300;
    if (c & 0xF800) output |= ((c & 0xF800) << (24 - 16)) | 0x00070000;
    return output;
  }
  case LTDC_FMT_ARGB1555: {
    register ltdc_color_t output = 0x00000000;
    if (c & 0x001F) output |= ((c & 0x001F) << ( 8 -  5)) | 0x00000007;
    if (c & 0x03E0) output |= ((c & 0x03E0) << (16 - 10)) | 0x00000700;
    if (c & 0x7C00) output |= ((c & 0x7C00) << (24 - 15)) | 0x00070000;
    if (c & 0x8000) output |= 0xFF000000;
    return output;
  }
  case LTDC_FMT_ARGB4444: {
    register ltdc_color_t output = 0x00000000;
    if (c & 0x000F) output |= ((c & 0x000F) << ( 8 -  4)) | 0x0000000F;
    if (c & 0x00F0) output |= ((c & 0x00F0) << (16 -  8)) | 0x00000F00;
    if (c & 0x0F00) output |= ((c & 0x0F00) << (24 - 12)) | 0x000F0000;
    if (c & 0xF000) output |= ((c & 0xF000) << (32 - 16)) | 0x0F000000;
    return output;
  }
  case LTDC_FMT_L8: {
    return (c & 0xFF) | 0xFF000000;
  }
  case LTDC_FMT_AL44: {
    register ltdc_color_t output = 0x00000000;
    if (c & 0x0F) output |= ((c & 0x0F) << ( 8 - 4)) | 0x0000000F;
    if (c & 0xF0) output |= ((c & 0xF0) << (32 - 8)) | 0x0F000000;
    return output;
  }
  case LTDC_FMT_AL88: {
    return ((c & 0x00FF) << ( 8 -  8)) |
           ((c & 0xFF00) << (32 - 16));
  }
  default:
    chDbgAssert(FALSE, "(), #1", "invalid format");
    return 0;
  }
}

#endif /* LTDC_NEED_CONVERSIONS */

/** @} */

#endif /* STM32_LTDC_USE_LTDC */
