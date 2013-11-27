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
 * @file    stm32_ltdc.h
 * @brief   LCD-TFT Controller Driver.
 * @pre     This driver needs a supported STM32 device.
 *
 * @addtogroup ltdc
 * @{
 */

#ifndef _STM32_LTDC_H_
#define _STM32_LTDC_H_

/**
 * @brief   Using the LTDC driver.
 */
#if !defined(STM32_LTDC_USE_LTDC) || defined(__DOXYGEN__)
#define STM32_LTDC_USE_LTDC     FALSE
#endif

#if STM32_LTDC_USE_LTDC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    LTDC layers
 * @{
 */
#define LTDC_L1                 0           /**< Layer 1.*/
#define LTDC_L2                 1           /**< Layer 2.*/
/** @} */

/**
 * @name    LTDC enable flags
 */
#define LTDC_EF_ENABLE          (1 <<  0)   /**< LTDC enabled.*/
#define LTDC_EF_DITHER          (1 << 16)   /**< Dithering enabled.*/
#define LTDC_EF_PIXCLK_INVERT   (1 << 28)   /**< Inverted pixel clock.*/
#define LTDC_EF_DATAEN_HIGH     (1 << 29)   /**< Active-high data enable.*/
#define LTDC_EF_VSYNC_HIGH      (1 << 30)   /**< Active-high vsync.*/
#define LTDC_EF_HSYNC_HIGH      (1 << 31)   /**< Active-high hsync.*/

#define LTDC_EF_MASK \
  (LTDC_EF_ENABLE | LTDC_EF_DITHER | LTDC_EF_PIXCLK_INVERT | \
   LTDC_EF_DATAEN_HIGH | LTDC_EF_VSYNC_HIGH | LTDC_EF_HSYNC_HIGH)
/** @} */

/**
 * @name    LTDC layer enable flags
 */
#define LTDC_LEF_ENABLE         (1 << 0)    /**< Layer enabled*/
#define LTDC_LEF_KEYING         (1 << 1)    /**< Color keying enabled.*/
#define LTDC_LEF_PALETTE        (1 << 4)    /**< Palette enabled.*/

#define LTDC_LEF_MASK \
  (LTDC_LEF_ENABLE | LTDC_LEF_KEYING | LTDC_LEF_PALETTE)
/** @} */

/**
 * @name    LTDC pixel formats
 * @{
 */
#define LTDC_FMT_ARGB8888       0           /**< ARGB-8888 format.*/
#define LTDC_FMT_RGB888         1           /**< RGB-888 format.*/
#define LTDC_FMT_RGB565         2           /**< RGB-565 format.*/
#define LTDC_FMT_ARGB1555       3           /**< ARGB-1555 format.*/
#define LTDC_FMT_ARGB4444       4           /**< ARGB-4444 format.*/
#define LTDC_FMT_L8             5           /**< L-8 format.*/
#define LTDC_FMT_AL44           6           /**< AL-44 format.*/
#define LTDC_FMT_AL88           7           /**< AL-88 format.*/
/** @} */

/**
 * @name    LTDC pixel format aliased raw masks
 * @{
 */
#define LTDC_XMASK_ARGB8888     0xFFFFFFFF  /**< ARGB-8888 aliased mask.*/
#define LTDC_XMASK_RGB888       0x00FFFFFF  /**< RGB-888 aliased mask.*/
#define LTDC_XMASK_RGB565       0x00F8FCF8  /**< RGB-565 aliased mask.*/
#define LTDC_XMASK_ARGB1555     0x80F8F8F8  /**< ARGB-1555 aliased mask.*/
#define LTDC_XMASK_ARGB4444     0xF0F0F0F0  /**< ARGB-4444 aliased mask.*/
#define LTDC_XMASK_L8           0x000000FF  /**< L-8 aliased mask.*/
#define LTDC_XMASK_AL44         0xF00000F0  /**< AL-44 aliased mask.*/
#define LTDC_XMASK_AL88         0xFF0000FF  /**< AL-88 aliased mask.*/
/** @} */

/**
 * @name    LTDC blending factors
 * @{
 */
#define LTDC_BLEND_FIX1_FIX2    0x0405      /**<      cnst1; 1 -      cnst2 */
#define LTDC_BLEND_FIX1_MOD2    0x0407      /**<      cnst1; 1 - a2 * cnst2 */
#define LTDC_BLEND_MOD1_FIX2    0x0605      /**< a1 * cnst1; 1 -      cnst2 */
#define LTDC_BLEND_MOD1_MOD2    0x0607      /**< a1 * cnst1; 1 - a2 * cnst2 */
/** @} */

/**
 * @name    LTDC parameter bounds
 * @{
 */

#define LTDC_MIN_SCREEN_WIDTH           1
#define LTDC_MIN_SCREEN_HEIGHT          1
#define LTDC_MAX_SCREEN_WIDTH           800
#define LTDC_MAX_SCREEN_HEIGHT          600

#define LTDC_MIN_HSYNC_WIDTH            1
#define LTDC_MIN_VSYNC_HEIGHT           1
#define LTDC_MAX_HSYNC_WIDTH            (1 << 12)
#define LTDC_MAX_VSYNC_HEIGHT           (1 << 11)

#define LTDC_MIN_HBP_WIDTH              0
#define LTDC_MIN_VBP_HEIGHT             0
#define LTDC_MAX_HBP_WIDTH              (1 << 12)
#define LTDC_MAX_VBP_HEIGHT             (1 << 11)

#define LTDC_MIN_ACC_HBP_WIDTH          1
#define LTDC_MIN_ACC_VBP_HEIGHT         1
#define LTDC_MAX_ACC_HBP_WIDTH          (1 << 12)
#define LTDC_MAX_ACC_VBP_HEIGHT         (1 << 11)

#define LTDC_MIN_HFP_WIDTH              0
#define LTDC_MIN_VFP_HEIGHT             0
#define LTDC_MAX_HFP_WIDTH              (1 << 12)
#define LTDC_MAX_VFP_HEIGHT             (1 << 11)

#define LTDC_MIN_ACTIVE_WIDTH           0
#define LTDC_MIN_ACTIVE_HEIGHT          0
#define LTDC_MAX_ACTIVE_WIDTH           (1 << 12)
#define LTDC_MAX_ACTIVE_HEIGHT          (1 << 11)

#define LTDC_MIN_ACC_ACTIVE_WIDTH       1
#define LTDC_MIN_ACC_ACTIVE_HEIGHT      1
#define LTDC_MAX_ACC_ACTIVE_WIDTH       (1 << 12)
#define LTDC_MAX_ACC_ACTIVE_HEIGHT      (1 << 11)

#define LTDC_MIN_ACC_TOTAL_WIDTH        1
#define LTDC_MIN_ACC_TOTAL_HEIGHT       1
#define LTDC_MAX_ACC_TOTAL_WIDTH        (1 << 12)
#define LTDC_MAX_ACC_TOTAL_HEIGHT       (1 << 11)

#define LTDC_MIN_LINE_INTERRUPT_POS     0
#define LTDC_MAX_LINE_INTERRUPT_POS     ((1 << 11) - 1)

#define LTDC_MIN_WINDOW_HSTART          0
#define LTDC_MIN_WINDOW_HSTART          0
#define LTDC_MAX_WINDOW_HSTOP           ((1 << 12) - 1)
#define LTDC_MAX_WINDOW_HSTOP           ((1 << 12) - 1)

#define LTDC_MIN_WINDOW_VSTART          0
#define LTDC_MIN_WINDOW_VSTART          0
#define LTDC_MAX_WINDOW_VSTOP           ((1 << 11) - 1)
#define LTDC_MAX_WINDOW_VSTOP           ((1 << 11) - 1)

#define LTDC_MIN_FRAME_WIDTH_BYTES      0
#define LTDC_MIN_FRAME_HEIGHT_LINES     0
#define LTDC_MIN_FRAME_PITCH_BYTES      0
#define LTDC_MAX_FRAME_WIDTH_BYTES      ((1 << 13) - 1 - 3)
#define LTDC_MAX_FRAME_HEIGHT_LINES     ((1 << 11) - 1)
#define LTDC_MAX_FRAME_PITCH_BYTES      ((1 << 13) - 1)

#define LTDC_MIN_PIXFMT_ID              0
#define LTDC_MAX_PIXFMT_ID              7

/** @} */

/**
 * @name    LTDC basic ARGB-8888 colors.
 * @{
 */
/* Microsoft Windows default 16-color palette.*/
#define LTDC_COLOR_BLACK        0xFF000000
#define LTDC_COLOR_MAROON       0xFF800000
#define LTDC_COLOR_GREEN        0xFF008000
#define LTDC_COLOR_OLIVE        0xFF808000
#define LTDC_COLOR_NAVY         0xFF000080
#define LTDC_COLOR_PURPLE       0xFF800080
#define LTDC_COLOR_TEAL         0xFF008080
#define LTDC_COLOR_SILVER       0xFFC0C0C0
#define LTDC_COLOR_GRAY         0xFF808080
#define LTDC_COLOR_RED          0xFFFF0000
#define LTDC_COLOR_LIME         0xFF00FF00
#define LTDC_COLOR_YELLOW       0xFFFFFF00
#define LTDC_COLOR_BLUE         0xFF0000FF
#define LTDC_COLOR_FUCHSIA      0xFFFF00FF
#define LTDC_COLOR_AQUA         0xFF00FFFF
#define LTDC_COLOR_WHITE        0xFFFFFFFF
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   The device has the LTDC peripheral.
 */
#if !defined(STM32_HAS_LTDC) || defined(__DOXYGEN__)
#ifdef STM32F429_439xx
#define STM32_HAS_LTDC                      TRUE
#else
#define STM32_HAS_LTDC                      FALSE
#endif
#endif

/**
 * @name    Configuration options
 * @{
 */

/**
 * @brief   LTDC event interrupt priority level setting.
 */
#if !defined(STM32_LTDC_EV_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_LTDC_EV_IRQ_PRIORITY          11
#endif

/**
 * @brief   LTDC error interrupt priority level setting.
 */
#if !defined(STM32_LTDC_ER_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define STM32_LTDC_ER_IRQ_PRIORITY          11
#endif

/** @} */

/**
 * @brief   Provides software color conversion functions.
 */
#if !defined(LTDC_NEED_CONVERSIONS) || defined(__DOXYGEN__)
#define LTDC_NEED_CONVERSIONS               FALSE
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#ifndef STM32F429_439xx
#error "Currently only STM32F429xx and STM32F439xx are supported"
#endif

#if !STM32_HAS_LTDC
#error "LTDC must be present when using the LTDC subsystem"
#endif

#if STM32_LTDC_USE_LTDC && !STM32_HAS_LTDC
#error "LTDC not present in the selected device"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/* Complex types forwarding.*/
typedef union ltdc_coloralias_t_ ltdc_coloralias_t;
typedef struct ltdc_window_t_ ltdc_window_t;
typedef struct ltdc_frame_t_ ltdc_frame_t;
typedef struct LTDCConfig_ LTDCConfig;
typedef enum ltdc_state_t_ ltdc_state_t;
typedef struct LTDCDriver_ LTDCDriver;

/**
 * @name    LTDC Data types
 * @{
 */

/**
 * @brief   LTDC generic color.
 */
typedef uint32_t ltdc_color_t;

/**
 * @brief   LTDC color aliases.
 * @detail  Mapped with ARGB-8888, except for luminance (L mapped onto B).
 *          Padding fields prefixed with <tt>'x'</tt>, which should be clear
 *          (all 0) before compression and set (all 1) after expansion.
 */
typedef union ltdc_coloralias_t_ {
  struct {
    unsigned  b     :  8;
    unsigned  g     :  8;
    unsigned  r     :  8;
    unsigned  a     :  8;
  }             argb8888;           /**< Mapped ARGB-8888 bits.*/
  struct {
    unsigned  b     :  8;
    unsigned  g     :  8;
    unsigned  r     :  8;
    unsigned  xa    :  8;
  }             rgb888;             /**< Mapped RGB-888 bits.*/
  struct {
    unsigned  xb    :  3;
    unsigned  b     :  5;
    unsigned  xg    :  2;
    unsigned  g     :  6;
    unsigned  xr    :  3;
    unsigned  r     :  5;
    unsigned  xa    :  8;
  }             rgb565;             /**< Mapped RGB-565 bits.*/
  struct {
    unsigned  xb    :  3;
    unsigned  b     :  5;
    unsigned  xg    :  3;
    unsigned  g     :  5;
    unsigned  xr    :  3;
    unsigned  r     :  5;
    unsigned  xa    :  7;
    unsigned  a     :  1;
  }             argb1555;           /**< Mapped ARGB-1555 values.*/
  struct {
    unsigned  xb    :  4;
    unsigned  b     :  4;
    unsigned  xg    :  4;
    unsigned  g     :  4;
    unsigned  xr    :  4;
    unsigned  r     :  4;
    unsigned  xa    :  4;
    unsigned  a     :  4;
  }             argb4444;           /**< Mapped ARGB-4444 values.*/
  struct {
    unsigned  l     :  8;
    unsigned  x     : 16;
    unsigned  xa    :  8;
  }             l8;                 /**< Mapped L-8 bits.*/
  struct {
    unsigned  xl    :  4;
    unsigned  l     :  4;
    unsigned  x     : 16;
    unsigned  xa    :  4;
    unsigned  a     :  4;
  }             al44;               /**< Mapped AL-44 bits.*/
  struct {
    unsigned  l     :  8;
    unsigned  x     : 16;
    unsigned  a     :  8;
  }             al88;               /**< Mapped AL-88 bits.*/
  ltdc_color_t  aliased;            /**< Aliased raw bits.*/
} ltdc_coloralias_t;

/**
 * @brief   LTDC layer identifier.
 */
typedef uint32_t ltdc_layerid_t;

/**
 * @brief   LTDC pixel format.
 */
typedef uint32_t ltdc_pixfmt_t;

/**
 * @brief   LTDC blending factor.
 */
typedef uint32_t ltdc_blendf_t;

/**
 * @brief   LTDC ISR callback.
 */
typedef void (*ltdc_isrcb_t)(LTDCDriver *ltdcp);

/**
 * @brief   LTDC window specifications.
 */
typedef struct ltdc_window_t_ {
  uint16_t      hstart;             /**< Horizontal start pixel (left).*/
  uint16_t      hstop;              /**< Horizontal stop pixel (right).*/
  uint16_t      vstart;             /**< Vertical start pixel (top).*/
  uint16_t      vstop;              /**< Vertical stop pixel (bottom).*/
} ltdc_window_t;

/**
 * @brief   LTDC frame specifications.
 */
typedef struct ltdc_frame_t_ {
  void          *bufferp;           /**< Frame buffer address.*/
  uint16_t      width;              /**< Frame width, in pixels.*/
  uint16_t      height;             /**< Frame height, in pixels.*/
  size_t        pitch;              /**< Line pitch, in bytes.*/
  ltdc_pixfmt_t fmt;                /**< Pixel format.*/
} ltdc_frame_t;

/**
 * @brief   LTDC configuration flags.
 */
typedef uint8_t ltdc_flags_t;

/**
 * @brief   LTDC startup layer configuration.
 */
typedef struct ltdc_layercfg_t_ {
  const ltdc_frame_t  *frame;       /**< Frame buffer specifications.*/
  const ltdc_window_t *window;      /**< Window specifications.*/
  ltdc_color_t        def_color;    /**< Default color, ARGB-8888.*/
  ltdc_color_t        key_color;    /**< Color key.*/
  const ltdc_color_t  *pal_colors;  /**< Palette colors, or @p NULL.*/
  uint16_t            pal_length;   /**< Palette length, or @p 0.*/
  uint8_t             const_alpha;  /**< Constant alpha factor.*/
  ltdc_blendf_t       blending;     /**< Blending factors.*/
  ltdc_flags_t        flags;        /**< Layer configuration flags.*/
} ltdc_laycfg_t;

/**
 * @brief   LTDC driver configuration.
 */
typedef struct LTDCConfig_ {
  /* Display specifications.*/
  uint16_t      screen_width;       /**< Screen pixel width.*/
  uint16_t      screen_height;      /**< Screen pixel height.*/
  uint16_t      hsync_width;        /**< Horizontal sync pixel width.*/
  uint16_t      vsync_height;       /**< Vertical sync pixel height.*/
  uint16_t      hbp_width;          /**< Horizontal back porch pixel width.*/
  uint16_t      vbp_height;         /**< Vertical back porch pixel height.*/
  uint16_t      hfp_width;          /**< Horizontal front porch pixel width.*/
  uint16_t      vfp_height;         /**< Vertical front porch pixel height.*/
  ltdc_flags_t  flags;              /**< Driver configuration flags.*/

  /* ISR callbacks.*/
  ltdc_isrcb_t  line_isrcb;         /**< Line Interrupt ISR, or @p NULL.*/
  ltdc_isrcb_t  rr_isrcb;           /**< Register Reload ISR, or @p NULL.*/
  ltdc_isrcb_t  fuerr_isrcb;        /**< FIFO Underrun ISR, or @p NULL.*/
  ltdc_isrcb_t  terr_isrcb;         /**< Transfer Error ISR, or @p NULL.*/

  /* Layer and color settings.*/
  ltdc_color_t  bg_color;           /**< Background color, RGB-888.*/
  const ltdc_laycfg_t *laycfgs[2];  /**< Layer configurations, or @p NULL.*/
} LTDCConfig;

/**
 * @brief   LTDC driver state.
 */
typedef enum ltdc_state_t_ {
  LTDC_UNINIT   = 0,                /**< Not initialized.*/
  LTDC_STOP     = 1,                /**< Stopped.*/
  LTDC_READY    = 2,                /**< Ready.*/
  LTDC_ACTIVE   = 3,                /**< Executing commands.*/
} ltdc_state_t;

/**
 * @brief   LTDC driver.
 */
typedef struct LTDCDriver_ {
  ltdc_state_t      state;          /**< Driver state.*/
  const LTDCConfig  *config;        /**< Driver configuration.*/

  /* Handy computations.*/
  ltdc_window_t     active_window;  /**< Active window coordinates.*/
} LTDCDriver;

/** @} */

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Makes an ARGB-8888 value from byte components.
 *
 * @param[in] a         alpha byte component
 * @param[in] r         red byte component
 * @param[in] g         green byte component
 * @param[in] b         blue byte component
 *
 * @return              color in ARGB-8888 format
 *
 * @api
 */
#define ltdcMakeARGB8888(a, r, g, b) \
  ((((ltdc_color_t)(a) & 0xFF) << 24) | \
   (((ltdc_color_t)(r) & 0xFF) << 16) | \
   (((ltdc_color_t)(g) & 0xFF) <<  8) | \
   (((ltdc_color_t)(b) & 0xFF) <<  0))

/**
 * @brief   Compute bytes per pixel.
 * @details Computes the bytes per pixel for the specified pixel format.
 *          Rounds to the ceiling.
 *
 * @param[in] fmt       pixel format
 *
 * @return              bytes per pixel
 *
 * @api
 */
#define ltdcBytesPerPixel(fmt) \
  ((ltdcBitsPerPixel(fmt) + 7) >> 3)

/**
 * TODO
 */
#define ltdcReloadAndWaitI(ltdcp, immediate) \
  { ltdcReloadI((ltdcp), (immediate)); \
    ltdcWaitReloadI(ltdcp); }

/**
 * TODO
 */
#define ltdcReloadAndWait(ltdcp, immediate) \
  { ltdcReload((ltdcp), (immediate)); \
    ltdcWaitReload(ltdcp); }


/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

extern LTDCDriver LTDCD1;

#ifdef __cplusplus
extern "C" {
#endif

  void ltdcInit(void);
  void ltdcObjectInit(LTDCDriver *ltdcp);
  ltdc_state_t ltdcGetStateI(LTDCDriver *ltdcp);
  ltdc_state_t ltdcGetState(LTDCDriver *ltdcp);
  void ltdcStart(LTDCDriver *ltdcp, const LTDCConfig *configp);
  void ltdcStop(LTDCDriver *ltdcp);

  ltdc_flags_t ltdcGetEnableFlagsI(LTDCDriver *ltdcp);
  void ltdcSetEnableFlagsI(LTDCDriver *ltdcp, ltdc_flags_t flags);
  bool_t ltdcIsReloadingI(LTDCDriver *ltdcp);
  void ltdcReloadI(LTDCDriver *ltdcp, bool_t immediate);
  void ltdcWaitReloadI(LTDCDriver *ltdcp);
  bool_t ltdcIsDitheringEnabledI(LTDCDriver *ltdcp);
  void ltdcEnableDitheringI(LTDCDriver *ltdcp);
  void ltdcDisableDitheringI(LTDCDriver *ltdcp);
  ltdc_color_t ltdcGetBackgroundColorI(LTDCDriver *ltdcp);
  void ltdcSetBackgroundColorI(LTDCDriver *ltdcp, ltdc_color_t c);
  uint16_t ltdcGetLineInterruptPosI(LTDCDriver *ltdcp);
  void ltdcSetLineInterruptPosI(LTDCDriver *ltdcp, uint16_t pos);
  void ltdcGetCurrentPosI(LTDCDriver *ltdcp, uint16_t *xp, uint16_t *yp);

  ltdc_flags_t ltdcLayerGetEnableFlagsI(LTDCDriver *ltdcp,
                                        ltdc_layerid_t layer);
  void ltdcLayerSetEnableFlagsI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                ltdc_flags_t flags);
  bool_t ltdcLayerIsEnabledI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerEnableI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerDisableI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  bool_t ltdcLayerIsPaletteEnabledI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerEnablePaletteI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerDisablePaletteI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetPaletteColorI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                 uint8_t slot, ltdc_color_t c);
  void ltdcLayerSetPaletteI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                            const ltdc_color_t colors[], uint16_t length);
  ltdc_pixfmt_t ltdcLayerGetPixelFormatI(LTDCDriver *ltdcp,
                                         ltdc_layerid_t layer);
  void ltdcLayerSetPixelFormatI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                ltdc_pixfmt_t fmt);
  bool_t ltdcLayerIsKeyingEnabledI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerEnableKeyingI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerDisableKeyingI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  ltdc_color_t ltdcLayerGetKeyingColorI(LTDCDriver *ltdcp,
                                        ltdc_layerid_t layer);
  void ltdcLayerSetKeyingColorI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                ltdc_color_t c);
  uint8_t ltdcLayerGetConstantAlphaI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetConstantAlphaI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                  uint8_t a);
  ltdc_color_t ltdcLayerGetDefaultColorI(LTDCDriver *ltdcp,
                                         ltdc_layerid_t layer);
  void ltdcLayerSetDefaultColorI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                 ltdc_color_t c);
  ltdc_blendf_t ltdcLayerGetBlendingFactorsI(LTDCDriver *ltdcp,
                                             ltdc_layerid_t layer);
  void ltdcLayerSetBlendingFactorsI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                    ltdc_blendf_t bf);
  void ltdcLayerGetWindowI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                           ltdc_window_t *winp);
  void ltdcLayerSetWindowI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                           const ltdc_window_t *winp);
  void ltdcLayerSetInvalidWindowI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerGetFrameI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                          ltdc_frame_t *framep);
  void ltdcLayerSetFrameI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                          const ltdc_frame_t *framep);
  void *ltdcLayerGetFrameAddressI(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetFrameAddressI(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                 void *bufferp);

  ltdc_flags_t ltdcGetEnableFlags(LTDCDriver *ltdcp);
  void ltdcSetEnableFlags(LTDCDriver *ltdcp, ltdc_flags_t flags);
  bool_t ltdcIsReloading(LTDCDriver *ltdcp);
  void ltdcReload(LTDCDriver *ltdcp, bool_t immediate);
  void ltdcWaitReload(LTDCDriver *ltdcp);
  bool_t ltdcIsDitheringEnabled(LTDCDriver *ltdcp);
  void ltdcEnableDithering(LTDCDriver *ltdcp);
  void ltdcDisableDithering(LTDCDriver *ltdcp);
  ltdc_color_t ltdcGetBackgroundColor(LTDCDriver *ltdcp);
  void ltdcSetBackgroundColor(LTDCDriver *ltdcp, ltdc_color_t c);
  uint16_t ltdcGetLineInterruptPos(LTDCDriver *ltdcp);
  void ltdcSetLineInterruptPos(LTDCDriver *ltdcp, uint16_t pos);
  void ltdcGetCurrentPos(LTDCDriver *ltdcp, uint16_t *xp, uint16_t *yp);

  ltdc_flags_t ltdcLayerGetEnableFlags(LTDCDriver *ltdcp,
                                       ltdc_layerid_t layer);
  void ltdcLayerSetEnableFlags(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               ltdc_flags_t flags);
  bool_t ltdcLayerIsEnabled(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerEnable(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerDisable(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  bool_t ltdcLayerIsPaletteEnabled(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerEnablePalette(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerDisablePalette(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetPaletteColor(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                uint8_t slot, ltdc_color_t c);
  void ltdcLayerSetPalette(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                           const ltdc_color_t colors[], uint16_t length);
  ltdc_pixfmt_t ltdcLayerGetPixelFormat(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetPixelFormat(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               ltdc_pixfmt_t fmt);
  bool_t ltdcLayerIsKeyingEnabled(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerEnableKeying(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerDisableKeying(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  ltdc_color_t ltdcLayerGetKeyingColor(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetKeyingColor(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                               ltdc_color_t c);
  uint8_t ltdcLayerGetConstantAlpha(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetConstantAlpha(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                 uint8_t a);
  ltdc_color_t ltdcLayerGetDefaultColor(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetDefaultColor(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                ltdc_color_t c);
  ltdc_blendf_t ltdcLayerGetBlendingFactors(LTDCDriver *ltdcp,
                                            ltdc_layerid_t layer);
  void ltdcLayerSetBlendingFactors(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                   ltdc_blendf_t bf);
  void ltdcLayerGetWindow(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                          ltdc_window_t *winp);
  void ltdcLayerSetWindow(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                          const ltdc_window_t *winp);
  void ltdcLayerSetInvalidWindow(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerGetFrame(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                         ltdc_frame_t *framep);
  void ltdcLayerSetFrame(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                         const ltdc_frame_t *framep);
  void *ltdcLayerGetFrameAddress(LTDCDriver *ltdcp, ltdc_layerid_t layer);
  void ltdcLayerSetFrameAddress(LTDCDriver *ltdcp, ltdc_layerid_t layer,
                                void *bufferp);

  size_t ltdcBitsPerPixel(ltdc_pixfmt_t fmt);
#if LTDC_NEED_CONVERSIONS || defined(__DOXYGEN__)
  ltdc_color_t ltdcFromARGB8888(ltdc_color_t c, ltdc_pixfmt_t fmt);
  ltdc_color_t ltdcToARGB8888(ltdc_color_t c, ltdc_pixfmt_t fmt);
#endif /* LTDC_NEED_CONVERSIONS */

#ifdef __cplusplus
}
#endif

#endif /* STM32_LTDC_USE_LTDC */

#endif /* _STM32_LTDC_H_ */

/** @} */
