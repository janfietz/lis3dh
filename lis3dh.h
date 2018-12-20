
/**
 * @file    lis3dh.h
 * @brief   LIS3DH Driver macros and structures.
 *
 * Driver for LIS3DH 3-axis "nano" accelerometer.
 * It can be used on STM32 platforms.
 *
 * @addtogroup LIS3DH
 * @{
 */

#ifndef _LIS3DH_H_
#define _LIS3DH_H_

#include "ch.h"
#include "hal.h"

#if HAL_USE_LIS3DH || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   LIS3DSH accelerometer subsystem characteristics.
 * @note    Sensitivity is expressed as milli-G/LSB whereas
 *          1 milli-G = 0.00980665 m/s^2.
 * @note    Bias is expressed as milli-G.
 *
 * @{
 */
#define LIS3DH_NUMBER_OF_AXES          3U

#define LIS3DH_2G                      2.0f
#define LIS3DH_4G                      4.0f
#define LIS3DH_6G                      6.0f
#define LIS3DH_8G                      8.0f
#define LIS3DH_16G                     16.0f

#define LIS3DH_SENS_2G                 0.06f
#define LIS3DH_SENS_4G                 0.12f
#define LIS3DH_SENS_6G                 0.18f
#define LIS3DH_SENS_8G                 0.24f
#define LIS3DH_SENS_16G                0.73f

#define LIS3DSH_ACC_BIAS                    0.0f

/**
 * @name    LIS3DSH communication interfaces related bit masks
 * @{
 */
#define LIS3DH_DI_MASK                     0xFF
#define LIS3DH_DI(n)                       (1 << n)
#define LIS3DH_AD_MASK                     0x3F
#define LIS3DH_AD(n)                       (1 << n)
#define LIS3DH_MS                          (1 << 6)
#define LIS3DH_RW                          (1 << 7)
/** @} */

/**
 * @name    LIS3DH register addresses
 * @{
 */
#define LIS3DH_STATUS_REG_AUX      0x07
#define LIS3DH_OUT_ADC1_L          0x08
#define LIS3DH_OUT_ADC1_H          0x09
#define LIS3DH_OUT_ADC2_L          0x0A
#define LIS3DH_OUT_ADC2_H          0x0B
#define LIS3DH_OUT_ADC3_L          0x0C
#define LIS3DH_OUT_ADC3_H          0x0D
#define LIS3DH_WHO_AM_I            0x0F
#define LIS3DH_CTRL_REG0           0x1E
#define LIS3DH_TEMP_CFG_REG        0x1F
#define LIS3DH_CTRL_REG1           0X20
#define LIS3DH_CTRL_REG2           0x21
#define LIS3DH_CTRL_REG3           0x22
#define LIS3DH_CTRL_REG4           0x23
#define LIS3DH_CTRL_REG5           0x24
#define LIS3DH_CTRL_REG6           0x25
#define LIS3DH_REFERENCE           0x26
#define LIS3DH_STATUS_REG          0x27
#define LIS3DH_OUT_X_L             0x28
#define LIS3DH_OUT_X_H             0x29
#define LIS3DH_OUT_Y_L             0x2A
#define LIS3DH_OUT_Y_H             0x2B
#define LIS3DH_OUT_Z_L             0x2C
#define LIS3DH_OUT_Z_H             0x2D
#define LIS3DH_FIFO_CTRL_REG       0x2E
#define LIS3DH_FIFO_SRC_REG        0x2F
#define LIS3DH_INT1_CFG            0x30
#define LIS3DH_INT1_SRC            0x31
#define LIS3DH_INT1_THS            0x32
#define LIS3DH_INT1_DURATION       0x33
#define LIS3DH_INT2_CFG            0x34
#define LIS3DH_INT2_SRC            0x35
#define LIS3DH_INT2_THS            0x36
#define LIS3DH_INT2_DURATION       0x37
#define LIS3DH_CLICK_CFG           0x38
#define LIS3DH_CLICK_SRC           0x39
#define LIS3DH_CLICK_THS           0x3A
#define LIS3DH_TIME_LIMIT          0x3B
#define LIS3DH_TIME_LATENCY        0x3C
#define LIS3DH_TIME_WINDOW         0x3D
#define LIS3DH_ACT_THS             0x3E
#define LIS3DH_ACT_DUR             0x3F


/**
 * @name    LIS3DH_TEMP_CFG_REG register bits definitions
 * @{
 */
#define LIS3DH_TEMP_CFG_REG_MASK         0xC0
#define LIS3DH_TEMP_CFG_TEMP_EN          (1 << 6)
#define LIS3DH_TEMP_CFG_ADC_EN           (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CTRL_REG0 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG0_SDO_PU_CONN     0x90
#define LIS3DH_CTRL_REG0_SDO_PU_DISCONN  0x10
/** @} */

/**
 * @name   LIS3DH_CTRL_REG1 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG1_MASK     0xFF
#define LIS3DH_CTRL_REG1_XEN        (1 << 0) /**< X-axis enable               */
#define LIS3DH_CTRL_REG1_YEN        (1 << 1) /**< Y-axis enable               */
#define LIS3DH_CTRL_REG1_ZEN        (1 << 2) /**< Z-axis enable               */
#define LIS3DH_CTRL_REG1_LP       (1 << 3) /**< Low-power mode enable       */
#define LIS3DH_CTRL_REG1_ODR_0    (1 << 4)
#define LIS3DH_CTRL_REG1_ODR_1    (1 << 5)
#define LIS3DH_CTRL_REG1_ODR_2    (1 << 6)
#define LIS3DH_CTRL_REG1_ODR_3    (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CTRL_REG2 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG2_MASK       0xFF
#define LIS3DH_CTRL_REG2_HP_IA1     (1 << 0)
#define LIS3DH_CTRL_REG2_HP_IA2     (1 << 1)
#define LIS3DH_CTRL_REG2_HPCLICK    (1 << 2)
#define LIS3DH_CTRL_REG2_FDS        (1 << 3)
#define LIS3DH_CTRL_REG2_HPCF_1     (1 << 4)
#define LIS3DH_CTRL_REG2_HPCF_2     (1 << 5)
#define LIS3DH_CTRL_REG2_HPM_0      (1 << 6)
#define LIS3DH_CTRL_REG2_HPM_1      (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CTRL_REG3 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG3_MASK          0xFE
#define LIS3DH_CTRL_REG3_I1_OVERRUN    (1 << 1)
#define LIS3DH_CTRL_REG3_I1_WTM        (1 << 2)
#define LIS3DH_CTRL_REG3_I1_321DA      (1 << 3)
#define LIS3DH_CTRL_REG3_I1_ZYXDA      (1 << 4)
#define LIS3DH_CTRL_REG3_I1_IA2        (1 << 5)
#define LIS3DH_CTRL_REG3_I1_IA1        (1 << 6)
#define LIS3DH_CTRL_REG3_I1_CLICK      (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CTRL_REG4 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG4_MASK     0xFF
#define LIS3DH_CTRL_REG4_SIM      (1 << 0)
#define LIS3DH_CTRL_REG4_ST_0     (1 << 1)
#define LIS3DH_CTRL_REG4_ST_1     (1 << 2)
#define LIS3DH_CTRL_REG4_HR       (1 << 3)
#define LIS3DH_CTRL_REG4_FS_MASK  0x38
#define LIS3DH_CTRL_REG4_FS_0     (1 << 4)
#define LIS3DH_CTRL_REG4_FS_1     (1 << 5)
#define LIS3DH_CTRL_REG4_BLE      (1 << 6)
#define LIS3DH_CTRL_REG4_BDU      (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CTRL_REG5 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG5_MASK       0xCF
#define LIS3DH_CTRL_REG5_D4D_INT2  (1 << 0)
#define LIS3DH_CTRL_REG5_LIR_INT2  (1 << 1)
#define LIS3DH_CTRL_REG5_D4D_INT1  (1 << 2)
#define LIS3DH_CTRL_REG5_LIR_INT1  (1 << 3)
#define LIS3DH_CTRL_REG5_FIFO_EN   (1 << 6)
#define LIS3DH_CTRL_REG5_BOOT      (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CTRL_REG6 register bits definitions
 * @{
 */
#define LIS3DH_CTRL_REG6_MASK         0xFA
#define LIS3DH_CTRL_REG6_INT_POLARITY (1 << 1)
#define LIS3DH_CTRL_REG6_I2_ACT       (1 << 3)
#define LIS3DH_CTRL_REG6_I2_BOOT      (1 << 4)
#define LIS3DH_CTRL_REG6_I2_IA2       (1 << 5)
#define LIS3DH_CTRL_REG6_I2_IA1       (1 << 6)
#define LIS3DH_CTRL_REG6_I2_CLICK     (1 << 7)
/** @} */

/**
 * @name   LIS3DH_CLICK_CFG register bits definitions
 * @{
 */
#define LIS3DH_CLICK_CFG_MASK     0x3F
#define LIS3DH_CLICK_CFG_XS       (1 << 0)
#define LIS3DH_CLICK_CFG_XD       (1 << 1)
#define LIS3DH_CLICK_CFG_YS       (1 << 2)
#define LIS3DH_CLICK_CFG_YD       (1 << 3)
#define LIS3DH_CLICK_CFG_ZS       (1 << 4)
#define LIS3DH_CLICK_CFG_ZD       (1 << 5)
/** @} */

/**
 * @name   LIS3DH_CLICK_SRC register bits definitions
 * @{
 */
#define LIS3DH_CLICK_SRC_MASK     0xFE
#define LIS3DH_CLICK_SRC_X        (1 << 0)
#define LIS3DH_CLICK_SRC_Y        (1 << 1)
#define LIS3DH_CLICK_SRC_Z        (1 << 2)
#define LIS3DH_CLICK_SRC_SIGN     (1 << 3)
#define LIS3DH_CLICK_SRC_SCLICK   (1 << 4)
#define LIS3DH_CLICK_SRC_DCLICK   (1 << 5)
#define LIS3DH_CLICK_SRC_IA       (1 << 6)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/
/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   LIS3DH SPI interface switch.
 * @details If set to @p TRUE the support for SPI is included.
 * @note    The default is @p TRUE.
 */
#if !defined(LIS3DH_USE_SPI) || defined(__DOXYGEN__)
#define LIS3DH_USE_SPI                     TRUE
#endif

/**
 * @brief   LIS3DH shared SPI switch.
 * @details If set to @p TRUE the device acquires SPI bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires SPI_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LIS3DH_SHARED_SPI) || defined(__DOXYGEN__)
#define LIS3DH_SHARED_SPI                  FALSE
#endif

/**
 * @brief   LIS3DH I2C interface switch.
 * @details If set to @p TRUE the support for I2C is included.
 * @note    The default is @p FALSE.
 */
#if !defined(LIS3DH_USE_I2C) || defined(__DOXYGEN__)
#define LIS3DH_USE_I2C                     FALSE
#endif

/**
 * @brief   LIS3DH shared I2C switch.
 * @details If set to @p TRUE the device acquires I2C bus ownership
 *          on each transaction.
 * @note    The default is @p FALSE. Requires I2C_USE_MUTUAL_EXCLUSION.
 */
#if !defined(LIS3DH_SHARED_I2C) || defined(__DOXYGEN__)
#define LIS3DH_SHARED_I2C                  FALSE
#endif
/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/
#if !(LIS3DH_USE_SPI ^ LIS3DH_USE_I2C)
#error "LIS3DH_USE_SPI and LIS3DH_USE_I2C cannot be both true or both false"
#endif

#if LIS3DH_USE_SPI && !HAL_USE_SPI
#error "LIS3DH_USE_SPI requires HAL_USE_SPI"
#endif

#if LIS3DH_SHARED_SPI && !SPI_USE_MUTUAL_EXCLUSION
#error "LIS3DH_SHARED_SPI requires SPI_USE_MUTUAL_EXCLUSION"
#endif

#if LIS3DH_USE_I2C && !HAL_USE_I2C
#error "LIS3DH_USE_I2C requires HAL_USE_I2C"
#endif

#if LIS3DH_SHARED_I2C && !I2C_USE_MUTUAL_EXCLUSION
#error "LIS3DH_SHARED_I2C requires I2C_USE_MUTUAL_EXCLUSION"
#endif

/**
 * @todo    Add support for LIS3DH over I2C.
 */
#if LIS3DH_USE_I2C
#error "LIS3DH over I2C still not supported"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/
/**
 * @brief   LIS3DH High-pass filter mode configuration
 */
typedef enum {
  LIS3DH_HPM_NM_1 = 0x00,   /**< Normal mode */
  LIS3DH_HPM_RS = 0x40,     /**< Reference signal for filtering */
  LIS3DH_HPM_NM_2 = 0x80,   /**< Normal mode */
  LIS3DH_HPM_AR = 0xCu      /**< Autoreset on interrupt event */
}lis3dh_hpm_t;

/**
 * @brief   LIS3DH full scale.
 */
typedef enum {
  LIS3DH_FS_2G = 0x00,         /**< Full scale ±2g.  */ 
  LIS3DH_FS_4G = 0x04,         /**< Full scale ±4g.  */ 
  LIS3DH_FS_8G = 0x08,         /**< Full scale ±8g.  */
  LIS3DH_FS_16G = 0x0c         /**< Full scale ±16g. */
}lis3dh_fs_t;

/**
 * @brief   LIS3DH output data rate.
 */
typedef enum {
  LIS3DH_ODR_PD = 0x00,        /**< ODR 100 Hz.                        */
  LIS3DH_ODR_1HZ = 0x10,   /**< ODR 1 Hz.                      */
  LIS3DH_ODR_10HZ = 0x20,    /**< ODR 10 Hz.                       */
  LIS3DH_ODR_25HZ = 0x30,    /**< ODR 25 Hz.                       */
  LIS3DH_ODR_50HZ = 0x40,      /**< ODR 50 Hz.                         */
  LIS3DH_ODR_100HZ = 0x50,     /**< ODR 100 Hz.                        */
  LIS3DH_ODR_200HZ = 0x60,     /**< ODR 200 Hz.                        */
  LIS3DH_ODR_400HZ = 0x70,     /**< ODR 400 Hz.                        */
  LIS3DH_ODR_1600HZ = 0x80     /**< ODR 1600 Hz.                       */
}lis3dh_odr_t;

/**
 * @brief   LIS3DH anti-aliasing bandwidth.
 */
typedef enum {
  LIS3DH_BW_800HZ = 0x00,      /**< AA filter BW 800Hz.                */
  LIS3DH_BW_200HZ = 0x40,      /**< AA filter BW 200Hz.                */
  LIS3DH_BW_400HZ = 0x80,      /**< AA filter BW 400Hz.                */
  LIS3DH_BW_50HZ = 0xC0        /**< AA filter BW 50Hz.                 */
}lis3dh_bw_t;

/**
 * @brief   LIS3DH block data update.
 */
typedef enum {
  LIS3DH_BDU_CONTINUOUS = 0x00,/**< Block data continuously updated.   */
  LIS3DH_BDU_BLOCKED = 0x80    /**< Block data updated after reading.  */
} lis3dh_bdu_t;

/**
 * @brief   Driver state machine possible states.
 */
typedef enum {
  LIS3DH_UNINIT = 0,                   /**< Not initialized.                   */
  LIS3DH_STOP = 1,                     /**< Stopped.                           */
  LIS3DH_READY = 2,                    /**< Ready.                             */
  LIS3DH_ACTIVE = 3,                   /**< Active.                            */
} lis3dhState_t;
/**
 * @brief   Type of a structure representing an lis3dhDriver driver.
 */
typedef struct lis3dhDriver LIS3DHDriver;

/**
 * @brief   Driver configuration structure.
 */
typedef struct {

#if (LIS3DH_USE_SPI) || defined(__DOXYGEN__)
  /**
   * @brief SPI driver associated to this LIS3DH.
   */
  SPIDriver                 *spip;
  /**
   * @brief SPI configuration associated to this LIS3DH.
   */
  const SPIConfig           *spicfg;
#endif /* LIS3DH_USE_SPI */
#if (LIS3DH_USE_I2C) || defined(__DOXYGEN__)
  /**
   * @brief I2C driver associated to this LIS3DH.
   */
  I2CDriver                 *i2cp;
  /**
   * @brief I2C configuration associated to this LIS3DH.
   */
  const I2CConfig           *i2ccfg;
#endif /* LIS3DH_USE_I2C */

 /**
   * @brief LIS3DH accelerometer subsystem initial full scale.
   */
  lis3dh_fs_t          fullscale;

    /**
   * @brief LIS3DSH output data rate selection.
   */
  lis3dh_odr_t         outputdatarate;
  /**
   * @brief   LIS3DH anti-aliasing bandwidth.
   */
  lis3dh_bw_t          antialiasing;
  /**
   * @brief   LIS3DH block data update.
   */
  lis3dh_bdu_t         blockdataupdate;

} lis3dhConfig;


/**
 * @brief   Structure representing an ws281x driver.
 */
struct lis3dhDriver {
  /**
   * @brief   Driver state.
   */
	lis3dhState_t                state;
  /**
   * @brief   Current configuration data.
   */
  const lis3dhConfig           *config;
  /* End of the mandatory fields.*/
};
/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void lis3dhInit(void);
  void lis3dhObjectInit(LIS3DHDriver *lis3dhp);
  void lis3dhStart(LIS3DHDriver *lis3dhp, const lis3dhConfig *config);
  void lis3dhStop(LIS3DHDriver *lis3dhp);
  msg_t lis3dhReadRaw(LIS3DHDriver *lis3dhp, int32_t axes[]);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_LIS3DH */

#endif /* _LIS3DH_H_ */

/** @} */
