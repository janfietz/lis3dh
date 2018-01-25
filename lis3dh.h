
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

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

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
 * @brief   Type of a structure representing an ws281xDriver driver.
 */
typedef struct lis3dhDriver lis3dhDriver;
/**
 * @brief   Driver configuration structure.
 */
typedef struct {

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
  void lis3dhObjectInit(lis3dhDriver *lis3dhp);
  void lis3dhStart(lis3dhDriver *lis3dhp, const lis3dhConfig *config);
  void lis3dhStop(lis3dhDriver *lis3dhp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_LIS3DH */

#endif /* _LIS3DH_H_ */

/** @} */
