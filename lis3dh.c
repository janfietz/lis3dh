/**
 * @file    lis3dh.c
 * @brief   LIS3DH Driver code.
 *
 * @addtogroup LIS3DH
 * @{
 */

#include "hal.h"
#include "lis3dh.h"

#if HAL_USE_LIS3DH || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/* Registers */
#define STATUS_REG_AUX      0x07
#define OUT_ADC1_L          0x08
#define OUT_ADC1_H          0x09
#define OUT_ADC2_L          0x0A
#define OUT_ADC2_H          0x0B
#define OUT_ADC3_L          0x0C
#define OUT_ADC3_H          0x0D
#define WHO_AM_I            0x0F
#define CTRL_REG0           0x1E
#define TEMP_CFG_REG        0x1F
#define CTRL_REG1           0X20
#define CTRL_REG2           0x21
#define CTRL_REG3           0x22
#define CTRL_REG4           0x23
#define CTRL_REG5           0x24
#define CTRL_REG6           0x25
#define REFERENCE           0x26
#define STATUS_REG          0x27
#define OUT_X_L             0x28
#define OUT_X_H             0x29
#define OUT_Y_L             0x2A
#define OUT_Y_H             0x2B
#define OUT_Z_L             0x2C
#define OUT_Z_H             0x2D
#define FIFO_CTRL_REG       0x2E
#define FIFO_SRC_REG        0x2F
#define INT1_CFG            0x30
#define INT1_SRC            0x31
#define INT1_THS            0x32
#define INT1_DURATION       0x33
#define INT2_CFG            0x34
#define INT2_SRC            0x35
#define INT2_THS            0x36
#define INT2_DURATION       0x37
#define CLICK_CFG           0x38
#define CLICK_SRC           0x39
#define CLICK_THS           0x3A
#define TIME_LIMIT          0x3B
#define TIME_LATENCY        0x3C
#define TIME WINDOW         0x3D
#define ACT_THS             0x3E
#define ACT_DUR             0x3F

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

/**
 * @brief   LIS3DH Driver initialization.
 * @note    This function is implicitly invoked by @p halInit(), there is
 *          no need to explicitly initialize the driver.
 *
 * @init
 */
void lis3dhInit(void) {}

/**
 * @brief   Initializes the standard part of a @p LIS3DHDriver structure.
 *
 * @param[out] lis3dhp     pointer to the @p LIS3DHDriver object
 *
 * @init
 */
void lis3dhObjectInit(lis3dhDriver *lis3dhp) {
    lis3dhp->state = LIS3DH_STOP;
    lis3dhp->config = NULL;
}

/**
 * @brief   Configures and activates the LIS3DH peripheral.
 *
 * @param[in] lis3dhp   pointer to the @p LIS3DHDriver object
 * @param[in] config    pointer to the @p LIS3DHConfig object
 *
 * @api
 */

void ws281xStart(LIS3DHDriver *lis3dhp, const lis3dhConfig *config) {
    osalDbgCheck((lis3dhp != NULL) && (config != NULL));

    osalSysLock();
    osalDbgAssert((lis3dhp->state == LIS3DH_STOP) ||
        (lis3dhp->state == LIS3DH_READY), "invalid state");
    lis3dhp->config = config;
    osalSysUnlock();

    osalSysLock();
    lis3dhp->state = LIS3DH_ACTIVE;
    osalSysUnlock();
}

/**
 * @brief   Deactivates the LIS3DH peripheral.
 *
 * @param[in] lis3dhp      pointer to the @p LIS3DHDriver object
 *
 * @api
 */
void lis3dhStop(LIS3DHDriver *lis3dhp) {
    osalDbgCheck(lis3dhp != NULL);

    osalSysLock();
    osalDbgAssert((lis3dhp->state == LIS3DH_STOP) ||
        (lis3dhp->state == LIS3DH_READY),
            "invalid state");

    lis3dhp->state = LIS3DH_STOP;
    osalSysUnlock();
}

#endif /* HAL_USE_LIS3DH */

/** @} */
