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

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
#if (LIS3DH_USE_SPI) || defined(__DOXYGEN__)
/**
 * @brief   Reads a generic register value using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer.
 */
static void lis3dhSPIReadRegister(SPIDriver *spip, uint8_t reg, size_t n,
                                   uint8_t* b) {
  uint8_t cmd;
  cmd = reg | LIS3DH_RW;
  if (n > 1) {
     cmd |= LIS3DH_MS;
  }
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiReceive(spip, n, b);
  spiUnselect(spip);
}

/**
 * @brief   Writes a value into a generic register using SPI.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       starting register address
 * @param[in] n         number of adjacent registers to write
 * @param[in] b         pointer to a buffer of values.
 */
static void lis3dhSPIWriteRegister(SPIDriver *spip, uint8_t reg, size_t n,
                                    uint8_t* b) {
  uint8_t cmd;
  cmd = reg;
  if (n > 1) {
     cmd |= LIS3DH_MS;
  }
  spiSelect(spip);
  spiSend(spip, 1, &cmd);
  spiSend(spip, n, b);
  spiUnselect(spip);
}
#endif /* LIS3DH_USE_SPI */

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
void lis3dhObjectInit(LIS3DHDriver *lis3dhp) {
    lis3dhp->config = NULL;
    lis3dhp->state = LIS3DH_STOP;
}

/**
 * @brief   Configures and activates the LIS3DH peripheral.
 *
 * @param[in] lis3dhp   pointer to the @p LIS3DHDriver object
 * @param[in] config    pointer to the @p LIS3DHConfig object
 *
 * @api
 */

void lis3dhStart(LIS3DHDriver *lis3dhp, const lis3dhConfig *config) {
    osalDbgCheck((lis3dhp != NULL) && (config != NULL));

    
    osalDbgAssert((lis3dhp->state == LIS3DH_STOP) ||
        (lis3dhp->state == LIS3DH_READY), "invalid state");
    lis3dhp->config = config;
    
    uint8_t cr = 0;
    {
        cr = LIS3DH_CTRL_REG1_XEN | LIS3DH_CTRL_REG1_YEN | LIS3DH_CTRL_REG1_ZEN |
            lis3dhp->config->outputdatarate;
    }

#if LIS3DH_USE_SPI
#if LIS3DH_SHARED_SPI
    spiAcquireBus(lis3dhp->config->spip);
#endif /* LIS3DH_SHARED_SPI */
    spiStart(lis3dhp->config->spip, lis3dhp->config->spicfg);

    lis3dhSPIWriteRegister(lis3dhp->config->spip, LIS3DH_CTRL_REG1, 1, &cr);
 
    cr = 0;
    {
        cr = lis3dhp->config->blockdataupdate | lis3dhp->config->fullscale;
    }
    lis3dhSPIWriteRegister(lis3dhp->config->spip, LIS3DH_CTRL_REG4, 1, &cr);

    if (config->temp_cfg > 0)
    {
        cr = config->temp_cfg;
        lis3dhSPIWriteRegister(lis3dhp->config->spip, LIS3DH_TEMP_CFG_REG, 1, &cr);
    }

#if LIS3DH_SHARED_SPI
    spiReleaseBus(lis3dhp->config->spip);
#endif /* LIS3DH_SHARED_SPI */
#endif /* LIS3DH_USE_SPI */

    /* This is the Accelerometer transient recovery time */
    osalThreadSleepMilliseconds(10);

    lis3dhp->state = LIS3DH_ACTIVE;
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

    if (lis3dhp->state == LIS3DH_READY) {
#if (LIS3DH_USE_SPI)
#if	LIS3DH_SHARED_SPI
    spiAcquireBus(lis3dhp->config->spip);
        spiStart(lis3dhp->config->spip,
             lis3dhp->config->spicfg);
#endif /* LIS3DH_SHARED_SPI */
        /* Disabling all axes and enabling power down mode.*/
        uint8_t cr1 = 0;
        lis3dhSPIWriteRegister(lis3dhp->config->spip, LIS3DH_CTRL_REG1,
                           1, &cr1);

        spiStop(lis3dhp->config->spip);
#if	LIS3DH_SHARED_SPI
        spiReleaseBus(lis3dhp->config->spip);
#endif /* LIS3DH_SHARED_SPI */    		
#endif /* LIS3DH_USE_SPI */
    }	

    lis3dhp->state = LIS3DH_STOP;
    osalSysUnlock();
}

/**
 * @brief   Retrieves raw data from the BaseAccelerometer.
 * @note    This data is retrieved from MEMS register without any algebraical
 *          manipulation.
 * @note    The axes array must be at least the same size of the
 *          BaseAccelerometer axes number.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[out] axes     a buffer which would be filled with raw data.
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */
msg_t lis3dhReadRaw(LIS3DHDriver *lis3dhp, int32_t axes[]) {
  
    msg_t msg = MSG_OK;

    osalDbgCheck((lis3dhp != NULL) && (axes != NULL));

    osalDbgAssert((lis3dhp->state == LIS3DH_ACTIVE),
                "acc_read_raw(), invalid state");

#if LIS3DH_USE_SPI
#if	LIS3DH_SHARED_SPI
    osalDbgAssert((lis3dhp->config->spip->state == SPI_READY),
                "acc_read_raw(), channel not ready");

    spiAcquireBus(lis3dhp->config->spip);
    spiStart(lis3dhp->config->spip,
           lis3dhp->config->spicfg);
#endif /* LIS3DH_SHARED_SPI */

    uint8_t buff [LIS3DH_NUMBER_OF_AXES * 2];
    lis3dhSPIReadRegister(lis3dhp->config->spip, LIS3DH_OUT_X_L,
                         LIS3DH_NUMBER_OF_AXES * 2, buff);

#if	LIS3DH_SHARED_SPI
    spiReleaseBus(lis3dhp->config->spip);
#endif /* LIS3DH_SHARED_SPI */
#endif /* LIS3DH_USE_SPI */

    int16_t tmp;
    uint8_t i;
    for(i = 0; i < LIS3DH_NUMBER_OF_AXES; i++) {
        tmp = buff[2 * i] + (buff[2 * i + 1] << 8);
        axes[i] = (int32_t)tmp;
    }
    return msg;
}

/**
 * @brief   Retrieves temperatue data from the accelerometer.
 *
 * @param[in] ip        pointer to @p BaseAccelerometer interface.
 * @param[out] temp     point to target temperatur value
 *
 * @return              The operation status.
 * @retval MSG_OK       if the function succeeded.
 * @retval MSG_RESET    if one or more I2C errors occurred, the errors can
 *                      be retrieved using @p i2cGetErrors().
 * @retval MSG_TIMEOUT  if a timeout occurred before operation end.
 */

msg_t lis3dhReadTemp(LIS3DHDriver *lis3dhp, int16_t* temp)
{
    msg_t msg = MSG_OK;

    osalDbgCheck(lis3dhp != NULL);

    osalDbgAssert((lis3dhp->state == LIS3DH_ACTIVE),
                "acc_read_raw(), invalid state");

#if LIS3DH_USE_SPI
#if	LIS3DH_SHARED_SPI
    osalDbgAssert((lis3dhp->config->spip->state == SPI_READY),
                "acc_read_raw(), channel not ready");

    spiAcquireBus(lis3dhp->config->spip);
    spiStart(lis3dhp->config->spip,
           lis3dhp->config->spicfg);
#endif /* LIS3DH_SHARED_SPI */

    uint8_t buff [2];
    lis3dhSPIReadRegister(lis3dhp->config->spip, LIS3DH_OUT_ADC3_L,
                         2, buff);

#if	LIS3DH_SHARED_SPI
    spiReleaseBus(lis3dhp->config->spip);
#endif /* LIS3DH_SHARED_SPI */
#endif /* LIS3DH_USE_SPI */

    int32_t rawTemp = (buff[0] + (buff[1] << 8)) >> 6;
    *temp = (((((rawTemp + 400) * 1000 )/ 800) * 125) / 1000) - 40;
    return msg;
}
#endif /* HAL_USE_LIS3DH */

/** @} */
