#include "debug.h"

#include <string.h>

#include "drivers/bus.h"
#include "drivers/time.h"
#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/compass/compass.h"
#include "drivers/compass/compass_mpu9250.h"
#include "drivers/compass/compass_ak8963.h"

static char debugData[10000];

void addDebugData(char* str)
{
    //snprintf(debugData, sizeof(debugData), "%s%s", debugData, str);
    strncat(debugData, str, sizeof(debugData) - strlen(debugData) - 1);
}

const char* getDebugData(void)
{
    return debugData;
}

#define AK8963_MAG_I2C_ADDRESS          0x0C

static bool debugMagMpu9250SlaveI2CRead(busDevice_t * busDev, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
    // Setting up MPU9250's I2C master to read from AK8963 via internal I2C bus
    busWrite(busDev, MPU_RA_I2C_SLV0_ADDR, addr | 0x80);     // set I2C slave address for read
    busWrite(busDev, MPU_RA_I2C_SLV0_REG, reg);                   // set I2C slave register
    busWrite(busDev, MPU_RA_I2C_SLV0_CTRL, len | 0x80);           // read number of bytes
    delay(10);                                                      // wait for transaction to complete
    busReadBuf(busDev, MPU_RA_EXT_SENS_DATA_00, buf, len);        // read I2C
    return true;
}

uint8_t debugMag(uint8_t reg)
{
    busDevice_t * dev = busDeviceOpen(BUSTYPE_SPI, DEVHW_MPU9250, 0);

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    delay(10);

    uint8_t byte;
    debugMagMpu9250SlaveI2CRead(dev, AK8963_MAG_I2C_ADDRESS, reg, &byte, 1);

    busSetSpeed(dev, BUS_SPEED_FAST);

    return byte;
}

uint8_t debugMag2(uint8_t reg)
{
    busDevice_t * dev = busDeviceOpen(BUSTYPE_SPI, DEVHW_MPU9250, 0);

    busSetSpeed(dev, BUS_SPEED_INITIALIZATION);
    delay(10);

    uint8_t byte;
    busReadBuf(dev, reg, &byte, 1);
    
    busSetSpeed(dev, BUS_SPEED_FAST);

    return byte;
}