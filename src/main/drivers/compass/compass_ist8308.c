/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "platform.h"

#ifdef USE_MAG_IST8308

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/bus.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_busdev.h"
#include "drivers/sensor.h"
#include "drivers/time.h"

#include "compass.h"
#include "compass_ist8308.h"

#define IST8308_ADDRESS                                 0x0C

#define IST8308_REG_WHOAMI                              0x00
#define IST8308_CHIP_ID                                 0x08


#define IST8308_REG_DATA                                0x11

#define IST8308_REG_CNTRL2                              0x31    //Control setting register 2
//bit4~bit0  Operation mode
#define     IST8308_OPMODE_STANDBY_MODE                 0x00
#define     IST8308_OPMODE_SINGLE_MODE                  0x01
#define     IST8308_OPMODE_CONTINUOS_MODE_10HZ          0x02
#define     IST8308_OPMODE_CONTINUOS_MODE_20HZ          0x04
#define     IST8308_OPMODE_CONTINUOS_MODE_50HZ          0x06
#define     IST8308_OPMODE_CONTINUOS_MODE_100HZ         0x08
#define     IST8308_OPMODE_CONTINUOS_MODE_200HZ         0x0A
#define     IST8308_OPMODE_CONTINUOS_MODE_8HZ           0x0B
#define     IST8308_OPMODE_CONTINUOS_MODE_1HZ           0x0C
#define     IST8308_OPMODE_CONTINUOS_MODE_0_5HZ         0x0D
#define     IST8308_OPMODE_SELF_TEST_MODE               0x10

#define IST8308_REG_CNTRL1                              0x30    //Control setting register 1
//bit6~bit5
#define     IST8308_NSF_DISABLE                         0x00
#define     IST8308_NSF_LOW                             0x20
#define     IST8308_NSF_MIDDLE                          0x40
#define     IST8308_NSF_HIGH                            0x60


static bool ist8308Init(magDev_t * magDev)
{

    busDevice_t *busdev = &magDev->busdev;

    // Set continous mode
    uint8_t regTemp;
    bool ack = busReadRegisterBuffer(busdev, IST8308_REG_CNTRL2, &regTemp, 1);
    regTemp &= ~0x1F;
    regTemp |= (IST8308_OPMODE_CONTINUOS_MODE_50HZ & 0x1F);
    ack = ack && busWriteRegister(busdev, IST8308_REG_CNTRL2, regTemp);

    delay(30);

    return ack;
}

static bool ist8308Read(magDev_t *magDev, int16_t *magData)
{
    uint8_t buf[6];

    busDevice_t *busdev = &magDev->busdev;

    bool ack = busReadRegisterBuffer(busdev, IST8308_REG_DATA, buf, 6);
    if (!ack) {
        // set magData to zero for case of failed read
        magData[X] = 0;
        magData[Y] = 0;
        magData[Z] = 0;

        return false;
    }

    magData[X] = (int16_t)(buf[1] << 8 | buf[0]);
    magData[Y] = (int16_t)(buf[3] << 8 | buf[2]);
    magData[Z] = (int16_t)(buf[5] << 8 | buf[4]);

    return true;
}

bool ist8308Detect(magDev_t *magDev)
{

    busDevice_t *busdev = &magDev->busdev;

    if (busdev->bustype == BUSTYPE_I2C && busdev->busdev_u.i2c.address == 0) {
        busdev->busdev_u.i2c.address = IST8308_ADDRESS;
    }

    uint8_t sig = 0;
    bool ack = busReadRegisterBuffer(busdev, IST8308_REG_WHOAMI, &sig, 1);
    if (ack && sig == IST8308_CHIP_ID) {
        magDev->init = ist8308Init;
        magDev->read = ist8308Read;
        return true;
    }

    return false;
}

#endif
