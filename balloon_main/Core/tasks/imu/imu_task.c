/**
* Copyright (c) 2020 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file bno055_support.c
* @date 10/01/2020
* @version  2.0.6
*
*/

/*---------------------------------------------------------------------------*
*  Includes
*---------------------------------------------------------------------------*/

#include "imu_task.h"
/*----------------------------------------------------------------------------*
*  The following APIs are used for reading and writing of
*   sensor data using I2C communication
*----------------------------------------------------------------------------*/

s32 bno055_data_init()
{


    /* Variable used to return value of
     * communication routine*/
    s32 comres = BNO055_ERROR;

    /* variable used to set the power mode of the sensor*/
    u8 power_mode = BNO055_INIT_VALUE;


    /*---------------------------------------------------------------------------*
     *********************** START INITIALIZATION ************************
     *--------------------------------------------------------------------------*/
#ifdef  BNO055_API

    /*  Based on the user need configure I2C interface.
     *  It is example code to explain how to use the bno055 API*/
    I2C_routine();
#endif
    if(xQueueIMU == NULL){
        xQueueIMU = xQueueCreate(4, sizeof(struct bno055_result_t));
    }else{
        xQueueReset(xQueueIMU);
    }


    if(xQueueIMU == NULL){
        comres += 1;
    }
    /*--------------------------------------------------------------------------*
     *  This API used to assign the value/reference of
     *  the following parameters
     *  I2C address
     *  Bus Write
     *  Bus read
     *  Chip id
     *  Page id
     *  Accel revision id
     *  Mag revision id
     *  Gyro revision id
     *  Boot loader revision id
     *  Software revision id
     *-------------------------------------------------------------------------*/
    comres += bno055_init(p_bno055);

    /*  For initializing the BNO sensor it is required to the operation mode
     * of the sensor as NORMAL
     * Normal mode can set from the register
     * Page - page0
     * register - 0x3E
     * bit positions - 0 and 1*/
    power_mode = BNO055_POWER_MODE_NORMAL;

    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(power_mode);

    /*----------------------------------------------------------------*
     ************************* END INITIALIZATION *************************
     *-----------------------------------------------------------------*/

    /************************* START READ RAW SENSOR DATA****************/

    /*  Using BNO055 sensor we can read the following sensor data and
     * virtual sensor data
     * Sensor data:
     * Accel
     * Mag
     * Gyro
     * Virtual sensor data
     * Euler
     * Quaternion
     * Linear acceleration
     * Gravity sensor */

    /*  For reading sensor raw data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * SENSOR MODE
     * 0x01 - BNO055_OPERATION_MODE_ACCONLY
     * 0x02 - BNO055_OPERATION_MODE_MAGONLY
     * 0x03 - BNO055_OPERATION_MODE_GYRONLY
     * 0x04 - BNO055_OPERATION_MODE_ACCMAG
     * 0x05 - BNO055_OPERATION_MODE_ACCGYRO
     * 0x06 - BNO055_OPERATION_MODE_MAGGYRO
     * 0x07 - BNO055_OPERATION_MODE_AMG
     * based on the user need configure the operation mode*/
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
    return comres;
}


s32 bno055_get_data(struct bno055_result_t * bno055Result){

    s32 comres = BNO055_ERROR;


    /*  Raw accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x08 to 0x0D*/
    comres += bno055_read_accel_xyz(&bno055Result->accel);

    /*  Raw mag X, Y and Z data can read from the register
     * page - page 0
     * register - 0x0E to 0x13*/
    comres += bno055_read_mag_xyz(&bno055Result->mag);

    /*  Raw gyro X, Y and Z data can read from the register
     * page - page 0
     * register - 0x14 to 0x19*/
    comres += bno055_read_gyro_xyz(&bno055Result->gyro);

    /************************* END READ RAW SENSOR DATA****************/

    /************************* START READ RAW FUSION DATA ********
     * For reading fusion data it is required to set the
     * operation modes of the sensor
     * operation mode can set from the register
     * page - page0
     * register - 0x3D
     * bit - 0 to 3
     * for sensor data read following operation mode have to set
     * FUSION MODE
     * 0x08 - BNO055_OPERATION_MODE_IMUPLUS
     * 0x09 - BNO055_OPERATION_MODE_COMPASS
     * 0x0A - BNO055_OPERATION_MODE_M4G
     * 0x0B - BNO055_OPERATION_MODE_NDOF_FMC_OFF
     * 0x0C - BNO055_OPERATION_MODE_NDOF
     * based on the user need configure the operation mode*/
    /*  Raw Quaternion W, X, Y and Z data can read from the register
     * page - page 0
     * register - 0x20 to 0x27 */
    comres += bno055_read_quaternion_wxyz(&bno055Result->quaternion);

    /*  Raw Linear accel X, Y and Z data can read from the register
     * page - page 0
     * register - 0x28 to 0x2D */
    comres += bno055_read_linear_accel_xyz(&bno055Result->linearAccel);

    /*  Raw Gravity sensor X, Y and Z data can read from the register
     * page - page 0
     * register - 0x2E to 0x33 */
    comres += bno055_read_gravity_xyz(&bno055Result->gravity);

    bno055Result->timestamp = HAL_GetTick();

    /************************* END READ RAW FUSION DATA  ************/

    /*  API used to read accel data output as double  - m/s2 and mg

    ************************* END DE-INITIALIZATION **********************
    *---------------------------------------------------------------------*/
    return comres;
}

void vTaskGetIMU(void *pvParameters ){
    s32 comres = BNO055_ERROR;


    comres += bno055_data_init();
    while (1){
        struct bno055_result_t bno055Result;
        comres += bno055_get_data(&bno055Result);
        if(xQueueSend(xQueueIMU, &bno055Result, 100) != pdPASS){
            comres += 1;
        }
    }
}

#ifdef  BNO055_API

/*--------------------------------------------------------------------------*
 *  The following API is used to map the I2C bus read, write, delay and
 *  device address with global structure bno055_t
 *-------------------------------------------------------------------------*/

/*-------------------------------------------------------------------------*
 *  By using bno055 the following structure parameter can be accessed
 *  Bus write function pointer: BNO055_WR_FUNC_PTR
 *  Bus read function pointer: BNO055_RD_FUNC_PTR
 *  Delay function pointer: delay_msec
 *  I2C address: dev_addr
 *--------------------------------------------------------------------------*/
s8 I2C_routine(void)
{
    p_bno055->bus_write = BNO055_I2C_bus_write;
    p_bno055->bus_read = BNO055_I2C_bus_read;
    p_bno055->delay_msec = BNO055_delay_msek;
    p_bno055->dev_addr = BNO055_I2C_ADDR1;

    return BNO055_INIT_VALUE;
}

/************** I2C buffer length******/

#define I2C_BUFFER_LEN 8
#define I2C0           5

/*-------------------------------------------------------------------*
 *
 *  This is a sample code for read and write the data by using I2C
 *  Use either I2C  based on your need
 *  The device address defined in the bno055.h file
 *
 *--------------------------------------------------------------------*/

/*  \Brief: The API is used as I2C bus write
 *  \Return : Status of the I2C write
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *   will data is going to be written
 *  \param reg_data : It is a value hold in the array,
 *      will be used for write the value into the register
 *  \param cnt : The no of byte of data to be write
 */
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
    s32 BNO055_iERROR = BNO055_INIT_VALUE;

    BNO055_iERROR = HAL_I2C_Mem_Write(i2c_bno, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 1000);
    return (s8) BNO055_iERROR;
}

/*
 * Please take the below APIs as your reference for
 * write the data using I2C communication
 * "BNO055_iERROR = I2C_WRITE_STRING(DEV_ADDR, ARRAY, CNT+1)"
 * add your I2C write APIs here
 * BNO055_iERROR is an return value of I2C read API
 * Please select your valid return value
 * In the driver BNO055_SUCCESS defined as 0
 * and FAILURE defined as -1
 * Note :
 * This is a full duplex operation,
 * The first read data is discarded, for that extra write operation
 * have to be initiated. For that cnt+1 operation done
 * in the I2C write string function
 * For more information please refer data sheet SPI communication:
 */


/*  \Brief: The API is used as I2C bus read
 *  \Return : Status of the I2C read
 *  \param dev_addr : The device address of the sensor
 *  \param reg_addr : Address of the first register,
 *  will data is going to be read
 *  \param reg_data : This data read from the sensor,
 *   which is hold in an array
 *  \param cnt : The no of byte of data to be read
 */
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;

    BNO055_iERROR = HAL_I2C_Mem_Read(i2c_bno, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 1000);

    return (s8)BNO055_iERROR;
}

/*  Brief : The delay routine
 *  \param : delay in ms
 */
void BNO055_delay_msek(u32 msek)
{
    HAL_Delay(msek);
    /*Here you can write your own delay routine*/
}

#endif
