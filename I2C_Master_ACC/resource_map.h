/******************************************************************************
* File Name:   resource_map.h
*
* Description: This file gives the I2C SCL and SDA pin map for all the supported
* kits.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#ifndef RESOURCE_MAP_H_
#define RESOURCE_MAP_H_

/* This code example requires two I2C ports, one as master and the other one
 * as slave. Set the I2C_MODE macro depending on the kit. Some kits support two
 * I2C ports which you can configure in I2C_MODE_BOTH whereas some support only
 * one port in which case you can configure the kit in either I2C_MODE_MASTER or
 * I2C_MODE_SLAVE. See README.md to know more on the kit specific configuration.
 */
#define I2C_MODE_BOTH    0
#define I2C_MODE_MASTER  1
#define I2C_MODE_SLAVE   2

#define I2C_MODE                    (I2C_MODE_MASTER)

#if (I2C_MODE == I2C_MODE_MASTER)
    #define mI2C_SCL                    (CYBSP_I2C_SCL)
    #define mI2C_SDA                    (CYBSP_I2C_SDA)
#endif

#if (I2C_MODE == I2C_MODE_SLAVE)
    #define sI2C_SCL                    (CYBSP_I2C_SCL)
    #define sI2C_SDA                    (CYBSP_I2C_SDA)
#endif

#if (I2C_MODE == I2C_MODE_BOTH)
    #define sI2C_SCL                    (CYBSP_I2C_SCL)
    #define sI2C_SDA                    (CYBSP_I2C_SDA)

    #if defined (TARGET_CY8CPROTO_062S3_4343W)
        #define mI2C_SCL                (P5_0)
        #define mI2C_SDA                (P5_1)
    #elif defined (TARGET_CYW9P62S1_43012EVB_01)
        #define mI2C_SCL                (P0_2)
        #define mI2C_SDA                (P0_3)
    #elif defined (TARGET_CY8CKIT_062S4)
        #define mI2C_SCL                (P10_0)
        #define mI2C_SDA                (P10_1)
    #elif defined (TARGET_CY8CKIT_062S2_43012)   || \
          defined (TARGET_CYW9P62S1_43438EVB_01) || \
          defined (TARGET_CY8CKIT_062_BLE)       || \
          defined (TARGET_CY8CKIT_062_WIFI_BT)   || \
          defined (TARGET_CY8CPROTO_063_BLE)     || \
          defined (TARGET_CY8CPROTO_062_4343W)   || \
          defined (TARGET_CY8CKIT_064B0S2_4343W) || \
          defined (TARGET_CYSBSYSKIT_01)         || \
          defined (TARGET_CYSBSYSKIT_DEV_01)     || \
          defined (TARGET_CY8CEVAL_062S2)        || \
          defined (TARGET_CY8CEVAL_062S2_LAI_4373M2)
        #define mI2C_SCL                (P9_0)
        #define mI2C_SDA                (P9_1)
    #else
        #error Unsupported kit. Define pins for I2C master or use in Master/Slave mode only.
    #endif
#endif

#endif /* RESOURCE_MAP_H_ */
