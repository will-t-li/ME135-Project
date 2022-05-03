/******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* I2C resource as Master using HAL APIs. The I2C master sends the
* command packets to the I2C slave to control an user LED.
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

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "resource_map.h"

/***************************************
*            Constants
****************************************/
#define CMD_TO_CMD_DELAY        (50UL)

/* I2C slave address to communicate with */

#define LSM6DSO_ADDR1				(0x6BUL)
#define LSM6DSO_ADDR2				(0x6AUL)

/* I2C bus frequency */
#define I2C_FREQ                (1000000UL)

/* I2C slave interrupt priority */
#define I2C_SLAVE_IRQ_PRIORITY  (7u)


/* Buffer and packet size */
#define PACKET_SIZE             (6UL)


/***************************************
*          Global Variables
****************************************/


#define MY_TCPWM_PWM_NUM1   (0UL)
#define MY_TCPWM_PWM_NUM2   (1UL)
#define MY_TCPWM_PWM_MASK  (1UL << MY_TCPWM_PWM_NUM1)


/*******************************************************************************
* Macros
*******************************************************************************/
/* PWM Frequency = 2Hz */
#define PWM_FREQUENCY (2u)
/* PWM Duty-cycle = 50% */
#define PWM_DUTY_CYCLE (50.0f)

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU.
*   1. I2C Master sends command packet to the slave
*   2. I2C Master reads the response packet to generate the next command
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    /* Set up the device based on configurator selections */
    result = cybsp_init();

    /* PWM object */
    // cyhal_pwm_t pwm_obj;


    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    result = cy_retarget_io_init( CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                                  CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("**************************\r\n");
    printf("PSoC 6 MCU I2C Master\r\n");
    printf("**************************\r\n\n");



#if ((I2C_MODE == I2C_MODE_BOTH) || (I2C_MODE == I2C_MODE_MASTER))
    cyhal_i2c_t mI2C;
    cyhal_i2c_cfg_t mI2C_cfg;
    uint8_t buffer1[PACKET_SIZE];
    uint8_t buffer2[PACKET_SIZE];
    uint16_t temp1;
    uint16_t z_acc1;
    uint16_t temp2;
	uint16_t z_acc2;
    double pwm_signal;

    /* Configure Pins */
    printf(">> Configuring Pins..... ");
    GPIO_PRT_Type* portAddr;

	/* Set the port address */
	portAddr = GPIO_PRT12;

	/* Set the drive mode to STRONG for pins P1[0], P1[2] and P1[3] (other pins in this port are HIGHZ) */
	CY_SET_REG32(&portAddr->CFG, CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE0_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE1_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE5_Pos );

	/* Set the pins P1[0], P1[2] and P1[3] to high and other pins in this port to low */
	CY_SET_REG32(&portAddr->OUT, GPIO_PRT_OUT_OUT0_Msk |
								 GPIO_PRT_OUT_OUT5_Msk);

    /* Configure I2C Master */
    printf(">> Configuring I2C Master..... ");
    mI2C_cfg.is_slave = false;
    mI2C_cfg.address = 0;
    mI2C_cfg.frequencyhal_hz = I2C_FREQ;
    result = cyhal_i2c_init( &mI2C, mI2C_SDA, mI2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    result = cyhal_i2c_configure( &mI2C, &mI2C_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        handle_error();
    }
    printf("Done\r\n\n");

#endif


    Cy_TCPWM_PWM_Init (TCPWM0, MY_TCPWM_PWM_NUM1, &tcpwm_0_cnt_0_config);

    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM1);
    /* Then start the PWM */
    Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_PWM_NUM1);

    Cy_TCPWM_PWM_Init (TCPWM0, MY_TCPWM_PWM_NUM2, &tcpwm_0_cnt_1_config);

	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM2);
	/* Then start the PWM */
	Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_PWM_NUM2);


    /* Enable interrupts */
    __enable_irq();
 

    for (;;)
    {
    	buffer1[0] = 0x11;
    	buffer1[1] = 0xA0;
    	buffer2[0] = 0x11;
		buffer2[1] = 0xA0;

    	cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR1,buffer1, 2, 0, true);
    	cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR2,buffer2, 2, 0, true);


        buffer1[0] = 0x10;
        buffer1[1] = 0xA0;
        buffer2[0] = 0x10;
		buffer2[1] = 0xA0;

        cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR1,buffer1, 2, 0, true);
        cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR2,buffer2, 2, 0, true);


        //buffer[0] = 0x2C; //Command for getting device serial number
        buffer1[0] = 0x2C;
        buffer1[1] = 0x2D;
        buffer2[0] = 0x2C;
		buffer2[1] = 0x2D;



        /* Send packet with command to the slave. */
        if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR1,
                                                  buffer1, 1, 0, true))
        {
            /* Read response packet from the slave. */
        	cyhal_system_delay_ms(100ul);
            if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR1,
                                                 buffer1, 2 , 0, true))
            {
                /* Check packet structure and status */

            	printf("First IMU");
            	printf("\n");
            	temp1 = buffer1[1] << 8;
            	z_acc1 = temp1 + buffer1[0];
            	printf("%d", z_acc1);
            	printf("\n");
            	pwm_signal = z_acc1 >> 10;
				printf("%f", pwm_signal);
				printf("\n");
            	z_acc1 = 0;
            }

            /* Give delay between commands. */
            cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
        }

        /* Send packet with command to the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR2,
												  buffer2, 1, 0, true))
		{
			/* Read response packet from the slave. */
			cyhal_system_delay_ms(100ul);
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR2,
												 buffer2, 2 , 0, true))
			{
				/* Check packet structure and status */

				printf("Second IMU");
				printf("\n");
				temp2 = buffer2[1] << 8;
				z_acc2 = temp2 + buffer2[0];
				printf("%d", z_acc2);
				printf("\n");
				pwm_signal = z_acc2 >> 10;
				printf("%f", pwm_signal);
				printf("\n");
				z_acc2 = 0;
			}

			/* Give delay between commands. */
			cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
		}


    }
}
