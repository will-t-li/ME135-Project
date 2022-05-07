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
#include "math.h"
/***************************************
*            Constants
****************************************/
#define CMD_TO_CMD_DELAY        (30UL)

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


#define MY_TCPWM_PWM_NUM1   (0UL) // left wheel drive
#define MY_TCPWM_PWM_NUM2   (1UL) // right wheel drive
#define MY_TCPWM_PWM_NUM3   (2UL) // left linear actuators
#define MY_TCPWM_PWM_NUM4   (3UL) // right linear actuators
#define MY_TCPWM_PWM_MASK  (1UL << MY_TCPWM_PWM_NUM1)

#define Kp  (1UL)
#define Kd  (0UL)
#define Ki  (0UL)
#define sampleTime  (0.005F)
#define targetAngle (0F)


/*******************************************************************************
* Macros
*******************************************************************************/

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
    //printf("\x1b[2J\x1b[;H");

    //printf("**************************\r\n");
    //printf("PSoC 6 MCU I2C Master\r\n");
    //printf("**************************\r\n\n");



#if ((I2C_MODE == I2C_MODE_BOTH) || (I2C_MODE == I2C_MODE_MASTER))
    cyhal_i2c_t mI2C;
    cyhal_i2c_cfg_t mI2C_cfg;
    uint8_t buffer1[PACKET_SIZE];
    uint8_t buffer2[PACKET_SIZE];
    uint8_t buffer3[PACKET_SIZE];
	uint8_t buffer4[PACKET_SIZE];
	uint8_t buffer5[PACKET_SIZE];
	uint8_t buffer6[PACKET_SIZE];
    int16_t tempz1;
    int16_t z_acc1;
    int16_t tempy1;
	int16_t y_acc1;
	int16_t tempx1;
	int16_t x_acc1;
    int16_t tempz2;
	int16_t z_acc2;
	int16_t tempy2;
	int16_t y_acc2;
	int16_t tempx2;
	int16_t x_acc2;
	int32_t compare_rear;
	int32_t compare_front;
	float current_angle;
	uint32_t target_angle = 0;
	int32_t error = 0;
	int32_t error_sum = 0;
    int16_t accY;
    int16_t accZ;
    bool neg_angle;
    int32_t acc_ratio = 0;
    int32_t prev_acc_ratio = 0;
    bool not_flat = 0;
    int8_t front_updown = 0;
    int8_t rear_updown = 0;


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
    //printf("Done\r\n\n");

#endif

        /* Configure Pins */
    //printf(">> Configuring Pins..... ");
    GPIO_PRT_Type* standbyaddr;
    GPIO_PRT_Type* frontacts_in1;
    GPIO_PRT_Type* frontacts_in2;
    GPIO_PRT_Type* rearacts_in1;
    GPIO_PRT_Type* rearacts_in2;

	/* Set the port address */
	standbyaddr = GPIO_PRT0;
	frontacts_in1 = GPIO_PRT12;
	frontacts_in2 = GPIO_PRT9;
	rearacts_in1 = GPIO_PRT10;
	rearacts_in2 = GPIO_PRT13;


	/* Set the drive mode to STRONG for pins 1 */
	CY_SET_REG32(&standbyaddr->CFG, CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE5_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE2_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE3_Pos );
	/* Set the pins 1 to high and other pins in this port to low */
	CY_SET_REG32(&standbyaddr->OUT, GPIO_PRT_OUT_OUT5_Msk |
								GPIO_PRT_OUT_OUT2_Msk |
								GPIO_PRT_OUT_OUT3_Msk |
								GPIO_PRT_OUT_OUT4_Msk);





	/* Setting up Pins for Linear Actuator Direction */

	/* Set the drive mode to STRONG for pins 12 */
	// These two are A_in_1 for front drivers
	CY_SET_REG32(&frontacts_in1->CFG, CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE0_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE1_Pos );

	/* Set the drive mode to STRONG for pins 9 */
	// These two are A_in_2 for front drivers
	CY_SET_REG32(&frontacts_in2->CFG, CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE0_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE5_Pos );

	/* Set the drive mode to STRONG for pins 10 */
	// These two are A_in_1 for rear drivers
	CY_SET_REG32(&rearacts_in1->CFG, CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE1_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE2_Pos );

	/* Set the drive mode to STRONG for pins 13 */
	// These two are A_in_2 for rear drivers
	CY_SET_REG32(&rearacts_in2->CFG, CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE4_Pos |
								 CY_GPIO_DM_STRONG_IN_OFF << GPIO_PRT_CFG_DRIVE_MODE6_Pos );

///////////////////////////////////////////////////////////////


	/*  Initializing Pins for Linear Actuator Direction */


	/* Set the pins 12 to high and other pins in this port to low */
	// default is driving it down
	CY_SET_REG32(&frontacts_in1->OUT, GPIO_PRT_OUT_OUT0_Msk |
									GPIO_PRT_OUT_OUT1_Msk);
	/* Set the pins 9 to low (other pins in this port are unchanged) */
	CY_SET_REG32(&frontacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
									 GPIO_PRT_OUT_CLR_OUT5_Msk);
	/* Set the pins 10 to high and other pins in this port to low */
	// default is driving it down
	CY_SET_REG32(&rearacts_in1->OUT, GPIO_PRT_OUT_OUT0_Msk |
									GPIO_PRT_OUT_OUT1_Msk);
	/* Set the pins 13 to low (other pins in this port are unchanged) */
	CY_SET_REG32(&rearacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT4_Msk |
									 GPIO_PRT_OUT_CLR_OUT6_Msk);


////////////////////////////////////////////////////////////////////




        /* Configure PWMs */
	// Pin 8.0
    Cy_TCPWM_PWM_Init (TCPWM0, MY_TCPWM_PWM_NUM1, &tcpwm_0_cnt_0_config);
    /* Enable the initialized PWM */
    Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM1);
    /* Then start the PWM */
    Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_PWM_NUM1);

    // Pin 6.2
    Cy_TCPWM_PWM_Init (TCPWM0, MY_TCPWM_PWM_NUM2, &tcpwm_0_cnt_1_config);
	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM2);
	/* Then start the PWM */
	Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_PWM_NUM2);

	// Pin 11.2
    Cy_TCPWM_PWM_Init (TCPWM0, MY_TCPWM_PWM_NUM3, &tcpwm_0_cnt_2_config);
	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM3);
	/* Then start the PWM */
	Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_PWM_NUM3);

	// Pin 11.4
    Cy_TCPWM_PWM_Init (TCPWM0, MY_TCPWM_PWM_NUM4, &tcpwm_0_cnt_3_config);
	/* Enable the initialized PWM */
	Cy_TCPWM_PWM_Enable(TCPWM0, MY_TCPWM_PWM_NUM4);
	/* Then start the PWM */
	Cy_TCPWM_TriggerStart_Single(TCPWM0, MY_TCPWM_PWM_NUM4);


    /* Enable interrupts */
    __enable_irq();
 

    for (;;)
    {
    	//Setting up I2C
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

        // z-axis
        buffer1[0] = 0x2C;
        buffer1[1] = 0x2D;
        buffer2[0] = 0x2C;
		buffer2[1] = 0x2D;
		// x-axis
		buffer3[0] = 0x28;
		buffer3[1] = 0x29;
		buffer4[0] = 0x28;
		buffer4[1] = 0x29;
		// y-axis
		buffer5[0] = 0x2A;
		buffer5[1] = 0x2B;
		buffer6[0] = 0x2A;
		buffer6[1] = 0x2B;


        /* Send packet with command to the slave. */
        if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR1,
                                                  buffer1, 1, 0, true))
        {
            /* Read response packet from the slave. */
        	cyhal_system_delay_us(100ul);
            if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR1,
                                                 buffer1, 2 , 0, true))
            {
                /* Check packet structure and status */
            	//printf("First IMU");
            	//printf("\n");
            	tempz1 = buffer1[1] << 8;
            	z_acc1 = tempz1 + buffer1[0];
            }
            /* Give delay between commands. */
            cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
        }
        /* Send packet with command to the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR2,
												  buffer2, 1, 0, true))
		{
			/* Read response packet from the slave. */
			cyhal_system_delay_us(100ul);
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR2,
												 buffer2, 2 , 0, true))
			{
				/* Check packet structure and status */
				//printf("Second IMU");
				//printf("\n");
				tempz2 = buffer2[1] << 8;
				z_acc2 = tempz2 + buffer2[0];
			}
			/* Give delay between commands. */
			cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
		}
		/* Send packet with command to the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR1,
												  buffer3, 1, 0, true))
		{
			/* Read response packet from the slave. */
			cyhal_system_delay_us(100ul);
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR1,
												 buffer3, 2 , 0, true))
			{
				/* Check packet structure and status */
				//printf("First IMU");
				//printf("\n");
				tempx1 = buffer3[0] << 8;
				x_acc1 = tempx1 + buffer3[1];
			}
			/* Give delay between commands. */
			cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
		}
		/* Send packet with command to the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR2,
												  buffer4, 1, 0, true))
		{
			/* Read response packet from the slave. */
			cyhal_system_delay_us(100ul);
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR2,
												 buffer4, 2 , 0, true))
			{
				/* Check packet structure and status */
				//printf("Second IMU");
				//printf("\n");
				tempx2 = buffer4[1] << 8;
				x_acc2 = tempx2 + buffer4[0];
			}
			/* Give delay between commands. */
			cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
		}
		/* Send packet with command to the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR1,
												  buffer5, 1, 0, true))
		{
			/* Read response packet from the slave. */
			cyhal_system_delay_us(100ul);
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR1,
												 buffer5, 2 , 0, true))
			{
				/* Check packet structure and status */
				//printf("First IMU");
				//printf("\n");
				tempy1 = buffer5[1] << 8;
				y_acc1 = tempy1 + buffer5[0];
			}
			/* Give delay between commands. */
			cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
		}
		/* Send packet with command to the slave. */
		if (CY_RSLT_SUCCESS == cyhal_i2c_master_write( &mI2C, LSM6DSO_ADDR2,
												  buffer6, 1, 0, true))
		{
			/* Read response packet from the slave. */
			cyhal_system_delay_us(100ul);
			if (CY_RSLT_SUCCESS == cyhal_i2c_master_read( &mI2C, LSM6DSO_ADDR2,
												 buffer6, 2 , 0, true))
			{
				/* Check packet structure and status */
				//printf("Second IMU");
				//printf("\n");
				tempy2 = buffer6[1] << 8;
				y_acc2 = tempy2 + buffer6[0];
			}
			/* Give delay between commands. */
			cyhal_system_delay_ms(CMD_TO_CMD_DELAY);
		}
		if (x_acc1 > 16393) {
			x_acc1 -= 65535;
		}
		if (y_acc1 > 16393) {
			y_acc1 -= 65535;
		}
		if (z_acc1 > 16393) {
			z_acc1 -= 65535;
		}
		if (x_acc2 > 16393) {
			x_acc2 -= 65535;
		}
		if (x_acc2 > 16393) {
			x_acc2 -= 65535;
		}
		if (x_acc2 > 16393) {
			x_acc2 -= 65535;
		}
		//printf("b%d, %d, %d, %d, %d, %de\r\n", x_acc1, y_acc1, z_acc1, x_acc2, y_acc2, z_acc2);
		//printf("\n");
		//printf("%lu", compare);
		//printf("\n");

		accY = (y_acc1 + y_acc2)/2;
		accZ = (z_acc1 + z_acc2)/2;
		acc_ratio = accY*1000/accZ;


		/////////////////////////////////////// Calculating angle for LabVIEW
		if (accY < 0) {
			neg_angle = 1;
			accY *= -1;
		}
		else {
			neg_angle = 0;
		}

	  // calculate the angle of inclination
	  current_angle = atan2(accY, accZ)*(180/3.1416);
	  if (neg_angle) {
		  current_angle *= -1;
		  accY *= -1;
	  }
		current_angle = floor(current_angle);
		current_angle = (current_angle < -20) ? -20 : ((current_angle > 20) ? 20 : current_angle);
		////////////////////////////////////////////////////


		//fetch current compare values
		compare_front = Cy_TCPWM_Counter_GetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM1); // 8.0
		compare_rear = Cy_TCPWM_Counter_GetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM2); // 6.2

		if ((prev_acc_ratio < acc_ratio - 20) || (prev_acc_ratio > acc_ratio + 20)) {
			not_flat = 1;
		}
		else if ((prev_acc_ratio > acc_ratio - 10) || (prev_acc_ratio < acc_ratio + 10)) {
			not_flat = 1;
		}
		else {
			not_flat = 0;
		}




		if (acc_ratio > 50) {
		//if (accZ < 16200 && accY > 0) {
			compare_front = 500;
			compare_rear = 500;
			front_updown = -1;
			rear_updown = 1;
			//FRONT DIRECTION DOWN
			/* Set the pins 9 to high and other pins in this port to low */
			CY_SET_REG32(&frontacts_in2->OUT, GPIO_PRT_OUT_OUT0_Msk |
											GPIO_PRT_OUT_OUT5_Msk);
			/* Set the pins 12 to low (other pins in this port are unchanged) */
			CY_SET_REG32(&frontacts_in1->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
											 GPIO_PRT_OUT_CLR_OUT1_Msk);
			//REAR DIRECTION UP
			/* Set the pins 10 to high and other pins in this port to low */
			CY_SET_REG32(&rearacts_in1->OUT, GPIO_PRT_OUT_OUT1_Msk |
											GPIO_PRT_OUT_OUT2_Msk);
			/* Set the pins 13 to low (other pins in this port are unchanged) */
			CY_SET_REG32(&rearacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT4_Msk |
											 GPIO_PRT_OUT_CLR_OUT6_Msk);
			printf("Positive Angle");
			printf("\r\n");
		}
		else if (acc_ratio < -50) {
		//else if (accZ < 16200 && accY < 0) {
			compare_front = 500;
			compare_rear = 500;
			front_updown = 1;
			rear_updown = -1;
			//FRONT DIRECTION UP
			/* Set the pins 12 to high and other pins in this port to low */
			CY_SET_REG32(&frontacts_in1->OUT, GPIO_PRT_OUT_OUT0_Msk |
											GPIO_PRT_OUT_OUT1_Msk);
			/* Set the pins 9 to low (other pins in this port are unchanged) */
			CY_SET_REG32(&frontacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
											 GPIO_PRT_OUT_CLR_OUT5_Msk);
			//REAR DIRECTION DOWN
			/* Set the pins 13 to high and other pins in this port to low */
			CY_SET_REG32(&rearacts_in2->OUT, GPIO_PRT_OUT_OUT4_Msk |
											GPIO_PRT_OUT_OUT6_Msk);
			/* Set the pins 10 to low (other pins in this port are unchanged) */
			CY_SET_REG32(&rearacts_in1->OUT_CLR, GPIO_PRT_OUT_CLR_OUT1_Msk |
											 GPIO_PRT_OUT_CLR_OUT2_Msk);
			printf("Negative Angle");
			printf("\r\n");
		}
		else {
			compare_front = 300;
			compare_rear = 300;
			if (not_flat) {
				front_updown = 0;
				rear_updown = 0;
				////////////////// Turn the brakes on
				/* Set the pins 12 to low (other pins in this port are unchanged) */
				CY_SET_REG32(&frontacts_in1->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
												 GPIO_PRT_OUT_CLR_OUT1_Msk);
				/* Set the pins 9 to low (other pins in this port are unchanged) */
				CY_SET_REG32(&frontacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
												 GPIO_PRT_OUT_CLR_OUT5_Msk);
				/* Set the pins 10 to low (other pins in this port are unchanged) */
				CY_SET_REG32(&rearacts_in1->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
												 GPIO_PRT_OUT_CLR_OUT1_Msk);
				/* Set the pins 13 to low (other pins in this port are unchanged) */
				CY_SET_REG32(&rearacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT4_Msk |
												 GPIO_PRT_OUT_CLR_OUT6_Msk);
				printf("Staying Still");
				printf("\r\n");
			}
			else {
				front_updown = 1;
				rear_updown = 1;
				/* Set the pins 12 to high and other pins in this port to low */
				CY_SET_REG32(&frontacts_in1->OUT, GPIO_PRT_OUT_OUT0_Msk |
												GPIO_PRT_OUT_OUT1_Msk);
				/* Set the pins 9 to low (other pins in this port are unchanged) */
				CY_SET_REG32(&frontacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT0_Msk |
												 GPIO_PRT_OUT_CLR_OUT5_Msk);
				/* Set the pins 10 to high and other pins in this port to low */
				CY_SET_REG32(&rearacts_in1->OUT, GPIO_PRT_OUT_OUT0_Msk |
												GPIO_PRT_OUT_OUT1_Msk);
				/* Set the pins 13 to low (other pins in this port are unchanged) */
				CY_SET_REG32(&rearacts_in2->OUT_CLR, GPIO_PRT_OUT_CLR_OUT4_Msk |
												 GPIO_PRT_OUT_CLR_OUT6_Msk);
				printf("Flat? Extending");
				printf("\r\n");
			}
		}

//		error = acc_ratio - target_angle;
//		error_sum = error_sum + error;
//		error_sum = (error_sum < -1000) ? -1000 : ((error_sum > 1000) ? 1000 : error_sum);
//		compare = Kp*(error) + Ki*(error_sum)*sampleTime - Kd*(current_angle-prev_angle)/sampleTime;
//		  if (neg_angle) {
//			  compare *= -1;
//		  }
		//compare = 500;

		Cy_TCPWM_Counter_SetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM1, compare_front); // front actuators
		Cy_TCPWM_Counter_SetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM2, 500); // front motors
		Cy_TCPWM_Counter_SetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM3, compare_rear + 100); // rear actuators
		Cy_TCPWM_Counter_SetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM4, 500); // rear motors
		compare_front = Cy_TCPWM_Counter_GetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM1); // 8.0
		compare_rear = Cy_TCPWM_Counter_GetCompare0Val(TCPWM0, MY_TCPWM_PWM_NUM3); // 11.2
		//prev_angle = current_angle;
		//printf("%d", not_flat);
		//printf("\r\n");
		// printf("%f", pwm_signal1);
		// printf("\n");
		//printf("%d, %d", prev_acc_ratio, acc_ratio);
		//printf("%d, %d", front_length, rear_length);
		//printf("\r\n");
		prev_acc_ratio = acc_ratio;
		printf("b%d, %d, %d, %d, %d, %d, %f, %ld, %ld, %d, %de\r\n", x_acc1, y_acc1, z_acc1, x_acc2, y_acc2, z_acc2, current_angle, compare_front, compare_rear, front_updown, rear_updown);
		z_acc1 = 0;
		z_acc2 = 0;
		y_acc1 = 0;
		y_acc2 = 0;
		x_acc1 = 0;
		x_acc2 = 0;
		/*
		 * angle = 0:
		 * 		if rear length and front length are both high, gpio = down and pwm = 100, reset lengths
		 * 		else pwm = 0, gpio = down
 * 		angle > 0:
 * 			pwm on for rear, gpio = up
 * 			add to rear length var
* 		angle < 0:
* 			pwm on for front, gpio = up
* 			add to front length var
		 */
    }
}
