/*!
    \file    main.c
    \brief   master receiver

    \version 2017-02-10, V1.0.0, firmware for GD32F30x
    \version 2018-10-10, V1.1.0, firmware for GD32F30x
    \version 2018-12-25, V2.0.0, firmware for GD32F30x
    \version 2020-09-30, V2.1.0, firmware for GD32F30x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/
//#include <Wire.h>  // for I2C
#include "heimann_sensordef_32x32.h"
//#include "heimann_lookuptable.h"

#include "systick.h"

//#include <math.h>
#include <stdio.h>
//#include <stdlib.h>

#include "main.h"
#include "gd32f30x.h"
#include "gd32f30x_usart.h"
#include "gd32f307c_eval.h"




/*!
    \brief      memory compare function
    \param[in]  src : source data
    \param[in]  dst : destination data
    \param[in]  length : the compare data length
    \param[out] none
    \retval     ErrStatus : ERROR or SUCCESS
*/
ErrStatus memory_compare(uint8_t* src, uint8_t* dst, uint16_t length) 
{
    while(length--){
        if(*src++ != *dst++){
            return ERROR;
        }
    }
    return SUCCESS;
}

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* enable GPIOB clock */
    rcu_periph_clock_enable(RCU_GPIOB);
    /* enable I2C1 clock */
    rcu_periph_clock_enable(RCU_I2C1);
    /* enable I2C0 clock */
    rcu_periph_clock_enable(RCU_I2C0);
}

/*!
    \brief      cofigure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    /* I2C0 and I2C1 GPIO ports */
    /* connect PB6 to I2C0_SCL */
    /* connect PB7 to I2C0_SDA */
    /* connect PB10 to I2C1_SCL */
    /* connect PB11 to I2C1_SDA */
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11);
}

/*!
    \brief      cofigure the I2C0 and I2C1 interfaces
    \param[in]  none
    \param[out] none
    \retval     none
*/
void i2c_config(void)//
{
    /* I2C clock configure */
    i2c_clock_config(I2C0, CLOCK_EEPROM, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SENSOR_ADDRESS);
    /* enable I2C0 */
    i2c_enable(I2C0);
    /* enable acknowledge */
    i2c_ack_config(I2C0, I2C_ACK_ENABLE);
    i2c_clock_config(I2C1, CLOCK_EEPROM, I2C_DTCY_2);
    /* I2C address configure */
    i2c_mode_addr_config(I2C1, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SENSOR_ADDRESS);
    /* enable I2C1 */
    i2c_enable(I2C1);
    /* enable acknowledge */
    i2c_ack_config(I2C1, I2C_ACK_ENABLE);
}






/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

int main(void)
{
    /* configure systick */
    systick_config();
    /* RCU config */
    rcu_config();
    /* GPIO config */
    gpio_config();
    /* I2C config */
    i2c_config();
    /* initilize the LEDs, USART and key */

    gd_eval_com_init(EVAL_COM0);

	/* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

   //susente 20210729
   /* USART interrupt configuration */
    nvic_irq_enable(USART0_IRQn, 0, 0);
   
   
   printf("\r susente 20210726 begin ###\n");

    setup();
    delay_1ms(1000);
    while (1){
	     loop();

		 #if 0
		i2CsetClock(EEPROM_CONFIG);
		 pu_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_CALIB);
		 pu_user = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_USER);
		printf("\r pu_calib pu_user [%x]: [%x]\n", pu_calib, pu_user);
		/*for(int i=0; i<16; i++)
		{
			byte EEPROM_test = read_EEPROM_byte(EEPROM_ADDRESS, i);
			printf("\r EEPROM_test %x: [%x]\n", i, EEPROM_test);

		}*/
		#endif

		#if 0 //SENSOR TEST
		i2CsetClock(SENSOR_CONFIG);
		delay_1ms(1000); 
		write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);
		write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, 0x22); //resist pull up
		write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);
		
		delay_1ms(27); //MUST HAVE DELAY BCS OF SENSOR
		//delay_1ms(timer_duration);
		//printf("\r timer_duration [%d]\n", timer_duration);
		//delay_1ms(50);
		read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
		printf("\r SENSOR_test [%d]\n", statusreg);
		
		uint8_t data_test[258];

		read_sensor_register( TOP_HALF, (uint8_t*)&data_test, 258);
		for(int i=0; i<258; i++)
		{
			printf("\r SENSOR_test %d: [%x]\n", i, data_test[i]);
		}
		#endif
		
		

		 //delay_1ms(20000);

		//loop('a');

		/*if(RESET == gd_eval_key_state_get(KEY_WAKEUP)){
            gd_eval_led_on(LED3);
            delay_1ms(500);
            gd_eval_led_off(LED3);
            gd_eval_led_toggle(LED4);
        }*/
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TBE));

    return ch;
}




