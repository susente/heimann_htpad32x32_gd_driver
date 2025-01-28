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
#include "heimann_lookuptable.h"

#include "systick.h"

#include <math.h>
#include <stdio.h>
//#include <stdlib.h>

#include "main.h"
#include "gd32f30x.h"
#include "gd32f30x_usart.h"
#include "gd32f307c_eval.h"




#define  bitRead(value, bit)   (((value) >> (bit)) & 0x01) 

#define I2C_10BIT_ADDRESS      0 //susente

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
//#define TRANSMIT_SIZE            (ARRAYNUM(txbuffer) - 1)
#define TRANSMIT_SIZE  (128) //32x4
#define RECEIVE_SIZE (1)

uint8_t txbuffer[32][4];;//temp_pix_uint32[32][32];
uint8_t rxbuffer[32];
uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = RECEIVE_SIZE;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0; 

char input = '\0';




typedef uint16_t word;
typedef uint8_t byte;


void read_pixel_data(void);
void pixel_masking(void);
void calculate_pixel_temp(void) ;
void sort_data(void);
void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n);
void write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input);
void print_pixel_temps(void);
void print_eeprom_value(void) ;
void print_eeprom_hex(void);
void print_calc_steps(void);

//added by susente
word calc_timer_duration(float bw, uint8_t clk, uint8_t mbit) ;
void calculate_pixcij();
void write_calibration_settings_to_sensor();
void read_eeprom() ;







// SENSOR CHARACTERISTICS
typedef struct {
  uint8_t number_row;    // number of raws
  uint8_t number_col;    // number of column
  uint8_t number_blocks; // number of blocks (top + down)
  uint8_t number_pixel;  // number of pixel
}characteristics;


characteristics sensor = {32, 32, 8, 1024};

// EEPROM DATA
uint8_t mbit_calib, bias_calib, clk_calib, bpa_calib, pu_calib, nrofdefpix, gradscale, vddscgrad, vddscoff, epsilon, arraytype;
int8_t globaloff;
uint8_t mbit_user, bias_user, clk_user, bpa_user, pu_user;
uint16_t tablenumber, vddth1, vddth2, ptatth1, ptatth2, ptatgr, globalgain;
int16_t thgrad[32][32];
int16_t thoffset[32][32];
int16_t vddcompgrad[8][32];
int16_t vddcompoff[8][32];
uint16_t pij[32][32];
uint16_t deadpixadr[24];
uint8_t deadpixmask[12] ;
uint8_t deadpixmask_test[8] ;

int32_t pixcij_int32[32][32];
uint32_t id, ptatoff;
float ptatgr_float, ptatoff_float, pixcmin, pixcmax, bw;

// SENSOR DATA
uint8_t data_top_block0[258], data_top_block1[258], data_top_block2[258], data_top_block3[258];
uint8_t data_bottom_block0[258], data_bottom_block1[258], data_bottom_block2[258], data_bottom_block3[258];
uint8_t electrical_offset_top[258], electrical_offset_bottom[258];
uint16_t eloffset[8][32];
uint16_t ptat_top_block0, ptat_top_block1, ptat_top_block2, ptat_top_block3;
uint16_t ptat_bottom_block0, ptat_bottom_block1, ptat_bottom_block2, ptat_bottom_block3;
uint16_t vdd_top_block0, vdd_top_block1, vdd_top_block2, vdd_top_block3;
uint16_t vdd_bottom_block0, vdd_bottom_block1, vdd_bottom_block2, vdd_bottom_block3;
uint16_t data_pixel[32][32];
uint8_t statusreg;

// CALCULATED VALUES
uint16_t ptat_av_uint16;
uint16_t vdd_av_uint16;
uint16_t ambient_temperature;
int32_t vij_pixc_int32[32][32];
uint32_t temp_pix_uint32[32][32];
int32_t vij_comp_int32[32][32];
int32_t vij_comp_s_int32[32][32];
int32_t vij_vddcomp_int32[32][32];

// OTHER
uint32_t gradscale_div;
uint32_t vddscgrad_div;
uint32_t vddscoff_div;
int vddcompgrad_n;
int vddcompoff_n;
char var = 'm';
uint16_t timer_duration;



/********************************************************************
   Function:        void setup()

   Description:     setup before main loop

   Dependencies:
 *******************************************************************/
#if 0
void setup() {

  // begin serial communication
  Serial.begin(115200);
  while (!Serial) {
    ;
  }


  printf("\nSETUP\n\n");
  // begin I2C communication (400kHz - eeprom / 1000kHz - sensor)
  printf("search device");
  uint8_t error = 1;
  while (error != 0) {
    Wire.end();
    delay(2000);
    Wire.begin();
    Wire.beginTransmission(SENSOR_ADDRESS);
    error = Wire.endTransmission();
    printf(".");
  }

  Wire.setClock(CLOCK_EEPROM); // I2C clock frequency 400kHz (for eeprom communication)


  printf("\nread eeprom");
  read_eeprom();


  Wire.setClock(CLOCK_SENSOR);
  // HINT: To increase the frame rate, here the I2C clock is higher than 1MHz from datasheet. If this causes any problems, set to the datasheet value.

  printf("\nwake up sensor");

  // to wake up sensor set configuration register to 0x01
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);


  printf("\ninitialization");
  write_calibration_settings_to_sensor();

  printf("\nstart sensor");
  // to start sensor set configuration register to 0x09
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);

  // other calculations before main loop
  gradscale_div = pow(2, gradscale);
  vddscgrad_div = pow(2, vddscgrad);
  vddscoff_div = pow(2, vddscoff);
  calculate_pixcij();

  timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib);

  // ERROR TABLENUMBER
  if (tablenumber != TABLENUMBER) {
    printf("\n\nHINT:\tConnected sensor does not match the selected look up table.");
    printf("\n\tThe calculated temperatures could be wrong!");
    printf("\n\tChange device in sensordef_32x32.h to sensor with tablenumber ");
    printf(tablenumber);
  }
  // ERROR BUFFER LENGTH
  if (BUFFER_LENGTH < 258) {
    printf("\n\nHINT:\tBUFFER_LENGTH in Wire.h library is not 258 or higher.");
    printf("\n\tThe calculated temperatures could be wrong!");
    printf("\n\tChange BUFFER_LENGTH in wire.h to 258 or higher");
  }



}

#endif
void i2CsetClock(uint8_t type)
{
		if(type == SENSOR_CONFIG)
		{
			i2c_clock_config(I2C0, CLOCK_SENSOR, I2C_DTCY_2);
			i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, SENSOR_ADDRESS);
		}
		else
		{
			i2c_clock_config(I2C0, CLOCK_EEPROM, I2C_DTCY_2);
			i2c_mode_addr_config(I2C0, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, EEPROM_ADDRESS);
		}
}




void setup() {
  printf("\r SETUP init \n");

  i2CsetClock(EEPROM_CONFIG);
  read_eeprom();



  i2CsetClock(SENSOR_CONFIG);
  // HINT: To increase the frame rate, here the I2C clock is higher than 1MHz from datasheet. If this causes any problems, set to the datasheet value.


  printf("\r wake up sensor\n");

  // to wake up sensor set configuration register to 0x01
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   0   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x01);


  printf("\r initialization\n");
  write_calibration_settings_to_sensor();


  printf("\r start sensor\n");
  // to start sensor set configuration register to 0x09
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09);

  // other calculations before main loop
  gradscale_div = pow(2, gradscale);
  vddscgrad_div = pow(2, vddscgrad);
  vddscoff_div = pow(2, vddscoff);
  calculate_pixcij();


  timer_duration = calc_timer_duration(bw, clk_calib, mbit_calib)/1000;

  // ERROR TABLENUMBER
  if (tablenumber != TABLENUMBER) {
    printf("\n\nHINT:\tConnected sensor does not match the selected look up table.");
    printf("\n\tThe calculated temperatures could be wrong!");
    printf("\n\tChange device in sensordef_32x32.h to sensor with tablenumber ");
    printf("\rtablenumber[%d]\n",tablenumber);
  }

  // ERROR BUFFER LENGTH
  #if 0
  if (BUFFER_LENGTH < 258) {
    printf("\n\nHINT:\tBUFFER_LENGTH in Wire.h library is not 258 or higher.");
    printf("\n\tThe calculated temperatures could be wrong!");
    printf("\n\tChange BUFFER_LENGTH in wire.h to 258 or higher");
  }
 #endif
}

void uart_get_cmd(void)
{ 
         while(RESET == usart_flag_get(USART0, USART_FLAG_TC));
         usart_interrupt_enable(USART0, USART_INT_RBNE);
		/* wait until USART receive the receiver_buffer */
		while(rxcount < rx_size);
	  rxcount = 0;
		//if(rxcount == rx_size)
		//	printf("\n\rUSART receive successfully!\n\r");
}


void send_one_frame_data_by_uart(void)
{
	int index = 48;
	  for (int m = 0; m < sensor.number_row; m++) 
	  {
    		for (int n = 0; n < sensor.number_col; n++) 
	     {
             txbuffer[n] [0]= (uint8_t)(((temp_pix_uint32[m][n]) &0xff));
		  txbuffer[n] [1]= (uint8_t)(((temp_pix_uint32[m][n])>>8 &0xff));
		  txbuffer[n] [2]= (uint8_t)(((temp_pix_uint32[m][n])>>16 &0xff));
		  txbuffer[n] [3]= (uint8_t)(((temp_pix_uint32[m][n])>>24 &0xff));
		  //printf("\r send_one_frame [%2x][%2x][%2x][%2x]  temp_pix:[%4x]",txbuffer[n] [0],txbuffer[n] [1],txbuffer[n] [2],txbuffer[n] [3],temp_pix_uint32[m][n]);
          }
		usart_interrupt_enable(USART0, USART_INT_TBE);
		while(txcount < tx_size);
		txcount = 0;
		delay_1ms(1);
      }
	  //printf("\r");
	  //delay_1ms(500);
	  
		 //while(1);
}



/********************************************************************
   Function:        void loop()

   Description:

   Dependencies:
 *******************************************************************/
void loop(void) {

  //printf("\r goto switch menu \n");
  //return;
  input = '\0';
  uart_get_cmd();
  //printf("\r enter looop[%c]\n", input);

  switch (input) {
    case 'm':
      // ---MENU---
      printf("\r MENU\n\n");
      printf("\ra... read eeprom (HEX)\n");
      printf("\rb... read eeprom (value)\n");
      printf("\rc... read pixel temps\n");
      printf("\rd... print all calc steps\n");
      break;

    case 'a':
      print_eeprom_hex();
      break;


    case 'b':
      print_eeprom_value();
      break;

    case 'c':
      // --- PIXEL TEMPS WITHOUT CALC STEPS
      read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 258);
      read_pixel_data();
      sort_data();
      calculate_pixel_temp();
      pixel_masking();
      print_pixel_temps();
      break;


    case 'd':
      // --- PIXEL TEMPS WITH CALC STEPS
      read_pixel_data();
      sort_data();
      calculate_pixel_temp();
      //pixel_masking() included in print_calc_steps
      print_calc_steps();
      break;

	case 's':
		//while(1)
			//{
	read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 258);
      read_pixel_data();
      sort_data();
      calculate_pixel_temp();
      pixel_masking();
      send_one_frame_data_by_uart();
			//}
      break;

	  default:
		//printf("\n\ndefault\n\n");
		break;
  }

  //var = Serial.read();

}


/********************************************************************
   Function:      calc_timer_duration(float bw, uint8_t clk, uint8_t mbit)

   Description:   calculate the duration of the timer which reads the sensor blocks

   Dependencies:  band width (bw)
                  clock (clk)
                  adc resolution (mbit)
 *******************************************************************/
word calc_timer_duration(float bw, uint8_t clk, uint8_t mbit) {
  float Fclk_float = 12000000 / 63 * clk + 1000000;    // calc clk in Hz
  float a, b, c;
  uint16_t calculated_timer_duration;

  a = (1 / NORM_BW);
  b = (32 * ( pow(2, mbit & 0x0F)+ 4) )/( Fclk_float);
  c = b / a;
  c = c / bw;
  c = SAFETY_FAC * c;

  calculated_timer_duration = c * 1000000; // c in s | timer_duration in µs

  return calculated_timer_duration;
}


/********************************************************************
   Function:        void read_pixel_data()

   Description:     read 2 complete pictures (first with ptat, second with vdd) and electrical Offset

   Dependencies:
 *******************************************************************/
void read_pixel_data(void) {

  // --- BLOCK 0 with PTAT ---
  //printf("\r read_pixel_data ************1\n");
  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09 );

  // wait for end of conversion bit (~27ms)
  //delay_1ms(timer_duration); // poll when 90% done
  delay_1ms(timer_duration);//susente add
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 258);


  //printf("\r read_pixel_data ************2\n");
  // --- BLOCK 1 with PTAT ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  1  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x19 );

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 258);


 //printf("\r read_pixel_data ************3\n");
  // --- BLOCK 2 with PTAT ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  0  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x29 );

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block2, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block2, 258);


  //printf("\r read_pixel_data ************4\n");
  // --- BLOCK 3 with PTAT ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  1  |   1   |    0     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x39 );

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block3, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block3, 258);


  //printf("\r read_pixel_data ************5\n");

  // SAVE PTAT
  ptat_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
  ptat_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
  ptat_top_block2 = data_top_block2[0] << 8  | data_top_block2[1];
  ptat_top_block3 = data_top_block3[0] << 8  | data_top_block3[1];
  ptat_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
  ptat_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];
  ptat_bottom_block2 = data_bottom_block2[0] << 8  | data_bottom_block2[1];
  ptat_bottom_block3 = data_bottom_block3[0] << 8  | data_bottom_block3[1];





  // --- BLOCK 0 with VDD ---

  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x09 + 0x04);

  // wait for end of conversion bit (~27ms)
  //delay_1ms(timer_duration); // poll when 90% done
  delay_1ms(timer_duration);
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block0, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block0, 258);


  //printf("\r read_pixel_data ************6\n");
  // --- BLOCK 1 with VDD ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  1  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x19 + 0x04);

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block1, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block1, 258);


  //printf("\r read_pixel_data ************7\n");
  // --- BLOCK 2 with VDD ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  0  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x29 + 0x04);

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block2, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block2, 258);


  //printf("\r read_pixel_data ************8\n");
  // --- BLOCK 3 with VDD ---

  // change block in configuration register (to block1)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  1  |  1  |   1   |    1     |   0   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x39 + 0x04);

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&data_top_block3, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&data_bottom_block3, 258);


  //printf("\r read_pixel_data ************9\n");
  // SAVE VDD
  vdd_top_block0 = data_top_block0[0] << 8  | data_top_block0[1];
  vdd_top_block1 = data_top_block1[0] << 8  | data_top_block1[1];
  vdd_top_block2 = data_top_block2[0] << 8  | data_top_block2[1];
  vdd_top_block3 = data_top_block3[0] << 8  | data_top_block3[1];
  vdd_bottom_block0 = data_bottom_block0[0] << 8  | data_bottom_block0[1];
  vdd_bottom_block1 = data_bottom_block1[0] << 8  | data_bottom_block1[1];
  vdd_bottom_block2 = data_bottom_block2[0] << 8  | data_bottom_block2[1];
  vdd_bottom_block3 = data_bottom_block3[0] << 8  | data_bottom_block3[1];


  //printf("\r read_pixel_data ************10\n");
  // --- EL.OFFSET ---

  // change block in configuration register (to block0)
  // |  7  |  6  |  5  |  4  |   3   |    2     |   1   |    0   |
  // |    RFU    |   Block   | Start | VDD_MEAS | BLIND | WAKEUP |
  // |  0  |  0  |  0  |  0  |   1   |    0     |   1   |    1   |
  write_sensor_byte(SENSOR_ADDRESS, CONFIGURATION_REGISTER, 0x0B );

  // wait for end of conversion bit (~27ms)
  delay_1ms(timer_duration); // poll when 90% done
  read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  while (bitRead(statusreg, 0) == 0) {
    read_sensor_register( STATUS_REGISTER, (uint8_t*)&statusreg, 1);
  }
  read_sensor_register( TOP_HALF, (uint8_t*)&electrical_offset_top, 258);
  read_sensor_register( BOTTOM_HALF, (uint8_t*)&electrical_offset_bottom, 258
  );


}











/********************************************************************
   Function:        void pixel_masking()

   Description:     repair dead pixel by using the average of the neighbors

   Dependencies:    number of defect pixel (nrofdefpix),
                    dead pixel address (deadpixadr),
                    dead pixel mask (deadpixmask),
                    pixel temperatures (temp_pix_uint32[32][32])
 *******************************************************************/
void pixel_masking(void) {


  uint8_t number_neighbours[24];
  uint32_t temp_defpix[24];
	int i;

  for (i = 0; i < nrofdefpix; i++) {
    number_neighbours[i] = 0;
    temp_defpix[i] = 0;

    // top half

    if (deadpixadr[i] < 512) {

      if ( (deadpixmask[i] & 1 )  == 1) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32)];
      }


      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) - 1];
      }

    }

    // bottom half
    else {

      if ( (deadpixmask[i] & 1 << 0 )  == 1 << 0) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32)];
      }

      if ( (deadpixmask[i] & 2 )  == 2 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 4 )  == 4 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 8 )  == 8 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) + 1];
      }

      if ( (deadpixmask[i] & 16 )  == 16 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32)];
      }

      if ( (deadpixmask[i] & 32 )  == 32 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) - 1][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 64 )  == 64 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5)][(deadpixadr[i] % 32) - 1];
      }

      if ( (deadpixmask[i] & 128 )  == 128 ) {
        number_neighbours[i]++;
        temp_defpix[i] = temp_defpix[i] + temp_pix_uint32[(deadpixadr[i] >> 5) + 1][(deadpixadr[i] % 32) - 1];
      }
    }

    temp_defpix[i] = temp_defpix[i] / number_neighbours[i];
    temp_pix_uint32[deadpixadr[i] >> 5][deadpixadr[i] % 32] = temp_defpix[i];

  }


}





/********************************************************************
   Function:        calculate_pixel_temp()

   Description:     compensate thermal, electrical offset and vdd and multiply sensitivity coeff
                    look for the correct temp in lookup table

   Dependencies:
 *******************************************************************/
void calculate_pixel_temp(void) {

  int64_t vij_pixc_and_pcscaleval;
  int64_t vdd_calc_steps;
  uint16_t table_row, table_col;
  int32_t vx, vy, ydist, dta;
	int i;
	int m,n;
  // find column of lookup table
  for (i = 0; i < NROFTAELEMENTS; i++) {
    if (ambient_temperature > XTATemps[i]) {
	//printf("\r calculate_pixel_temp[%d]\n",ambient_temperature,XTATemps[i]);
      table_col = i;
    }
  }
  //printf("\r cal ++++1 [%d][%d]\n", ambient_temperature,XTATemps[table_col]);
  dta = (int32_t)(ambient_temperature - XTATemps[table_col]);
  //printf("\r cal ++++2 [%d]\n",dta);
  ydist = (int32_t)ADEQUIDISTANCE;

	
  for (m = 0; m < sensor.number_row; m++) {
    for (n = 0; n < sensor.number_col; n++) {

      // --- THERMAL OFFSET ---
      // compensate thermal drifts (see datasheet, chapter: 11.2 Thermal Offset)
      vij_comp_int32[m][n] = (data_pixel[m][n] - (thgrad[m][n] * ptat_av_uint16) / gradscale_div - thoffset[m][n]);


      // --- ELECTRICAL OFFSET
      // compensate electrical offset (see datasheet, chapter: 11.3 Electrical Offset)
      // top half
      if (m < sensor.number_row / 2) {
        vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m % 4][n];
      }
      // bottom half
      else {
        vij_comp_s_int32[m][n] = vij_comp_int32[m][n] - eloffset[m % 4 + 4][n];
      }



      // --- VDD ---
      // select VddCompGrad and VddCompOff for pixel m,n:
      // top half
      if (m < sensor.number_row / 2) {
        vddcompgrad_n = vddcompgrad[m % 4][n];
        vddcompoff_n = vddcompoff[m % 4][n];
      }
      // bottom half
      else {
        vddcompgrad_n = vddcompgrad[m % 4 + 4][n];
        vddcompoff_n = vddcompoff[m % 4 + 4][n];
      }
      // compensate vdd (see datasheet, chapter: 11.4 Vdd Compensation)
      vdd_calc_steps = vddcompgrad_n * ptat_av_uint16;
      vdd_calc_steps = vdd_calc_steps / vddscgrad_div;
      vdd_calc_steps = vdd_calc_steps + vddcompoff_n;
      vdd_calc_steps = vdd_calc_steps * ( vdd_av_uint16 - vddth1 - ((vddth2 - vddth1) / (ptatth2 - ptatth1)) * (ptat_av_uint16  - ptatth1));
      vdd_calc_steps = vdd_calc_steps / vddscoff_div;
      vij_vddcomp_int32[m][n] = vij_comp_s_int32[m][n] - vdd_calc_steps;

      // --- SENSITIVITY ---
      // multiply sensitivity coeff for each pixel (see datasheet, chapter: 11.5 Object Temperature)
      vij_pixc_and_pcscaleval = (int64_t)vij_vddcomp_int32[m][n] * (int64_t)PCSCALEVAL;
      vij_pixc_int32[m][n] =  (int32_t)(vij_pixc_and_pcscaleval / (int64_t)pixcij_int32[m][n]);


      // --- LOOKUPTABLE ---
      // find correct temp for this sensor in lookup table and do a bilinear interpolation (see datasheet, chapter: 11.7 Look-up table)
      table_row = vij_pixc_int32[m][n] + TABLEOFFSET;
      table_row = table_row >> ADEXPBITS;
      // bilinear interpolation
      vx = ((((int32_t)TempTable[table_row][table_col + 1] - (int32_t)TempTable[table_row][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row][table_col];
      vy = ((((int32_t)TempTable[table_row + 1][table_col + 1] - (int32_t)TempTable[table_row + 1][table_col]) * (int32_t)dta) / (int32_t)TAEQUIDISTANCE) + (int32_t)TempTable[table_row + 1][table_col];
      temp_pix_uint32[m][n] = (uint32_t)((vy - vx) * ((int32_t)(vij_pixc_int32[m][n] + TABLEOFFSET) - (int32_t)YADValues[table_row]) / ydist + (int32_t)vx);

      // --- GLOBAL OFFSET ---
      temp_pix_uint32[m][n] = temp_pix_uint32[m][n] + globaloff;

    }
  }


}

/********************************************************************
   Function:        void sort_data()

   Description:     sort the raw data blocks in 2d array and calculate ambient temperature, ptat and vdd

   Dependencies:
 *******************************************************************/
void sort_data() {

  uint32_t sum;
	int n;
  for (n = 0; n < sensor.number_col; n++) {


    // --- PIXEL DATA TOP HALF ---
    // block 0
    data_pixel[0][n] = data_top_block0[2 * n + 2] << 8 | data_top_block0[2 * n + 3];
    data_pixel[1][n] = data_top_block0[2 * (n + 32) + 2] << 8 | data_top_block0[2 * (n + 32) + 3];
    data_pixel[2][n] = data_top_block0[2 * (n + 64) + 2] << 8 | data_top_block0[2 * (n + 64) + 3];
    data_pixel[3][n] = data_top_block0[2 * (n + 96) + 2] << 8 | data_top_block0[2 * (n + 96) + 3];

    // block 1
    data_pixel[4][n] = data_top_block1[2 * n + 2] << 8 | data_top_block1[2 * n + 3];
    data_pixel[5][n] = data_top_block1[2 * (n + 32) + 2] << 8 | data_top_block1[2 * (n + 32) + 3];
    data_pixel[6][n] = data_top_block1[2 * (n + 64) + 2] << 8 | data_top_block1[2 * (n + 64) + 3];
    data_pixel[7][n] = data_top_block1[2 * (n + 96) + 2] << 8 | data_top_block1[2 * (n + 96) + 3];

    // block 2
    data_pixel[8][n] = data_top_block2[2 * n + 2] << 8 | data_top_block2[2 * n + 3];
    data_pixel[9][n] = data_top_block2[2 * (n + 32) + 2] << 8 | data_top_block2[2 * (n + 32) + 3];
    data_pixel[10][n] = data_top_block2[2 * (n + 64) + 2] << 8 | data_top_block2[2 * (n + 64) + 3];
    data_pixel[11][n] = data_top_block2[2 * (n + 96) + 2] << 8 | data_top_block2[2 * (n + 96) + 3];

    // block 3
    data_pixel[12][n] = data_top_block3[2 * n + 2] << 8 | data_top_block3[2 * n + 3];
    data_pixel[13][n] = data_top_block3[2 * (n + 32) + 2] << 8 | data_top_block3[2 * (n + 32) + 3];
    data_pixel[14][n] = data_top_block3[2 * (n + 64) + 2] << 8 | data_top_block3[2 * (n + 64) + 3];
    data_pixel[15][n] = data_top_block3[2 * (n + 96) + 2] << 8 | data_top_block3[2 * (n + 96) + 3];

    // --- PIXEL DATA BOTTOM HALF ---
    // block 3
    data_pixel[16][n] = data_bottom_block3[192 + 2 * n + 2] << 8 | data_bottom_block3[192 + 2 * n + 3];
    data_pixel[17][n] = data_bottom_block3[128 + 2 * n + 2] << 8 | data_bottom_block3[128 + 2 * n + 3];
    data_pixel[18][n] = data_bottom_block3[64 + 2 * n + 2] << 8 | data_bottom_block3[64 + 2 * n + 3];
    data_pixel[19][n] = data_bottom_block3[0 + 2 * n + 2] << 8 | data_bottom_block3[0 + 2 * n + 3];

    // block 2
    data_pixel[20][n] = data_bottom_block2[192 + 2 * n + 2] << 8 | data_bottom_block2[192 + 2 * n + 3];
    data_pixel[21][n] = data_bottom_block2[128 + 2 * n + 2] << 8 | data_bottom_block2[128 + 2 * n + 3];
    data_pixel[22][n] = data_bottom_block2[64 + 2 * n + 2] << 8 | data_bottom_block2[64 + 2 * n + 3];
    data_pixel[23][n] = data_bottom_block2[0 + 2 * n + 2] << 8 | data_bottom_block2[0 + 2 * n + 3];

    // block 1
    data_pixel[24][n] = data_bottom_block1[192 + 2 * n + 2] << 8 | data_bottom_block1[192 + 2 * n + 3];
    data_pixel[25][n] = data_bottom_block1[128 + 2 * n + 2] << 8 | data_bottom_block1[128 + 2 * n + 3];
    data_pixel[26][n] = data_bottom_block1[64 + 2 * n + 2] << 8 | data_bottom_block1[64 + 2 * n + 3];
    data_pixel[27][n] = data_bottom_block1[0 + 2 * n + 2] << 8 | data_bottom_block1[0 + 2 * n + 3];

    // block 0
    data_pixel[28][n] = data_bottom_block0[192 + 2 * n + 2] << 8 | data_bottom_block0[192 + 2 * n + 3];
    data_pixel[29][n] = data_bottom_block0[128 + 2 * n + 2] << 8 | data_bottom_block0[128 + 2 * n + 3];
    data_pixel[30][n] = data_bottom_block0[64 + 2 * n + 2] << 8 | data_bottom_block0[64 + 2 * n + 3];
    data_pixel[31][n] = data_bottom_block0[0 + 2 * n + 2] << 8 | data_bottom_block0[0 + 2 * n + 3];


    // --- ELECTRICAL OFFSET ---
    // top half
    eloffset[0][n] = electrical_offset_top[2 * n + 2] << 8 | electrical_offset_top[2 * n + 3];
    eloffset[1][n] = electrical_offset_top[2 * (n + 32) + 2] << 8 | electrical_offset_top[2 * (n + 32) + 3];
    eloffset[2][n] = electrical_offset_top[2 * (n + 64) + 2] << 8 | electrical_offset_top[2 * (n + 64) + 3];
    eloffset[3][n] = electrical_offset_top[2 * (n + 96) + 2] << 8 | electrical_offset_top[2 * (n + 96) + 3];
    // bottom half
    eloffset[4][n] = electrical_offset_bottom[2 * (n + 96) + 2] << 8 | electrical_offset_bottom[2 * (n + 96) + 3];
    eloffset[5][n] = electrical_offset_bottom[2 * (n + 64) + 2] << 8 | electrical_offset_bottom[2 * (n + 64) + 3];
    eloffset[6][n] = electrical_offset_bottom[2 * (n + 32) + 2] << 8 | electrical_offset_bottom[2 * (n + 32) + 3];
    eloffset[7][n] = electrical_offset_bottom[2 * n + 2] << 8 | electrical_offset_bottom[2 * n + 3];


  }




  // calculate ptat average (datasheet, chapter: 11.1 Ambient Temperature )
  sum = ptat_top_block0 + ptat_top_block1 + ptat_top_block2 + ptat_top_block3 + ptat_bottom_block0 + ptat_bottom_block1 + ptat_bottom_block2 + ptat_bottom_block3;
  ptat_av_uint16 = sum / 8;


  // calculate ambient_temperature (datasheet, chapter: 11.1 Ambient Temperature )
  ambient_temperature = ptat_av_uint16 * ptatgr_float + ptatoff_float;


  // calculate vdd average (datasheet, chapter: 11.4 Vdd Compensation )
  sum = vdd_top_block0 + vdd_top_block1 + vdd_top_block2 + vdd_top_block3 + vdd_bottom_block0 + vdd_bottom_block1 + vdd_bottom_block2 + vdd_bottom_block3;
  vdd_av_uint16 = sum / 8;


}



/********************************************************************
   Function:        void calculate_pixcij()

   Description:     calculate sensitivity coefficients for each pixel

   Dependencies:    minimum sensitivity coefficient (pixcmin),
                    maximum sensitivity coefficient (pixcmax),
                    sensitivity coefficient (pij[32][32]),
                    emissivity factor (epsilon),
                    factor for fine tuning of the sensitivity (globalgain)
 *******************************************************************/
void calculate_pixcij() {

	int m,n;
  for (m = 0; m < 32; m++) {
    for (n = 0; n < 32; n++) {

      // calc sensitivity coefficients (see datasheet, chapter: 11.5 Object Temperature)
      pixcij_int32[m][n] = (int32_t)pixcmax - (int32_t)pixcmin;
      pixcij_int32[m][n] = pixcij_int32[m][n] / 65535;
      pixcij_int32[m][n] = pixcij_int32[m][n] * pij[m][n];
      pixcij_int32[m][n] = pixcij_int32[m][n] + pixcmin;
      pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0  * epsilon / 100;
      pixcij_int32[m][n] = pixcij_int32[m][n] * 1.0  * globalgain / 10000;

    }
  }

}








/********************************************************************
   Function:        void read_EEPROM_byte(int deviceaddress, unsigned int eeaddress )

   Description:     read eeprom register

   Dependencies:    epprom address (deviceaddress)
                    eeprom register (eeaddress)
 *******************************************************************/
byte read_EEPROM_byte(int deviceaddress, unsigned int eeaddress ) {
     byte rdata = 0xFF;
      //printf("\r read_EEPROM_byte \n");
	int n = 1;
      /* wait until I2C bus is idle */
	  while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
  
  	 //i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);
	 if(2 == n)
	 {
		i2c_ackpos_config(I2C0,I2C_ACKPOS_NEXT);
	  }
  
	  /* send a start condition to I2C bus */
	  i2c_start_on_bus(I2C0);
  
	  /* wait until SBSEND bit is set */
	  while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
	  /* send slave address to I2C bus */
	  i2c_master_addressing(I2C0, EEPROM_ADDRESS, I2C_TRANSMITTER);
	  /* wait until ADDSEND bit is set */
	  while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
	  //printf("\n read_sensor_register #####2 \n");
	  /* clear ADDSEND bit */
	  i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	  /* wait until the transmit data buffer is empty */
	  while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
  
       i2c_enable(I2C0);
	  // printf("\r while 1 n:[%d] ##########3\n", n);

	  //i2c_data_transmit(I2C0, eeaddress >> 8);
	  i2c_data_transmit(I2C0, eeaddress >> 8);
	  while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));//I2C_FLAG_BTC

	  i2c_data_transmit(I2C0, eeaddress & 0xff);
	  while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));//I2C_FLAG_BTC
	  
	  i2c_start_on_bus(I2C0);
	  /* wait until SBSEND bit is set */
	  
	  while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
	  //printf("\r while 1 n:[%d] ##########2\n", n);
  
	  /* send slave address to I2C bus */
	  i2c_master_addressing(I2C0, EEPROM_ADDRESS, I2C_RECEIVER);
	  
	  if(n < 3)
	  {
		   /* disable acknowledge */
		   i2c_ack_config(I2C0,I2C_ACK_DISABLE);
	   }
	  
	  while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
	  i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	  
	 /* send a stop condition to I2C bus */
	  if(1 == n)
	  {
       	 /* send a stop condition to I2C bus */
        	i2c_stop_on_bus(I2C0);
        }

	  while(n)
	  {
	  		if(3 == n)
	  		{
			 	/* wait until BTC bit is set */
            		while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
				/* disable acknowledge */
            		i2c_ack_config(I2C0,I2C_ACK_DISABLE);
			}
	  		if(2 == n)
	  		{
			 	/* wait until BTC bit is set */
            		while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
				/* disable acknowledge */
            		i2c_stop_on_bus(I2C0);
			}
			//delay_1ms(5);
		  	if(i2c_flag_get(I2C0, I2C_FLAG_RBNE))
	  		{
              		rdata = i2c_data_receive(I2C0);
				n--;
	  		}
	  }

	  /* wait until stop condition generate */ 
	  while(I2C_CTL0(I2C0)&0x0200);
	  
	  /* enable acknowledge */
	  i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	  i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);

  return rdata;
}



/********************************************************************
   Function:        void read_eeprom()

   Description:     read all values from eeprom

   Dependencies:
 *******************************************************************/


void read_eeprom() {
  printf("\r read eeprom #####1\n");
  byte b[4];
  bw = (read_EEPROM_byte(EEPROM_ADDRESS, E_BW2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_BW1)) / 100;
  id = read_EEPROM_byte(EEPROM_ADDRESS, E_ID4) << 24 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID3) << 16 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_ID1);
  mbit_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_MBIT_CALIB);
  bias_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_BIAS_CALIB);
  clk_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_CLK_CALIB);
  bpa_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_BPA_CALIB);
  pu_calib = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_CALIB);
  mbit_user = read_EEPROM_byte(EEPROM_ADDRESS, E_MBIT_USER);
  bias_user = read_EEPROM_byte(EEPROM_ADDRESS, E_BIAS_USER);
  clk_user = read_EEPROM_byte(EEPROM_ADDRESS, E_CLK_USER);
  bpa_user = read_EEPROM_byte(EEPROM_ADDRESS, E_BPA_USER);
  pu_user = read_EEPROM_byte(EEPROM_ADDRESS, E_PU_USER);
  vddth1 = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH1_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH1_1);
  vddth2 = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH2_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDTH2_1);
  vddscgrad = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDSCGRAD);
  vddscoff = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDSCOFF);
  ptatth1 = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH1_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH1_1);
  ptatth2 = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH2_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PTATTH2_1);
  nrofdefpix = read_EEPROM_byte(EEPROM_ADDRESS, E_NROFDEFPIX);
  gradscale = read_EEPROM_byte(EEPROM_ADDRESS, E_GRADSCALE);
  tablenumber = read_EEPROM_byte(EEPROM_ADDRESS, E_TABLENUMBER2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_TABLENUMBER1);
  arraytype = read_EEPROM_byte(EEPROM_ADDRESS, E_ARRAYTYPE);
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATGR_4);
  ptatgr_float = *(float*)b;
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PTATOFF_4);
  ptatoff_float = *(float*)b;
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMIN_4);
  pixcmin = *(float*)b;
  b[0] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_1);
  b[1] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_2);
  b[2] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_3);
  b[3] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIXCMAX_4);
  pixcmax = *(float*)b;
  epsilon = read_EEPROM_byte(EEPROM_ADDRESS, E_EPSILON);
  globaloff = read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALOFF);
  globalgain = read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALGAIN_2) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_GLOBALGAIN_1);
  // --- DeadPixAdr ---
	int i;
  for (i = 0; i < nrofdefpix; i++) {
    deadpixadr[i] = read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXADR + 2 * i + 1 ) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXADR + 2 * i);
    if (deadpixadr[i] > 512) {    // adaptedAdr:
      deadpixadr[i] = 1024 + 512 - deadpixadr[i] + 2 * (deadpixadr[i] % 32 ) - 32;
    }
  }
  // --- DeadPixMask ---
  for (int i = 0; i < nrofdefpix; i++) {
    deadpixmask[i] = read_EEPROM_byte(EEPROM_ADDRESS, E_DEADPIXMASK + i);
  }

  printf("\r read eeprom #####2\n");
  // --- Thgrad_ij ---
  int m = 0;
  int n = 0;
  uint16_t addr_i = 0x0740; // start address
  // top half
  for (int i = 0; i < 512; i++) {
    addr_i = 0x0740 + 2 * i;
    thgrad[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 2 * i);
    n++;
    if (n == 32) {
      n = 0;
      m++;
    }
  }
  // bottom half
   printf("\r read eeprom #####3\n");
  for (int i = 0; i < sensor.number_col; i++) {

    thgrad[31][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * i);
    thgrad[30][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 1 * 64 + 2 * i);
    thgrad[29][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 2 * 64 + 2 * i);
    thgrad[28][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0400 + 3 * 64 + 2 * i);

    thgrad[27][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * i);
    thgrad[26][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 1 * 64 + 2 * i);
    thgrad[25][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 2 * 64 + 2 * i);
    thgrad[24][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0500 + 3 * 64 + 2 * i);

    thgrad[23][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * i);
    thgrad[22][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 1 * 64 + 2 * i);
    thgrad[21][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 2 * 64 + 2 * i);
    thgrad[20][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0600 + 3 * 64 + 2 * i);

    thgrad[19][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * i);
    thgrad[18][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 1 * 64 + 2 * i);
    thgrad[17][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 2 * 64 + 2 * i);
    thgrad[16][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THGRAD + 0x0700 + 3 * 64 + 2 * i);

  }
  printf("\r read eeprom #####4\n");

  // --- ThOffset_ij ---
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < 512; i++) {
    addr_i = 0x0F40 + 2 * i;
    thoffset[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 2 * i);
    n++;
    if (n == 32) {
      n = 0;
      m++;
    }
  }
  printf("\r read eeprom #####5\n");
  // bottom half
  for (int i = 0; i < sensor.number_col; i++) {
    thoffset[31][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * i);
    thoffset[30][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 1 * 64 + 2 * i);
    thoffset[29][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 2 * 64 + 2 * i);
    thoffset[28][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0400 + 3 * 64 + 2 * i);

    thoffset[27][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * i);
    thoffset[26][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 1 * 64 + 2 * i);
    thoffset[25][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 2 * 64 + 2 * i);
    thoffset[24][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0500 + 3 * 64 + 2 * i);

    thoffset[23][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * i);
    thoffset[22][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 1 * 64 + 2 * i);
    thoffset[21][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 2 * 64 + 2 * i);
    thoffset[20][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0600 + 3 * 64 + 2 * i);

    thoffset[19][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * i);
    thoffset[18][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 1 * 64 + 2 * i);
    thoffset[17][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 2 * 64 + 2 * i);
    thoffset[16][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_THOFFSET + 0x0700 + 3 * 64 + 2 * i);
  }


  //---VddCompGrad---
  printf("\r read eeprom #####6\n");
  // top half
  for (int i = 0; i < sensor.number_col; i++) {
    // top half
    vddcompgrad[0][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * i);
    vddcompgrad[1][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 1 * 64 + 2 * i);
    vddcompgrad[2][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 2 * 64 + 2 * i);
    vddcompgrad[3][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 3 * 64 + 2 * i);
    // bottom half (backwards)
    vddcompgrad[7][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 4 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 4 * 64 + 2 * i);
    vddcompgrad[6][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 5 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 5 * 64 + 2 * i);
    vddcompgrad[5][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 6 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 6 * 64 + 2 * i);
    vddcompgrad[4][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 7 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPGRAD + 7 * 64 + 2 * i);

  }

  //---VddCompOff---
  printf("\r read eeprom #####7\n");

  // top half
  for (int i = 0; i < sensor.number_col; i++) {
    // top half
    vddcompoff[0][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * i);
    vddcompoff[1][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 1 * 64 + 2 * i);
    vddcompoff[2][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 2 * 64 + 2 * i);
    vddcompoff[3][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 3 * 64 + 2 * i);
    // bottom half
    vddcompoff[7][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 4 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 4 * 64 + 2 * i);
    vddcompoff[6][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 5 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 5 * 64 + 2 * i);
    vddcompoff[5][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 6 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 6 * 64 + 2 * i);
    vddcompoff[4][i] = read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 7 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_VDDCOMPOFF + 7 * 64 + 2 * i);

  }

  printf("\r read eeprom #####8\n");

  // --- P_ij ---
  m = 0;
  n = 0;
  // top half
  for (int i = 0; i < 512; i++) {
    addr_i = 0x0F40 + 2 * i;
    pij[m][n] = read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 2 * i);
    n++;
    if (n == 32) {
      n = 0;
      m++;
    }
  }
   printf("\r read eeprom #####9\n");
  // bottom half
  for (int i = 0; i < sensor.number_col; i++) {
    pij[31][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * i);
    pij[30][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 1 * 64 + 2 * i);
    pij[29][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 2 * 64 + 2 * i);
    pij[28][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0400 + 3 * 64 + 2 * i);

    pij[27][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * i);
    pij[26][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 1 * 64 + 2 * i);
    pij[25][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 2 * 64 + 2 * i);
    pij[24][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0500 + 3 * 64 + 2 * i);

    pij[23][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * i);
    pij[22][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 1 * 64 + 2 * i);
    pij[21][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 2 * 64 + 2 * i);
    pij[20][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0600 + 3 * 64 + 2 * i);

    pij[19][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * i);
    pij[18][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 1 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 1 * 64 + 2 * i);
    pij[17][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 2 * 64 + 2 * i);
    pij[16][i] =  read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 3 * 64 + 2 * i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, E_PIJ + 0x0700 + 3 * 64 + 2 * i);

  }


}

/********************************************************************
   Function:        void write_user_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_user_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_user);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_user);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_user);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_user);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_user);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_user);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_user);
}

/********************************************************************
   Function:        void write_calibration_settings_to_sensor()

   Description:     write calibration data (from eeprom) to trim registers (sensor)

   Dependencies:
 *******************************************************************/
void write_calibration_settings_to_sensor() {

  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, 0x22); //added by susente
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER1, mbit_calib);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER2, bias_calib);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER3, bias_calib);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER4, clk_calib);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER5, bpa_calib);
  delay_1ms(5);
  write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER6, bpa_calib);
  //delay_1ms(5);
  //write_sensor_byte(SENSOR_ADDRESS, TRIM_REGISTER7, pu_calib);
}


/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)

   Description:     read sensor register

   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n) {

      /* wait until I2C bus is idle */
	  while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
  
	 if(2 == n)
	 {
		i2c_ackpos_config(I2C0,I2C_ACKPOS_NEXT);
	  }
  
	  /* send a start condition to I2C bus */
	  i2c_start_on_bus(I2C0);
  
	  /* wait until SBSEND bit is set */
	  while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
	  /* send slave address to I2C bus */
	  i2c_master_addressing(I2C0, SENSOR_ADDRESS, I2C_TRANSMITTER);
	  /* wait until ADDSEND bit is set */
	  while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));

	  /* clear ADDSEND bit */
	  i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	  /* wait until the transmit data buffer is empty */
	  while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
  
       i2c_enable(I2C0);

	  i2c_data_transmit(I2C0, addr);
	  while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));//I2C_FLAG_BTC

	  i2c_start_on_bus(I2C0);
	  /* wait until SBSEND bit is set */
	  
	  while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
  
	  /* send slave address to I2C bus */
	  i2c_master_addressing(I2C0, SENSOR_ADDRESS, I2C_RECEIVER);

	  if(n < 3)
	  {
		   /* disable acknowledge */
		   i2c_ack_config(I2C0,I2C_ACK_DISABLE);
	   }
	  while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));

	  i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);

	  
	 /* send a stop condition to I2C bus */
	  if(1 == n)
	  {
       	 /* send a stop condition to I2C bus */
        	i2c_stop_on_bus(I2C0);
        }

	  while(n)
	  {
	  		if(3 == n)
	  		{
			 	/* wait until BTC bit is set */
            		while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
				/* disable acknowledge */
            		i2c_ack_config(I2C0,I2C_ACK_DISABLE);
			}
	  		if(2 == n)
	  		{
			 	/* wait until BTC bit is set */
            		while(!i2c_flag_get(I2C0, I2C_FLAG_BTC));
				/* disable acknowledge */
            		i2c_stop_on_bus(I2C0);
			}
			//delay_1ms(5);
		  	if(i2c_flag_get(I2C0, I2C_FLAG_RBNE))
	  		{
              		*dest++= i2c_data_receive(I2C0);
				n--;
				//printf("\r read_EEPROM_byte rdata[%x], n:[%d] \n", *(dest-1), n);
	  		}
	  }

	  /* wait until stop condition generate */ 
	  while(I2C_CTL0(I2C0)&0x0200);
	  
	  /* enable acknowledge */
	  i2c_ack_config(I2C0, I2C_ACK_ENABLE);
	  i2c_ackpos_config(I2C0, I2C_ACKPOS_CURRENT);

}



/********************************************************************
   Function:        void write_SENDOR_byte(int deviceaddress, unsigned int eeaddress )

   Description:     write sensor register

   Dependencies:    device address (deviceaddress)
                    register address (registeraddress)
                    input byte (input)
 *******************************************************************/
void write_EEPROM_byte(int deviceaddress, unsigned int eeaddress, uint8_t input) {

	 /* wait until I2C bus is idle */
	 while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
	 /* send a start condition to I2C bus */
	 i2c_start_on_bus(I2C0);
	 /* wait until SBSEND bit is set */
	 while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
	 /* send slave address to I2C bus */
	 i2c_master_addressing(I2C0, deviceaddress, I2C_TRANSMITTER);
	 /* wait until ADDSEND bit is set */
	 while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
	 /* clear ADDSEND bit */
	 i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
	 /* wait until the transmit data buffer is empty */
	 while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
  
	/* register address transmission */
	  i2c_data_transmit(I2C0, eeaddress >>8);
	/* wait until the TBE bit is set */
	 while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
	 /* register address transmission */
	  i2c_data_transmit(I2C0, eeaddress &&0xFF);
	/* wait until the TBE bit is set */
	 while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
  
	  /* data transmission */
	  i2c_data_transmit(I2C0, input);
	/* wait until the TBE bit is set */
	 while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
	
	 /* send a stop condition to I2C bus */
	 i2c_stop_on_bus(I2C0);
	 while(I2C_CTL0(I2C0)&0x0200);

}

/********************************************************************
   Function:        void read_sensor_register( uint16_t addr, uint8_t *dest, uint16_t n)

   Description:     read sensor register

   Dependencies:    register address (addr),
                    number of bytes (n)
 *******************************************************************/
void write_sensor_byte(uint8_t deviceaddress, uint8_t registeraddress, uint8_t input) {

    //editted by susente 
    /* wait until I2C bus is idle */
    while(i2c_flag_get(I2C0, I2C_FLAG_I2CBSY));
    /* send a start condition to I2C bus */
    i2c_start_on_bus(I2C0);
    /* wait until SBSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_SBSEND));
    /* send slave address to I2C bus */
    i2c_master_addressing(I2C0, deviceaddress, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_ADDSEND));
    /* clear ADDSEND bit */
    i2c_flag_clear(I2C0, I2C_FLAG_ADDSEND);
    /* wait until the transmit data buffer is empty */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

   /* register address transmission */
     i2c_data_transmit(I2C0, registeraddress);
   /* wait until the TBE bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));

	 /* data transmission */
     i2c_data_transmit(I2C0, input);
   /* wait until the TBE bit is set */
    while(!i2c_flag_get(I2C0, I2C_FLAG_TBE));
   
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(I2C0);
    while(I2C_CTL0(I2C0)&0x0200);

}




























/********************************************************************

   END OF READ-OUT PROGRAMM

   the following functions are used to chat with serial monitor

 *******************************************************************/










































/********************************************************************
   Function:        print_pixel_temps()

   Description:     print temperature on serial monitor

   Dependencies:
 *******************************************************************/
void print_pixel_temps() {


  printf("\n\n\n\r---PRINT PIXEL TEMPERATURE---\n");

  printf("\n\n\rpixel temperature (dK)\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%2d ",temp_pix_uint32[m][n]);
    }
    printf("\n\r");
  }

  printf("\n\n\n\n\rdone (m... back to menu)\n\n\n");
}




/********************************************************************
   Function:        print_calc_steps()

   Description:     print every needed step for temperature calculation + pixel masking

   Dependencies:
 *******************************************************************/
void print_calc_steps() {

#if 1
  printf("\n\n\n\r---PRINT ALL STEPS---");

  printf("\n\n\n\r1) read row pixel data (V_ij):\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",data_pixel[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }

  printf("\n\n\n\r2) read electrical offset (elOffset_ij):\n\n\r");
  for (int m = 0; m < 8; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",eloffset[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }

  printf("\n\n\n\r3) calculate ambient temperature (Ta):\n\n");
  printf("\rPTAT_av = 1/8*(");
  printf("%d",ptat_top_block0);
  printf(" + ");
  printf("%d",ptat_top_block1);
  printf(" + ");
  printf("%d",ptat_top_block2);
  printf(" + ");
  printf("%d",ptat_top_block3);
  printf(" + ");
  printf("%d",ptat_bottom_block0);
  printf(" + ");
  printf("%d",ptat_bottom_block1);
  printf(" + ");
  printf("%d",ptat_bottom_block2);
  printf(" + ");
  printf("%d",ptat_bottom_block3);
  printf(") = ");
  printf("%d",ptat_av_uint16);
  printf("\n\n\rTa = ");
  printf("%d",ptat_av_uint16);
  printf(" * ");
  printf("%f",ptatgr_float);
  printf(" + ");
  printf("%f",ptatoff_float);
  printf(" = ");
  printf("%d",ambient_temperature);
  printf(" (Value is given in dK)");


  printf("\n\n\n\r4) compensate thermal offset (V_ij_comp):\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",vij_comp_int32[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }

  printf("\n\n\n\r5) compensate electrical offset (V_ij_comp_s):\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",vij_comp_s_int32[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }


  printf("\n\n\n\r6) vdd compensation (V_ij_vddcomp):\n\n\r");
  printf("VDD_av = 1/8*(");
  printf("%d",vdd_top_block0);
  printf(" + ");
  printf("%d",vdd_top_block1);
  printf(" + ");
  printf("%d",vdd_top_block2);
  printf(" + ");
  printf("%d",vdd_top_block3);
  printf(" + ");
  printf("%d",vdd_bottom_block0);
  printf(" + ");
  printf("%d",vdd_bottom_block1);
  printf(" + ");
  printf("%d",vdd_bottom_block2);
  printf(" + ");
  printf("%d",vdd_bottom_block3);
  printf(") = ");
  printf("%d",vdd_av_uint16);
  printf("\n\n\r");



  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",vij_vddcomp_int32[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }


  printf("\n\n\n\r7) calculate sensitivity coefficients (pixc_ij):\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",pixcij_int32[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }

  printf("\n\n\n\r8) multiply scaling coeff and sensitivity coeff to compensated pixel voltages (V_ij_pixc):\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",vij_pixc_int32[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }

  printf("\n\n\n\r9) calcluate final pixel temperature (in dK) with lookup table and bilinear interpolation:\n\n\r");
  for (int m = 0; m < sensor.number_row; m++) {
    for (int n = 0; n < sensor.number_col; n++) {
      printf("%d",temp_pix_uint32[m][n]);
      printf(" ");
    }
    printf("\n\r");
  }


  pixel_masking();

  printf("\n\n\n\r10) pixel masking (if there is a defect pixel):\n\n\r");

  if (nrofdefpix > 0) {
    for (int m = 0; m < sensor.number_row; m++) {
      for (int n = 0; n < sensor.number_col; n++) {
        printf("%d",temp_pix_uint32[m][n]);
        printf(" ");
      }
      printf("\n\r");
    }
  }
  else {
    printf("no defect pixel");
  }


  printf("\n\n\n\n\rdone (m... back to menu)\n\n\n");
  #endif
}



/********************************************************************
   Function:        print_eeprom_hex()

   Description:     print eeprom contint as hex values

   Dependencies:
 *******************************************************************/
void print_eeprom_hex() {
  printf("\n\n\n\r---PRINT EEPROM (HEX)---\n");
  printf("\n\n\rEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n\r");

  // line
  for (int i = 0; i < 75; i++) {
    printf("- ");
  }

  for (int i = 0; i <= 0x13FF; i++) {


    if (i % 16 == 0) {
      printf("\n\r");

      if (i < 0x0080) {
        printf("\rHEADER\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x00D0) {
        printf("\rDEADPIX\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x0340) {
        printf("\rFREE\t0x");
       // printf(i, HEX);
       printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x0540) {
        printf("\rVDDGRAD\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x0740) {
        printf("\rVDDOFF\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x0F40) {
        printf("\rTHGRAD\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x1740) {
        printf("\rTHOFF\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x1F40) {
        printf("\rPij\t0x");
        //printf(i, HEX);
        printf("0x%.2x",i);
        printf("\t|\t");
      }
    }
    else {
      printf("\t");
    }

    //printf(read_EEPROM_byte(EEPROM_ADDRESS, i), HEX);
    printf("0x%.2x",read_EEPROM_byte(EEPROM_ADDRESS, i));

  }

  printf("\n\n\n\n\rdone (m... back to menu)\n\n\n");
}




/********************************************************************
   Function:        print_eeprom_value()

   Description:     print all needed values in their saved form

   Dependencies:
 *******************************************************************/
void print_eeprom_value() {

#if 1
  printf("\n\n\n---PRINT EEPROM (VALUE)---\n");
  printf("\nHINT: Here values longer than 8 bit are printed in their first block.\n");
  printf("\n\nEEPROM 32x32\t\t0x00\t0x01\t0x02\t0x03\t0x04\t0x05\t0x06\t0x07\t0x08\t0x09\t0x0A\t0x0B\t0x0C\t0x0D\t0x0E\t0x0F\n");

  // line
  for (int i = 0; i < 75; i++) {
    printf("- ");
  }
  // HEADER
  // 1st line
  printf("\n");
  printf("HEADER\t0x00");
  printf("\t|\t");
  printf("%d",pixcmin);
  printf("\t\t\t");
  printf("%d",pixcmax);
  printf("\t\t\t");
  printf("%d",gradscale);
  printf("\t\t\t");
  printf("%d",tablenumber);
  printf("\t\t");
  printf("%d",epsilon);
  // 2nd line
  printf("\n");
  printf("HEADER\t0x10");
  printf("\t|\t\t\t\t\t\t\t\t\t\t\t");
  printf("%d",mbit_calib);
  printf("\t");
  printf("%d",bias_calib);
  printf("\t");
  printf("%d",clk_calib);
  printf("\t");
  printf("%d",bpa_calib);
  printf("\t");
  printf("%d",pu_calib);
  // 3rd line
  printf("\n");
  printf("HEADER\t0x20");
  printf("\t|\t\t\t");
  printf("%d",arraytype);
  printf("\t\t\t\t");
  printf("%d",vddth1);
  printf("\t\t");
  printf("%d",vddth2);
  // 4th line
  printf("\n");
  printf("HEADER\t0x30");
  printf("\t|\t\t\t\t\t");
  printf("%5d ",ptatgr_float);
  printf("\t\t\t\t");
  printf("%d",ptatoff_float);
  printf("\t\t\t\t");
  printf("%d",ptatth1);
  printf("\t\t");
  printf("%d",ptatth2);
  // 5th line
  printf("\n");
  printf("HEADER\t0x40");
  printf("\t|\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t");
  printf("%d",vddscgrad);
  printf("\t");
  printf("%d",vddscoff);
  // 6th line
  printf("\n");
  printf("HEADER\t0x50");
  printf("\t|\t\t\t\t\t");
  printf("%d",globaloff);
  printf("\t");
  printf("%d",globalgain);
  // 7th line
  printf("\n");
  printf("HEADER\t0x60");
  printf("\t|\t");
  printf("%d",mbit_user);
  printf("\t");
  printf("%d",bias_user);
  printf("\t");
  printf("%d",clk_user);
  printf("\t");
  printf("%d",bpa_user);
  printf("\t");
  printf("%d",pu_user);
  // 8th line
  printf("\n");
  printf("HEADER\t0x70");
  printf("\t|\t\t\t\t\t");
  printf("%d",id);
  printf("\t\t\t\t\t\t\t\t\t\t\t");
  printf("%d",nrofdefpix);



  // OTHER (16bit)
  for (int i = 0x0080; i <= 0x00AF; i = i + 2) {

    if (i % 16 == 0) {
      printf("\n");
      printf("DEADPIX\t0x");
      printf("0x%.2x",i);
      printf("\t|\t");
    }
    else {
      printf("\t\t");
    }
    printf("%d",read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i));
  }


  // OTHER (8bit)
  for (int i = 0x00B0; i <= 0x033F; i++) {

    if (i % 16 == 0) {
      printf("\n");
      if (i < 0x00D0) {
        printf("DEADPIX\t0x");
      }
      else {
        printf("FREE\t0x");
      }
      printf("0x%.2x",i);
      printf("\t|\t");
    }
    else {
      printf("\t");
    }
    printf("%d", read_EEPROM_byte(EEPROM_ADDRESS, i));
  }


  // OTHER (16bit)
  for (int i = 0x0340; i <= 0x1F3F; i = i + 2) {

    if (i % 16 == 0) {
      printf("\n");


      if (i < 0x0540) {
        printf("VDDGRAD\t0x");
         printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x0740) {
        printf("VDDOFF\t0x");
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x0F40) {
        printf("THGRAD\t0x");
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x1740) {
        printf("THOFF\t0x");
        printf("0x%.2x",i);
        printf("\t|\t");
      }
      else if (i < 0x1F40) {
        printf("Pij\t0x");
        printf("0x%.2x",i);
        printf("\t|\t");
      }
    }

    else {
      printf("\t\t");
    }



    if (i >= 0x0340 && i < 0x1740) {
      printf("%d",(int16_t)(read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i)));
    }
    else {
      printf("%d",read_EEPROM_byte(EEPROM_ADDRESS, i + 1) << 8 | read_EEPROM_byte(EEPROM_ADDRESS, i));
    }


  }

  printf("\n\n\n\ndone (m... back to menu)\n\n\n");
#endif
}


