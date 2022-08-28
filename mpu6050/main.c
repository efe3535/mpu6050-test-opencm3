#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include "i2c-init.h"

int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);
void get_buffered_line(void);

#define BUFLEN 127

static uint16_t start_ndx;
static uint16_t end_ndx;
static char buf[BUFLEN+1];
#define buf_len ((end_ndx - start_ndx) % BUFLEN)
static inline int inc_ndx(int n) { return ((n + 1) % BUFLEN); }
static inline int dec_ndx(int n) { return (((n + BUFLEN) - 1) % BUFLEN); }


static void clock_setup(void) {
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);
	rcc_periph_clock_enable(RCC_GPIOA);	
}

static void gpio_setup(void) {
	gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO5);
}

static void usart_setup(void) {
		rcc_periph_clock_enable(RCC_USART2);	
	    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
        gpio_set_af(GPIOA, GPIO_AF7, GPIO2 | GPIO3);
	    usart_set_baudrate(USART2, 115200);
        usart_set_databits(USART2, 8);
        usart_set_stopbits(USART2, USART_STOPBITS_1);
        usart_set_mode(USART2, USART_MODE_TX_RX);
        usart_set_parity(USART2, USART_PARITY_NONE);
        usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
        usart_enable(USART2);
}



int main(void) {
	clock_setup();
	gpio_setup();
	i2c_setup();
	usart_setup();

	gpio_toggle(GPIOA, GPIO5);
	
	uint8_t data = 0;
	uint8_t cmd = WHO_AM_I_REG;
	i2c_transfer7(I2C1, MPU6050_ADDR, &cmd, 1, &data, 1);
	
	if(data == 104) {
		gpio_toggle(GPIOA, GPIO5);

		uint8_t cmd_arr[] = {PWR_MGMT_1_REG, 0,SMPLRT_DIV_REG,0x07, ACCEL_CONFIG_REG, 0, GYRO_CONFIG_REG, 0};
	//	uint8_t cmd_arr[] = {PWR_MGMT_1_REG, 0, ACCEL_CONFIG_REG, 0x10, 0x1b, 0x10};
		i2c_transfer7(I2C1, MPU6050_ADDR, cmd_arr,2, NULL, 0);
	}
	gpio_toggle(GPIOA, GPIO5);
	for(int i=0;i<250000;i++) {__asm("nop");}
	gpio_toggle(GPIOA, GPIO5);
	while(1) {
		char myaccelX[128];
		char myaccelY[128];
		char myaccelZ[128];
		uint8_t vals[6];
		cmd = 0x3B; //0x3B
/*
		char myaccel1[128];
		char myaccel2[128];
		char myaccel3[128];
		char myaccel4[128];
		char myaccel5[128];
		char myaccel6[128];
*/
		uint8_t tmp[6];


		i2c_transfer7(I2C1, MPU6050_ADDR, &cmd, 1, tmp, 6);


		
/*		sprintf(myaccelX ,"%lf", ((int16_t)(tmp[0]<<8 | tmp[1]))/16384.0);
		sprintf(myaccelY ,"%lf", ((int16_t)(tmp[2]<<8 | tmp[3]))/16384.0);
		sprintf(myaccelZ ,"%lf", ((int16_t)(tmp[4]<<8 | tmp[5]))/14418.0);
*/

		//sprintf(myaccelX ,"%f", (double)((int16_t)(tmp[0]<<8 | tmp[1]))/16384.0);

		//my_dtostrf(((int16_t)(tmp[0]<<8 | tmp[1]))/16384.0, 3,4,  myaccelX);
		/*
		sprintf(myaccelX ,"%d", ((int16_t)(tmp[0]<<8 | tmp[1])));
		sprintf(myaccelY ,"%d", ((int16_t)(tmp[2]<<8 | tmp[3])));
		sprintf(myaccelZ ,"%d", ((int16_t)(tmp[4]<<8 | tmp[5])));
		*/
	
		sprintf(myaccelX ,"%d", ((int16_t)(tmp[0]<<8 | tmp[1])));
		sprintf(myaccelY ,"%d", ((int16_t)(tmp[2]<<8 | tmp[3])));
		sprintf(myaccelZ ,"%d", ((int16_t)(tmp[4]<<8 | tmp[5])));
		


		/*	
		sprintf(myaccel1 ,"%u", ((uint16_t)(tmp[0])));
		sprintf(myaccel2 ,"%u", ((uint16_t)(tmp[1])));
		sprintf(myaccel3 ,"%u", ((uint16_t)(tmp[2])));
		sprintf(myaccel4 ,"%u", ((uint16_t)(tmp[3])));
		sprintf(myaccel5 ,"%u", ((uint16_t)(tmp[4])));
		sprintf(myaccel6 ,"%u", ((uint16_t)(tmp[5])));
*/
		
		
		for(int j=0; j<(int)strlen(myaccelX);j++) {
			usart_send_blocking(USART2,myaccelX[j]);
		}
		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');
		
		for(int j=0; j<(int)strlen(myaccelY);j++) {
			usart_send_blocking(USART2,myaccelY[j]);
		}
		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');
		
		for(int j=0; j<(int)strlen(myaccelZ);j++) {
			usart_send_blocking(USART2,myaccelZ[j]);
		}
		
		
		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');
		
		cmd = 0x41;		
		uint8_t tmp2[2];
		int16_t temp;
		uint8_t temp_calc;
		char temps[128];
		i2c_transfer7(I2C1, MPU6050_ADDR, &cmd, 1, tmp2, 2);
		

		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');

		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');
		temp = (int16_t) (tmp2[0]<<8 | tmp2[1]);
		temp_calc = (uint8_t)(float)((int16_t)temp/(float)340.0+(float)36.53);
		sprintf(temps, "%u", temp_calc);
		for(int i=0;i<(int)(strlen(temps));i++) {
			usart_send_blocking(USART2, temps[i]);
		}

		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');
		for(int i=0;i<2500000;i++) {__asm("nop");}

		usart_send_blocking(USART2,'\r');
		usart_send_blocking(USART2,'\n');
	}
	
	return 0;
}

