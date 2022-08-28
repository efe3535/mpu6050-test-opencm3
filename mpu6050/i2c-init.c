#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include "i2c-init.h"

void i2c_setup(void) {
        rcc_periph_clock_enable(RCC_I2C1);
        rcc_periph_clock_enable(RCC_GPIOB);
		rcc_periph_reset_pulse(RST_I2C1);
        gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO8 | GPIO9);
        gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, GPIO8 | GPIO9);
        gpio_set_af(GPIOB, GPIO_AF4, GPIO8 | GPIO9);
        i2c_peripheral_disable(I2C1);
        i2c_reset(I2C1);
        i2c_set_standard_mode(I2C1);
        i2c_enable_ack(I2C1);
        i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2); /* default, no need to do this really */
        i2c_set_clock_frequency(I2C1, 45); // old 8
        i2c_set_speed(I2C1, i2c_speed_fm_400k, 45);
        i2c_peripheral_enable(I2C1);
}

void mpu6050_send(uint8_t cmd, uint8_t data, uint8_t cmdlen, uint8_t datalen) {
	i2c_transfer7(I2C1, MPU6050_ADDR, &cmd, cmdlen, &data, datalen);
}
