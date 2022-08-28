# Instructions
 1. `cd mpu6050`
 2. `make -C libopencm3` # (Only needed once)
 3. `cd mpu6050 && make -j4`

# Connections

| SDA | I2C1 PB9    |
|-----|-------------|
| SCL | I2C1 PB8    |
| VCC | STM32F4 5V  |
| GND | STM32F4 GND |
