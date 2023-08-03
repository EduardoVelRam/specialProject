#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdlib.h>

#define SLEEP_TIME_MS   1000

#define ICP20100_I2C_ADDRESS   0x63 /* p. 20 Datasheet*/
#define ICP20100_TEMP_REGISTER 0xFD /* p. 43 Datasheet*/
#define ICP20100_PRES_REGISTER 0XFA /* p. 43 Datasheet*/

#define I2C_NODE DT_NODELABEL(i2c0)
static const struct device *i2c0_dev = DEVICE_DT_GET(I2C_NODE);

static uint8_t i2c_buffer[2];  /* Probablemente deba subirse el espacio de memoria */

float icp20100_temp(uint8_t teemp){
    float temperature = (temp&0x0F)(float)*65/262144+25; /* p. 35 Datasheet*/
    return temperature;
}

float icp20100_pres(uint8_t pres){
    float pressure = (pres&0x0F)*40/131072+(float);  /* p. 34 Datasheet*/
    return pressure;
}

void main(void){

    int err;

    if (!device_is_ready(i2c_dev)){
        printk("i2c_dev not ready\n");
        return;
    }

    while (true) {

        i2c_buffer[1]=ICP20100_TEMP_REGISTER;  /* Revisar */
		i2c_buffer[2]=ICP20100_PRES_REGISTER;  /* Revisar */

        do{
            err = i2c_read(i2c_dev, i2c_buffer, 3, ICP20100_I2C_ADDRESS); /* Son 3 bytes porque se leen 20 bits */
            if (err < 0){ printk("ICP20100 read failed: %d\n", err); break; }

			float temperature = icp20100_temp(i2c_buffer[0]);
            printk("ICP20100: %.2f Cel \n", temperature);


			err = i2c_read(i2c_dev, i2c_buffer, 3, ICP20100_I2C_ADDRESS); /* Igual se leen 20 bits */
            if (err < 0){ printk("ICP20100 read failed: %d\n", err); break; }

			float pressure = icp20100_temp(i2c_buffer[1]);
            printk("ICP20100: %.2f KPa \n", pressure);

        }while(false);
        k_msleep(SLEEP_TIME_MS)
    }
}