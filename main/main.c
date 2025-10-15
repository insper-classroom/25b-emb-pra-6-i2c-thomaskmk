#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"

#include "Fusion.h"
#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 16;
const int I2C_SCL_GPIO = 17;

QueueHandle_t xQueuePos;

typedef struct adc {
    int axis;
    int val;
} adc_t;


static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p) {
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    
    while (true) { 

        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };      

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);


        adc_t data;

        // Pitch: X
        float X = euler.angle.pitch;
        if (X > -8 && X < 8 ) X = 0;
        
        data.axis = 0;
        data.val = (int)X;

        //printf("X: %f\n", X);

        if (X) xQueueSend(xQueuePos, &data, 0);

        // Roll: Y
        float Y = euler.angle.roll;
        if (Y > -8 && Y < 8) Y = 0;

        data.axis = 1;
        data.val = (int)Y; 

        if (Y) xQueueSend(xQueuePos, &data, 0);

        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    adc_t data;

    while (1) {
        if (xQueueReceive(xQueuePos, &data, pdMS_TO_TICKS(100))) {
            uart_putc_raw(uart0, data.axis);
            uart_putc_raw(uart0, data.val);
            uart_putc_raw(uart0, data.val >> 8);
            uart_putc_raw(uart0, -1);
        }        
    }
}

int main() {
    stdio_init_all();

    xQueuePos = xQueueCreate(3, sizeof(adc_t));
    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true)
        ;
}
