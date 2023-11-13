/******************************************************************************
 * This is the code for interfacing a 16x2 LCD with a I2C module using the
 * I2C protocol, and printing "HELLO" on the LCD Screen.
 *
 * Configurations are as follows:
 * PB6 - I2C1_SCL
 * PB7 - I2C1_SDA
 * (See Page 58 of the 203 Page user stm32f407vg user manual)
 *
 * For the I2C Module (PC58574T) to LCD the connections are as follows:
 * P0 - RS
 * P1 - R/W
 * P2 - E
 * P3 - Back Light
 * P4 - D4
 * P5 - D5
 * P6 - D6
 * P7 - D7
 * So, the data byte that we send through the I2C module will be mapped to the
 * LCD based on the above configuration. This will be helpful for understanding
 * how the data and commands are sent to the LCD in the LCD_Write_Data and
 * LCD_Write_Cmd function.
 *****************************************************************************/

//Header Files
#include <stm32f4xx.h>
#define ARM_MATH_CM4

uint32_t local_time, sensor_time;
double distance;

// Function to read distance from HC-SR04 sensor
#define MAX_SENSOR_TIMEOUT 10000  // Maximum timeout in microseconds (adjust as needed)

uint32_t read_distance(void) {
    local_time = 0;
    uint32_t timeout = 0;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin LOW
    HAL_Delay(2);  // wait for 2 us

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    HAL_Delay(10);  // wait for 10 us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);  // pull the TRIG pin LOW

    // Wait for the ECHO pin to go HIGH
    while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) && (timeout < MAX_SENSOR_TIMEOUT)) {
        timeout++;
    }

    // Measure the duration only if the ECHO pin is HIGH
    if (timeout < MAX_SENSOR_TIMEOUT) {
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2)) {
            local_time++;  // measure time for which the pin is HIGH
        }
    } else {
        // Timeout occurred, handle the error (return an error code or handle it as needed)
        local_time = 0;  // Set distance to 0 or another value indicating an error
    }

    return local_time;
}


// Function to initialize the LCD


void LCD_Init() {
	// Initialization code for LCD, assuming your LCD initialization code is here

}

// Function to print data on LCD
void LCD_Print(double distance) {
    // Code to convert the distance data to string and print it on the LCD
    // Assuming you have a function to convert double to string and LCD printing function
//     char buffer[20];
//     snprintf(buffer, sizeof(buffer), "%.2f", distance);
//     LCD_Print_Text(buffer);

}

int main(void) {
    // Initialize HAL and other peripherals
    HAL_Init();
    HAL_Delay(100);  // Delay for stability

    // Initialize the LCD
    LCD_Init();

    while (1) {
        sensor_time = read_distance();
        distance = ((sensor_time * 0.034) / 2) * 100;  // Speed of sound = 343 m/s

        // Print the distance on the LCD
        LCD_Print(distance);
//        LCD_Print_Text(distance);
        // Delay for a specific time before taking the next reading
        HAL_Delay(1000);  // Delay for 1 second, adjust as needed
    }
}
