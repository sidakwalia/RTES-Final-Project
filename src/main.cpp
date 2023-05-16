#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "drivers/stm32f429i_discovery_ts.h"

// Initialize LCD.
LCD_DISCO_F429ZI lcd;

// Initialize TouchScreen.
TS_StateTypeDef ts_state;

// Function to display numbers.
void displayNumbers() {
    char str[12];
    lcd.Clear(LCD_COLOR_WHITE);
    for(int i = 1; i <= 5; i++) {
        sprintf(str, "%d", i);
        lcd.DisplayStringAt(0, LINE(i), (uint8_t *)str, CENTER_MODE);
    }
}

// Function to check the input number based on y-coordinate of touch.
int checkInputNumber(int y) {
    int line_height = lcd.GetYSize() / 5;
    printf("Line height: %d\n", line_height);
    int number = (y / line_height) + 1;
    return number;
}

int main() {
    // Initialize the TouchScreen.
    BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());

    displayNumbers();

    while (1) {
        BSP_TS_GetState(&ts_state);
       if (ts_state.TouchDetected) {
    printf("Touch detected at X: %d, Y: %d\n", ts_state.X, ts_state.Y);
    int inputNumber = checkInputNumber(ts_state.Y);
    printf("Input number: %d\n", inputNumber);
    wait_us(2000000); // Wait for 2 seconds to avoid multiple detections
}
    }
}
