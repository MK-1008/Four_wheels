#include "key_ctrl.h"
#include "main.h"

#define DEBOUNCE_DELAY 30 // 定义消抖延迟为30ms
enum ControlMode controlMode = MANUAL;
static uint8_t lastButtonState = 0;
uint8_t reading;

//enum ControlMode getControlMode() {
//    return controlMode;
//}

	
void read_button_state(void) 
{
    static uint32_t lastDebounceTime = 0;
    static uint8_t buttonState = 0;

    reading = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15);

    if (reading != lastButtonState) {
        lastDebounceTime = HAL_GetTick();
    }

    if ((HAL_GetTick() - lastDebounceTime) > DEBOUNCE_DELAY) {
        if (reading != buttonState) {
            buttonState = reading;

            if (buttonState == GPIO_PIN_SET) { // Assuming low means pressed
                controlMode = (controlMode == MANUAL) ? AUTOMATIC : MANUAL;

                if (controlMode == AUTOMATIC) {
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); // Turn on LED
                } else {
                    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);   // Turn off LED
                }
            }
        }
    }

    lastButtonState = reading;
}
