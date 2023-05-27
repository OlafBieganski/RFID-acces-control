#include "keypad.h"

void keypad_init(){
	for(uint8_t l_row = 0; l_row < NUM_OF_ROWS; l_row++)
		HAL_GPIO_WritePin(row_ports_array[l_row],
						  row_pins_array[l_row],
						  GPIO_PIN_SET);
}

int8_t keypad_scan_pressed_index(){
	for(uint8_t l_row = 0; l_row < NUM_OF_ROWS; l_row++){
		HAL_GPIO_WritePin(row_ports_array[l_row],
						  row_pins_array[l_row],
						  GPIO_PIN_RESET);
		for(uint8_t l_col = 0; l_col < NUM_OF_COLUMNS; l_col++){
			if(HAL_GPIO_ReadPin(column_ports_array[l_col],
								column_pins_array[l_col]) == GPIO_PIN_RESET){
				return (l_row * 3 + l_col);
			}
		}
	}
	return NO_BUTTON_PRESSED;
}

char keypad_index_to_char(int8_t index){
	if(index < 0 || index > 11) return KEYPAD_ERROR_CHAR;
	else return keypad_index_char_array[index];
}





