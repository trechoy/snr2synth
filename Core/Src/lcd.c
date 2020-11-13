#include "lcd.h"

void lcd_changeColor(char color, UART_HandleTypeDef huart)
{
	unsigned char start[] = {0x7C, 0x2B};
	HAL_UART_Transmit(&huart, (uint8_t *)start, 2, 20);
	if (color == 'r') {
		unsigned char red[] = {0xff, 0x0f, 0x0f};
		HAL_UART_Transmit(&huart, (uint8_t *) red, 3, 20);
	}
	else if (color == 'g') {
		unsigned char grn[] = {0x00, 0x0ff, 0x00};
		HAL_UART_Transmit(&huart, (uint8_t *) grn, 3, 20);
	}
	else if (color == 'b') {
		unsigned char blue[] = {0x00, 0xaf, 0xff};
		HAL_UART_Transmit(&huart, (uint8_t *) blue, 3, 20);
	}
	else if (color == '0')
	{
		unsigned char off[] = {0x00, 0x00, 0x00};
		HAL_UART_Transmit(&huart, (uint8_t *) off, 3, 20);
	}
	else {
		unsigned char wht[] = {0xff, 0xff, 0xff};
		HAL_UART_Transmit(&huart, (uint8_t *) wht, 3, 20);
	}
}

void lcd_changeColorRGB(unsigned char r, unsigned char g, unsigned char b, UART_HandleTypeDef huart)
{
	unsigned char * msg[] = {0x7C, 0x2B, r, g, b};
	HAL_UART_Transmit(&huart, (uint8_t *) msg, 5, 10);
}

void lcd_clear(UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t *) "|-", strlen("|-"), 15);
}

void lcd_showMessage(char * msg, UART_HandleTypeDef huart)
{
	lcd_clear(huart);
	HAL_UART_Transmit(&huart, (uint8_t *) msg , strlen(msg), 50);
}

void lcd_appendMessage(char * msg, UART_HandleTypeDef huart)
{
	HAL_UART_Transmit(&huart, (uint8_t *) msg , strlen(msg), 50);
}
