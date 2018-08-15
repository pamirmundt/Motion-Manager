#ifndef LCD_H
#define LCD_H

#include "stm32f3xx_hal.h"
#include <stdbool.h>

// Instruction Set 1: (RE=0: Basic Instruction)
#define DISPLAY_CLEAR           0x01 // Fill DDRAM with "20H" and set DDRAM address counter (AC) to "00H"
#define RETURN_HOME             0x02 // Set DDRAM address counter (AC) to "00H", and put cursor
// to origin &#65533;Gthe content of DDRAM are not changed
#define ENTRY_MODE_SET          0x04 // Set cursor position and display shift when doing write or read
// operation
#define DISPLAY_CONTROL         0x08 // D=1: Display ON, C=1: Cursor ON, B=1: Character Blink ON
#define CURSOR_DISPLAY_CONTROL  0x10 // Cursor position and display shift control; the content of
// DDRAM are not changed
#define FUNCTION_SET            0x20 // DL=1 8-bit interface, DL=0 4-bit interface
// RE=1: extended instruction, RE=0: basic instruction
#define SET_CGRAM_ADDRESS       0x40 // Set CGRAM address to address counter (AC)
// Make sure that in extended instruction SR=0
#define SET_DDRAM_ADDRESS       0x80 // Set DDRAM address to address counter (AC)
// AC6 is fixed to 0
 
// Instruction set 2: (RE=1: extended instruction)
#define STANDBY                 0x01 // Enter standby mode, any other instruction can terminate.
// COM1&#65533;c32 are halted
#define SCROLL_OR_RAM_ADDR_SEL  0x02 // SR=1: enable vertical scroll position
// SR=0: enable CGRAM address (basic instruction)
#define REVERSE_BY_LINE         0x04 // Select 1 out of 4 line (in DDRAM) and decide whether to
// reverse the display by toggling this instruction
// R1,R0 initial value is 0,0
#define EXTENDED_FUNCTION_SET   0x20 // DL=1 :8-bit interface, DL=0 :4-bit interface
// RE=1: extended instruction, RE=0: basic instruction
#define SET_SCROLL_ADDRESS      0x40 // G=1 :graphic display ON, G=0 :graphic display OFF
#define SET_GRAPHIC_RAM_ADDRESS 0x80 // Set GDRAM address to address counter (AC)
// Set the vertical address first and followed the horizontal
// address by consecutive writings
// Vertical address range: AC5&#65533;cAC0, Horizontal address range: AC3&#65533;cAC0
 
// Parameters regarding Instruction Sets 1 & 2
#define DISPLAY_SHIFT_S         0x01 // Set 1, ENTRY_MODE_SET
#define INCREASE_DECREASE_ID    0x02 // Set 1, ENTRY_MODE_SET
#define CURSOR_BLINK_ON_B       0x01 // Set 1, DISPLAY_CONTROL
#define CURSOR_ON_C             0x02 // Set 1, DISPLAY_CONTROL
#define DISPLAY_ON_D            0x04 // Set 1, DISPLAY_CONTROL
#define SHIFT_RL                0x04 // Set 1, CURSOR_DISPLAY_CONTROL
#define CURSOR_SC               0x08 // Set 1, CURSOR_DISPLAY_CONTROL
#define EXTENDED_INSTRUCTION_RE 0x04 // Set 1, FUNCTION_SET; Set 2, EXTENDED_FUNTION_SET
#define DATA_LENGTH_DL          0x10 // Set 1, FUNCTION_SET; Set 2, EXTENDED_FUNTION_SET
#define REVERSE_BY_LINE_R0      0x01 // Set 2, REVERSE_BY_LINE
#define REVERSE_BY_LINE_R1      0x02 // Set 2, REVERSE_BY_LINE
#define EN_VERTICAL_SCROLL_SR   0x01 // Set 2, SCROLL_OR_RAM_ADDR_SEL
#define GRAPHIC_ON_G            0x02 // Set 2, EXTENDED_FUNTION_SET
 
#define BUSY_FLAG_BF            0x80

typedef struct LCD
{
	SPI_HandleTypeDef * SPI;
} LCD;

void InitLcdSPI(struct LCD * lcd, SPI_HandleTypeDef * spi);

void SpiWrite(SPI_HandleTypeDef * spi, uint8_t txData);

void WriteInstruction(struct LCD * lcd, uint8_t Command);

void WriteRam(struct LCD * lcd, uint8_t data);

void InitDisplay(struct LCD * lcd);

void SetGraphicsMode(struct LCD * lcd);

void SetTextMode(struct LCD * lcd);

void ClearScreen(struct LCD * lcd);

void ReturnHome(struct LCD * lcd);

void Standby(struct LCD * lcd);

// Text
void DisplayStringLeftAlligned(struct LCD * lcd, int Row, int Column, unsigned char *ptr, int length);

void DisplayStringRightAlligned(struct LCD * lcd, int Row, int Column, unsigned char *ptr, int length);

void DisplayChar(struct LCD * lcd, int Row, int Column, unsigned char inpChr);

// Graphics
void FillGDRAM(struct LCD * lcd, unsigned char * bitmap);

void FillGDRAM_Turned(struct LCD * lcd, unsigned char * bitmap);

void ClearGDRAM(struct LCD * lcd);

void DivideHorizontal(struct LCD * lcd);

void DivideVertical(struct LCD * lcd);

void DivideQuadrant(struct LCD * lcd);

void DivideT(struct LCD * lcd);

void DivideInverseT(struct LCD * lcd);

void DivideHalfInverseT(struct LCD * lcd);

void HighlightTopLeftText(LCD * lcd);

void HighlightTopRightText(LCD * lcd);

void HighlightBottomText(LCD * lcd);

void HighlightMenuItem(LCD * lcd, uint8_t idx, bool fill);

#endif

