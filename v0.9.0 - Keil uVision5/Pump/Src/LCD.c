#include "LCD.h"

void InitLcdSPI(struct LCD * lcd, SPI_HandleTypeDef * spi){
	lcd->SPI = spi;
}

void SpiWrite(SPI_HandleTypeDef * spi, uint8_t txData){
	HAL_SPI_Transmit(spi, &txData, sizeof(uint8_t), 100);
}

void WriteInstruction(struct LCD * lcd, uint8_t Command)
{
	// Synchronizing Bit string
	// 1 1 1 1 1 0 0 0
	uint8_t syncBitString = 0xf8;
	SpiWrite(lcd->SPI, syncBitString);
	//HAL_SPI_Transmit(&hspi1, &syncBitString, sizeof(uint8_t), 100);
	
	// High Data
	// D7 D6 D5 D4 0 0 0 0
	uint8_t highData = (Command & 0xf0);
	SpiWrite(lcd->SPI, highData);
	//HAL_SPI_Transmit(&hspi1, &highData, sizeof(highData), 100);
	
	//Low Data
	// D3 D2 D1 D0 0 0 0 0
	uint8_t lowData = Command << 4;
	SpiWrite(lcd->SPI, lowData);
	//HAL_SPI_Transmit(&hspi1, &lowData, sizeof(lowData), 100);
	
	// Display Clear Execution Time
	// 1.6ms - 1600us
	if(Command == DISPLAY_CLEAR){
		DWT_Delay_us(1600);
	}
	// Execution Time of Other Commands
	// 72us
	else
	{
		DWT_Delay_us(72);
	}
}

void WriteRam(struct LCD * lcd, uint8_t data)
{
	// Synchronizing Bit string or RS high
	// 1 1 1 1 1 0 0 0 | 0 0 0 0 0 0 1 0 = 0xfa
	uint8_t syncBitString = (0xf8 | 0x02);
	SpiWrite(lcd->SPI, syncBitString);
	//HAL_SPI_Transmit(lcd->SPI, &syncBitString, sizeof(uint8_t), 100);
	
	// High Data
	// D7 D6 D5 D4 0 0 0 0
	uint8_t highData = (data & 0xf0);
	SpiWrite(lcd->SPI, highData);
	//HAL_SPI_Transmit(&hspi1, &highData, sizeof(highData), 100);
	
	//Low Data
	// D3 D2 D1 D0 0 0 0 0
	uint8_t lowData = data << 4;
	SpiWrite(lcd->SPI, lowData);
	//HAL_SPI_Transmit(&hspi1, &lowData, sizeof(lowData), 100);
	
	// Execution Time
	// 72us
	DWT_Delay_us(72);
}

void InitDisplay(struct LCD * lcd){
	// Wait min 40ms before Init
	HAL_Delay(40);
	
	//Function set [DL=1 8-bit interface; DL=0 4-bit interface;
	//              RE=1: extended instruction; RE=0: basic instruction]
	// RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
	//  0  0   0   0   1  DL   X  RE   0   0
	WriteInstruction(lcd, FUNCTION_SET | DATA_LENGTH_DL);
	
	// Wait 100ms
	DWT_Delay_us(100);
	
	//Function set [DL=1 8-bit interface; DL=0 4-bit interface;
	//              RE=1: extended instruction; RE=0: basic instruction]
	// RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
	//  0  0   0   0   1  DL   X  RE   0   0
	WriteInstruction(lcd, FUNCTION_SET | DATA_LENGTH_DL);
	
	//Wait 37us
	DWT_Delay_us(37);
	
	// DisplayControl [D=1: Display ON; C=1: Cursor ON; B=1: Character Blink ON]
	// RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
	//  0  0   0   0   0   0   1   D   C   B
	WriteInstruction(lcd, DISPLAY_CONTROL | DISPLAY_ON_D);
	
	// Wait 100us
	DWT_Delay_us(100);
	
	// Display Clear
	WriteInstruction(lcd, 0x01);
	
	// Wait 10ms
	HAL_Delay(10);
	
	//Set cursor position and display shift when doing write or read operation
	// RS RW DB7 DB6 DB5 DB4 DB3 DB2 DB1 DB0
	//  0  0   0   0   0   0   0   1 I/D   S
	WriteInstruction(lcd, ENTRY_MODE_SET | INCREASE_DECREASE_ID);
}

void SetGraphicsMode(struct LCD * lcd) {
	WriteInstruction(lcd, EXTENDED_FUNCTION_SET | DATA_LENGTH_DL);
	WriteInstruction(lcd, EXTENDED_FUNCTION_SET | DATA_LENGTH_DL | EXTENDED_INSTRUCTION_RE); //RE=1 (Extended funtion set)
	WriteInstruction(lcd, EXTENDED_FUNCTION_SET | DATA_LENGTH_DL | EXTENDED_INSTRUCTION_RE | GRAPHIC_ON_G);
}

void SetTextMode(struct LCD * lcd){
	// RE=0 (Basic funtion set)
	WriteInstruction(lcd, FUNCTION_SET | DATA_LENGTH_DL);
}

void ClearScreen(struct LCD * lcd){
	WriteInstruction(lcd, FUNCTION_SET | DATA_LENGTH_DL); // RE=0 (Basic funtion set)
	WriteInstruction(lcd, DISPLAY_CLEAR);
}

void ReturnHome(struct LCD * lcd){
	WriteInstruction(lcd, FUNCTION_SET | DATA_LENGTH_DL); //RE=0 (Basic funtion set)
	WriteInstruction(lcd, RETURN_HOME);
}

void Standby(struct LCD * lcd){
	WriteInstruction(lcd, EXTENDED_FUNCTION_SET | DATA_LENGTH_DL | EXTENDED_INSTRUCTION_RE); //RE=1 (Extended funtion set)
	WriteInstruction(lcd, STANDBY);
}

// Text
void DisplayStringLeftAlligned(struct LCD * lcd, int Row, int Column, unsigned char *ptr, int length)
{
	switch (Row){
		case 0:
			WriteInstruction(lcd, 0x80 | (Column/2));
			break;
		case 1:
			WriteInstruction(lcd, 0x90 | (Column/2));
			break;
		case 2:
			WriteInstruction(lcd, 0x88 | (Column/2));
			break;
		case 3:
			WriteInstruction(lcd, 0x98 | (Column/2));
			break;
		default:
			WriteInstruction(lcd, 0x80);
			break;
			
	}

	if (Column%2!=0)
		WriteRam(lcd, ' ');

	for (int i=0; i<length; i++) {
		WriteRam(lcd, (unsigned int)ptr[i]);
	}
}

void DisplayStringRightAlligned(struct LCD * lcd, int Row, int Column, unsigned char *ptr, int length)
{
		
		Column = Column - length + 1;
		
		switch (Row){
		case 0:
			WriteInstruction(lcd, 0x80 | (Column)/2);
			break;
		case 1:
			WriteInstruction(lcd, 0x90 | (Column/2));
			break;
		case 2:
			WriteInstruction(lcd, 0x88 | (Column/2));
			break;
		case 3:
			WriteInstruction(lcd, 0x98 | (Column/2));
			break;
		default:
			WriteInstruction(lcd, 0x80);
			break;
			
	}

	if (Column%2!=0)
		WriteRam(lcd, ' ');

	for (int i=0; i<length; i++) {
		WriteRam(lcd, (unsigned int)ptr[i]);
	}
}

void DisplayChar(struct LCD * lcd, int Row, int Column, unsigned char inpChr){
	
	int i=0;
 
	switch (Row) {
			case 0:
					Column =0x80 | (Column/2); // SET_DDRAM_ADDRESS
					break;
			case 1:
					Column =0x90 | (Column/2);
					break;
			case 2:
					Column =0x88 | (Column/2);
					break;
			case 3:
					Column =0x98 | (Column/2);
					break;
			default:
					Column=0x80;
					break;
	}
	
	if (Column%2!=0) {
			Column-=1;
			i=1;
	}
	
	WriteInstruction(lcd, (unsigned int)Column);

	if (i==1) {
			WriteRam(lcd, ' ');
	}
	WriteRam(lcd, (unsigned int)inpChr);
}

// Graphics
void FillGDRAM(struct LCD * lcd, unsigned char * bitmap){
	unsigned char i, j, k ;
 
	for ( i = 0 ; i < 2 ; i++ ) {
			for ( j = 0 ; j < 32 ; j++ ) {
					WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | j) ;
					if ( i == 0 ) {
							WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS) ;
					} else {
							WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x08) ;
					}
					for ( k = 0 ; k < 16 ; k++ ) {
							WriteRam(lcd, *bitmap++ ) ;
					}
			}
	}
}

void FillGDRAM_Turned(struct LCD * lcd, unsigned char * bitmap){
	int i, j, k, m, offset_row, mask ;
	unsigned char data;

	for ( i = 0 ; i < 2 ; i++ ) { //upper and lower page
			for ( j = 0 ; j < 32 ; j++ ) { //32 lines per page
					WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | j) ;
					if ( i == 0 ) {
							WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS) ;
					} else {
							WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x08) ;
					}
					mask=1<<(j%8); // extract bitnumber
					//printf("mask: %d\r\n",mask);
					for ( k = 0 ; k < 16 ; k++ ) { //16 bytes per line
							offset_row=((i*32+j)/8)*128 + k*8; //y coordinate/8 = row 0-7 * 128 = byte offset, read 8 bytes
							data=0;
							for (m = 0 ; m < 8 ; m++) { // read 8 bytes from source

									if ((bitmap[offset_row+m] & mask)) { //pixel = 1
											data|=(128>>m);
									}
							}
							WriteRam(lcd, data) ;
					}
			}
	}
}

void ClearGDRAM(struct LCD * lcd){
	unsigned char i, j, k ;
 
	for ( i = 0 ; i < 2 ; i++ ) {
			for ( j = 0 ; j < 32 ; j++ ) {
					WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | j) ;
					if ( i == 0 ) {
							WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS) ;
					} else {
							WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x08) ;
					}
					for ( k = 0 ; k < 16 ; k++ ) {
							WriteRam(lcd, 0);
					}
			}
	}
}

void DivideHorizontal(struct LCD * lcd){
	for (uint8_t x = 0; x < 8; x++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 31 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);

		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | (x | 0x88));
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);
	}
}

void DivideVertical(struct LCD * lcd){
	for (uint8_t y = 0; y < 32; y++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);

		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03 | 0x88);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);
	}
}

void DivideQuadrant(struct LCD * lcd){
	for (uint8_t y = 0; y < 31; y++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);
	}
	for (uint8_t y = 1; y < 32; y++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03 | 0x88);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);
	}
	for (uint8_t x = 0; x < 8; x++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 31 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);

		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | (x | 0x88));
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);
	}
}

void DivideT(struct LCD * lcd){
	for (uint8_t y = 1; y < 32; y++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03 | 0x88);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);
	}
	for (uint8_t x = 0; x < 8; x++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 31 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);

		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | (x | 0x88));
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);
	}
}

void DivideInverseT(struct LCD * lcd){
	for (uint8_t y = 0; y < 31; y++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);
	}
	for (uint8_t x = 0; x < 8; x++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 31 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);

		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | (x | 0x88));
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);
	}
}

void DivideHalfInverseT(struct LCD * lcd){
	for (uint8_t y = 16; y < 31; y++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03);
		WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x01);
		WriteRam(lcd, 0x80);
	}
	for (uint8_t x = 0; x < 8; x++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 31 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);

		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0 );
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | (x | 0x88));
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFF);
	}
}

void HighlightTopLeftText(LCD * lcd){
	// Top Left Highlight
	for(uint8_t y = 0; y < 16; y++){
		for (uint8_t x = 0; x < 3; x++) {
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
			WriteRam(lcd, 0xFF);
			WriteRam(lcd, 0xFF);
		}
	}
	for(uint8_t y = 0; y < 16; y++){
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x03);
		WriteRam(lcd, 0xFF);
		WriteRam(lcd, 0xFE);
	}
}

void HighlightTopRightText(LCD * lcd){
	// Top Right Highlight
	for(uint8_t y = 0; y < 16; y++){
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | 0x04);
		WriteRam(lcd, 0x7F);
		WriteRam(lcd, 0xFF);
	}
	for(uint8_t y = 0; y < 16; y++){
		for (uint8_t x = 5; x < 8; x++) {
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x);
			WriteRam(lcd, 0xFF);
			WriteRam(lcd, 0xFF);
		}
	}
}

void HighlightBottomText(LCD * lcd){
	for(uint8_t y = 0; y < 16; y++){
		for (uint8_t x = 0; x < 8; x++) {
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
			WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | x | 0x88);
			WriteRam(lcd, 0xFF);
			WriteRam(lcd, 0xFF);
		}
	}
}

void HighlightMenuItem(LCD * lcd, uint8_t idx, bool fill){
	idx &= 0x03;
	
	uint8_t y = idx * 16;
	uint8_t x_addr = 0x80;
	
	// adjust coordinates and address
	if (y >= 32) {
		y -= 32;
		x_addr = 0x88; // Page 2
	}

	for (uint8_t x = 0; x < 8; x++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y);
		WriteInstruction(lcd, x_addr | x);
		fill ? WriteRam(lcd, 0xFF) : WriteRam(lcd, 0x00);
		fill ? WriteRam(lcd, 0xFF) : WriteRam(lcd, 0x00);
		
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y + 15);
		WriteInstruction(lcd, x_addr | x);
		fill ? WriteRam(lcd, 0xFF) : WriteRam(lcd, 0x00);
		fill ? WriteRam(lcd, 0xFF) : WriteRam(lcd, 0x00);
	}
	
	for (uint8_t y1 = y + 1; y1 < y + 15; y1++) {
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y1);
		WriteInstruction(lcd, x_addr);
		fill ? WriteRam(lcd, 0x80) : WriteRam(lcd, 0x00);
		WriteRam(lcd, 0x00);
		
		WriteInstruction(lcd, SET_GRAPHIC_RAM_ADDRESS | y1);
		WriteInstruction(lcd, x_addr + 7);
		WriteRam(lcd, 0x00);
		fill ? WriteRam(lcd, 0x01) : WriteRam(lcd, 0x00);
	}
}
