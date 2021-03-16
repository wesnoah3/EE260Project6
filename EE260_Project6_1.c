#include "stm32f4xx.h"

#define RS 0x20     //0b 0000 0010
#define EN 0x80      //0b 0000 1000

void delayMs(int);
void keypad_init(void);
char keypad_getkey(void);
char hexKeys[] = {0x0,'D','F','0','*','C','9','8','7','B','6','5','4','A','3','2','1'};
char c;
void USART2_init(void);
char USART2_read(void);
void USART2_write(int);
void LCD_nibble_write(char data, unsigned char control);
void LCD_command(unsigned char command);
void LCD_data(char data);
void LCD_init(void);
void PORTS_init(void);

int main(void) {
	unsigned char key;
	keypad_init();
	USART2_init();
	LCD_init();
	
	while(1) {
		key = keypad_getkey();
		if (key != 0) {
			c = hexKeys[key];
			delayMs(1);
			USART2_write(c);
			LCD_data(c);
		}
		delayMs(350);
	}
	
}

void keypad_init(void) { //ENABLE PINS FOR KEYPAD
    RCC->AHB1ENR |=  0x04;	        /* enable GPIOC clock */
    GPIOC->MODER &= ~0x0000FFFF;    /* clear pin mode to input */
    GPIOC->PUPDR =   0x00000055;    /* enable pull up resistors for column pins */
}

void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA3 for USART2 RX */
    GPIOA->AFR[0] &= ~0xF000;
    GPIOA->AFR[0] |=  0x7000;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00C0;
    GPIOA->MODER  |=  0x0080;   /* enable alternate function for PA3 */
	
	  /* Configure PA2 for USART2_TX */
    GPIOA->AFR[0] &= ~0x0F00;
    GPIOA->AFR[0] |=  0x0700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x0030;
    GPIOA->MODER  |=  0x0020;   /* enable alternate function for PA2 */

	  /* configure UART2, Rx and TX */
	  USART2->BRR = 0x0683;       /* 9600 baud @ 16 MHz */
		//USART2->BRR = 0x008B;     /* 115200 baud @ 16 MHz */
    USART2->CR1 = 0x0004;       /* enable Rx, 8-bit data */
    USART2->CR1 |= 0x0008;      /* enable Tx, 8-bit data */	
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
			
}

char USART2_read(void) {
    while (!(USART2->SR & 0x0020)) {}   // wait until char arrives
    return USART2->DR;
}

void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (ch & 0xFF);
}

char keypad_getkey(void) {
    int row, col;
    const int row_mode[] = {0x00000100, 0x00000400, 0x00001000, 0x00004000}; /* one row is output */
    const int row_low[] =  {0x00100000, 0x00200000, 0x00400000, 0x00800000}; /* one row is low */
    const int row_high[] = {0x00000010, 0x00000020, 0x00000040, 0x00000080}; /* one row is high */

    /* check to see any key pressed */
    GPIOC->MODER = 0x00005500;      /* make all row pins output */
    GPIOC->BSRR =  0x00F00000;      /* drive all row pins low */
		delayMs(1);		
    //delay();                        /* wait for signals to settle */
    col = GPIOC->IDR & 0x000F;      /* read all column pins */
    GPIOC->MODER &= ~0x0000FF00;    /* disable all row pins drive */
    if (col == 0x000F)              /* if all columns are high */
        return 0;                       /* no key pressed */

    /* If a key is pressed, it gets here to find out which key.
     * It activates one row at a time and read the input to see
     * which column is active. */
    for (row = 0; row < 4; row++) {
        GPIOC->MODER &= ~0x0000FF00;    /* disable all row pins drive */
        GPIOC->MODER |= row_mode[row];  /* enable one row at a time */
        GPIOC->BSRR = row_low[row];     /* drive the active row low */
			  delayMs(1);
        //delay();                        /* wait for signal to settle */
        col = GPIOC->IDR & 0x000F;      /* read all columns */
        GPIOC->BSRR = row_high[row];    /* drive the active row high */
        if (col != 0x000F) break;       /* if one of the input is low, some key is pressed. */
    }
    GPIOC->BSRR = 0x000000F0;           /* drive all rows high before disable them */
    GPIOC->MODER &= ~0x0000FF00;        /* disable all rows */
    if (row == 4)
        return 0;                       /* if we get here, no key is pressed */

    /* gets here when one of the rows has key pressed, check which column it is */
    if (col == 0x000E) return row * 4 + 1;    /* key in column 0 */
    if (col == 0x000D) return row * 4 + 2;    /* key in column 1 */
    if (col == 0x000B) return row * 4 + 3;    /* key in column 2 */
    if (col == 0x0007) return row * 4 + 4;    /* key in column 3 */

    return 0;   /* just to be safe */
}

void LCD_init(void) {
	PORTS_init();
	
	delayMs(20);
	LCD_nibble_write(0x30, 0);
	delayMs(5);
	LCD_nibble_write(0x30, 0);
	delayMs(1);
	LCD_nibble_write(0x30, 0);
	delayMs(1);
	LCD_nibble_write(0x20, 0);
	delayMs(1);
	LCD_command(0x28);
	LCD_command(0x06);
	LCD_command(0x01);
	LCD_command(0x0F);
}

void PORTS_init(void) {
	RCC->AHB1ENR |= 0x06;
	
	GPIOB->MODER &= ~0x0000CC00;
	GPIOB->MODER |= 0x00004400;
	GPIOB->BSRR = 0x00800000;
	
	GPIOC->MODER &= ~0x0000FF00;
	GPIOC->MODER |= 0x00005500;
}

void LCD_nibble_write(char data, unsigned char control) {
	GPIOC->BSRR = 0x00F00000;
	GPIOC->BSRR = data & 0xF0;
	
	if (control & RS)
		GPIOB->BSRR = RS;
	else
		GPIOB->BSRR = RS << 16;
	
	GPIOB->BSRR = EN;
	delayMs(0);
	GPIOB->BSRR = EN << 16;
}

void LCD_command(unsigned char command) {
	LCD_nibble_write(command & 0xF0, 0);
	LCD_nibble_write(command << 4, 0);
	
	if (command < 4)
		delayMs(2);
	else
		delayMs(1);
}

void LCD_data(char data) {
	LCD_nibble_write(data & 0xF0, RS);
	LCD_nibble_write(data << 4, RS);
	
	delayMs(1);
}

void delayMs(int n) {
    int i;
    for (; n > 0; n--)
        for (i = 0; i < 2000; i++) ;
} 
