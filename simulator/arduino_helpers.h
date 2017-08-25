#ifndef ARDUINO_HELPERS_H
#define ARDUINO_HELPERS_H

/*
 * These definitions were copied from Arduino's header files for the mega board
 */

const char digital_pin_to_port[] = {
	// PORTLIST
	// -------------------------------------------
	'E'	, // PE 0 ** 0 ** USART0_RX
	'E'	, // PE 1 ** 1 ** USART0_TX
	'E'	, // PE 4 ** 2 ** PWM2
	'E'	, // PE 5 ** 3 ** PWM3
	'G'	, // PG 5 ** 4 ** PWM4
	'E'	, // PE 3 ** 5 ** PWM5
	'H'	, // PH 3 ** 6 ** PWM6
	'H'	, // PH 4 ** 7 ** PWM7
	'H'	, // PH 5 ** 8 ** PWM8
	'H'	, // PH 6 ** 9 ** PWM9
	'B'	, // PB 4 ** 10 ** PWM10
	'B'	, // PB 5 ** 11 ** PWM11
	'B'	, // PB 6 ** 12 ** PWM12
	'B'	, // PB 7 ** 13 ** PWM13
	'J'	, // PJ 1 ** 14 ** USART3_TX
	'J'	, // PJ 0 ** 15 ** USART3_RX
	'H'	, // PH 1 ** 16 ** USART2_TX
	'H'	, // PH 0 ** 17 ** USART2_RX
	'D'	, // PD 3 ** 18 ** USART1_TX
	'D'	, // PD 2 ** 19 ** USART1_RX
	'D'	, // PD 1 ** 20 ** I2C_SDA
	'D'	, // PD 0 ** 21 ** I2C_SCL
	'A'	, // PA 0 ** 22 ** D22
	'A'	, // PA 1 ** 23 ** D23
	'A'	, // PA 2 ** 24 ** D24
	'A'	, // PA 3 ** 25 ** D25
	'A'	, // PA 4 ** 26 ** D26
	'A'	, // PA 5 ** 27 ** D27
	'A'	, // PA 6 ** 28 ** D28
	'A'	, // PA 7 ** 29 ** D29
	'C'	, // PC 7 ** 30 ** D30
	'C'	, // PC 6 ** 31 ** D31
	'C'	, // PC 5 ** 32 ** D32
	'C'	, // PC 4 ** 33 ** D33
	'C'	, // PC 3 ** 34 ** D34
	'C'	, // PC 2 ** 35 ** D35
	'C'	, // PC 1 ** 36 ** D36
	'C'	, // PC 0 ** 37 ** D37
	'D'	, // PD 7 ** 38 ** D38
	'G'	, // PG 2 ** 39 ** D39
	'G'	, // PG 1 ** 40 ** D40
	'G'	, // PG 0 ** 41 ** D41
	'L'	, // PL 7 ** 42 ** D42
	'L'	, // PL 6 ** 43 ** D43
	'L'	, // PL 5 ** 44 ** D44
	'L'	, // PL 4 ** 45 ** D45
	'L'	, // PL 3 ** 46 ** D46
	'L'	, // PL 2 ** 47 ** D47
	'L'	, // PL 1 ** 48 ** D48
	'L'	, // PL 0 ** 49 ** D49
	'B'	, // PB 3 ** 50 ** SPI_MISO
	'B'	, // PB 2 ** 51 ** SPI_MOSI
	'B'	, // PB 1 ** 52 ** SPI_SCK
	'B'	, // PB 0 ** 53 ** SPI_SS
	'F'	, // PF 0 ** 54 ** A0
	'F'	, // PF 1 ** 55 ** A1
	'F'	, // PF 2 ** 56 ** A2
	'F'	, // PF 3 ** 57 ** A3
	'F'	, // PF 4 ** 58 ** A4
	'F'	, // PF 5 ** 59 ** A5
	'F'	, // PF 6 ** 60 ** A6
	'F'	, // PF 7 ** 61 ** A7
	'K'	, // PK 0 ** 62 ** A8
	'K'	, // PK 1 ** 63 ** A9
	'K'	, // PK 2 ** 64 ** A10
	'K'	, // PK 3 ** 65 ** A11
	'K'	, // PK 4 ** 66 ** A12
	'K'	, // PK 5 ** 67 ** A13
	'K'	, // PK 6 ** 68 ** A14
	'K'	, // PK 7 ** 69 ** A15
};

const int digital_pin_to_bit_mask[] = {
	// PIN IN PORT
	// -------------------------------------------
	0	, // PE 0 ** 0 ** USART0_RX
	1	, // PE 1 ** 1 ** USART0_TX
	4	, // PE 4 ** 2 ** PWM2
	5	, // PE 5 ** 3 ** PWM3
	5	, // PG 5 ** 4 ** PWM4
	3	, // PE 3 ** 5 ** PWM5
	3	, // PH 3 ** 6 ** PWM6
	4	, // PH 4 ** 7 ** PWM7
	5	, // PH 5 ** 8 ** PWM8
	6	, // PH 6 ** 9 ** PWM9
	4	, // PB 4 ** 10 ** PWM10
	5	, // PB 5 ** 11 ** PWM11
	6	, // PB 6 ** 12 ** PWM12
	7	, // PB 7 ** 13 ** PWM13
	1	, // PJ 1 ** 14 ** USART3_TX
	0	, // PJ 0 ** 15 ** USART3_RX
	1	, // PH 1 ** 16 ** USART2_TX
	0	, // PH 0 ** 17 ** USART2_RX
	3	, // PD 3 ** 18 ** USART1_TX
	2	, // PD 2 ** 19 ** USART1_RX
	1	, // PD 1 ** 20 ** I2C_SDA
	0	, // PD 0 ** 21 ** I2C_SCL
	0	, // PA 0 ** 22 ** D22
	1	, // PA 1 ** 23 ** D23
	2	, // PA 2 ** 24 ** D24
	3	, // PA 3 ** 25 ** D25
	4	, // PA 4 ** 26 ** D26
	5	, // PA 5 ** 27 ** D27
	6	, // PA 6 ** 28 ** D28
	7	, // PA 7 ** 29 ** D29
	7	, // PC 7 ** 30 ** D30
	6	, // PC 6 ** 31 ** D31
	5	, // PC 5 ** 32 ** D32
	4	, // PC 4 ** 33 ** D33
	3	, // PC 3 ** 34 ** D34
	2	, // PC 2 ** 35 ** D35
	1	, // PC 1 ** 36 ** D36
	0	, // PC 0 ** 37 ** D37
	7	, // PD 7 ** 38 ** D38
	2	, // PG 2 ** 39 ** D39
	1	, // PG 1 ** 40 ** D40
	0	, // PG 0 ** 41 ** D41
	7	, // PL 7 ** 42 ** D42
	6	, // PL 6 ** 43 ** D43
	5	, // PL 5 ** 44 ** D44
	4	, // PL 4 ** 45 ** D45
	3	, // PL 3 ** 46 ** D46
	2	, // PL 2 ** 47 ** D47
	1	, // PL 1 ** 48 ** D48
	0	, // PL 0 ** 49 ** D49
	3	, // PB 3 ** 50 ** SPI_MISO
	2	, // PB 2 ** 51 ** SPI_MOSI
	1	, // PB 1 ** 52 ** SPI_SCK
	0	, // PB 0 ** 53 ** SPI_SS
	0	, // PF 0 ** 54 ** A0
	1	, // PF 1 ** 55 ** A1
	2	, // PF 2 ** 56 ** A2
	3	, // PF 3 ** 57 ** A3
	4	, // PF 4 ** 58 ** A4
	5	, // PF 5 ** 59 ** A5
	6	, // PF 6 ** 60 ** A6
	7	, // PF 7 ** 61 ** A7
	0	, // PK 0 ** 62 ** A8
	1	, // PK 1 ** 63 ** A9
	2	, // PK 2 ** 64 ** A10
	3	, // PK 3 ** 65 ** A11
	4	, // PK 4 ** 66 ** A12
	5	, // PK 5 ** 67 ** A13
	6	, // PK 6 ** 68 ** A14
	7	, // PK 7 ** 69 ** A15
};

#endif
