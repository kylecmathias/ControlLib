#ifndef CLBBITS_HPP
#define CLBBITS_HPP

/*
To write to a bit using the bits macro, use BIT0 << BIT_MACRO


*/

#define BIT0 0b00000001
#define BIT1 0b00000010
#define BIT2 0b00000100
#define BIT3 0b00001000
#define BIT4 0b00010000
#define BIT5 0b00100000
#define BIT6 0b01000000
#define BIT7 0b10000000

//probably unneeded but just in case for 16 bit registers
#define BIT8 BIT0 << 8
#define BIT9 BIT1 << 8
#define BIT10 BIT2 << 8
#define BIT11 BIT3 << 8
#define BIT12 BIT4 << 8
#define BIT13 BIT5 << 8
#define BIT14 BIT6 << 8
#define BIT15 BIT7 << 8

#endif