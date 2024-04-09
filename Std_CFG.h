/*
 * File Name: Std_CFG.h
 * Version  : V 1.0
 * Created	:  Apr 9, 2024
 * Author	: Ahmed Alsikely
 */

#ifndef STD_CFG_H_
#define STD_CFG_H_

/*---------------------------------- Defines ------------------------------------------*/
#define  NULL	((void *)0)

/**************************************** Macros Functions *****************************************/
#define SET_BIT(REG,BIT)   ((REG) |= (0x1UL << BIT))
#define CLEAR_BIT(REG,BIT) ((REG) &= ~(0x1UL << BIT))
#define TOGGLE_BIT(REG,BIT) ((REG) ^= (0x1UL << BIT))
/*--------------------------------- Data Types ----------------------------------------*/

typedef unsigned char      uint8_t ;
typedef unsigned char      char_t ;
typedef signed char        sint8_t ;
typedef unsigned short     uint16_t;
typedef signed short int   sint16_t;
typedef unsigned long int  uint32_t;
typedef signed long int    sint32_t;
typedef unsigned long long uint64_t;
typedef signed long long   sint64_t;
typedef float 			   float32_t;
typedef double             float64_t;

typedef enum{
	R_OK,
	R_NOK,
	NULL_POINTER,
	WRONG_PARAMETER,
	NOT_READY,
	WrongPLLM_PARAMETER,
	WrongPLLN_PARAMETER,
	WrongPLLP_PARAMETER,
	WrongPLLQ_PARAMETER,
	WrongPLLSRC_PARAMETER,
}Status_t;




#endif /* STD_CFG_H_ */