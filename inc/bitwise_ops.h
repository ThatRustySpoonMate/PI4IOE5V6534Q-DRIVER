/*
 * bitwise_ops.h
 *
 *  Created on: 18 Jul. 2023
 *      Author: Ethan
 */

#ifndef INC_BITWISE_OPS_H_
#define INC_BITWISE_OPS_H_

#define bitset(byte,nbit)   ((byte) |=  (1U<<(nbit)))
#define bitclear(byte,nbit) ((byte) &= ~(1U<<(nbit)))
#define bitflip(byte,nbit)  ((byte) ^=  (1U<<(nbit)))
#define bitcheck(byte,nbit) ((byte) &   (1U<<(nbit))) // Puts value of bit at pos nbit into byte

/* Assumes a 16 bit value */
#define LOWBYTE(x)   ((unsigned char) (x))
#define HIGHBYTE(x)  ((unsigned char) (((unsigned int) (x)) >> 8))

#endif /* INC_BITWISE_OPS_H_ */
