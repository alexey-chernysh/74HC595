/*
 * common.h
 *
 */

#ifndef COMMON_H_
#define COMMON_H_

#define MAX_CHAR 0x7F
#define MIN_CHAR 0x80

#define MAX_INT    0x7FFF
#define MIN_INT    0x8000
#define INVERTED_8 0xFFF7

#define MAX_UINT   0xFFFF

#define MAX_LONG 0x7FFFFFFF
#define MIN_LONG 0x80000000

#define MAKE_BINARY(a,b,c,d,e,f,g,h) (((a)<<7)|(b)<<6)|((c)<<5)|((d)<<4)|((e)<<3|((f)<<2)|((g<<1)|h))

#include <intrinsics.h>
#include <iostm8s105k4.h>  // подключение заголовочного файла с объявлениями регистров, масок и битов
#include <stdbool.h> // подключение заголовочного файла для ипользования типа bool

#endif /* COMMON_H_ */
