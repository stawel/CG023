#ifndef XN_DEBUG_H
#define XN_DEBUG_H

#include <stdint.h>

void xn_debug_init();
void xn_debug_setchannel(uint8_t channel);

void xn_debug_print_string(char * ptr);
void xn_debug_print_long(long x);
void xn_debug_print_u8(uint8_t x);
void xn_debug_print_float(float x);
void xn_debug_printnl();

void xn_debug_irq_handler(uint8_t status);

#ifdef ENABLE_DEBUG

#define xn_debug_print(x) _Generic((x), unsigned long: xn_debug_print_long, char *: xn_debug_print_string, uint8_t: xn_debug_print_u8, float: xn_debug_print_float) (x)

#define LogDebug_();
#define LogDebug_1(x, args...) xn_debug_print(x);
#define LogDebug_2(x, args...) xn_debug_print(x); LogDebug_1(args);
#define LogDebug_3(x, args...) xn_debug_print(x); LogDebug_2(args);
#define LogDebug_4(x, args...) xn_debug_print(x); LogDebug_3(args);
#define LogDebug_5(x, args...) xn_debug_print(x); LogDebug_4(args);
#define LogDebug_6(x, args...) xn_debug_print(x); LogDebug_5(args);
#define LogDebug_7(x, args...) xn_debug_print(x); LogDebug_6(args);
#define LogDebug_8(x, args...) xn_debug_print(x); LogDebug_7(args);
#define LogDebug_9(x, args...) xn_debug_print(x); LogDebug_8(args);
#define LogDebug_10(x, args...) xn_debug_print(x); LogDebug_9(args);
#define LogDebug_11(x, args...) xn_debug_print(x); LogDebug_10(args);
#define LogDebug_12(x, args...) xn_debug_print(x); LogDebug_11(args);
#define LogDebug_13(x, args...) xn_debug_print(x); LogDebug_12(args);
#define LogDebug_14(x, args...) xn_debug_print(x); LogDebug_13(args);
#define LogDebug_15(x, args...) xn_debug_print(x); LogDebug_14(args);


#define _NUM_ARGS2(X,X64,X63,X62,X61,X60,X59,X58,X57,X56,X55,X54,X53,X52,X51,X50,X49,X48,X47,X46,X45,X44,X43,X42,X41,X40,X39,X38,X37,X36,X35,X34,X33,X32,X31,X30,X29,X28,X27,X26,X25,X24,X23,X22,X21,X20,X19,X18,X17,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define NUM_ARGS(...) _NUM_ARGS2(0, __VA_ARGS__ ,64,63,62,61,60,59,58,57,56,55,54,53,52,51,50,49,48,47,46,45,44,43,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

#define LogDebug_N2(N, args...) LogDebug_ ## N(args)
#define LogDebug_N(N, args...) LogDebug_N2(N, args)
#define LogDebug(args...) do { LogDebug_N(NUM_ARGS(args), args); xn_debug_printnl(); } while(0)
#define LogDebug2(args...) do { LogDebug_N(NUM_ARGS(args), args); } while(0)

#define runDebug(x) x

#else

#define LogDebug(...)
#define LogDebug2(...)
#define runDebug(x)

#endif


#endif // XN_DEBUG_H
