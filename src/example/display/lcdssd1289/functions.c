#include "functions.h"

void uint16tostr(char buf[], uint32_t d, uint8_t base);

void uint16tostr(char buf[], uint32_t d, uint8_t base)
{
    uint32_t div = 1;
    uint8_t index = 0;
    while (d/div >= base) div *= base;
    while (div != 0)
    {
            uint32_t num = d/div;
            d = d%div;
            div /= base;
            if (num > 9)
                    buf[index] = (num-10) + 65;
            else
                    buf[index] = num + 48;
            index++;
    }
    buf[index] = 0;	// end of string
}
