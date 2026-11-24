#include "platform.h"
#include <limits.h>
#include <stdio.h>
static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { fprintf(stderr,"FAIL line%d: %s\n",__LINE__,#x); return 1; } } while(0)
int main(void) {
    platform_time_state s;
    platform_time_reset(&s,0x00fffff0u);
    CHECK(platform_time_advance(&s,0x00ffffb0u)==1);
    CHECK(platform_time_advance(&s,0x00ffff71u)==1);
    CHECK(platform_time_advance(&s,0x00ffff70u)==2);
    CHECK(s.remainder_cycles==0);
    /* Downcounter wrap: 8 -> FFFFC8 is64 elapsed CPU cycles. */
    platform_time_reset(&s,8);
    CHECK(platform_time_advance(&s,0x00ffffc8u)==1);
    CHECK(platform_time_advance(&s,0x12ffff88u)==2); /* counter upper bits ignored */
    /* Distinct serialized callers share fractional cycles without losing time. */
    platform_time_reset(&s,1000);
    CHECK(platform_time_advance(&s,969)==0);
    CHECK(platform_time_advance(&s,936)==1);
    CHECK(platform_time_advance(&s,920)==1);
    CHECK(platform_time_advance(&s,872)==2);
    s.microseconds=UINT32_MAX;
    CHECK(platform_time_advance(&s,808)==0);
    CHECK(platform_time_advance(&s,808)==0); /* repeated reads add nothing */
    platform_time_reset(&s,0x00ffffffu);
    CHECK(platform_time_advance(&s,0)==262143);
    CHECK(s.remainder_cycles==63);
    CHECK(platform_time_advance(&s,0x00ffffffu)==262144);
    printf("platform time: %u checks PASS; wrap/remainder/serialized callers/uint32 wrap\n",checks);
    return 0;
}
