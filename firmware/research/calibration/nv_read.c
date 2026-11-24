#include "nv_read.h"
static uint16_t u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static uint32_t u32(const uint8_t *p) { return (uint32_t)u16(p)|((uint32_t)u16(p+2)<<16); }
static bool is_active(const uint8_t *p) { return u32(p)==0xABABu && u32(p+4)==0; }
bool xl330_nv_open(xl330_nv_snapshot *s,const uint8_t *p,size_t size) {
    if (!s) return false;
    *s=(xl330_nv_snapshot){0};
    if (!p || size<4096) return false;
    if (!is_active(p)) { if (!is_active(p+2048)) return false; s->bank=1; }
    p+=s->bank*2048u;
    if (u32(p+8)>65535u || u32(p+12)!=0) return false;
    s->generation=u16(p+8);
    for (unsigned offset=16;offset<2048;offset+=8) {
        const uint32_t word=u32(p+offset), upper=u32(p+offset+4);
        if (word==UINT32_MAX && upper==UINT32_MAX) continue;
        const uint16_t key=u16(p+offset);
        if (key>=64 || upper!=0 || (word>>24)!=0) return false;
        s->bytes[key]=(uint8_t)(word>>16);
        s->present|=(uint64_t)1<<key;
    }
    s->ready=true;
    return true;
}
bool xl330_nv_get(const xl330_nv_snapshot *s,uint8_t key,uint8_t *value) {
    if (!s || !s->ready || key>=64 || !value) return false;
    *value=s->bytes[key];
    return true;
}
bool xl330_nv_config_read(const xl330_nv_snapshot *s,xl330_nv_config *c) {
    static const uint8_t keys[]={7,8,10,11,20,21,22,23,36,37,38,39};
    if (!s || !s->ready || !c) return false;
    for (unsigned i=0;i<sizeof keys/sizeof keys[0];++i)
        if (!(s->present&((uint64_t)1<<keys[i]))) return false;
    const uint8_t mode=s->bytes[11],drive=s->bytes[10];
    if (s->bytes[7]>252 || s->bytes[8]>7 || (drive&~13u)!=0 ||
        !(mode==0 || mode==1 || mode==3 || mode==4 || mode==5 || mode==16)) return false;
    const uint32_t home=u32(s->bytes+20);
    const int64_t signed_home=home<=INT32_MAX ? (int64_t)home : (int64_t)home-INT64_C(4294967296);
    *c=(xl330_nv_config){
        .homing_offset_raw=(int32_t)signed_home,.pwm_limit=u16(s->bytes+36),
        .current_limit=u16(s->bytes+38),.id=s->bytes[7],.baud_index=s->bytes[8],
        .drive_mode=drive,.operating_mode=mode};
    return true;
}
