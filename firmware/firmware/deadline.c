#include "deadline.h"
bool deadline_tick(deadline_budget *d,bool missed){
 uint8_t mask=(uint8_t)(1u<<(d->index&7u));uint8_t *cell=&d->bits[d->index>>3];
 if(*cell&mask){*cell&=(uint8_t)~mask;d->window--;}
 if(missed){*cell|=mask;d->window++;d->total++;}
 if(d->window>d->window_max)d->window_max=d->window;
 if(++d->index==DEADLINE_WINDOW)d->index=0;
 return d->window>DEADLINE_MISS_LIMIT;
}
bool bucket_tick(bucket_budget *b,bool bad){
 if(bad){b->bucket[b->current]++;b->window++;b->total++;}
 if(b->window>b->window_max)b->window_max=b->window;
 bool over=b->window>DEADLINE_MISS_LIMIT;
 if(++b->ticks==BUCKET_TICKS){b->ticks=0;if(++b->current==BUCKET_COUNT)b->current=0;b->window-=b->bucket[b->current];b->bucket[b->current]=0;}
 return over;
}
