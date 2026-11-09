
#include "heading_controller.h"
#include <math.h>
#include <stdio.h>
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL %d\n",__LINE__);}}while(0)
int main(void){
 unsigned gaps[]={0,1,99,100,101,150,300,499,500,501};float c,e;
 for(unsigned i=0;i<10;i++){
  HeadingHoldController h={0};CK(HeadingController_HoldStep(&h,0,0,0,1000,&c,&e));
  unsigned ok=HeadingController_HoldStep(&h,0,2,0,1000+gaps[i],&c,&e);
  CK(ok==(gaps[i]<500));if(ok){CK(fabsf(c)<=1.01f);CK(fabsf(c)<=2);}
 }
 HeadingHoldController h={0};CK(HeadingController_HoldStep(&h,0,0,0,0xfffffff0U,&c,&e));
 CK(HeadingController_HoldStep(&h,0,1,0,84,&c,&e));
 CK(!HeadingController_HoldStep(&h,0,NAN,0,100,&c,&e));
 CK(!HeadingController_HoldStep(&h,0,11,0,100,&c,&e));
 printf("HEADING checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
