#include <stdint.h>
#include <stdio.h>
uint32_t FieldMap_TaskCodeValid(const char *code)
{
    uint32_t i,masks[4]={0U,0U,0U,0U},group,digit;
    if(!code) return 0U;
    for(i=0U;i<15U;i++) {
        if(i==3U || i==7U || i==11U) {if(code[i]!='+') return 0U;continue;}
        group=i/4U;
        if(code[i]<'1' || code[i]>'6') return 0U;
        digit=(uint32_t)(code[i]-'0');
        if((group&1U) && digit>3U) return 0U;
        if(masks[group]&(1UL<<digit)) return 0U;
        masks[group]|=1UL<<digit;
    }
    return code[15]=='\0' && masks[1]==14U && masks[3]==14U && masks[0]==masks[2];
}

int main(void){int bad=FieldMap_TaskCodeValid("216+231+354+123"),good=FieldMap_TaskCodeValid("216+231+612+123");printf("reported=%d corrected=%d\n",bad,good);return bad||!good;}
