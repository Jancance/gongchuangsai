/* 用户确认：原速连续下降，伸缩先启动。时序经本机MATLAB验证。 */
#ifndef SOURCE_PICK_EXTENSION_LEAD_20261115_H
#define SOURCE_PICK_EXTENSION_LEAD_20261115_H
static unsigned SourcePick_ExtensionLeadMs(unsigned region)
{
 static const unsigned ms[3]={810U,780U,660U};
 return ms[region];
}
#endif
