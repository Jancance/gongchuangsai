static const float vectors[3][4]={{3.0213867495e+02f,7.9569963817e+01f,1.6437703237e+02f,1.9666731421e+02f},{3.0695643253e+02f,-9.1577269767e+01f,1.6440499019e+02f,1.6653949301e+02f},{1.5723805964e+02f,-1.0934326891e+01f,5.6841350564e+00f,1.8133132924e+02f}};
static const float shift[2]={-1.3400000000e+01f,-1.4900000000e+01f};

#include <stdio.h>
#include "source_pick_geometry.h"
static unsigned checks,failures;
#define CK(x) do{checks++;if(!(x)){failures++;printf("FAIL line %u\n",__LINE__);}}while(0)
int main(void){
 SourcePickGeometry g={165,170,180,0,360,10},bad;
 SourcePickTarget t,saved;
 SourcePickTolerance fine={.2f,.1f,.5f,.5f,2,100};
 unsigned k;float err;
 for(k=0;k<3;k++){
  CK(SourcePick_Solve(&g,vectors[k][0],vectors[k][1],shift[0],shift[1],0,&t)==SPG_MARGIN);
  CK(fabsf(t.extension_mm-vectors[k][2])<.0005f);
  CK(fabsf(t.logical_deg-vectors[k][3])<.0005f);
  CK(t.geometry_in_range==1 && !t.requested_margin_met && !t.motion_allowed);
  CK(!SourcePick_FirmwareSupports(&t,180,50));
  CK(SourcePick_FirmwareSupports(&t,360,170)); /* 比较假设能力，未写入固件。 */
  CK(SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,80,80,1,&err));
  CK(err<.0001f);
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,0,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,80,0,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,80,80,0,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg+1.6f,t.extension_mm,0,0,100,80,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm+.11f,0,0,100,80,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,.51f,0,100,80,80,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,.51f,100,80,80,1,&err));
  CK(SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,9,0xfffffff0U,0xfffffff0U,1,&err));
  CK(!SourcePick_SampleWithinTolerance(&g,&t,&fine,t.logical_deg,t.extension_mm,0,0,100,101,80,1,&err));
 }
 CK(SourcePick_Solve(&g,165,0,0,0,0,&t)==SPG_MARGIN && t.extension_mm==0 && !t.motion_allowed);
 CK(SourcePick_Solve(&g,335,0,0,0,0,&t)==SPG_MARGIN && t.extension_mm==170);
 CK(SourcePick_Solve(&g,164.99f,0,0,0,0,&t)==SPG_STROKE && !t.geometry_in_range);
 CK(SourcePick_Solve(&g,335.01f,0,0,0,0,&t)==SPG_STROKE && !t.geometry_in_range);
 CK(SourcePick_Solve(&g,235,0,0,0,90,&t)==SPG_OK && fabsf(t.logical_deg-90)<.0001f);
 CK(!t.motion_allowed && t.requested_margin_met);
 saved=t;bad=g;bad.max_extension_mm=NAN;
 CK(SourcePick_Solve(&bad,235,0,0,0,0,&t)==SPG_ARGUMENT && !t.geometry_in_range && !t.motion_allowed);
 CK(SourcePick_Solve(&g,NAN,0,0,0,0,&t)==SPG_ARGUMENT);
 CK(SourcePick_Solve(&g,INFINITY,0,0,0,0,&t)==SPG_ARGUMENT);
 CK(SourcePick_Solve(&g,0,0,0,0,0,&t)==SPG_ARGUMENT);
 CK(SourcePick_Solve(&g,235,0,0,0,0,0)==SPG_ARGUMENT);
 CK(!SourcePick_FirmwareSupports(0,360,170));
 saved.logical_deg=-1;CK(!SourcePick_FirmwareSupports(&saved,360,170));
 saved.logical_deg=90;saved.extension_mm=-1;CK(!SourcePick_FirmwareSupports(&saved,360,170));
 saved.extension_mm=70;saved.motion_allowed=1;CK(!SourcePick_FirmwareSupports(&saved,360,170));
 CK(!SourcePick_SampleWithinTolerance(&g,&saved,&fine,90,70,0,0,10,0,0,1,&err));
 CK(fabsf(SPG_AngleError(359,1)+2)<.0001f);
 CK(fabsf(SPG_AngleError(1,359)-2)<.0001f);
 printf("SOURCE_GEOMETRY checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
