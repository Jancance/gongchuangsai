static const float vectors[3][4]={{2.8093068779e+02f,8.1257155424e+01f,1.6509863533e+02f,1.9583048521e+02f},{2.8245057109e+02f,-9.1139175072e+01f,1.6510583943e+02f,1.6517513451e+02f},{1.3310732117e+02f,-6.6045296679e+00f,4.9109377771e+00f,1.8037831311e+02f}};
static const float shift[2]={-3.2800000000e+01f,-7.7000000000e+00f};

#include <stdio.h>
#include "source_pick_geometry.h"
int main(void){
 SourcePickGeometry g={161,170,180,0,360,10};SourcePickTarget target;
 unsigned k,checks=0,failures=0;
 for(k=0;k<3;k++){
  unsigned rc=SourcePick_Solve(&g,vectors[k][0],vectors[k][1],shift[0],shift[1],0,&target);
  checks++;if(rc!=SPG_MARGIN)failures++;
  checks++;if(fabsf(target.extension_mm-vectors[k][2])>.0005f)failures++;
  checks++;if(fabsf(target.logical_deg-vectors[k][3])>.0005f)failures++;
  checks++;if(target.motion_allowed || !target.geometry_in_range || target.requested_margin_met)failures++;
  checks++;if(SourcePick_FirmwareSupports(&target,180,50))failures++;
 }
 printf("CURRENT_PAIR checks=%u failures=%u\n",checks,failures);return failures?1:0;
}
