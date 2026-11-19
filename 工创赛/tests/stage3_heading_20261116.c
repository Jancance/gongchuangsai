#include <stdio.h>
#include <assert.h>
#include "heading_session.h"
int main(void){
 HeadingSession s;HeadingSessionOutput o;HeadingSessionInput in={0};
 in.now_ms=in.sample_ms=100;in.sequence=1;in.healthy=in.platform_ok=1;in.max_rpm=3;
 HeadingSession_Init(&s);assert(!HeadingSession_Start(&s,&in,1.2f,&o));
 HeadingSession_Init(&s);assert(HeadingSession_StartFine(&s,&in,1.2f,&o));assert(s.controller.target_deg>1.19f);
 HeadingSession_Init(&s);assert(HeadingSession_StartFine(&s,&in,-1.2f,&o));assert(s.controller.target_deg< -1.19f);
 HeadingSession_Init(&s);assert(!HeadingSession_StartFine(&s,&in,1.0f,&o));
 HeadingSession_Init(&s);assert(!HeadingSession_StartFine(&s,&in,10.01f,&o));
 HeadingSession_Init(&s);assert(HeadingSession_Start(&s,&in,5.0f,&o));
 HeadingSession_Init(&s);in.healthy=0;assert(!HeadingSession_StartFine(&s,&in,1.2f,&o));
 puts("PASS actual heading session: ordinary >=5 unchanged; fine +/-1.2; limits and health retained");
 return 0;
}
