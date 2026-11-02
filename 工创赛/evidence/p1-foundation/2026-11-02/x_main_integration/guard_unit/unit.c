#include "chassis_direction_guard.h"
static uint32_t checks,first_failure;
#define CHECK(x) do{checks++;if(!(x)&&!first_failure)first_failure=__LINE__;}while(0)
uint32_t test_entry(void){
 ChassisDirectionGuard s;
 DirectionGuard_Reset(&s);CHECK(DirectionGuard_Check(&s,1000,1000,3,1)==DIRECTION_NO_TARGET);
 DirectionGuard_Ack(&s,-10,900);CHECK(DirectionGuard_Check(&s,1000,1000,3,1)==0);CHECK(s.pending==1&&s.reverse_samples==1);
 CHECK(DirectionGuard_Check(&s,1010,1000,3,1)==0);CHECK(s.reverse_samples==1);
 DirectionGuard_Ack(&s,-11,1100);CHECK(s.pending==1);CHECK(s.first_reverse_ms==1000);
 CHECK(DirectionGuard_Check(&s,1200,1200,0,1)==0);CHECK(!s.pending);
 CHECK(DirectionGuard_Check(&s,1300,1300,3,1)==0);CHECK(s.pending);
 CHECK(DirectionGuard_Check(&s,1400,1400,3,1)==DIRECTION_PERSISTENT);CHECK(s.reverse_samples==2);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,-10,900);
 CHECK(DirectionGuard_Check(&s,1000,1000,3,1)==0);CHECK(!DirectionGuard_Expired(&s,1599));
 DirectionGuard_Ack(&s,-9,1500);CHECK(DirectionGuard_Check(&s,1600,1000,3,1)==DIRECTION_PERSISTENT);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,-10,900);
 CHECK(DirectionGuard_Check(&s,1000,1000,6,1)==DIRECTION_REVERSE);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,-10,900);
 CHECK(DirectionGuard_Check(&s,1000,1000,3,0)==DIRECTION_REVERSE);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,-21,900);
 CHECK(DirectionGuard_Check(&s,1000,1000,1,1)==DIRECTION_REVERSE);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,10,900);
 CHECK(DirectionGuard_Check(&s,1000,1000,-5,1)==0);CHECK(s.pending);
 CHECK(DirectionGuard_Check(&s,1100,1100,1,1)==0);CHECK(!s.pending);
 DirectionGuard_Ack(&s,-10,1200);CHECK(DirectionGuard_Check(&s,1200,1100,100,0)==0);CHECK(!s.pending);
 CHECK(DirectionGuard_Check(&s,1300,1300,-3,0)==0);
 CHECK(DirectionGuard_Check(&s,1400,1400,1,1)==0);CHECK(s.pending);
 DirectionGuard_Ack(&s,0,1450);CHECK(!s.pending);CHECK(DirectionGuard_Check(&s,1500,1500,1,0)==0);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,-10,0xFFFFFF00U);
 CHECK(DirectionGuard_Check(&s,0xFFFFFFF0U,0xFFFFFFF0U,3,1)==0);
 CHECK(!DirectionGuard_Expired(&s,0x00000247U));CHECK(DirectionGuard_Expired(&s,0x00000248U));
 CHECK(DirectionGuard_Check(&s,0x00000248U,0xFFFFFFF0U,3,1)==DIRECTION_PERSISTENT);
 DirectionGuard_Reset(&s);DirectionGuard_Ack(&s,-10,900);
 CHECK(DirectionGuard_Check(&s,1000,1000,3,1)==0);
 CHECK(DirectionGuard_Check(&s,1100,1000,3,0)==DIRECTION_REVERSE);
 DirectionGuard_Reset(&s);CHECK(!s.valid&&!s.pending);
 return first_failure?(0x80000000U|first_failure):checks;
}
