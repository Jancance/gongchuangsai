#include "src/sync_batch.h"
static uint32_t checks,failure;
#define CHECK(x) do{checks++;if(!(x)&&!failure)failure=__LINE__;}while(0)
static void ready(SyncBatch *b,uint32_t now) {
 uint32_t i;SyncBatch_Begin(b,7,now);
 for(i=0;i<4;i++)CHECK(SyncBatch_Ack(b,7,b->generation,i,1,now+i*20));
}
uint32_t test_entry(void) {
 SyncBatch b={0};uint32_t i,g;
 for(i=0;i<4;i++) {
  SyncBatch_Begin(&b,7,1000);g=b.generation;
  CHECK(!SyncBatch_Claim(&b,7,g,1000));
  {uint32_t k;for(k=0;k<i;k++)CHECK(SyncBatch_Ack(&b,7,g,k,1,1000+k*20));}
  CHECK(!SyncBatch_Claim(&b,7,g,1060));
  CHECK(!SyncBatch_Ack(&b,7,g,i,5,1060));CHECK(!SyncBatch_Claim(&b,7,g,1070));
 }
 ready(&b,1000);g=b.generation;
 CHECK(!SyncBatch_Claim(&b,8,g,1070));CHECK(!SyncBatch_Claim(&b,7,g-1,1070));
 CHECK(SyncBatch_Claim(&b,7,g,1070));CHECK(!SyncBatch_Claim(&b,7,g,1071));
 CHECK(b.phase==SYNC_BATCH_CLAIMED);CHECK(SyncBatch_Finish(&b,1,1071));
 CHECK(!SyncBatch_Finish(&b,1,1072));CHECK(!SyncBatch_Claim(&b,7,g,1073));
 ready(&b,1000);CHECK(!SyncBatch_Claim(&b,7,b.generation,1100));
 ready(&b,1000);CHECK(SyncBatch_Claim(&b,7,b.generation,1099));CHECK(!SyncBatch_Finish(&b,1,1149));
 ready(&b,1000);CHECK(SyncBatch_Claim(&b,7,b.generation,1070));CHECK(!SyncBatch_Finish(&b,5,1071));
 ready(&b,1000);g=b.generation;SyncBatch_Abort(&b);CHECK(!SyncBatch_Claim(&b,7,g,1070));
 ready(&b,1000);CHECK(b.generation!=g);CHECK(!SyncBatch_Claim(&b,7,g,1070));
 SyncBatch_Begin(&b,7,1000);CHECK(!SyncBatch_Ack(&b,7,b.generation,1,1,1010));
 SyncBatch_Begin(&b,7,1000);g=b.generation;CHECK(SyncBatch_Ack(&b,7,g,0,1,1000));CHECK(!SyncBatch_Ack(&b,7,g,0,1,1010));
 SyncBatch_Begin(&b,7,1000);CHECK(!SyncBatch_Ack(&b,8,b.generation,0,1,1000));
 SyncBatch_Begin(&b,7,1000);CHECK(!SyncBatch_Ack(&b,7,b.generation-1,0,1,1000));
 SyncBatch_Begin(&b,7,1000);CHECK(!SyncBatch_Ack(&b,7,b.generation,4,1,1000));
 SyncBatch_Begin(&b,7,1000);CHECK(!SyncBatch_Ack(&b,7,b.generation,0,1,1100));
 ready(&b,0xfffffff0U);CHECK(SyncBatch_Claim(&b,7,b.generation,0x36U));CHECK(SyncBatch_Finish(&b,1,0x37U));
 b.generation=0xffffffffU;SyncBatch_Begin(&b,7,1000);CHECK(b.generation==1);
 return failure?(0x80000000U|failure):checks;
}
