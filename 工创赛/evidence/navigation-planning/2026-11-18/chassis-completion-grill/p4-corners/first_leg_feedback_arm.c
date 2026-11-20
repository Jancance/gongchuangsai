#include "fusion_motion.h"
#include <math.h>

volatile unsigned replay_assertions,replay_case,replay_detail;
volatile float replay_actual,replay_expected;
#define CHECK(x) do { replay_assertions++; if(!(x)) return __LINE__; } while(0)

typedef struct {
    FusionMotionFeedbackInput input;
    FusionMotionVector correction;
    int raw[4];
    unsigned finish;
} FeedbackCase;

/* Expected values from the MATLAB controller equations with H=.08, K=77*pi/60,
 * lever=209 and LR/RR/LF/RF ordering. Yaw fields are radians or radians/s. */
static const FeedbackCase cases[]={
    {{{0},{0},{0},{{0}},{{8,0},{0,8},{-8,0},{0,-8}},0},
        {0},{248,-248,-248,-248},0},
    {{{0.6f,-0.8f,1.571668991f},{0,0,1.570796327f},{0},
      {{0}},{{1,3},{2,4},{3,5},{4,6}},0},
        {0.84f,-1.12f,0},{57,185,247,57},0},
    {{{30,40,-0.0872664626f},{0},{0},{{0}},{{0}},0},
        {1.344f,1.792f,-0.013962634f},{15,-8,6,1},0},
    {{{30,40,-0.0872664626f},{0},{6.6f,8.8f,-0.048869219f},{{0}},{{0}},0},
        {7.2f,9.6f,-0.052359878f},{69,-33,21,15},0},
    {{{0,0,-3.124139361f},{0,0,3.124139361f},{0},{{0}},{{1,1},{1,1},{1,1},{1,1}},0},
        {0,0,0.013962634f},{-69,8,-6,-55},0},
    {{{1.5f,2,0},{0},{0},{{0}},{{8,0},{0,8},{-8,0},{0,-8}},.reference_complete=1,.capture_complete=1},
        {1.344f,1.792f,0},{0,0,0,0},1},
    {{{2.51f,0,0},{0},{0},{{0}},{{0}},.reference_complete=1,.capture_complete=1},
        {2.24f,0,0},{6,6,6,6},0},
    {{{0,0,0.0017627825f},{0},{0},{{0}},{{8,0},{0,8},{-8,0},{0,-8}},.reference_complete=1,.capture_complete=1},
        {0},{0,0,0,0},1},
    {{{0,0,-0.00331612558f},{0},{0,0,0.04f},{{0}},{{0}},
      .slot_yaw_delta={0.008f,0.008f,0.008f,0.008f}},
        {0},{-52,52,-52,52},0},
    {{{0,0,0.00366519143f},{0},{0},{{0}},{{0}},.reference_complete=1,.capture_complete=1},
        {0,0,0.00806342115f},{-4,4,-4,4},0},
    {{{0,0,90.0f*0.01745329251994329577f},{0,0,89.8f*0.01745329251994329577f},
      {0,0,0.04f},{{0}},{{0}},.reference_complete=1,.capture_complete=1},
        {0},{0,0,0,0},1},
    {{{0},{0},{0},{{0}},{{8,0},{0,8},{-8,0},{0,-8}},.reference_complete=1},
        {0},{248,-248,-248,-248},0},
};

unsigned test_entry(void)
{
    const unsigned forward[4]={0,1,0,1};
    FusionMotionFeedbackBatch batch;
    FusionMotionFeedbackInput invalid=cases[0].input;
    unsigned i,j;
    for(i=0;i<sizeof(cases)/sizeof(cases[0]);i++) {
        const FeedbackCase *v=&cases[i];
        replay_case=i;
        CHECK(FusionMotion_FeedbackBatch(&v->input,&batch));
        CHECK(fabsf(batch.correction.x-v->correction.x)<1e-5f);
        CHECK(fabsf(batch.correction.y-v->correction.y)<1e-5f);
        CHECK(fabsf(batch.correction.yaw-v->correction.yaw)<1e-6f);
        CHECK(batch.finish_requested==v->finish);
        for(j=0;j<4;j++) {
            const int raw=v->raw[j];
            replay_detail=j;
            replay_actual=batch.wheels.rpm[j];
            replay_expected=raw*.1f;
            CHECK(fabsf(replay_actual-replay_expected)<1e-5f);
            CHECK(batch.commands[j].address==j+1);
            CHECK(batch.commands[j].direction==(forward[j]^(raw<0)));
            CHECK(batch.commands[j].speed_raw==(unsigned)(raw<0 ? -raw : raw));
        }
    }
    invalid.slot_to[3].x=1000;
    batch.finish_requested=2;
    CHECK(!FusionMotion_FeedbackBatch(&invalid,&batch));
    CHECK(batch.finish_requested==2);
    CHECK(!FusionMotion_FeedbackBatch(0,&batch));
    CHECK(!FusionMotion_FeedbackBatch(&invalid,0));
    return 0;
}
