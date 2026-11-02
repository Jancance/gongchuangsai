static void feed_bytes(const char *frame)
{
 uint32_t i;
 for(i=0;frame[i];i++)App_SerialMotionByte((uint8_t)frame[i]);
}
static void serial_line(const char *prefix)
{
 char frame[RQ_LINE_MAX];uint32_t n=(uint32_t)strlen(prefix);
 (void)snprintf(frame,sizeof(frame),"%s%04X",prefix,RuntimeRoute_Crc(prefix,n));
 feed_bytes(frame);App_SerialMotionByte(10U);
}
static void upload_route(uint32_t id,uint32_t count)
{
 static const RuntimeRouteRow all[8]={
 {4,18144,200,35000},{6,18000,200,35000},{3,-90000,30000,35000},{3,-90000,30000,35000},
 {4,18144,200,35000},{3,90000,30000,35000},{3,90000,30000,35000},{5,18000,200,35000}};
 char text[150];uint32_t i;
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,BEGIN,%lu,",(unsigned long)test_tick,(unsigned long)id,(unsigned long)count);
 serial_line(text);CHECK(s_route_ack_result==RQ_OK);CHECK(!g_vm_test_request);
 serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);
 for(i=0;i<count;i++) {
  (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,ROW,%lu,%lu,%ld,%lu,%lu,",
   (unsigned long)test_tick,(unsigned long)id,(unsigned long)i,(unsigned long)all[i].kind,
   (long)all[i].amount,(unsigned long)all[i].speed,(unsigned long)all[i].timeout_ms);
  serial_line(text);CHECK(s_route_ack_result==RQ_OK);
  serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);
 }
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,COMMIT,%lu,",(unsigned long)test_tick,
  (unsigned long)id,(unsigned long)RuntimeRoute_Digest(id,count,all));
 serial_line(text);CHECK(s_route_ack_result==RQ_OK);CHECK(s_runtime_route.ready);
 CHECK(MOTION_ACTION_COUNT==count);CHECK(App_MotionPlanValid());CHECK(!g_vm_test_request&&!test_power);
 serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);CHECK(s_runtime_route.ready);
}
static void setup_route(void)
{
 reset_scheduler();ChassisAuto_Init();RuntimeRoute_Init(&s_runtime_route);
 g_serial_motion_state=g_serial_motion_result=g_serial_motion_rejected=g_serial_motion_accepted=0;
 g_serial_motion_link_lost=0;s_serial_tx_seq=100;s_serial_tx_valid=s_serial_proto_ok=1;
 s_heading_source.rate_dps=0;s_route_ack_pending=0;
}
static void start_route(void)
{
 char text[150];
 (void)snprintf(text,sizeof(text),"RC1,99,%lu,ARM,",(unsigned long)test_tick);
 serial_line(text);CHECK(g_serial_motion_state==1);
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,%lu,GO,%lu,",(unsigned long)test_tick,
  (unsigned long)s_runtime_route.committed.id,(unsigned long)s_runtime_route.committed.digest);
 serial_line(text);CHECK(s_route_ack_result==RQ_OK);CHECK(g_combined_test.request==1);
 serial_line(text);CHECK(s_route_ack_result==RQ_DUPLICATE);CHECK(g_combined_test.request==1);
}
static void finish_route(uint32_t count)
{
 uint32_t i;
 for(i=0;i<count;i++) {
  if(i)test_tick+=1001;
  App_CombinedTestStep();
  if(i==count-1 && g_combined_test.state==1)App_CombinedTestStep();
  CHECK(g_combined_test.completed==i+1);
 }
 App_SerialMotionSafety();CHECK(g_combined_test.state==3);CHECK(g_serial_motion_state==3);
 CHECK(!g_vm_test_request&&!test_power);CHECK(App_SerialCompletedStopped());
}
uint32_t test_entry(void)
{
 RuntimeRoute q;RuntimeRouteMessage m={0};RuntimeRoutePrepared prepared;
 uint32_t i,n,old;char text[150];
 for(n=1;n<=8;n++) {
  setup_route();upload_route(1,n);start_route();finish_route(n);
  CHECK(step_calls==n);CHECK(finish_calls==1);
 }
 /* A new route after normal completion reuses only successful backend state. */
 g_chassis_auto_session=mock_status.session;g_chassis_auto_state=CHASSIS_AUTO_DONE;
 g_chassis_auto_result=1;g_chassis_auto_cause=0;g_chassis_auto_segment=2;
 g_chassis_auto_stop_mask[2]=15;
 for(i=0;i<4;i++)g_chassis_auto_driver_rpm[i]=0;
 upload_route(2,2);CHECK(g_chassis_auto_state==CHASSIS_AUTO_IDLE);start_route();finish_route(2);
 CHECK(step_calls==10);CHECK(finish_calls==2);
 /* Fault/cancel states cannot be reset by upload. */
 old=s_runtime_route.committed.id;g_combined_test.state=4;g_combined_test.cause=5;
 g_chassis_auto_state=CHASSIS_AUTO_FAULT;g_chassis_auto_result=2;g_chassis_auto_cause=8;
 (void)snprintf(text,sizeof(text),"RQ1,99,%lu,3,BEGIN,1,",(unsigned long)test_tick);
 serial_line(text);CHECK(s_route_ack_result==RQ_BUSY);CHECK(s_runtime_route.committed.id==old);
 CHECK(g_chassis_auto_state==CHASSIS_AUTO_FAULT);CHECK(!ChassisAuto_RearmCompleted(g_chassis_auto_session));
 setup_route();upload_route(1,2);start_route();
 serial_line("RQ1,99,1000,2,BEGIN,1,");CHECK(s_route_ack_result==RQ_BUSY);
 CHECK(s_runtime_route.committed.id==1&&s_runtime_route.committed.count==2);
 /* Keepalive loss cancels the whole route and never advances another segment. */
 test_tick+=1000;App_SerialMotionSafety();CHECK(g_combined_test.state==5);
 CHECK(g_serial_motion_link_lost);CHECK(!g_vm_test_request&&!test_power);
 setup_route();upload_route(1,1);start_route();
 test_tick=s_serial_keepalive_ms+35000;App_CombinedTestStep();
 /* The independent route deadline also cuts a blocked first segment. */
 g_combined_test.state=1;g_combined_test.index=0;s_combined_stage_ms=test_tick-35000;
 App_CombinedTestSafety();CHECK(g_combined_test.state==4&&g_combined_test.cause==2);
 /* CRC, parser bounds, stage timeout, missing/changed/duplicate rows. */
 RuntimeRoute_Init(&q);m.id=1;m.op=RQ_BEGIN;m.count=1;
 CHECK(RuntimeRoute_Stage(&q,&m,0)==RQ_OK);m.op=RQ_COMMIT;
 CHECK(RuntimeRoute_Prepare(&q,&m,1,&prepared)==RQ_MISSING);
 m.op=RQ_ROW;m.index=0;m.row=(RuntimeRouteRow){4,1000,200,35000};
 CHECK(RuntimeRoute_Stage(&q,&m,2)==RQ_OK);CHECK(RuntimeRoute_Stage(&q,&m,3)==RQ_DUPLICATE);
 m.row.amount=2000;CHECK(RuntimeRoute_Stage(&q,&m,4)==RQ_ID);
 m.op=RQ_COMMIT;m.digest=1;CHECK(RuntimeRoute_Prepare(&q,&m,5,&prepared)==RQ_DIGEST);
 m.digest=RuntimeRoute_Digest(1,1,q.staged);CHECK(RuntimeRoute_Prepare(&q,&m,10000,&prepared)==RQ_EXPIRED);
 RuntimeRoute_Tick(&q,10000);CHECK(!q.ready&&!q.staging);
 m.op=RQ_BEGIN;CHECK(RuntimeRoute_Stage(&q,&m,10001)==RQ_ID);
 m.id=2;CHECK(RuntimeRoute_Stage(&q,&m,10001)==RQ_OK);
 m.op=RQ_ROW;m.index=1;CHECK(RuntimeRoute_Stage(&q,&m,10002)==RQ_RANGE);
 m.index=0;m.row.kind=7;CHECK(RuntimeRoute_Stage(&q,&m,10002)==RQ_RANGE);
 CHECK(RuntimeRoute_Decode("RQ1,1,1,1,BEGIN,1,0000",&m)==RQ_FORMAT);
 CHECK(RuntimeRoute_Decode("RQ1,42949672960,1,1,BEGIN,1,0000",&m)==RQ_FORMAT);
 /* Stale command tokens and unknown serial START cannot launch a route. */
 setup_route();serial_line("RQ1,99,0,1,BEGIN,1,");CHECK(s_route_ack_result==RQ_STALE);
 serial_line("RC1,99,1000,ARM,");CHECK(g_serial_motion_state==0);
 CHECK(!g_combined_test.request&&!g_vm_test_request);
 upload_route(1,1);serial_line("RC1,99,1000,ARM,");CHECK(g_serial_motion_state==1);
 serial_line("RC1,99,1000,START,");CHECK(!g_combined_test.request&&!g_vm_test_request);
 serial_line("RQ1,99,1000,1,GO,1,");CHECK(s_route_ack_result==RQ_ID);CHECK(!g_combined_test.request);
 return first_failure?(0x80000000U|first_failure):checks;
}
