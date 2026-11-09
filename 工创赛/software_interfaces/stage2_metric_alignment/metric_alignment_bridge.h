#ifndef STAGE2_METRIC_ALIGNMENT_BRIDGE_H
#define STAGE2_METRIC_ALIGNMENT_BRIDGE_H
#include "metric_alignment.h"
#include "vehicle_mission.h"
/* 候选适配层：由现有任务A单一拥有inbox；不在UART ISR中调用。
 * 实际VehicleMission_ReadAlignmentRequest已检查第2段停车/持位/无故障。
 * 本文件未挂入当前固件；未来串口适配器也只能提交观测，不直接发运动。
 */
static uint32_t MetricAlignmentBridge_Open(MetricAlignmentInbox *c,
 const MetricAlignmentProfile *profile,uint32_t unique_request_id,uint32_t now_ms)
{
 VehicleAlignmentRequest current;MetricAlignmentRequest r;
 if(!c || !profile)return MA_BAD_ARGUMENT;
 MetricAlignment_Invalidate(c);
 if(!VehicleMission_ReadAlignmentRequest(&current) || current.position_executor_enabled)
  return MA_NOT_STOPPED;
 if(!(current.nominal_target_x_mm>=0 && current.nominal_target_x_mm<2400 &&
      current.nominal_target_y_mm>=0 && current.nominal_target_y_mm<2400))return MA_BAD_ARGUMENT;
 memset(&r,0,sizeof(r));r.request_id=unique_request_id;r.run_id=current.run_id;
 r.stage=current.stage;r.start_id=current.start_id;r.map_version=current.layout_version;
 r.goal_cell=current.goal_cell;r.issued_ms=now_ms;r.timeout_ms=1000;
 r.nominal_x_milli_mm=(int32_t)(current.nominal_target_x_mm*1000.0f);
 r.nominal_y_milli_mm=(int32_t)(current.nominal_target_y_mm*1000.0f);r.profile=*profile;
 return MetricAlignment_Begin(c,&r,1);
}
static uint32_t MetricAlignmentBridge_Current(MetricAlignmentInbox *c)
{
 VehicleAlignmentRequest current;
 if(!c)return 0;
 if(!c->pending || !VehicleMission_ReadAlignmentRequest(&current) || current.position_executor_enabled ||
    current.run_id!=c->request.run_id || current.stage!=c->request.stage ||
    current.start_id!=c->request.start_id || current.layout_version!=c->request.map_version ||
    current.goal_cell!=c->request.goal_cell ||
    current.nominal_target_x_mm*1000.0f!=(float)c->request.nominal_x_milli_mm ||
    current.nominal_target_y_mm*1000.0f!=(float)c->request.nominal_y_milli_mm){
  MetricAlignment_Invalidate(c);return 0;
 }
 return 1;
}
static uint32_t MetricAlignmentBridge_Submit(MetricAlignmentInbox *c,const char *wire,
 uint32_t length,uint32_t now_ms)
{
 MetricAlignmentObservation o;uint32_t result;
 if(!MetricAlignmentBridge_Current(c))return MA_NOT_STOPPED;
 result=MetricAlignment_Decode(wire,length,&o);if(result)return result;
 return MetricAlignment_Accept(c,&o,now_ms,1);
}
static uint32_t MetricAlignmentBridge_Read(MetricAlignmentInbox *c,uint32_t now_ms,
 MetricAlignmentObservation *out)
{
 if(!MetricAlignmentBridge_Current(c))return MA_NOT_STOPPED;
 return MetricAlignment_Read(c,now_ms,1,out);
}
#endif
