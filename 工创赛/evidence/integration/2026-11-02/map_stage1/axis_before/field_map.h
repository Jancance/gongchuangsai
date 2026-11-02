#ifndef FIELD_MAP_H
#define FIELD_MAP_H
#include <stdint.h>
#include "vehicle_tuning.h"
typedef struct { float x_mm,y_mm; } FieldPoint;
typedef struct {
    FieldPoint start,stop;
    float left_cm,speed_cm_s,nominal_scan_gap_mm;
    uint32_t start_grid,goal_grid,config_id;
} FieldStage1Plan;
/* 原始格中心与实际停车点分开。0/4停车点是蓝色区中心；扫码点另取中点。 */
uint32_t FieldMap_LocalMode(void);
uint32_t FieldMap_CellCenter(uint32_t id,FieldPoint *point);
uint32_t FieldMap_Park(uint32_t id,FieldPoint *point);
uint32_t FieldMap_Stage1(FieldStage1Plan *plan);
/* 只作当前静态图的轴向扫掠检查，不代表动态障碍/完整赛道验收。 */
uint32_t FieldMap_AxisClear(FieldPoint a,FieldPoint b,float half_mm);
uint32_t FieldMap_TaskCodeValid(const char *code);
#endif
