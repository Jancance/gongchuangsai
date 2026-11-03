#ifndef VEHICLE_TUNING_H
#define VEHICLE_TUNING_H
/* 整车现场调参入口。单位写在名称和注释中；修改后编译下载。
 * 当前只实现第一阶段：启停区1前进后左移到扫码点；不自动进入第二阶段。
 * 地图按用户图暂作均分估算，不声称与未来雷达真实布局完全一致。 */
#define VEHICLE_STAGE1_ENABLE 1U
#include "../../USER/vehicle_parameters.h"
#ifndef VEHICLE_SCAN_BYPASS
#define VEHICLE_SCAN_BYPASS 1U
#endif
#define VEHICLE_MAP_VERSION 2U
#define FIELD_SIZE_MM 2400.0f
#define FIELD_GRID_SIZE_MM 480.0f
/* 用户最终坐标：+X从启停区2指向启停区1，+Y从启停区2指向粗加工区。
 * 车头朝+Y/原料区，左移沿-X；本项目坐标不冒充雷达设备坐标。 */
#define FIELD_START1_X_MM 2250.0f
#define FIELD_START1_Y_MM 150.0f
#define FIELD_START2_X_MM 150.0f
#define FIELD_START2_Y_MM 150.0f
/* 用户确认：因扫码模块贴车尾、工作距离至少100mm，第一阶段
 * 先前进15cm再左移105cm，不旋转；扫码点为两区中点沿+Y前移150mm。
 * 修改前进距离改FORWARD_CM；修改左移距离可调整SCAN_X_MM。
 * 默认扫码停车点(1200,300)mm；原两区中点(1200,150)只作几何参考。 */
#define VEHICLE_STAGE1_FORWARD_CM 15.0f
#define FIELD_SCAN_X_MM ((FIELD_START1_X_MM+FIELD_START2_X_MM)*0.5f)
#define FIELD_SCAN_Y_MM (FIELD_START1_Y_MM+VEHICLE_STAGE1_FORWARD_CM*10.0f)
/* 三个工位默认取格中心，后续现场在此独立修改停车坐标。 */
#define FIELD_RAW_X_MM 2160.0f
#define FIELD_RAW_Y_MM 1200.0f
#define FIELD_COARSE_X_MM 240.0f
#define FIELD_COARSE_Y_MM 1200.0f
#define FIELD_STORAGE_X_MM 1200.0f
#define FIELD_STORAGE_Y_MM 2160.0f
/* 第一阶段保持车头朝+Y，前进=+Y，左移=-X；只支持PA1启停区1。 */
#define VEHICLE_STAGE1_ZONE 1U
#define VEHICLE_STAGE1_ENTRY_SPEED_CM_S 5.0f
#define VEHICLE_STAGE1_SPEED_CM_S 10.0f
#define VEHICLE_STAGE1_MOVE_TIMEOUT_MS 60000U
#define VEHICLE_STAGE1_SCAN_TIMEOUT_MS 8000U
#define VEHICLE_STAGE1_SCAN_FRESH_MS 500U
/* 既有用户接受的轮组近似外廓，不是新测量的整车外廓。
 * 扫码窗口贴车尾，暂按中心后方半车长计算；厚度/突出量待现场校准。
 * 不能用左右居中的X推算读距。板已由用户确认为Y=0。
 * 默认停车Y=300，名义窗口Y=300-141.5=158.5mm，不能称实测。 */
#define VEHICLE_BODY_LENGTH_MM 283.0f
#define VEHICLE_BODY_WIDTH_MM 254.0f
#define FIELD_SCANNER_REAR_OFFSET_MM (VEHICLE_BODY_LENGTH_MM*0.5f)
#define FIELD_QR_PLANE_Y_MM 0.0f
#define VEHICLE_SCAN_MIN_STANDOFF_MM 100.0f
#define VEHICLE_STAGE1_SCAN_AFTER_STOP 0U
#define VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS 5000U
/* 当前几何检查采用300mm方形包络；需实测整车/展开扫掠再验收。 */
#define FIELD_BODY_HALF_MM 150.0f
#if VEHICLE_STAGE1_ENABLE > 1U || (VEHICLE_STAGE1_ZONE < 1U || VEHICLE_STAGE1_ZONE > 2U)
#error Stage1_currently_supports_start_zone1_only
#endif
#if VEHICLE_STAGE1_SCAN_AFTER_STOP > 1U
#error Invalid_stage1_scan_option
#endif
#endif
