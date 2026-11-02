#ifndef VEHICLE_TUNING_H
#define VEHICLE_TUNING_H
/* 整车现场调参入口。单位写在名称和注释中；修改后编译下载。
 * 当前只实现第一阶段：启停区1左移到扫码点；不自动进入第二阶段。
 * 地图按用户图暂作均分估算，不声称与未来雷达真实布局完全一致。 */
#define VEHICLE_STAGE1_ENABLE 1U
#define VEHICLE_MAP_VERSION 1U
#define FIELD_SIZE_MM 2400.0f
#define FIELD_GRID_SIZE_MM 480.0f
/* 雷达图坐标：X向上，Y向右；尺寸图需旋转后对应，不能混用轴。 */
#define FIELD_START1_X_MM 150.0f
#define FIELD_START1_Y_MM 2250.0f
#define FIELD_START2_X_MM 150.0f
#define FIELD_START2_Y_MM 150.0f
/* 用户已确认：扫码点暂为两个蓝色启停区中心的中点。
 * 改扫码距离：修改下面Y坐标，例如1150代表从区1左移110厘米。
 * 直接左移要求扫码X与区1X相同，否则拒绝启动，不偷偷增加前后动作。 */
#define FIELD_SCAN_X_MM ((FIELD_START1_X_MM+FIELD_START2_X_MM)*0.5f)
#define FIELD_SCAN_Y_MM ((FIELD_START1_Y_MM+FIELD_START2_Y_MM)*0.5f)
/* 三个工位默认取格中心，后续现场在此独立修改停车坐标。 */
#define FIELD_RAW_X_MM 1200.0f
#define FIELD_RAW_Y_MM 2160.0f
#define FIELD_COARSE_X_MM 1200.0f
#define FIELD_COARSE_Y_MM 240.0f
#define FIELD_STORAGE_X_MM 2160.0f
#define FIELD_STORAGE_Y_MM 1200.0f
/* 第一阶段：车头朝雷达图+X/原料区，车尾扫码模块朝二维码板。
 * 不旋转，车体左移=地图Y减小。当前只允许PA1选择启停区1。 */
#define VEHICLE_STAGE1_ZONE 1U
#define VEHICLE_STAGE1_SPEED_CM_S 10.0f
#define VEHICLE_STAGE1_MOVE_TIMEOUT_MS 45000U
#define VEHICLE_STAGE1_SCAN_TIMEOUT_MS 8000U
#define VEHICLE_STAGE1_SCAN_FRESH_MS 500U
/* 用户描述扫码窗口X约等于启停区1中心X；板按图暂取X=0边线。
 * 因而名义间距150mm，实际未尺量；不是传感器实时测距反馈。
 * 这两个值用于停车几何检查，安装/板位置变化后必须重新修改。 */
#define FIELD_SCANNER_START_X_MM FIELD_START1_X_MM
#define FIELD_QR_PLANE_X_MM 0.0f
#define VEHICLE_SCAN_MIN_STANDOFF_MM 100.0f
#define VEHICLE_STAGE1_SCAN_AFTER_STOP 1U
/* 当前几何检查采用300mm方形包络；需实测整车/展开扫掠再验收。 */
#define FIELD_BODY_HALF_MM 150.0f
#if VEHICLE_STAGE1_ENABLE > 1U || VEHICLE_STAGE1_ZONE != 1U
#error Stage1_currently_supports_start_zone1_only
#endif
#if VEHICLE_STAGE1_SCAN_AFTER_STOP > 1U
#error Invalid_stage1_scan_option
#endif
#endif
