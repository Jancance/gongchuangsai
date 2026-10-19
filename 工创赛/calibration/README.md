# P2 标定文件规则

## 桌面标定打印板（2026-09-26）

打印文件：`output/pdf/A4_checkerboard_10x7_15mm.pdf`。按用户后续要求，已改为A4纵向纯棋盘格居中版，无文字和尺寸线。横10×纵7个外部方格，每格标称15.00mm，棋盘本体150×105mm；OpenCV内部角点为 `(9, 6)` 共54个，不要把外部格数当角点数。打印选实际大小100%，禁止适应页面或缩放；用尺量5个连续格子应为75mm，并核对横纵方向，贴平硬板并留至少15mm白边。

配套名义参数在同目录JSON；打印机输出、平整度和相机实拍可读性尚未验证。当前相机固定桌面高度约190mm，该板是近距离台架候选，不保证所有倾斜姿态都能入镜。整页不必入镜，棋盘及周围白边需完整可见。PDF本身已渲染检查，OpenCV能在渲染图检出54个角点；这不是实机标定完成证据。

## 标定数据管理

每个标定文件只对应一个固定组合：相机机身、镜头、分辨率、裁剪方式、安装姿态和安装高度。任何一项变化后旧标定立即失效。

2026-10-02补充（CAL-05，工程合同）：场地原料转盘上表面离地高度准备覆盖80～100mm，需填实测值；与车载270°盘分别记录。平面标定还必须绑定所观测的工作平面及特征高度。仅工作平面升降不表示镜头内参改变，但旧平面的H不能直接继续使用；须重新标定/使用匹配且已验证的profile。相机/镜头/安装变化时，仍重新核对相关标定。详见[高度与跨组整合](../docs/P1_P2_HEIGHT_AND_INTEGRATION_PLAN_2026-10-02.md)。当前工具尚未自动读取以下新增元数据或按盘高求三维投影。

推荐命名：

```text
<source_id>_<width>x<height>_<mount_revision>_<YYYYMMDD>.yaml
```

文件至少包含：

```text
calibration_id
source_id
camera_vid_pid
camera_serial
image_size
pixel_format
camera_matrix
distortion_coefficients
reprojection_rms_px
reprojection_p95_px
mount_revision
mount_height_mm
T_camera_to_robot
H_image_to_ground
work_plane_id
height_datum
raw_turntable_surface_z_mm
observed_feature_plane_z_mm
material_height_mm
geometry_revision
created_at
validation_dataset
validation_result
```

内参、主相机工作平面/手眼链、前视鸟瞰单应性分别保存；不得用同一批标定图同时宣称训练精度和独立验证精度。目标门槛为重投影 RMS 不高于 0.5 px、95 分位不高于 1 px，但抓取、放置和回区必须另以毫米和真实动作结果验收。

新增字段是待实现的元数据约定，不是已存在于所有标定文件。原料盘面高度以场地地面Z=0、向上为正；如果检测物料顶面，特征平面要计入物料高度。每次改高后保存独立中心/边缘毫米误差、匹配的P1几何版本，撤销旧目标与预测历史后才恢复动作。
