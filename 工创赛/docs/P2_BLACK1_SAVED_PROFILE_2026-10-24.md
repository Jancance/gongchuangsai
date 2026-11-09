# black1启动时效与几何来源：旧图板端剖析

## 已验证事实和纠正

121400单绿292帧检测层有green、无black；首两帧171.824/152.248ms过期被拒绝，后290帧单green/T001。同轮console正常收尾，未见JPEG/错误告警。源码main.py在采集前已导入颜色标定，先前仅导入project_detector的Windows剖析包含额外延迟导入，不能解释真实主程序首帧超时。不得据此增加无效的预导入或宣称已修复。

24次绿色来源切换对应相邻中心最大差3.518px，每次velocity_px_s和predicted_px均为空，符合现有跨来源禁速度保护。该差不是物理位移或已标定精度。现阶段不改检测器/跟踪器，不把两种中心直接作为统一抓取点。

## 工程诊断（E）

output/p2_black1_saved_profile.py先校验4算法模块哈希和3配置来源SHA，再按真实主程序导入链加载main模块，但不调用main/initialize/open_camera。回放现存首帧及sample06，每张分别无历史与使用保存的前帧内部轨迹作提示，普通计时默认各8次；cProfile另行调用，避免把剖析开销混入普通计时。记录版本、线程数、逐次耗时、来源/中心、调用栈与脚本SHA，不更改线程/参数/150ms门、不生成动作、不打开相机或串口。已有输出拒绝覆盖，另用--output新文件名。

Windows普通CLI及阻断cv2.VideoCapture/serial.Serial构造函数的回放均通过，结果见当日black1-green30-121400/main-import-replay-windows.json与no-hardware-profile-windows.json；帮助和输入范围检查可用。仅为软件诊断验证，不代表Orange Pi耗时或端到端延迟通过。几何来源证据geometry-source-review.json。

Windows PowerShell上传：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_black1_saved_profile.py" orangepi@10.252.117.211:/home/orangepi/
```

香橙派终端（不需重拍、移动物料或启动预览）：

```bash
python3 ~/p2_black1_saved_profile.py --app ~/p2-generic120-black1-20261024/vision_app --evidence ~/p2-generic120-black1-20261024/color-follow-audits/20261024-121400-134643
```

输出默认保存~/black1-saved-profile-121400.json。拿到报告后才判断板端瓶颈；该旧图回放不能证明主机采集缓冲、GUI/记录器或首帧调度是原因，也不能验收曝光到结果延迟。已有版本保留，无需部署替换包或回退运行配置。IP是最近会话值，换网络需重新确认。

## 需求与未闭环

R已核VIS-01、FIELD-07；E已核VIS-02/03/09/11/12/13/15、CAL-02/03、RUN-06。Q QR-04等仍待技术答疑确认。板端瓶颈/启动时效、统一几何抓取点、新黑恢复分支独立实物覆盖、困难六色、毫米/随臂与真实抓放均未闭环，动作禁用。软件剖析不修改官方规则或升级实物验收。
