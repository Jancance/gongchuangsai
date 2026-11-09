# 摄像头1.1续接：blue2板端证据复核（E）

输入 `evidence/p2-camera/2026-10-27/blue2/20261027-175625-750469.zip`，CRC通过，SHA256为 `4fd546aa583d5ff2c96bc4084077e0961a0fdd4d859c5756622bc77a95c07910`。4运行模块、2诊断模块与3配置源哈希匹配发布，3份实际配置也匹配。复核脚本 `scripts/review_camera135_blue2_20261027.py`，输出同证据目录 `review.json`；原ZIP保留，解压目录 `run-20261027-175625-750469`。

首末主机read完成跨度175.743秒，1425处理帧：1395 FOLLOWING_PIXEL，30 STALE_RESULT（2.11%）。正常帧1384次双颜色，11次仅蓝；正常帧蓝色缺失0，浅蓝缺失11（33/68/239/257/351/453/896/966/1258/1305/1348）。正常帧无蓝/浅蓝重复候选。30次过期均撤销全部目标，28次检测器已有两色、1次仅蓝、1次仅浅蓝；27次蓝丢失/重现及27次换号不能解释为蓝检测器反复漏检，最长已闭合缺失0.300秒。150ms主机年龄门保留，无动作许可、无ready。

检测调用中位/95分位/最大104.181/111.077/176.814ms；主机结果年龄124.434/146.753/185.510ms；诊断开销0.330/0.396/11.348ms。它们不是曝光到结果总延迟。与前轮场景不同，不以跨轮统计证明blue2新分支解决所有漏蓝或优化因果。

已视觉查看first-frame-raw.png与longest-blue-loss-raw.png，两物料在圆环纸上完整可见。正常蓝色1395次全部来自combined_silhouette，中心极差X/Y约0.243/0.246px；合并轮廓含物料及圆环图案，不能解释为顶面/抓点定位精度，也不能用本轮证明0.94补查分支已被触发。浅蓝1310次current_pale_circle、52次chromatic_circle、22次combined_silhouette，中心极差52.255/22px；存在候选来源切换，未证明是真实物料运动。当前保存原图不足以确定11次漏浅蓝的精确拒绝条件，亦不把中央圆环的UNKNOWN当作黑物料漏检。

## 下一项：只改观察颜色，获取真正漏浅蓝原图

保持相机、灯光、物料摆放和所有检测/跟踪参数不变。在香橙派VNC终端运行完整命令（现有相机程序已退出）：

```bash
python3 /home/orangepi/p2-generic135-rejection-audit-20261026/p2_color_follow_audit.py --app /home/orangepi/p2-generic135-blue2-20261026/vision_app --watch-color lightblue --sample-every 5 --max-samples 4 --output-dir /home/orangepi/camera135-lightblue-diagnostics
```

观察约90秒，在预览窗口按Q正常退出。目标证据为first-nonstale-lightblue-missing前后原图/JSON，过期不占该事件；同色重复候选事件仍自动保存。如果本轮未触发，如实保留，不宣称问题消失。退出后Windows PowerShell取回新目录的ZIP：

```powershell
scp "orangepi@10.252.117.211:/home/orangepi/camera135-lightblue-diagnostics/*.zip" "C:\Users\Tenderness\Documents\ChatGPT\工创赛\evidence\p2-camera\2026-10-27\lightblue\"
```

IP沿用上一轮成功取回地址，若网络切换则先核对VNC当前连接地址。运行仍像素诊断、串口及动作禁用；取证完成后先回放再决定修改，不直接放宽阈值。

已检查VIS-01（R）、VIS-02/03/09/12/13/15、CAL-03、RUN-06（E），无新增竞赛规则。官方QR-04等Q继续待技术答疑确认。未闭环：漏浅蓝原图/拒绝原因、候选中心语义与切换、过期/换号、历史重复浅蓝、独立六色困难场景、内参/残余畸变/地面毫米与随臂标定及真实抓放。当前仅台架观察，未改变机构验收状态。
