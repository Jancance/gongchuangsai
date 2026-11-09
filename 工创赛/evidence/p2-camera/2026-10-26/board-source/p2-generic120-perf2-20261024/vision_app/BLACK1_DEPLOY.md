# 120°黑料静态漏检：black1候选

来源：112215静态30秒的356帧中14帧漏黑，原图sample-02-before-raw对应第62帧；111235包含移动的记录另有黑料漏检和过期。两张漏检图均在ring拒绝开/关时复现既有形状/边缘恢复失败。此次只处理当前黑料候选生成，不签署动态时效或物理抓取通过。

## 修订与适用边界（E）

从ring1复制完整程序到独立black1目录，只改project_detector.py，并在material_color_thresholds.json新增black_face_support_candidate.enabled。原目录、六色HSV、35%有效像素门、150ms时效门、相机初始化/曝光流程、跟踪器、六模式和四工具保留。

仅在原检测及旧黑色边缘恢复均没有black时，搜索最多两块当前帧紧凑拒绝轮廓。灰度半分辨率Hough只提供圆假设；颜色/边缘仍在原图核验。圆面要求黑票≥65%、黑范围外其他单色票≤10%、有效像素比例≥35%且至少500像素、当前父轮廓支持≥50%、72方向至少75%有符合强度/方向的梯度边缘、残差≤max(2px,半径4%)。多个可支持圆拒绝为未知；不使用旧框直接输出或复用旧中心。以上为工程候选，不是竞赛规则。

通过时返回当前父轮廓凸包候选，附face_evidence。凸包中心仍可能包含侧壁/印刷环，明确为current_parent_hull_not_calibrated_grasp_point，不能用于机械臂抓取。保留原UNKNOWN拒绝行；新候选最后仍经过ring1环纹检查。来源切换由原跟踪器标记geometry_reference_changed，原跟随器不跨不同几何来源计算速度。旧轨迹年龄、未知身份、最终新鲜度和动作禁用均保留。

已存原图仅为开发集；缩小图中侧壁可能形成第二个有效暗圆，此时保持未知。尚未证明任意缩放、斜视、反光、遮挡或六色独立泛化性能。候选增加了计算，Windows离线耗时不能作为香橙派性能证明，板端必须继续检查150ms拒绝。

## 下一项：独立目录板端30秒

保持相机/支架/纸/双物料静止、光照不变，关闭其他相机预览。Windows PowerShell上传：

```powershell
scp "C:\Users\Tenderness\Documents\ChatGPT\工创赛\output\p2_vision_generic120_black1_20261024.zip" orangepi@10.252.117.211:/home/orangepi/
```

香橙派终端：

```bash
python3 -m zipfile -e ~/p2_vision_generic120_black1_20261024.zip ~/p2-generic120-black1-20261024
python3 ~/p2-generic120-black1-20261024/vision_app/run.py --check
python3 -u ~/p2_green83_ring_capture.py --app ~/p2-generic120-black1-20261024/vision_app --seconds 30 2>&1 | tee ~/black1-green-black-static30-console.log
```

目标目录若已有现场修改，用全新目录名并同步--app，不覆盖。IP为最近用户确认值，换网重查。取证沿用已部署83包装器及两脚本，无须新传入口。watch green仍记录所有颜色/候选，不能以longest_green_loss=0推断black无失检；须回传Evidence ZIP与控制台日志，逐帧核查black/green输出、ID、重捕获未知、时效及新分支证据。

回退：退出black1，使用原ring1目录运行同包装器；或关闭black_face_support_candidate.enabled。关闭开关的检测结果已做ring1完全一致验证。不要复制新检测器覆盖旧目录。

## 验证与未闭环

9项定向测试包含两张真实漏检图无需历史即可恢复、两轮所有去重绿黑原图、两轮印刷环负样本、开关关闭一致、历史六色及旧黑料恢复、蓝高光/空场/旧提示拒绝、两个支持圆/边界/无暗色证据拒绝、当前凸包跟踪、旋转及缩小歧义拒绝。测试数据数量、包哈希、六模式/四工具/check/配置核对见evidence/p2-camera/2026-10-24/black1/validation.json；不是全部处理帧原图或独立实物验收。

已检查R VIS-01/FIELD-07；E VIS-02/03/09/11/12/13/15、CAL-02/03、RUN-06。Q QR-04等继续标注待技术答疑确认。black1板端连续检出/耗时、空环误报再验证、困难六色、毫米/随臂补偿及真实抓放未闭环；串口/动作禁用。构建脚本scripts/build_p2_generic120_black1.py，源片段scripts/p2_black_face_candidate.py，测试tests/test_generic120_black1.py。
