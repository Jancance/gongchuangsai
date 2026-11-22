# GitHub最新版本已同步（2026-11-20）

- 远端main：a4409b05ce404b57e9d0807061d8ddc946121d77，已联网fetch并fast-forward，完整检出28252个受控文件。
- 正式工程：D:\新建文件夹4\工创赛\代码\stm32\3.stm32f407工程/gongxun-f407.uvprojx；目标改用Vehicle_Integrated（正式）或Vehicle_Offline（离线）。旧Vehicle_LiftRuler不是此版目标。
- 旧工程完整备份：D:\新建文件夹4\工创赛\代码\stm32\3.stm32f407工程.before_github_20261120-223836。
- 完整仓库本地副本：C:\Users\Tenderness\Documents\Codex\2026-11-19\hi-2\work\gongchuangsai_publish_20261120；该副本与GitHub本次提交一致，工作区干净。
- D:/Codex/工创赛下现有源码/配置/交接数据已按远端更新，所有被替换的不同文件先保存在D:\Codex\工创赛\tmp\before_github_pull_20261120-223836；本地独有试验不删除。

## 本次验证

正式路径2264个受控工程文件与远端逐项SHA256一致。Vehicle_Integrated和Vehicle_Offline重建均0错误0警告。Integrated AXF SHA256：5b165562fa6d848bb69c5c5b536b6e4575252fb3ff8fad43ff063dd2e7bd5e71。整图Pi包full_map_pi_20261120的60个清单文件均通过SHA256检查。本次仅同步和构建，没有烧录、复位、连接串口或启动小车，也没有部署Pi。

## 现在先做什么

下一步先准备香橙派整图视觉，不直接按PC13全图试跑。

1. 香橙派上电联网，准备HDMI或已有远程桌面；保持车辆停稳，不按PC13。先核旧视觉自启、实际相机和串口owner以及现场HSV配置，再按FULL_MAP.md切换整图程序，备份并保留原HSV。
2. 完成粗加工、暂存平放和码垛的占位样本，以及暂存格22的目标/微调响应标定。当前storage-align.json中target_confirmed_by_user=false、response_measured_on_vehicle=false；storage-observation.json的模式2/3/4均verified=false、rings为空。因此新版有意不会凭空判放稳，当前不能把软件编译通过当全流程可发车。
3. 上位视觉就绪后，由用户自行烧录Vehicle_Integrated，再先做区2分站联调：扫码→原料三抓装盘→粗加工三放三收，随后验证暂存底层与码垛，最后双批八段回区。实际两区/三分钟验收另记录；旧170.940秒纯底盘不等于带抓放整场用时。

最新源码已包含APP_ZONE2_MISSION_ENABLE=1的双批调度与站点票据。旧记录的“模块未接路线”是此前版本边界，不能移作此版状态；同样，新版整图源码已经接入也不代表整图实物已通过。上次机械臂调参/补偿数据保存在完整旧工程备份及本聊天历史证据中，未丢弃。暂存对位以粗加工数值为候选起点，必须现场标定，不能直接将verified改true绕过。

## 最新记录入口

- 正式工程旁GitHub最新交接/长期记忆.md（历史按日期区分）。
- 完整仓库/工创赛/evidence/integration/2026-11-20/source-rough-merge/FULL_MAP_PLAN.md。
- D:/Codex/工创赛/output/full_map_pi_20261120/FULL_MAP.md（整图视觉安装、调色及未验项）。

远端README中的D:/Desktop/工创赛是提交者原机路径；本机仍按用户指定D盘正式工程路径使用。本次不修改远端仓库，不将未验收内容标成已部署。
