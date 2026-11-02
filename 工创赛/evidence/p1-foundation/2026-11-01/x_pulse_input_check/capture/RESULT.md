## 2026-11-01 左前P_Pul关闭后准备单变量采集

用户反馈左前已将P_Pul改PUL_OFF，尚待读回。新独立tmp/x_lf_puloff_capture_20261101基于先前空轴被动采集版，默认VM0，白名单只读1A/3B/42核对X闭环/校准编码器ready/P_Pul0后才采43系统状态。PID和其他驱动参数不写、无运动或使能命令；Safety/Power及30RPM/30度附加阈值、触发后2秒关闭均保持。等待扰动窗口由30秒延长60秒，为现场读提示预留时间，无限重试/自动重启均无。原采集与证据不覆盖。

fetch成功HEAD=origin/main。关闭菜单请求并读回PE2低，随后烧录新镜像9998d89d5b0c80fdfc4e97c50ea0a6256057146fb533159575760322854634d8，43844字节回读一致，默认VM请求0/PE2低，主控无故障；本次增量构建0错0警告。证据menu_vm_off.json、download_vm_off.json、prepare.json、build.log。待显式启动后读回preflight，再让用户对固定左前空轴施加一次扰动。尚未证明关闭脉冲口能改善振荡。R/E/Q沿用，不删保护；仅本地未上传。

