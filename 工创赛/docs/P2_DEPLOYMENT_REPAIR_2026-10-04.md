# 桌面工作台中文文件名解压修复

日期：2026-10-04。来源：用户粘贴的香橙派终端日志，原附件为`D:/Codex/home/attachments/79b57118-8876-43b6-90fe-c29f8869311a/已粘贴的文本.txt`。

2026-10-05更新：用户板端截图确认恢复6文件，--check正常返回1280×960、串口DISABLED、去畸变false、毫米几何未验证。文件名修复及板端配置/模块检查已闭环，证据见`evidence/p2-integrated-20261005-startup/board-check.png`。下文10月4日“待回传”为历史状态；新版ZIP的实际unzip兼容行为仍未实测，用户用原包Python补齐成功。下一步为F桌面相机预览，尚无新工作台实时采集/跟踪证据。

## 现象与证据

板端已收到10月3日ZIP并完成unzip，但中文文件名显示西里尔字母与框线字符，`run.py --check`在导入`颜色标定`时报ModuleNotFoundError。日志中的乱码逐字匹配原UTF-8文件名按CP866解码的结果。原ZIP实际有UTF-8标志且Python读取文件名正确，因此不是源码缺失。用户当前IPv4为10.31.154.211，仅为本次会话观测，不作为固定地址。

## 当前用户下一步

在RealVNC香橙派终端完整复制以下代码。只恢复ZIP中尚不存在的正确文件名，保留已存在文件和用户配置；不删除乱码文件，不安装模块，不开相机或串口。输入日志目前没有显示自定义代码修改。

```bash
python3 - <<'PY'
from pathlib import Path
from zipfile import ZipFile
target = Path('/home/orangepi/p2-integrated-20261003')
with ZipFile('/home/orangepi/vision_app_decxin_2026-10-03.zip') as archive:
    if archive.testzip() is not None:
        raise RuntimeError('ZIP integrity check failed')
    count = 0
    for item in archive.infolist():
        if not (target / item.filename).exists():
            archive.extract(item, target)
            count += 1
    print('Restored files:', count)
PY
python3 /home/orangepi/p2-integrated-20261003/vision_app/run.py --check
```

预计恢复6个中文文件。检查应返回image_size=[1280,960]、serial=DISABLED、undistort=false、metric_geometry_verified=false，且无Traceback。具体恢复数量取决于已存在文件，数量本身不是通过条件。板端检查通过后另行启动F桌面预览；当前无需装机或机械臂。

## 本地修复与验证

- `scripts/package_integrated_vision.py`给中文路径增加Info-ZIP Unicode Path扩展字段0x7075（版本1、原文件名字节CRC32、UTF-8路径），并保留UTF-8标志。默认输出新版`output/vision_app_decxin_2026-10-04.zip`；10月3日原包保留。应用源码、阈值、相机和安全门禁未修改。
- 新版45个应用文件，151179字节；SHA256为`af956baef3462bdc6d8b5584a14577754f259cf80544d4397689bd173081e6c9`。ZIP CRC和逐文件哈希通过。
- 新增3项包装回归通过：Unicode字段在本地/中央目录一致且CRC有效；源内容/清单一致；补齐正确工具名并保留已改配置及旧乱码文件。
- 独立临时目录用10月3日真实包模拟CP866错名，复现ModuleNotFoundError；按以上Python补齐方式恢复6文件后，`--check`返回正常JSON且退出码0。
- 本轮未重复全部70项视觉算法测试，因为仅打包脚本及文档改变。新版在用户实际unzip上的兼容行为尚未确认，Python修复在本地已验证，用户板端结果待回传。

## 基线与未闭环

已检查VIS-01（R，六色入口保留）、VIS-03/12与RUN-06（E，本机参数、跟踪新鲜度和故障拒绝保持）；QR-04（Q）继续待技术答疑确认。本轮证据是用户日志、编码复现、3项软件测试和模拟修复后--check；不能据此认定板端采集、桌面跟踪、装机、毫米标定、机械臂跟随抓取或场地验收通过。到装机/机械臂/场地切换前仍需提醒用户。
