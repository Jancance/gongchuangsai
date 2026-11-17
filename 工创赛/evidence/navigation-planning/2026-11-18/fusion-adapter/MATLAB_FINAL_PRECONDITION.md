# MATLAB最终前置：v8

2026-11-18，最终模型 `verify_fusion_adapter_v8.m`，实际本机MATLAB R2020a `9.8.0.1323502`，740检查/120向量，exit0。开始01:32:22.886335+08:00，结束01:32:29.791944+08:00。脚本SHA256 `81888abffe3faee2fec80cfca6c5509bf02d361f2f6cf84cb4728522a1dcd356`。

完整M1..M7检查范围见 `MATLAB_V7_REPORT.md`；v8只修正同绑定/同begin/同sample核心冲突与updates回退组合优先级：先核心冲突拒绝且state不变，核心相同后才允许计数回退prime。新增该单反例，M4从26到27，总计M1=37、M2=12、M3=71、M4=27、M5=35、M6=22、M7=536，总和740。

实际运行证据 `matlab_receipt_v8.json` / `matlab_v8.log` / `matlab_summary_v8.json`；回执明确执行前后产品fusion_adapter.c/.h均不存在。MATLAB向量 `adapter_vectors_v8.csv` 与v7一致（SHA256 `31e85fe56dd8e6c2501bfe6241a977c0613a84ac619155b00cc969976a82ee8b`），量化结果 `quantization_scan_v8.csv`。初版与v2..v7全部保留，不覆盖失败证据或先前成功回执。

仍只是pure copied-value ESTIMATED前置；无runtime caller、真实轮速/HWT瞬时速度、雷达bundle、A/J执行或物理停稳证明。真实ARM C/Reference集成及NULL合同后做；root/high release前不写产品。
