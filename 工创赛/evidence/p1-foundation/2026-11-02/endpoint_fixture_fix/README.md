# 端点仿真夹具修正证据入口

最终结果见 [final/RESULT.md](final/RESULT.md) 和 [final/final_review.json](final/final_review.json)。

最终夹具同时修正合法反弹的ACK目标判断，以及持续异常在STOP/SETTLE期间提前结束注入的问题。
无前馈、零系数、1%候选各40场景，共120模型、3,472,244条断言符合预期。

本目录第一轮 full_*、RESULT.md、final_review.json 是预备结果，保留追溯；其中1%候选的持续异常场景尚未持续注入到停车阶段，不作为最终验收证据。
quick_010 是原单次反弹失败场景的先行验证。

旧夹具的失败记录保留在同级 longitudinal_feedforward_v4，不覆盖或改写。
本轮没有修改主工程或独立候选固件，没有访问硬件，没有提交或推送GitHub。
