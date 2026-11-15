# 独立观测候选 run2：通信失败，单列保存

2026-11-15；同一候选 `0ebf9e4e…874b3`，本趟无下载、统一reset一次。fresh HWT −0.005°预检健康；PLAN8、ARM、START各发送一次，之后ack_timeout。既有client发送KEEP×8、超时安全STOP×1，没有自动重发START，也未读正常尾部快照。

后续经另行授权，只进行一次running-core只读采集，无halt/reset/resume、控制/VM或外设寄存器写入，也未读USART DR。此时START实际已accepted3，RX消费67 B且前后不变、link_lost1/result8；任务、CAN任务、喂狗及TX计数继续增加，VM/PE2为0、backend idle/session0，RTOS/safety/assert/CFSR/HFSR为0。USART/PA9/10/RCC状态保存在`failure_read_receipt.json`。

只读采集比失败链退出晚约331秒，中间包含用户中断与授权/摆位确认。它是此后读取到的保存状态，不是假称失败瞬间同步读数。最初最后TM只显示accepted2和VM0，不能代表失联后的实时状态；实时读取补证START已消费，但后续KEEP/STOP未见MCU RX消费。USB/COM/USART的具体原因未定。

本趟不计入位置精度比较。用户随后单独确认停稳摆好并授权的run3另目录记录，不是本趟自动重试。原始raw/jsonl、发送日志、once markers与失败回执全部保留；派生结论见`failure_analysis.json`。
