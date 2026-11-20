# Radar transaction ordering

2026-11-18. The installed MATLAB R2020a ran both models before corresponding product edits. The captured completion lines were:

```text
observer_order_model checks=235 PASS MATLAB 2020a
observer_model checks=528 PASS MATLAB 2020a
```

MATLAB printed nonfatal warnings about its user preference and toolbox cache files. The wrapper could not persist the raw transcript and digest receipt in the evidence directory during that run. The completion lines above were captured from the tool session.

| Model or product input | SHA256 before product edits |
| --- | --- |
| observer_order_model.m | `3bfc99dd34f9ea3a8675ee2bfacc370db67a2683ca5a6584105958792abf2ef1` |
| observer_model.m | `a039fa975d0f7e4373c2b4065929c7644789a8c01560899aeb0707e1f21654fd` |
| radar_client.c | `9fc22ec6d5e72db15a29313b22fbd99ec11b4315775a90e7d86de1dcbce65e20` |
| radar_observer.c | `f3584ea5e52ed2f7a690a77ba7a88bff780720165d360e8d5f1ae7360a823a25` |
| radar_observer.h | `6b354067c9bc9c58dd5001cbb8d8598caf4d67aa4758171bd4ffa0ad25cb646e` |

The regression covers a readiness loss followed by recovery within one owner poll, configuration admission during the active STOP transaction, and transaction events received before their matching command begins DMA TX. Unsolicited state updates remain independent of transaction completion. V6 GRID and ACK do not carry a host request ticket. The launch guard excludes frames parsed before DMA start, while delayed post-launch frames remain subject to the existing protocol checks and scan sequence freshness rules.
