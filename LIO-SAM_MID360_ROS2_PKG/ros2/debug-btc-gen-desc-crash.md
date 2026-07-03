# Debug Session: btc-gen-desc-crash

**Status**: [OPEN]
**Date**: 2026-06-29
**Symptom**: `alaserPGO` node crashes with exit code -11 (SIGSEGV) during `GenerateBtcDescs` for frame 0 (first keyframe, 1189 cloud points).
**Root Cause**: TBD

## Hypotheses

| ID | Hypothesis | Status |
|----|-----------|--------|
| A | 空指针/野指针：`GenerateBtcDescs` 内部访问了未初始化的指针（如 `voxel_map` 中的 `OctoTree*`） | pending |
| B | `init_voxel_map` 的 OpenMP 并行化导致数据竞争，`voxel_map` 中 `OctoTree*` 指针被破坏 | pending |
| C | 点云数据异常：`input_cloud` 为空或包含 NaN/Inf 点，导致后续计算崩溃 | pending |
| D | `config_setting_` 未正确初始化，导致关键参数为默认值0引发除零或越界 | pending |
| E | `data_base_` 或 `frame_positions_` 等成员变量在 `GenerateBtcDescs` 调用前未初始化 | pending |

## Evidence Log

(To be populated after instrumentation)