# mocap_to_real（部署资源目录）

本目录**只保留真机部署资源**（标定、音效、udev、位姿 JSON）。

原先约 83 个 Python 兼容壳（`sys.modules` 别名 / `runpy` 启动器）已迁到：

```text
legacy/mocap_to_real/
```

扁平 `from joint_config import …` 仍由 `marsdog_control.compat.ensure_legacy_path()`
把 `legacy/mocap_to_real` 放进 `sys.path` 解析；正式入口请用：

```bash
PYTHONPATH=src python -m marsdog_control.apps.walk
PYTHONPATH=src python -m marsdog_control.apps.sim.sim_walk
```

实体离线测试副本：`manual_tests/legacy/test_runtime.py`、
`manual_tests/legacy/test_imu_dm_pipeline.py`。
