# tests/ — 离线与真机测试层

## 目录

| 路径 | 含义 | 默认 CI |
|---|---|---|
| `tests/test_*.py` | 离线单元测试 | ✅ `unittest discover` |
| `tests/parity/` | 假硬件全环金样 | ✅ |
| `tests/Motor_test/` | 真机电机跟随 bench（**不是** unittest） | ❌ 人工跑 |
| `manual_tests/legacy/` | 从 mocap 迁出的旧测试/脚本（部分过期/需真机） | ❌ 不进默认 discover |

## 默认命令

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src:mocap_to_real python3 -m unittest discover -s tests -p "test_*.py"
```

## 真机诊断（不在本目录）

总线体检等真源在：

`src/marsdog_control/apps/tools/diagnostics/`

例如：

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src python3 -m marsdog_control.apps.tools.diagnostics.static_test
```
