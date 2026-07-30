# Marsdog v3 — 常用命令

每条命令都带完整 `cd`，可直接复制粘贴。仓库根目录默认：

`/home/cat/project/marsdogv3_test1`

若你的路径不同，把下面所有路径改成你的仓库根即可。

---

## 0. 运行环境（pinocchio 等）— 新板必做一次

aarch64 上 PyPI **没有**可用的 Pinocchio 轮子，请用 conda 环境（已写入仓库）：

| 文件 | 作用 |
|---|---|
| `environment.yml` | 顶层依赖（日常创建环境用这个） |
| `environment.lock.yml` | 完整锁定（可选，更严复现） |
| `scripts/setup_env.sh` | 一键装 Miniforge + 创建 `marsdog` 环境 |

```bash
cd /home/cat/公共的/marsdog_v3_test3 && ./scripts/setup_env.sh
```

装好后 `./run_walk.sh` 会自动用 `~/miniforge3/envs/marsdog/bin/python`。

**不要把 `~/miniforge3` 推进 git**（数 GB）。保留本机该目录，下次换代码几乎不用重装；只有新板或删了 Miniforge 才再跑 `setup_env.sh`。

---

## 0.1 `mocap_to_real` 现在是什么？

**控制/工具代码真源已全部不在这里。**  
`mocap_to_real/*.py` = **兼容启动器/壳**（转发到 `src/marsdog_control`）。  
仍留在该目录的只有**部署附属物**：`usb_device_map.json`、`trim_cal.json`、`udev/`、`sounds/`、少量 shell/文档/图片。

| 项目 | 真源位置 |
|---|---|
| 主程序 walk | `src/marsdog_control/apps/walk.py` |
| 串口 / CAN / IMU / 手柄 | `src/marsdog_control/config/bus_config.py` |
| 总线体检 static_test | `src/marsdog_control/apps/tools/diagnostics/static_test.py` |
| 其它诊断/标定/分析/bench | `src/marsdog_control/apps/tools/**` |
| 换线映射缓存（数据文件） | `mocap_to_real/usb_device_map.json`（仍由 bus_config 读取） |

旧命令 `cd mocap_to_real && python3 XXX.py` **仍可用**（壳会跳进 src），推荐改用 `-m marsdog_control...`。

---

## 1. 打开主程序（让狗用遥控器走起来）

### 1.1 手柄行走（推荐）

```bash
cd /home/cat/project/marsdogv3_test1 && ./run_walk.sh --no-tail
```

等价写法：

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src:mocap_to_real python3 -m marsdog_control.apps.walk --no-tail
```

兼容旧入口（行为相同）：

```bash
cd /home/cat/project/marsdogv3_test1 && python3 mocap_to_real/walk.py --no-tail
```

### 1.2 悬空自检（不要手柄、不写日志）

```bash
cd /home/cat/project/marsdogv3_test1 && ./run_walk.sh --no-gamepad --no-tail --no-log
```

或：

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src:mocap_to_real python3 -m marsdog_control.apps.walk --no-gamepad --no-tail --no-log
```

### 1.3 遥控器 / 键盘怎么用

1. 机器人上电，USB-CAN / IMU / 手柄接收器都插好  
2. 用上面 §1.1 启动，等打印 `[ok] 已站立`、`IMU 闭环已启用`  
3. **手柄**：推前进摇杆 → 切入行走（默认 NaturalSoftTrot）；回中可回站立  
4. **键盘（可选）**：
   - `SPACE` / `s`：站立 ↔ 行走切换  
   - `+/-`：步频  
   - `u/d`：体高  
   - `q` / `ESC`：退出（会回站立 + 缓速失能）  
5. 结束请用 `Ctrl+C` 或 `q`，不要直接拔电

> 不要加 `--no-gamepad`，否则手柄不会生效。

---

## 2. 总线体检 `static_test`（与主程序同一通道）

只读探测各电机 / IMU / 手柄是否在线，**不会让狗走起来**。

**真源**在 `src/marsdog_control/apps/tools/diagnostics/static_test.py`（已不再住在 `mocap_to_real`）。

推荐入口：

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src python3 -m marsdog_control.apps.tools.diagnostics.static_test
```

兼容入口（薄启动器，转发到上面同一实现）：

```bash
cd /home/cat/project/marsdogv3_test1/mocap_to_real && python3 static_test.py
```

看汇总表：四肢主关节 + tarsus + 腰/颈应尽量 `ONLINE`。  
头部电机（15/16/17）离线一般不影响走路。

---

## 3. 换过 USB 口之后：重新扫串口

插拔 / 换 Hub 口后编号会变，**主程序和 static_test 都会跟着缓存走**。

```bash
# 1) 机器人已上电，只读扫描（识别角色）
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src python3 -m marsdog_control.apps.tools.diagnostics.setup_usb_devices
```

```bash
# 2) 结果正确后安装 udev 固定名（需要 sudo）
cd /home/cat/project/marsdogv3_test1 && sudo PYTHONPATH=src python3 -m marsdog_control.apps.tools.diagnostics.setup_usb_devices --install
```

```bash
# 3) 再体检确认
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src python3 -m marsdog_control.apps.tools.diagnostics.static_test
```

映射写在 `mocap_to_real/usb_device_map.json`，由 `bus_config` 统一读取。

---

## 4. 离线回归（不连真机）

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src:mocap_to_real python3 -m unittest discover -s tests -p "test_*.py"
```

期望：`Ran … OK`（含 parity 与 `src` 自洽守卫）。

### 4.1 完整质量门禁（compileall + pyflakes + 单测 + parity + coverage）

首次用需装一次开发依赖（不是运行时依赖，跟 `pyserial` 等无关）：

```bash
pip3 install --user -r /home/cat/project/marsdogv3_test1/requirements-dev.txt
```

然后随时跑：

```bash
cd /home/cat/project/marsdogv3_test1 && ./scripts/check.sh
```

提交前必过；`--fast` 跳过 coverage（改动很小、只想快速确认没炸时用）。
`.gitlab-ci.yml` 跑的是同一个脚本，仓库注册 runner 后会自动在每次 push 触发。

---

## 5. 查看当前解析到的串口（确认主程序/体检同源）

```bash
cd /home/cat/project/marsdogv3_test1 && PYTHONPATH=src:mocap_to_real python3 -c "
from marsdog_control.config import bus_config as b
import os
for n in ['LZ_CAN_A_DEVICE','LZ_CAN_B_DEVICE','EVO_CAN_DEVICE','DM_CAN_DEVICE','IMU_DEVICE','GAMEPAD_DEVICE']:
    v=getattr(b,n); print(f'{n:16} {v} -> {os.path.realpath(v) if os.path.exists(v) else \"缺失\"}')
"
```

主程序和 `static_test` 读的就是这些值。

---

## 6. 目录心智（改哪里）

| 要改什么 | 去哪 |
|---|---|
| 主入口 / CLI | `src/marsdog_control/apps/walk.py`、`walk_cli.py` |
| 稳态一拍 | `src/marsdog_control/runtime/walk_loop.py` |
| 串口路径 | `src/marsdog_control/config/bus_config.py` + `mocap_to_real/usb_device_map.json` |
| 步态 / IK | `src/marsdog_control/motion/` |
| 手柄语义 | `src/marsdog_control/input/` |
| 诊断工具 | `mocap_to_real/`（壳 + 工具；实现真源在 `src`） |

更细的重构说明见 `REFACTOR_STATUS.md`；真机流程见 `Marsdog真机部署与验证手册.md`。
