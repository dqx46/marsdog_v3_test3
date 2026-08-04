"""固定 2D 俯视示意图 — 标注电机 ONLINE / OFFLINE + 总线。

不读 URDF mesh，坐标按四腿+头腰拓扑手摆，便于板子上快速出图。
坐标系: +Y 朝前(头), +X 朝右; 图上左侧=机器人左侧(FL/RL)。
"""

from __future__ import annotations

import os
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from marsdog_control.config.joints import JOINT_BY_ID

# results 值: (online, bus, deg, extra)
StatusMap = Mapping[int, Tuple]

# 俯视示意图节点 (x, y)；单位任意，只看相对位置。
# 腿链从髋向外: hip → thigh/roll → calf → tarsus
_NODE_XY: Dict[int, Tuple[float, float]] = {
    # 前左 FL (上左)
    1: (-1.6, 1.55),   # fl_hip_pitch
    2: (-2.15, 1.85),  # fl_thigh_roll
    3: (-2.55, 2.25),  # fl_calf
    4: (-2.85, 2.65),  # fl_tarsus
    # 前右 FR (上右)
    5: (1.6, 1.55),
    6: (2.15, 1.85),
    7: (2.55, 2.25),
    8: (2.85, 2.65),
    # 后左 RL (下左)
    9: (-1.6, -1.35),   # rl_hip
    10: (-2.15, -1.75),  # rl_thigh
    11: (-2.55, -2.15),  # rl_calf
    22: (-2.85, -2.55),  # rl_tarsus (spare)
    # 后右 RR (下右)
    12: (1.6, -1.35),
    13: (2.15, -1.75),
    14: (2.55, -2.15),
    23: (2.85, -2.55),  # rr_tarsus (spare)
    # 头 / 颈 / 腰 (中线偏前)
    18: (0.0, 0.85),    # neck_pitch
    15: (-0.45, 1.55),  # head_pitch
    16: (0.0, 1.85),    # head_yaw
    17: (0.45, 1.55),   # head_roll
    19: (-0.45, 0.15),  # waist_yaw
    20: (0.0, 0.15),    # waist_pitch
    21: (0.45, 0.15),   # waist_roll
}

# 骨架连线 (motor_id pairs) — 仅视觉引导
_EDGES: Sequence[Tuple[int, int]] = (
    (1, 2), (2, 3), (3, 4),
    (5, 6), (6, 7), (7, 8),
    (9, 10), (10, 11), (11, 22),
    (12, 13), (13, 14), (14, 23),
    (18, 16), (15, 16), (17, 16),
    (18, 20), (19, 20), (21, 20),
    (1, 20), (5, 20), (9, 20), (12, 20),
)

# 总线短名 / 颜色 / 说明 (顺序 = 图例顺序; 标题用 ASCII, 板子常无中文字体)
_BUS_META: Sequence[Tuple[str, str, str, str]] = (
    # bus_key, short, color, title
    ("lz_can_a", "lz_a", "#1565c0", "CAN-A LingZu"),
    ("incos_can", "incos", "#ef6c00", "INCOS"),
    ("lz_can_b", "lz_b", "#6a1b9a", "CAN-B LingZu"),
    ("evo_can", "evo", "#00838f", "EVO"),
    ("dm_can", "dm", "#5d4037", "DM Tarsus"),
    ("none", "spare", "#9e9e9e", "spare"),
)

_BUS_SHORT = {b: s for b, s, _, _ in _BUS_META}
_BUS_COLOR = {b: c for b, _, c, _ in _BUS_META}
_BUS_TITLE = {b: t for b, _, _, t in _BUS_META}

_COLOR_ONLINE = "#2e7d32"
_COLOR_OFFLINE = "#c62828"
_COLOR_UNKNOWN = "#9e9e9e"
_COLOR_SPARE = "#bdbdbd"
_COLOR_EDGE = "#90a4ae"
_COLOR_BODY = "#eceff1"


def _short_joint(motor_id: int) -> str:
    j = JOINT_BY_ID.get(motor_id)
    if j is None:
        return str(motor_id)
    name = j.name
    for prefix in ("fl_", "fr_", "rl_", "rr_"):
        if name.startswith(prefix):
            name = name[len(prefix):]
            break
    abbrev = {
        "hip_pitch": "hip",
        "thigh_roll": "roll",
        "thigh": "thigh",
        "calf": "calf",
        "tarsus": "tars",
        "head_pitch": "h_p",
        "head_yaw": "h_y",
        "head_roll": "h_r",
        "neck_pitch": "neck",
        "waist_yaw": "w_y",
        "waist_pitch": "w_p",
        "waist_roll": "w_r",
        "hip": "hip",
    }.get(name, name[:6])
    return f"{motor_id}:{abbrev}"


def _bus_of(motor_id: int, results: StatusMap) -> str:
    if motor_id in results and results[motor_id][1]:
        return str(results[motor_id][1])
    j = JOINT_BY_ID.get(motor_id)
    return j.bus if j else "unknown"


def _bus_ids(bus: str) -> List[int]:
    return sorted(mid for mid, j in JOINT_BY_ID.items() if j.bus == bus)


def _bus_status_line(bus: str, results: StatusMap) -> Tuple[str, str]:
    """返回 (图例一行文字, 文字颜色)。整总线 bus_fail 时标红。"""
    ids = _bus_ids(bus)
    if not ids:
        return f"{_BUS_TITLE.get(bus, bus)}", "#455a64"
    probed = [mid for mid in ids if mid in results]
    if not probed:
        id_str = ",".join(str(i) for i in ids)
        short = _BUS_SHORT.get(bus, bus)
        return f"{short}  {_BUS_TITLE.get(bus, bus)}  [{id_str}]", "#455a64"

    n_on = sum(1 for mid in probed if results[mid][0])
    n_tot = len(probed)
    all_fail = all(
        (not results[mid][0]) and str(results[mid][3]) == "bus_fail"
        for mid in probed
    )
    id_str = ",".join(str(i) for i in ids)
    short = _BUS_SHORT.get(bus, bus)
    title = _BUS_TITLE.get(bus, bus)
    if all_fail:
        return f"{short}  {title}  [{id_str}]  BUS FAIL", _COLOR_OFFLINE
    if n_on < n_tot:
        return (
            f"{short}  {title}  [{id_str}]  {n_on}/{n_tot}",
            _COLOR_OFFLINE,
        )
    return f"{short}  {title}  [{id_str}]  {n_on}/{n_tot}", "#1b5e20"


def render_motor_status_figure(
    results: StatusMap,
    *,
    out_path: str,
    title: Optional[str] = None,
    show: bool = False,
    include_spare: bool = False,
) -> Optional[str]:
    """根据 static_test results 画俯视示意图并保存 PNG。

    Returns:
        成功时返回输出路径；matplotlib 不可用时返回 None。
    """
    try:
        import matplotlib
        if show:
            try:
                matplotlib.use("TkAgg")
            except Exception:
                matplotlib.use("Agg")
        else:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
        from matplotlib.patches import Circle, FancyBboxPatch
    except ImportError:
        print("[viz] matplotlib 不可用，跳过示意图")
        return None

    ids = sorted(_NODE_XY.keys())
    if not include_spare:
        ids = [i for i in ids if JOINT_BY_ID.get(i) and JOINT_BY_ID[i].bus != "none"]

    fig, ax = plt.subplots(figsize=(10.5, 11.0), dpi=120)
    ax.set_aspect("equal")
    ax.set_xlim(-3.7, 4.2)
    ax.set_ylim(-3.35, 3.55)
    ax.axis("off")

    # 机身示意框
    body = FancyBboxPatch(
        (-1.05, -0.95), 2.1, 2.15,
        boxstyle="round,pad=0.08,rounding_size=0.25",
        linewidth=1.2, edgecolor="#78909c", facecolor=_COLOR_BODY, zorder=0,
    )
    ax.add_patch(body)
    ax.text(0.0, 0.55, "BODY", ha="center", va="center",
            fontsize=9, color="#607d8b", zorder=1)

    # 区域标签
    for txt, xy in (
        ("FL", (-2.55, 1.25)),
        ("FR", (2.55, 1.25)),
        ("RL", (-2.55, -0.95)),
        ("RR", (2.55, -0.95)),
        ("HEAD ↑", (0.0, 2.55)),
        ("REAR ↓", (0.0, -2.95)),
    ):
        ax.text(xy[0], xy[1], txt, ha="center", va="center",
                fontsize=8, color="#546e7a", fontweight="bold", zorder=1)

    # 骨架线
    drawn = set(ids)
    for a, b in _EDGES:
        if a not in drawn or b not in drawn:
            continue
        xa, ya = _NODE_XY[a]
        xb, yb = _NODE_XY[b]
        ax.plot([xa, xb], [ya, yb], color=_COLOR_EDGE, lw=1.4, zorder=2)

    offline_ids = []
    for mid in ids:
        x, y = _NODE_XY[mid]
        j = JOINT_BY_ID.get(mid)
        spare = bool(j and j.bus == "none")
        bus = _bus_of(mid, results)
        bus_color = _BUS_COLOR.get(bus, "#37474f")

        if mid in results:
            online = bool(results[mid][0])
            color = _COLOR_ONLINE if online else _COLOR_OFFLINE
            if not online:
                offline_ids.append(mid)
        elif spare:
            color = _COLOR_SPARE
            online = None
        else:
            color = _COLOR_UNKNOWN
            online = None

        r = 0.13 if online is False else 0.11
        # 外圈 = 总线色；填充 = 在线状态
        ring = Circle(
            (x, y), r + 0.035, facecolor=bus_color, edgecolor="none", zorder=3)
        ax.add_patch(ring)
        face = Circle(
            (x, y), r,
            facecolor=color,
            edgecolor="#b71c1c" if online is False else "#263238",
            linewidth=1.6 if online is False else 0.8,
            zorder=4,
        )
        ax.add_patch(face)

        joint_lbl = _short_joint(mid)
        bus_lbl = _BUS_SHORT.get(bus, bus[:6])
        tc = _COLOR_OFFLINE if online is False else "#263238"
        weight = "bold" if online is False else "normal"
        ax.annotate(
            joint_lbl, (x, y), textcoords="offset points",
            xytext=(0, 10), ha="center", va="bottom",
            fontsize=7.5, color=tc, fontweight=weight, zorder=5,
        )
        ax.annotate(
            bus_lbl, (x, y), textcoords="offset points",
            xytext=(0, -12), ha="center", va="top",
            fontsize=6.5, color=bus_color, fontweight="bold", zorder=5,
        )

    n_online = sum(1 for mid, v in results.items() if v[0])
    n_total = len(results)
    n_off = n_total - n_online
    if title is None:
        title = f"Marsdog motor status  ({n_online}/{n_total} online"
        if n_off:
            title += f", {n_off} OFFLINE"
        title += ")"
    ax.set_title(title, fontsize=12, pad=10)

    # 状态图例 (右下)
    legend = [
        Line2D([0], [0], marker="o", color="w", label="ONLINE",
               markerfacecolor=_COLOR_ONLINE, markeredgecolor="#37474f", markersize=9),
        Line2D([0], [0], marker="o", color="w", label="OFFLINE",
               markerfacecolor=_COLOR_OFFLINE, markeredgecolor="#b71c1c", markersize=11),
        Line2D([0], [0], marker="o", color="w", label="not probed",
               markerfacecolor=_COLOR_UNKNOWN, markeredgecolor="#37474f", markersize=9),
    ]
    ax.legend(handles=legend, loc="lower right", framealpha=0.9, fontsize=8)

    # 总线图例 (右上)：色点 + 短名 + ID + 在线统计
    ax.text(
        4.05, 3.35, "BUS",
        ha="right", va="top", fontsize=9, fontweight="bold", color="#37474f",
        zorder=6,
    )
    y0 = 3.05
    bus_row = 0
    for bus, _short, color, _title in _BUS_META:
        if bus == "none" and not include_spare:
            continue
        if not _bus_ids(bus) and bus != "none":
            continue
        line, tc = _bus_status_line(bus, results)
        yy = y0 - bus_row * 0.28
        ax.plot(2.15, yy, "o", color=color, markersize=8, zorder=6)
        ax.text(
            2.35, yy, line, ha="left", va="center",
            fontsize=7.2, color=tc, family="monospace", zorder=6,
        )
        bus_row += 1

    ax.text(
        4.05, y0 - bus_row * 0.28 - 0.08,
        "ring=bus  fill=online/offline",
        ha="right", va="top", fontsize=7, color="#607d8b", zorder=6,
    )

    if offline_ids:
        lines = []
        for mid in offline_ids:
            j = JOINT_BY_ID.get(mid)
            name = j.name if j else "?"
            bus = _bus_of(mid, results)
            short = _BUS_SHORT.get(bus, bus)
            lines.append(f"{mid} {name}  [{short}]")
        ax.text(
            -3.55, -3.2, "OFFLINE:\n" + "\n".join(lines),
            ha="left", va="bottom", fontsize=8, color=_COLOR_OFFLINE,
            family="monospace",
            bbox=dict(boxstyle="round,pad=0.35", facecolor="#ffebee",
                      edgecolor=_COLOR_OFFLINE, alpha=0.95),
            zorder=6,
        )

    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out_path, bbox_inches="tight", facecolor="white")
    if show:
        try:
            plt.show()
        except Exception as exc:  # noqa: BLE001 — 无显示环境时忽略
            print(f"[viz] show 失败({exc}); 已保存 {out_path}")
    plt.close(fig)
    return out_path


def default_plot_path() -> str:
    """默认写到仓库根下 static_test_status.png。"""
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.normpath(os.path.join(here, "..", "..", "..", "..", ".."))
    return os.path.join(root, "static_test_status.png")
