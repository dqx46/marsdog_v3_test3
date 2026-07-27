#!/usr/bin/env python3
"""Generate Marsdog control architecture diagram as Visio .vsdx."""

from __future__ import annotations

import zipfile
from pathlib import Path
from xml.sax.saxutils import escape

OUT = Path(__file__).with_name("marsdog_control_architecture.vsdx")

# Page size (inches)
PAGE_W, PAGE_H = 16.0, 11.0


def cell(n: str, v, u: str | None = None, f: str | None = None) -> str:
    attrs = [f'N="{n}"', f'V="{v}"']
    if u is not None:
        attrs.append(f'U="{u}"')
    if f is not None:
        attrs.append(f'F="{f}"')
    return "<Cell " + " ".join(attrs) + "/>"


def rect(
    sid: int,
    x: float,
    y: float,
    w: float,
    h: float,
    text: str,
    *,
    fill: str = "#DEEBF7",
    line: str = "#2F5496",
    bold: bool = False,
    font_size_pt: float = 10.0,
) -> str:
    """Axis-aligned rounded rectangle; (x,y) is center in inches (Visio Pin)."""
    size_in = font_size_pt / 72.0  # Visio Character.Size is in inches (pt/72)
    weight = 1 if bold else 0
    txt = escape(text).replace("\n", "&#10;")
    return f"""
    <Shape ID="{sid}" NameU="Box_{sid}" Type="Shape" LineStyle="3" FillStyle="3" TextStyle="3">
      {cell("PinX", x, "IN")}
      {cell("PinY", y, "IN")}
      {cell("Width", w, "IN")}
      {cell("Height", h, "IN")}
      {cell("LocPinX", w / 2, "IN", "Width*0.5")}
      {cell("LocPinY", h / 2, "IN", "Height*0.5")}
      {cell("Angle", 0)}
      {cell("FlipX", 0)}
      {cell("FlipY", 0)}
      {cell("ResizeMode", 0)}
      {cell("FillForegnd", fill)}
      {cell("FillBkgnd", "#FFFFFF")}
      {cell("FillPattern", 1)}
      {cell("LineWeight", "0.01", "IN")}
      {cell("LineColor", line)}
      {cell("LinePattern", 1)}
      {cell("Rounding", 0.05, "IN")}
      {cell("VerticalAlign", 1)}
      {cell("Para.HorzAlign", 1)}
      {cell("TxtPinX", w / 2, "IN", "Width*0.5")}
      {cell("TxtPinY", h / 2, "IN", "Height*0.5")}
      {cell("TxtWidth", w, "IN", "Width*1")}
      {cell("TxtHeight", h, "IN", "Height*1")}
      <Section N="Character">
        <Row IX="0">
          {cell("Font", "Microsoft YaHei")}
          {cell("Color", "#1F1F1F")}
          {cell("Style", weight)}
          {cell("Size", f"{size_in:.6f}", "IN")}
        </Row>
      </Section>
      <Section N="Geometry" IX="0">
        <Cell N="NoFill" V="0"/>
        <Cell N="NoLine" V="0"/>
        <Cell N="NoShow" V="0"/>
        <Cell N="NoSnap" V="0"/>
        <Row T="RelMoveTo" IX="1">
          <Cell N="X" V="0" F="Width*0"/><Cell N="Y" V="0" F="Height*0"/>
        </Row>
        <Row T="RelLineTo" IX="2">
          <Cell N="X" V="1" F="Width*1"/><Cell N="Y" V="0" F="Height*0"/>
        </Row>
        <Row T="RelLineTo" IX="3">
          <Cell N="X" V="1" F="Width*1"/><Cell N="Y" V="1" F="Height*1"/>
        </Row>
        <Row T="RelLineTo" IX="4">
          <Cell N="X" V="0" F="Width*0"/><Cell N="Y" V="1" F="Height*1"/>
        </Row>
        <Row T="RelLineTo" IX="5">
          <Cell N="X" V="0" F="Width*0"/><Cell N="Y" V="0" F="Height*0"/>
        </Row>
      </Section>
      <Text>{txt}</Text>
    </Shape>"""


def connector(
    sid: int,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    *,
    color: str = "#5B5B5B",
    label: str = "",
) -> str:
    """Straight connector from (x1,y1) to (x2,y2); pin at midpoint."""
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    dx, dy = x2 - x1, y2 - y1
    import math

    length = math.hypot(dx, dy) or 0.01
    angle = math.atan2(dy, dx)
    # Visio 1-D shape: BeginX/EndX relative to pin
    bx, by = x1 - cx, y1 - cy
    ex, ey = x2 - cx, y2 - cy
    txt = escape(label) if label else ""
    text_xml = f"<Text>{txt}</Text>" if label else "<Text/>"
    return f"""
    <Shape ID="{sid}" NameU="Conn_{sid}" Type="Shape" LineStyle="0" FillStyle="0" TextStyle="0">
      {cell("PinX", cx, "IN")}
      {cell("PinY", cy, "IN")}
      {cell("Width", length, "IN")}
      {cell("Height", 0, "IN")}
      {cell("LocPinX", length / 2, "IN", "Width*0.5")}
      {cell("LocPinY", 0, "IN", "Height*0.5")}
      {cell("Angle", angle)}
      {cell("BeginX", x1, "IN")}
      {cell("BeginY", y1, "IN")}
      {cell("EndX", x2, "IN")}
      {cell("EndY", y2, "IN")}
      {cell("LineWeight", "0.012", "IN")}
      {cell("LineColor", color)}
      {cell("LinePattern", 1)}
      {cell("BeginArrow", 0)}
      {cell("EndArrow", 4)}
      {cell("EndArrowSize", 2)}
      {cell("ObjType", 2)}
      <Section N="Geometry" IX="0">
        <Cell N="NoFill" V="1"/>
        <Cell N="NoLine" V="0"/>
        <Row T="MoveTo" IX="1">
          <Cell N="X" V="{bx}" U="IN"/><Cell N="Y" V="{by}" U="IN"/>
        </Row>
        <Row T="LineTo" IX="2">
          <Cell N="X" V="{ex}" U="IN"/><Cell N="Y" V="{ey}" U="IN"/>
        </Row>
      </Section>
      {text_xml}
    </Shape>"""


def build_page_shapes() -> str:
    shapes: list[str] = []
    sid = 1

    def add_rect(*args, **kwargs):
        nonlocal sid
        shapes.append(rect(sid, *args, **kwargs))
        sid += 1
        return sid - 1

    def add_conn(*args, **kwargs):
        nonlocal sid
        shapes.append(connector(sid, *args, **kwargs))
        sid += 1
        return sid - 1

    # Title
    add_rect(
        8.0,
        10.4,
        14.5,
        0.55,
        "Marsdog Control 程序框架   |   marsdog_control  +  mocap_to_real",
        fill="#D6DCE4",
        line="#1F4E79",
        bold=True,
        font_size_pt=14,
    )

    # Row Y coordinates (bottom-origin inches)
    y_app = 9.4
    y_rt = 8.2
    y_pipe = 6.55
    y_motion = 4.85
    y_ctrl = 3.55
    y_hw = 2.15
    y_note = 0.85

    # === Entry ===
    add_rect(2.2, y_app, 3.2, 0.7, "入口 apps/walk.py\nRuntimeApp", fill="#FFF2CC", line="#BF8F00", bold=True)
    add_rect(6.0, y_app, 3.4, 0.7, "默认 --legacy-loop\nmocap_to_real/walk.main", fill="#FCE4D6", line="#C65911", bold=True)
    add_rect(10.0, y_app, 3.4, 0.7, "--no-legacy-loop\nRuntimePipeline (未接管)", fill="#E2EFDA", line="#548235")
    add_rect(13.8, y_app, 2.8, 0.7, "compat.py\n新旧 import 别名", fill="#EDEDED", line="#666666")

    add_conn(2.2, y_app - 0.35, 2.2, y_rt + 0.4)
    add_conn(6.0, y_app - 0.35, 6.0, y_rt + 0.4, color="#C65911")

    # === Runtime / Config ===
    add_rect(2.2, y_rt, 3.2, 0.8, "runtime/\nFSM · startup · shutdown", fill="#DDEBF7", line="#2F5496", bold=True)
    add_rect(6.0, y_rt, 3.4, 0.8, "config/\nschema · joints · devices", fill="#DDEBF7", line="#2F5496")
    add_rect(10.0, y_rt, 3.4, 0.8, "core/types\nRobotState · UserCommand\nMotionTarget · ControlOutput", fill="#D6DCE4", line="#405E7A", bold=True)
    add_rect(13.8, y_rt, 2.8, 0.8, "io/\ninput · logging", fill="#DDEBF7", line="#2F5496")

    # === Pipeline strip ===
    add_rect(
        8.0,
        y_pipe + 1.05,
        14.5,
        0.35,
        "固定实时管线 RuntimePipeline.tick()  （目标架构；真机默认仍走 legacy 循环内等价步骤）",
        fill="#FFF2CC",
        line="#BF8F00",
        font_size_pt=9,
    )

    pipe = [
        (1.5, "1 Input\npoll 命令"),
        (3.7, "2 FSM\n模式转移"),
        (5.9, "3 Balance\nIMU 调平"),
        (8.1, "4 Motion\n足端轨迹+IK"),
        (10.3, "5 Safety\n限位/摔倒"),
        (12.5, "6 Executor\n增益+前馈"),
        (14.6, "7 Hardware\nsend MIT"),
    ]
    centers = []
    for x, t in pipe:
        add_rect(x, y_pipe, 1.9, 0.95, t, fill="#BDD7EE", line="#2F5496", bold=True, font_size_pt=9)
        centers.append(x)

    for a, b in zip(centers, centers[1:]):
        add_conn(a + 0.95, y_pipe, b - 0.95, y_pipe, color="#2F5496")

    # arrow from FSM area down to pipeline
    add_conn(2.2, y_rt - 0.4, 1.5, y_pipe + 0.5)

    # === Motion detail ===
    add_rect(
        8.0,
        y_motion + 0.85,
        14.5,
        0.32,
        "motion/   步态与运动学",
        fill="#C6EFCE",
        line="#548235",
        bold=True,
        font_size_pt=10,
    )
    add_rect(2.4, y_motion, 3.6, 0.85, "gait_recipes.py\n预设 / ControllerSet", fill="#E2EFDA", line="#548235")
    add_rect(6.4, y_motion, 3.8, 0.85, "gait_controller.py\n_leg_xz / _swing_z\n足端轨迹规划", fill="#C6EFCE", line="#375623", bold=True)
    add_rect(10.4, y_motion, 3.2, 0.85, "kinematics.py\n前腿3链 / 后腿2D IK", fill="#E2EFDA", line="#548235")
    add_rect(13.8, y_motion, 2.6, 0.85, "motion_planner.py\nblend · 限速", fill="#E2EFDA", line="#548235")

    add_conn(8.1, y_pipe - 0.5, 6.4, y_motion + 0.45, color="#548235", label="")

    # === Control + Safety ===
    add_rect(3.2, y_ctrl, 4.5, 0.85, "control/\nimu_balance · gravity_comp\nimpedance · executor", fill="#F8CBAD", line="#C65911")
    add_rect(8.0, y_ctrl, 3.6, 0.85, "safety/supervisor\n限位 · 跳变 · ESTOP", fill="#FF6B6B", line="#C00000", bold=True)
    add_rect(12.4, y_ctrl, 4.5, 0.85, "预留 backends/ · policies/\n仿真·回放·RL（未实现）", fill="#EDEDED", line="#7F7F7F")

    add_conn(10.3, y_pipe - 0.5, 8.0, y_ctrl + 0.45, color="#C00000")
    add_conn(12.5, y_pipe - 0.5, 3.2, y_ctrl + 0.45, color="#C65911")

    # === Hardware ===
    add_rect(
        8.0,
        y_hw + 0.85,
        14.5,
        0.32,
        "hardware/   驱动与行为",
        fill="#D9E2F3",
        line="#2F5496",
        bold=True,
        font_size_pt=10,
    )
    add_rect(2.2, y_hw, 2.8, 0.85, "motors/\n灵足·EVO·达妙\nCAN", fill="#DDEBF7", line="#2F5496")
    add_rect(5.2, y_hw, 2.6, 0.85, "sensors/\nWT901 IMU", fill="#DDEBF7", line="#2F5496")
    add_rect(8.0, y_hw, 2.6, 0.85, "robot_hw\nread_state", fill="#BDD7EE", line="#1F4E79", bold=True)
    add_rect(10.8, y_hw, 2.6, 0.85, "actuation\nsend_all", fill="#BDD7EE", line="#1F4E79", bold=True)
    add_rect(13.6, y_hw, 2.8, 0.85, "behavior/\ntail · audio", fill="#DDEBF7", line="#2F5496")

    add_conn(14.6, y_pipe - 0.5, 10.8, y_hw + 0.45, color="#1F4E79")
    add_conn(14.6, y_pipe - 0.5, 8.0, y_hw + 0.45, color="#1F4E79")

    # === Notes ===
    add_rect(
        8.0,
        y_note,
        14.5,
        1.0,
        "数据契约: RobotState → UserCommand → FSM → MotionTarget → Safety → ControlOutput(MIT) → 电机\n"
        "迁移状态: 实现多在 src/marsdog_control；真机主循环默认仍在 mocap_to_real/walk.py\n"
        "验证: tests/parity 黄金快照对拍 motion/executor",
        fill="#FFFBF0",
        line="#BF8F00",
        font_size_pt=9,
    )

    return "\n".join(shapes)


def page_xml() -> str:
    shapes = build_page_shapes()
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<PageContents xmlns="http://schemas.microsoft.com/office/visio/2012/main"
              xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
              xml:space="preserve">
  <Shapes>
{shapes}
  </Shapes>
</PageContents>
"""


def document_xml() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<VisioDocument xmlns="http://schemas.microsoft.com/office/visio/2012/main"
               xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
               xml:space="preserve">
  <DocumentSettings TopPage="0" DefaultTextStyle="14" DefaultLineStyle="13" DefaultFillStyle="13"/>
  <Colors>
    <ColorEntry IX="0" RGB="#000000"/>
    <ColorEntry IX="1" RGB="#FFFFFF"/>
  </Colors>
  <StyleSheets>
    <StyleSheet ID="0" NameU="No Style" IsCustomNameU="1" Name="No Style" IsCustomName="1">
      <Cell N="EnableLineProps" V="1"/>
      <Cell N="EnableFillProps" V="1"/>
      <Cell N="EnableTextProps" V="1"/>
    </StyleSheet>
    <StyleSheet ID="3" NameU="Normal" IsCustomNameU="1" Name="Normal" IsCustomName="1">
      <Cell N="LineWeight" V="0.01" U="IN"/>
      <Cell N="LineColor" V="#000000"/>
      <Cell N="LinePattern" V="1"/>
      <Cell N="FillForegnd" V="#FFFFFF"/>
      <Cell N="FillPattern" V="1"/>
      <Cell N="VerticalAlign" V="1"/>
    </StyleSheet>
  </StyleSheets>
  <DocumentSheet NameU="TheDoc" Name="TheDoc">
    <Cell N="PageWidth" V="11" U="IN"/>
    <Cell N="PageHeight" V="8.5" U="IN"/>
  </DocumentSheet>
</VisioDocument>
"""


def pages_xml() -> str:
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Pages xmlns="http://schemas.microsoft.com/office/visio/2012/main"
       xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
       xml:space="preserve">
  <Page ID="0" NameU="Marsdog架构" Name="Marsdog架构" IsCustomNameU="1" IsCustomName="1">
    <PageSheet>
      <Cell N="PageWidth" V="{PAGE_W}" U="IN"/>
      <Cell N="PageHeight" V="{PAGE_H}" U="IN"/>
      <Cell N="ShdwSep" V="0.0625"/>
      <Cell N="PageScale" V="1" U="IN"/>
      <Cell N="DrawingScale" V="1" U="IN"/>
    </PageSheet>
    <Rel r:id="rId1"/>
  </Page>
</Pages>
"""


def content_types() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/visio/document.xml" ContentType="application/vnd.ms-visio.drawing.main+xml"/>
  <Override PartName="/visio/pages/pages.xml" ContentType="application/vnd.ms-visio.pages+xml"/>
  <Override PartName="/visio/pages/page1.xml" ContentType="application/vnd.ms-visio.page+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>
"""


def rels_root() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.microsoft.com/visio/2010/relationships/document" Target="visio/document.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>
"""


def rels_document() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.microsoft.com/visio/2010/relationships/pages" Target="pages/pages.xml"/>
</Relationships>
"""


def rels_pages() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.microsoft.com/visio/2010/relationships/page" Target="page1.xml"/>
</Relationships>
"""


def rels_page1() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"/>
"""


def core_xml() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
                   xmlns:dc="http://purl.org/dc/elements/1.1/"
                   xmlns:dcterms="http://purl.org/dc/terms/"
                   xmlns:dcmitype="http://purl.org/dc/dcmitype/"
                   xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>Marsdog Control Architecture</dc:title>
  <dc:creator>marsdogv3_test1</dc:creator>
  <cp:lastModifiedBy>marsdogv3_test1</cp:lastModifiedBy>
</cp:coreProperties>
"""


def app_xml() -> str:
    return """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties"
            xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Microsoft Visio</Application>
  <Pages>1</Pages>
</Properties>
"""


def windows_xml() -> str:
    return f"""<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Windows ClientWidth="1920" ClientHeight="1080"
         xmlns="http://schemas.microsoft.com/office/visio/2012/main"
         xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
         xml:space="preserve">
  <Window ID="0" WindowType="Drawing" WindowState="0" WindowLeft="0" WindowTop="0"
          WindowWidth="1920" WindowHeight="1080" ContainerType="Page" Page="0"
          ViewScale="0.6" ViewCenterX="{PAGE_W/2}" ViewCenterY="{PAGE_H/2}">
    <ShowRulers>1</ShowRulers>
    <ShowGrid>1</ShowGrid>
    <ShowPageBreaks>0</ShowPageBreaks>
    <ShowGuides>1</ShowGuides>
    <ShowConnectionPoints>1</ShowConnectionPoints>
    <ShowPageOutline>1</ShowPageOutline>
    <GlueSettings>9</GlueSettings>
    <SnapSettings>65847</SnapSettings>
    <SnapExtensions>34</SnapExtensions>
  </Window>
</Windows>
"""


def main() -> None:
    files = {
        "[Content_Types].xml": content_types(),
        "_rels/.rels": rels_root(),
        "docProps/core.xml": core_xml(),
        "docProps/app.xml": app_xml(),
        "visio/document.xml": document_xml(),
        "visio/windows.xml": windows_xml(),
        "visio/_rels/document.xml.rels": rels_document(),
        "visio/pages/pages.xml": pages_xml(),
        "visio/pages/_rels/pages.xml.rels": rels_pages(),
        "visio/pages/page1.xml": page_xml(),
        "visio/pages/_rels/page1.xml.rels": rels_page1(),
    }

    # Patch Content_Types + document rels for windows.xml
    files["[Content_Types].xml"] = content_types().replace(
        '</Types>',
        '  <Override PartName="/visio/windows.xml" ContentType="application/vnd.ms-visio.windows+xml"/>\n</Types>',
    )
    files["visio/_rels/document.xml.rels"] = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.microsoft.com/visio/2010/relationships/pages" Target="pages/pages.xml"/>
  <Relationship Id="rId2" Type="http://schemas.microsoft.com/visio/2010/relationships/windows" Target="windows.xml"/>
</Relationships>
"""

    if OUT.exists():
        OUT.unlink()
    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for name, data in files.items():
            zf.writestr(name, data.encode("utf-8"))
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
