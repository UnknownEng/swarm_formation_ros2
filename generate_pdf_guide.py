#!/usr/bin/env python3
# Copyright 2026 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Automated PDF Quickstart & Execution Guide Generator using ReportLab."""

import os
from reportlab.lib.pagesizes import letter
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, Image, PageBreak
)
from reportlab.pdfgen import canvas


class NumberedCanvas(canvas.Canvas):
    """Canvas that computes total pages dynamically for footer page numbers."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._saved_page_states = []

    def showPage(self):
        self._saved_page_states.append(dict(self.__dict__))
        self._startPage()

    def save(self):
        num_pages = len(self._saved_page_states)
        for state in self._saved_page_states:
            self.__dict__.update(state)
            self.draw_page_decorations(num_pages)
            super().showPage()
        super().save()

    def draw_page_decorations(self, page_count):
        self.saveState()
        self.setFont("Helvetica", 8)
        self.setFillColor(colors.HexColor("#718096"))

        # Header (pages 2+)
        if self._pageNumber > 1:
            self.drawString(54, 752, "Decentralized Swarm Shape Formation | User Quickstart & Execution Guide")
            self.drawRightString(558, 752, "NUST - Mansoor Ahmed Rind")
            self.setStrokeColor(colors.HexColor("#CBD5E0"))
            self.setLineWidth(0.5)
            self.line(54, 746, 558, 746)

        # Footer (all pages)
        self.setStrokeColor(colors.HexColor("#CBD5E0"))
        self.setLineWidth(0.5)
        self.line(54, 45, 558, 45)
        page_str = f"Page {self._pageNumber} of {page_count}"
        self.drawRightString(558, 32, page_str)
        self.drawString(54, 32, "Confidential & Proprietary - Robotics & AI Department, SMME, NUST")
        self.restoreState()


def create_code_block(code_text: str, style_sheet) -> Table:
    """Wrap code text in a padded, bordered grey box."""
    code_style = ParagraphStyle(
        'CodeStyle',
        parent=style_sheet['Normal'],
        fontName='Courier',
        fontSize=7.8,
        leading=10.0,
        textColor=colors.HexColor("#1A202C")
    )
    formatted_lines = code_text.strip().replace('\n', '<br/>').replace(' ', '&nbsp;')
    p = Paragraph(f"<code>{formatted_lines}</code>", code_style)
    t = Table([[p]], colWidths=[504])
    t.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor("#EDF2F7")),
        ('BOX', (0, 0), (-1, -1), 0.75, colors.HexColor("#CBD5E0")),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    return t


def build_pdf(filename: str):
    doc = SimpleDocTemplate(
        filename,
        pagesize=letter,
        leftMargin=54,
        rightMargin=54,
        topMargin=54,
        bottomMargin=54
    )

    styles = getSampleStyleSheet()

    # Custom typography
    title_style = ParagraphStyle(
        'DocTitle',
        parent=styles['Heading1'],
        fontName='Helvetica-Bold',
        fontSize=18,
        leading=22,
        textColor=colors.HexColor("#1A365D"),
        spaceAfter=2
    )
    subtitle_style = ParagraphStyle(
        'DocSubTitle',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=10,
        leading=13,
        textColor=colors.HexColor("#4A5568"),
        spaceAfter=8
    )
    h1_style = ParagraphStyle(
        'SectionH1',
        parent=styles['Heading2'],
        fontName='Helvetica-Bold',
        fontSize=11.5,
        leading=14.5,
        textColor=colors.HexColor("#2B6CB0"),
        spaceBefore=10,
        spaceAfter=4,
        keepWithNext=True
    )
    h2_style = ParagraphStyle(
        'SectionH2',
        parent=styles['Heading3'],
        fontName='Helvetica-Bold',
        fontSize=9.5,
        leading=12.5,
        textColor=colors.HexColor("#2D3748"),
        spaceBefore=7,
        spaceAfter=3,
        keepWithNext=True
    )
    body_style = ParagraphStyle(
        'BodyDark',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=8.5,
        leading=11.5,
        textColor=colors.HexColor("#2D3748"),
        spaceAfter=4
    )
    th_style = ParagraphStyle(
        'TableHeader',
        parent=styles['Normal'],
        fontName='Helvetica-Bold',
        fontSize=7.5,
        leading=9.5,
        textColor=colors.white,
        alignment=1
    )
    td_style = ParagraphStyle(
        'TableCell',
        parent=styles['Normal'],
        fontName='Helvetica',
        fontSize=7.5,
        leading=9.5,
        textColor=colors.HexColor("#2D3748")
    )
    td_center_style = ParagraphStyle(
        'TableCellCenter',
        parent=td_style,
        alignment=1
    )
    callout_style = ParagraphStyle(
        'CalloutText',
        parent=styles['Normal'],
        fontName='Helvetica-Oblique',
        fontSize=8.0,
        leading=10.5,
        textColor=colors.HexColor("#2C5282")
    )

    story = []

    # ==================== PAGE 1 ====================
    story.append(Paragraph("Decentralized Swarm Shape Formation in Gazebo", title_style))
    story.append(Paragraph("Zero-to-Hero Quickstart, GitHub Download, and Physical Simulation Guide", subtitle_style))

    # Meta banner
    meta_text = (
        "<b>Author:</b> Mansoor Ahmed Rind (NUST) &nbsp;|&nbsp; "
        "<b>Platform:</b> Ubuntu 24.04 LTS &nbsp;|&nbsp; "
        "<b>ROS 2:</b> Jazzy Jalisco &nbsp;|&nbsp; "
        "<b>Simulator:</b> Gazebo Harmonic 8.x"
    )
    meta_p = Paragraph(meta_text, body_style)
    meta_table = Table([[meta_p]], colWidths=[504])
    meta_table.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor("#EBF8FF")),
        ('BOX', (0, 0), (-1, -1), 0.75, colors.HexColor("#BEE3F8")),
        ('TOPPADDING', (0, 0), (-1, -1), 4),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
        ('LEFTPADDING', (0, 0), (-1, -1), 8),
        ('RIGHTPADDING', (0, 0), (-1, -1), 8),
    ]))
    story.append(meta_table)
    story.append(Spacer(1, 6))

    # 1. Project Scope & Capabilities
    story.append(Paragraph("1. Project Scope & Architecture", h1_style))
    p1 = (
        "This project provides a physics-verified, decentralized multi-robot shape formation system "
        "operating inside a high-complexity search-and-rescue disaster site. A swarm of <b>5 differential-drive mobile robots</b> "
        "self-organizes into a regular pentagon (radius <i>R</i> = 2.0 m) <b>without GPS, external mocap, or a central coordinator</b>. "
        "The project rigorously benchmarks three operational modes:"
    )
    story.append(Paragraph(p1, body_style))

    scope_points = [
        "<b>Part 1 (Baseline Consensus):</b> Standard unweighted linear consensus. Physical contacts and inertia cause rigid-body crashes (d = 0.397 m), proving uncertified consensus fails in physics.",
        "<b>Part 2 (Real-World Faults):</b> Introduces 15% packet drop, 50-150ms delay, sensor noise (sigma = 0.18m), and a 5-second blackout window for Robot 4. Shows severe trajectory degradation.",
        "<b>Part 3 (Resilient Method + Dual-Layer APF Barrier):</b> Combines Huber consensus, Exponential Moving Average (EMA) filtering, dynamic heartbeat exclusion, and a Dual-Layer Artificial Potential Field (APF). Guarantees zero crashes (d_min = 0.676 m >= 0.50 m) and 3.6 cm formation precision."
    ]
    for pt in scope_points:
        story.append(Paragraph(f"• {pt}", body_style))
    story.append(Spacer(1, 4))

    # 2. Step 1: System Prerequisites
    story.append(Paragraph("2. Step 1: Installing Prerequisites on Ubuntu 24.04", h1_style))
    story.append(Paragraph(
        "Open a terminal on Ubuntu 24.04 LTS (or WSL2) and install ROS 2 Jazzy, Gazebo Harmonic, and build tools:",
        body_style
    ))
    code_install = (
        "# 1. Update system & add ROS 2 Jazzy repository\n"
        "sudo apt update && sudo apt install -y curl gnupg2 lsb-release build-essential git\n"
        "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg\n"
        "echo \"deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null\n"
        "\n# 2. Install ROS 2 Jazzy, Gazebo Harmonic & Transport Bridges\n"
        "sudo apt update && sudo apt install -y \\\n"
        "  ros-jazzy-desktop ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \\\n"
        "  python3-colcon-common-extensions python3-numpy python3-matplotlib python3-pytest"
    )
    story.append(create_code_block(code_install, styles))
    story.append(Spacer(1, 4))

    # 3. Step 2: Download Repository
    story.append(Paragraph("3. Step 2: Downloading and Preparing the Workspace", h1_style))
    story.append(Paragraph(
        "Clone the repository directly into your home workspace directory (<code>~/swarm_ws</code>):",
        body_style
    ))
    code_clone = (
        "cd ~\n"
        "git clone https://github.com/Mansoor-Ahmed-Rind/swarm_formation_ws.git swarm_ws\n"
        "cd ~/swarm_ws && ls -la src/swarm_formation"
    )
    story.append(create_code_block(code_clone, styles))
    story.append(Spacer(1, 4))

    # 4. Step 3: Build & Lint
    story.append(Paragraph("4. Step 3: Building the Package & Quality Checks", h1_style))
    story.append(Paragraph(
        "Compile the ROS 2 workspace using <code>colcon</code> and verify 100% test compliance:",
        body_style
    ))
    code_build = (
        "cd ~/swarm_ws && source /opt/ros/jazzy/setup.bash\n"
        "colcon build --symlink-install\n"
        "source install/setup.bash\n"
        "colcon test --packages-select swarm_formation && colcon test-result --verbose"
    )
    story.append(create_code_block(code_build, styles))

    story.append(PageBreak())

    # ==================== PAGE 2 ====================
    # 5. Step 4: Running Simulations
    story.append(Paragraph("5. Step 4: Running the Simulations", h1_style))

    story.append(Paragraph("Mode A: Interactive 3D Gazebo Simulation with GUI (Live Demonstration)", h2_style))
    story.append(Paragraph(
        "To watch the 5 physical robots actively navigate the collapsed building disaster site with live 3D physics:",
        body_style
    ))
    code_run_gui = (
        "cd ~/swarm_ws && source install/setup.bash\n"
        "ros2 launch swarm_formation gazebo_swarm_launch.py \\\n"
        "  world:=src/swarm_formation/worlds/collapsed_building.sdf \\\n"
        "  headless:=false \\\n"
        "  use_resilient_method:=true \\\n"
        "  enable_faults:=true"
    )
    story.append(create_code_block(code_run_gui, styles))
    story.append(Spacer(1, 4))

    story.append(Paragraph("Mode B: Automated Headless Benchmark Suite (All 3 Parts + Plots)", h2_style))
    story.append(Paragraph(
        "To execute all three comparative experiments sequentially, log telemetry to CSV, and automatically generate plots:",
        body_style
    ))
    code_run_all = (
        "cd ~/swarm_ws\n"
        "chmod +x run_gazebo_experiments.sh\n"
        "./run_gazebo_experiments.sh"
    )
    story.append(create_code_block(code_run_all, styles))
    story.append(Spacer(1, 4))

    story.append(Paragraph("Mode C: Part 1 Baseline Crash Demonstration", h2_style))
    story.append(Paragraph(
        "To observe the collision failure when linear consensus operates without safety barriers:",
        body_style
    ))
    code_run_crash = (
        "ros2 launch swarm_formation gazebo_swarm_launch.py \\\n"
        "  world:=src/swarm_formation/worlds/collapsed_building.sdf \\\n"
        "  headless:=false use_resilient_method:=false enable_faults:=false"
    )
    story.append(create_code_block(code_run_crash, styles))
    story.append(Spacer(1, 8))

    # 6. Quantitative Results
    story.append(Paragraph("6. Quantitative Benchmark Results", h1_style))
    story.append(Paragraph(
        "Physical metrics logged across the three experimental setups in Gazebo Harmonic:",
        body_style
    ))

    # Results Table with Paragraphs
    table_raw = [
        ["Benchmark Metric", "Part 1: Baseline", "Part 2: Real Faults", "Part 3: Resilient APF", "Safety Specification"],
        ["Min Distance (d_min)", "0.3969 m (CRASH)", "0.5683 m (Near-miss)", "0.6757 m (SAFE)", ">= 0.5000 m"],
        ["Crash Violations", "1 collision event", "0 events", "0 events (100% SAFE)", "0 crash events"],
        ["Final RMS Shape Error", "0.9580 m (Jammed)", "0.0684 m", "0.0368 m (3.6 cm)", "<= 0.1500 m"],
        ["Convergence Time", "Failed (Interlocked)", "15.14 s", "19.00 s", "< 45.0 s"],
        ["Safety Certificate", "FAILED", "UNCERTIFIED", "CERTIFIED SAFE", "Zero Crashes"]
    ]
    formatted_table = []
    for row_idx, row in enumerate(table_raw):
        row_cells = []
        for col_idx, cell in enumerate(row):
            if row_idx == 0:
                row_cells.append(Paragraph(cell, th_style))
            elif col_idx == 0:
                row_cells.append(Paragraph(cell, td_style))
            else:
                row_cells.append(Paragraph(cell, td_center_style))
        formatted_table.append(row_cells)

    t_results = Table(formatted_table, colWidths=[124, 95, 95, 100, 90])
    t_results.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor("#2B6CB0")),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E0")),
        ('BACKGROUND', (0, 1), (-1, 1), colors.HexColor("#FFF5F5")),  # Crash red tint
        ('BACKGROUND', (0, 3), (-1, 3), colors.HexColor("#F0FFF4")),  # Safe green tint
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
    ]))
    story.append(t_results)

    story.append(PageBreak())

    # ==================== PAGE 3 ====================
    # 7. Comparative Plot Image
    img_path = "/home/kali/swarm_ws/gazebo_swarm_comparison.png"
    if os.path.exists(img_path):
        story.append(Paragraph("7. Comparative Diagnostics Visualization", h1_style))
        story.append(Paragraph(
            "Multi-panel diagnostic comparison generated automatically from Gazebo Harmonic telemetry logs:",
            body_style
        ))
        img = Image(img_path, width=490, height=275)
        story.append(img)
        story.append(Spacer(1, 8))

    # 8. What is Expected vs Not Expected
    story.append(Paragraph("8. Evaluator Guide: What is Expected vs. What is NOT Expected", h1_style))

    guide_raw = [
        ["Observation", "What is EXPECTED (Normal Verified)", "What is NOT Expected (Non-Error)"],
        [
            "Collision Safety",
            "In Part 3, min distance strictly remains >= 0.50m (0.6757m achieved). Zero physical crashes.",
            "Do NOT expect Part 1 (baseline) to succeed without crashing; it is designed to fail to prove APF necessity."
        ],
        [
            "Robot Dynamics",
            "Robots rotate in place to align yaw heading before moving forward smoothly.",
            "Do NOT expect omnidirectional sideways sliding; platforms are differential-drive unicycles."
        ],
        [
            "Swarm Centroid",
            "Swarm forms around its own barycenter (initial centroid ~ +0.44m, -0.44m).",
            "Do NOT expect formation centered at (0, 0); system is decentralized with zero GPS or world anchor."
        ],
        [
            "Agent 4 Dropout",
            "At t = 5s to 10s, Robot 4 stops transmitting and coasts. Remaining robots stay stable. Robot 4 rejoins at 10s.",
            "Do NOT consider Robot 4 halting a defect; this is the programmed blackout fault test."
        ],
        [
            "Shutdown Warning",
            "Clean CSV export and plot generation; launch ends cleanly.",
            "Do NOT panic at 'escalating to SIGTERM' at shutdown; this is standard ROS 2 Gz bridge thread cleanup."
        ]
    ]
    formatted_guide = []
    for row_idx, row in enumerate(guide_raw):
        row_cells = []
        for col_idx, cell in enumerate(row):
            if row_idx == 0:
                row_cells.append(Paragraph(cell, th_style))
            else:
                row_cells.append(Paragraph(cell, td_style))
        formatted_guide.append(row_cells)

    t_guide = Table(formatted_guide, colWidths=[90, 207, 207])
    t_guide.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor("#1A365D")),
        ('GRID', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E0")),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('BACKGROUND', (0, 1), (-1, 1), colors.HexColor("#F7FAFC")),
        ('BACKGROUND', (0, 3), (-1, 3), colors.HexColor("#F7FAFC")),
    ]))
    story.append(t_guide)
    story.append(Spacer(1, 8))

    # 9. Troubleshooting & FAQ
    story.append(Paragraph("9. Troubleshooting & Performance Tips", h1_style))
    troubles = [
        "<b>Black screen or GUI freeze in WSL2:</b> Run <code>export LIBGL_ALWAYS_SOFTWARE=1</code> before launching Gazebo.",
        "<b>Stale background processes:</b> If ports are locked, run: <code>killall -9 gz ruby gzserver gzclient; pkill -9 -f gazebo_node</code>.",
        "<b>Regenerate plots manually:</b> Run <code>python3 generate_gazebo_comparison.py</code> inside <code>~/swarm_ws</code>.",
        "<b>Read full IEEE paper:</b> Inspect <code>ieee_research_report.md</code> in the repository root for complete academic proofs."
    ]
    for tr in troubles:
        story.append(Paragraph(f"• {tr}", body_style))
    story.append(Spacer(1, 8))

    # Sign-off box
    sign_off = (
        "<b>Repository:</b> github.com/Mansoor-Ahmed-Rind/swarm_formation_ws &nbsp;|&nbsp; "
        "<b>Contact:</b> mansoor.rind@smme.nust.edu.pk<br/>"
        "NUST School of Mechanical & Manufacturing Engineering (SMME), Islamabad, Pakistan"
    )
    p_sign = Paragraph(sign_off, callout_style)
    t_sign = Table([[p_sign]], colWidths=[504])
    t_sign.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, -1), colors.HexColor("#EDF2F7")),
        ('BOX', (0, 0), (-1, -1), 0.5, colors.HexColor("#CBD5E0")),
        ('ALIGN', (0, 0), (-1, -1), 'CENTER'),
        ('TOPPADDING', (0, 0), (-1, -1), 5),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 5),
        ('LEFTPADDING', (0, 0), (-1, -1), 10),
        ('RIGHTPADDING', (0, 0), (-1, -1), 10),
    ]))
    story.append(t_sign)

    doc.build(story, canvasmaker=NumberedCanvas)
    print(f"Successfully generated PDF: {filename}")


if __name__ == '__main__':
    output_pdf = "/home/kali/swarm_ws/decentralized_swarm_quickstart_guide.pdf"
    build_pdf(output_pdf)
