import os
import re
from reportlab.platypus import SimpleDocTemplate, Paragraph, Table, TableStyle
from reportlab.lib.units import mm
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib import colors
from svglib.svglib import svg2rlg

# -----------------------------
# Configuration
# -----------------------------
INPUT_FOLDER = "collarobot_perception/images/tags"
OUTPUT = "aruco_5cm_with_names.pdf"
TARGET_SIZE_MM = 50  # exact 5 cm
MARKERS_PER_ROW = 3

# -----------------------------
# Helper: extract numeric ID
# -----------------------------
def extract_id(filename):
    name_no_ext = os.path.splitext(filename)[0]
    match = re.search(r"(\d+)$", name_no_ext)
    return int(match.group(1)) if match else -1

# -----------------------------
# Collect and sort SVG files
# -----------------------------
svg_files = sorted(
    [f for f in os.listdir(INPUT_FOLDER) if f.lower().endswith(".svg")],
    key=extract_id
)

# -----------------------------
# PDF setup
# -----------------------------
doc = SimpleDocTemplate(OUTPUT)
elements = []

text_style = ParagraphStyle(
    name="MarkerLabel",
    fontName="Helvetica",
    fontSize=9,
    textColor=colors.black,
    alignment=1,  # center
)

data = []
row = []

# -----------------------------
# Build table content
# -----------------------------
for i, filename in enumerate(svg_files):
    path = os.path.join(INPUT_FOLDER, filename)
    drawing = svg2rlg(path)

    # Scale marker to exactly 50 mm width
    scale = (TARGET_SIZE_MM * mm) / drawing.width
    drawing.scale(scale, scale)
    drawing.width = TARGET_SIZE_MM * mm
    drawing.height = TARGET_SIZE_MM * mm

    # Clean label (numeric ID at end)
    marker_id = extract_id(filename)
    label = Paragraph(str(marker_id), text_style)

    # Stack marker + label in one cell
    cell = [drawing, label]
    row.append(cell)

    # Add full row
    if (i + 1) % MARKERS_PER_ROW == 0:
        data.append(row)
        row = []

# Fill last row if incomplete
if row:
    while len(row) < MARKERS_PER_ROW:
        row.append("")
    data.append(row)

# -----------------------------
# Create table layout
# -----------------------------
table = Table(
    data,
    colWidths=[60 * mm] * MARKERS_PER_ROW
)

table.setStyle(TableStyle([
    ("ALIGN", (0, 0), (-1, -1), "CENTER"),
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("LEFTPADDING", (0, 0), (-1, -1), 2),
    ("RIGHTPADDING", (0, 0), (-1, -1), 2),
    ("TOPPADDING", (0, 0), (-1, -1), 2),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
]))

elements.append(table)

# -----------------------------
# Build PDF
# -----------------------------
doc.build(elements)

print(f"PDF created successfully: {OUTPUT}")