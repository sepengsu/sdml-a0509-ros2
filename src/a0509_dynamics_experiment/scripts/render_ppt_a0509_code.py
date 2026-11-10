"""Create two slide-sized code screenshots from the actual A0509 model files."""

from __future__ import annotations

from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from pygments import lex
from pygments.lexers import XmlLexer
from pygments.token import Comment, Literal, Name, Token


ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / "docs/media/a0509_urdf_mjcf"
URDF = ROOT / "src/doosan-robot2/dsr_description2/urdf/a0509.urdf"
MJCF = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/a0509.xml"
SIZE = (2400, 1350)
MAX_COLUMNS = 123


def load_font(candidates: list[str], size: int) -> ImageFont.FreeTypeFont:
    for candidate in candidates:
        if Path(candidate).is_file():
            return ImageFont.truetype(candidate, size)
    raise FileNotFoundError(f"No font found among: {candidates}")


CODE_FONT = load_font(
    ["C:/Windows/Fonts/CascadiaCode.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"],
    28,
)
UI_FONT = load_font(
    ["C:/Windows/Fonts/segoeui.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"],
    29,
)
SMALL_FONT = load_font(
    ["C:/Windows/Fonts/segoeui.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"],
    23,
)


def token_color(token: Token) -> str:
    if token in Comment:
        return "#6A9955"
    if token in Name.Tag:
        return "#569CD6"
    if token in Name.Attribute:
        return "#9CDCFE"
    if token in Literal.String:
        return "#CE9178"
    return "#D4D4D4"


def styled_lines(source_lines: list[str]) -> list[list[tuple[str, str]]]:
    """Lex the unmodified excerpt, expanding tabs as a code editor displays them."""
    result: list[list[tuple[str, str]]] = [[]]
    for token, value in lex("\n".join(source_lines), XmlLexer()):
        color = token_color(token)
        for char in value:
            if char == "\n":
                result.append([])
            elif char == "\t":
                count = 4 - len(result[-1]) % 4
                result[-1].extend([(" ", color)] * count)
            else:
                result[-1].append((char, color))
    return result[: len(source_lines)]


def wrap_line(chars: list[tuple[str, str]]) -> list[list[tuple[str, str]]]:
    if len(chars) <= MAX_COLUMNS:
        return [chars]
    sections = []
    remaining = chars
    while len(remaining) > MAX_COLUMNS:
        break_at = next(
            (index for index in range(MAX_COLUMNS, 70, -1) if remaining[index][0] == " "),
            MAX_COLUMNS,
        )
        sections.append(remaining[:break_at])
        remaining = [(" ", "#D4D4D4")] * 10 + remaining[break_at:].copy()
    sections.append(remaining)
    return sections


def render(source: Path, first: int, last: int, label: str, destination: str) -> None:
    lines = source.read_text(encoding="utf-8").splitlines()
    excerpt = lines[first - 1 : last]
    visual_lines = []
    for number, chars in enumerate(styled_lines(excerpt), start=first):
        for part_number, segment in enumerate(wrap_line(chars)):
            visual_lines.append((number if part_number == 0 else None, segment))
    if len(visual_lines) > 23:
        raise ValueError(f"{source.name}: {len(visual_lines)} visual lines exceed the slide")

    canvas = Image.new("RGB", SIZE, "#080D16")
    draw = ImageDraw.Draw(canvas)
    draw.rounded_rectangle((65, 62, 2335, 1288), radius=19, fill="#1E1E1E", outline="#394454", width=2)
    draw.rounded_rectangle((66, 63, 2334, 162), radius=18, fill="#252526")
    draw.rectangle((66, 135, 2334, 162), fill="#252526")
    for x, color in [(103, "#FF5F57"), (139, "#FEBC2E"), (175, "#28C840")]:
        draw.ellipse((x - 10, 104 - 10, x + 10, 104 + 10), fill=color)
    draw.rounded_rectangle((245, 118, 560, 194), radius=8, fill="#1E1E1E")
    draw.text((275, 137), f"<>  {source.name}", font=UI_FONT, fill="#E6E9ED")
    draw.text((1995, 94), f"A0509  /  {label}", font=SMALL_FONT, fill="#A9B3C4")
    draw.rectangle((66, 194, 2334, 255), fill="#1E1E1E")
    breadcrumb = "  /  ".join(source.relative_to(ROOT / "src/doosan-robot2").parts)
    draw.text((250, 209), breadcrumb, font=SMALL_FONT, fill="#97A4B5")
    draw.rectangle((66, 255, 218, 1240), fill="#181818")
    draw.rounded_rectangle((107, 311, 141, 354), radius=3, outline="#65717F", width=3)
    draw.line((115, 326, 133, 326), fill="#65717F", width=2)
    draw.line((115, 338, 133, 338), fill="#65717F", width=2)
    draw.ellipse((108, 385, 136, 413), outline="#65717F", width=3)
    draw.line((132, 410, 147, 425), fill="#65717F", width=3)
    draw.rectangle((65, 1240, 2335, 1288), fill="#007ACC")
    draw.text((94, 1248), f"A0509  ·  {label}", font=SMALL_FONT, fill="#FFFFFF")
    draw.text((2010, 1248), "XML  |  UTF-8", font=SMALL_FONT, fill="#FFFFFF")

    code_x = 338
    number_x = 266
    top = 279
    line_height = 42
    char_width = CODE_FONT.getlength("M")
    for position, (line_number, segment) in enumerate(visual_lines):
        y = top + position * line_height
        if line_number is not None:
            draw.text((number_x, y), str(line_number).rjust(3), font=CODE_FONT, fill="#69707A")
        for column, (char, color) in enumerate(segment):
            if char != " ":
                draw.text((int(code_x + column * char_width), y), char, font=CODE_FONT, fill=color)

    OUTPUT.mkdir(parents=True, exist_ok=True)
    path = OUTPUT / destination
    canvas.save(path)
    print(path)


if __name__ == "__main__":
    render(URDF, 30, 50, "URDF", "a0509_urdf_code.png")
    render(MJCF, 24, 42, "MJCF", "a0509_mjcf_code.png")
