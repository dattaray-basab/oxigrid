#!/usr/bin/env python3
from pathlib import Path
import re

README = Path("README.md")
DOC_MAP = Path("doc_map.md")
MARKER_START = "<!-- DOC_MAP_START -->"
MARKER_END = "<!-- DOC_MAP_END -->"

TITLE_MAP = {
    "analytics": "Analytics",
    "battery": "Battery",
    "digitaltwin": "Digital Twin",
    "harmonics": "Harmonics",
    "io": "IO",
    "monitoring": "Monitoring",
    "network": "Network",
    "optimize": "Optimize",
    "planning": "Planning",
    "powerelectronics": "Power Electronics",
    "powerflow": "Power Flow",
    "powerquality": "Power Quality",
    "protection": "Protection",
    "renewable": "Renewable",
    "security": "Security",
    "simulation": "Simulation",
    "stability": "Stability",
    "testcases": "Testcases",
    "units": "Units",
}


def generate_doc_map() -> str:
    lines = ["# Module Documentation Map", ""]
    for directory in sorted(Path("src").iterdir()):
        if not directory.is_dir():
            continue
        doc = directory / f"{directory.name}.md"
        if not doc.exists():
            continue
        title = TITLE_MAP.get(directory.name, directory.name.capitalize())
        lines.append(f"- [{title}]({doc.as_posix()})")
    lines.append("")
    return "\n".join(lines)


def update_doc_map_file() -> None:
    DOC_MAP.write_text(generate_doc_map())
    print(f"Generated {DOC_MAP}")


def update_readme() -> None:
    if not README.exists():
        raise FileNotFoundError(f"{README} not found")
    if not DOC_MAP.exists():
        raise FileNotFoundError(f"{DOC_MAP} not found")

    content = README.read_text()
    section = DOC_MAP.read_text().strip()
    replacement = f"{MARKER_START}\n{section}\n{MARKER_END}"

    pattern = re.compile(
        rf"{re.escape(MARKER_START)}.*?{re.escape(MARKER_END)}",
        re.DOTALL,
    )
    if not pattern.search(content):
        raise RuntimeError("README.md does not contain the required doc map markers")
    new_content = pattern.sub(replacement, content)
    README.write_text(new_content)
    print(f"Updated {README}")


if __name__ == "__main__":
    update_doc_map_file()
    update_readme()
