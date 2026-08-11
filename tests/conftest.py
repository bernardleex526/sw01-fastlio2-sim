from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"


def load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def parse_xml(path: Path):
    return ET.parse(path).getroot()
