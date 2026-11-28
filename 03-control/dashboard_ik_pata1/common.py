"""Utilidades compartidas: carga de config.yaml y rutas."""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent


def load_config(path=None):
    path = Path(path) if path else ROOT / "config.yaml"
    with open(path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    cfg["_root"] = str(path.parent)
    return cfg


def resolve(cfg, rel):
    """Ruta relativa a la carpeta de config.yaml."""
    return (Path(cfg["_root"]) / rel).resolve()
