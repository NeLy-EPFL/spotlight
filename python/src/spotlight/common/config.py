"""Config loader for the spotlight package (assets/tools_config.yaml)."""

import yaml

import spotlight
from pathlib import Path


def load_spotlight_config() -> dict:
    config_path = get_assets_dir() / "tools_config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(
            f"Configuration file {config_path} does not exist. Make sure the "
            "spotlight package is installed correctly."
        )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def get_assets_dir() -> Path:
    """The package's shared assets directory (`src/spotlight/assets/`).

    Returns:
        Path to `assets/`, e.g. for loading `tools_config.yaml` or a body-plan JSON.
    """
    return Path(spotlight.__path__[0]).expanduser() / "assets"
