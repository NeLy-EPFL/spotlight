import yaml

import spotlight_tools
from pathlib import Path


def load_spotlight_tools_config() -> dict:
    spotlight_package_dir = Path(spotlight_tools.__path__[0]).expanduser()
    config_path = spotlight_package_dir.parent.parent / "config/config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(
            f"Configuration file {config_path} does not exist. Make sure the "
            "spotlight-tools package is installed correctly."
        )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config
