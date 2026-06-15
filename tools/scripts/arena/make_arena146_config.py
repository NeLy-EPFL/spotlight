from spotlight_tools import get_assets_dir
from spotlight_tools.arena import ArenaConfig


arena_spec_path = get_assets_dir() / "arena_configs/arena146/arena146_spec.pdf"
arena_config = ArenaConfig(arena_spec_path)
arena_config.save(arena_spec_path.parent)
