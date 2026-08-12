from spotlight_tools import get_assets_dir
from spotlight_tools.arena import ArenaConfig


for arena_spec_path in (get_assets_dir() / "arena_configs/").glob("*/arena_spec.pdf"):
    print(f"Processing {arena_spec_path}...")
    arena_config = ArenaConfig(arena_spec_path)
    arena_config.save(arena_spec_path.parent)
