# Doing experiments

> [!TIP]
> 
> **Tl;dr:**
> 
```bash
align-cameras -p ~/Spotlight/profiles/default -a ~/Spotlight/arenas/arena146
run-arena-registration-scan -p ~/Spotlight/profiles/default -a ~/Spotlight/arenas/arena146
# source ~/project/spotlight-dev/spotlight-tools/.venv/bin/activate
fit-arena-registration -a $HOME/Spotlight/arenas/arena146
run-spotlight -p ~/Spotlight/profiles/default -a ~/Spotlight/arenas/arena146
```