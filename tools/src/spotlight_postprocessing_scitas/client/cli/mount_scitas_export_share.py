import os
from pathlib import Path
from subprocess import run

import tyro

import spotlight_postprocessing_scitas.common.config as config


def mount_scitas_export_share(
    username: str | None = None,
    mountpoint: Path = config.SCITAS_EXPORT_MOUNTPOINT_LOCAL,
    samba_addr: str = config.SCITAS_EXPORT_SAMBA_ADDR,
) -> None:
    """Mount the SCITAS /export share via Samba and create working dir for Spotlight.

    Args:
        username: Username for Samba authentication. If None, will use the current user.
    """
    if username is None:
        username = os.environ.get("USER")
        if username is None:
            raise ValueError("username is not specified and env var 'USER' is not set.")
    mountpoint = mountpoint.expanduser()

    mount_output = run(["mount"], capture_output=True, text=True, check=True).stdout
    if samba_addr not in mount_output:
        run(["sudo", "mkdir", "-p", mountpoint], check=True)
        options = f"mfsymlinks,username={username},noperm,vers=2.1,domain=intranet"
        run(
            [
                "sudo",
                "mount",
                "-t",
                "cifs",
                samba_addr,
                "-o",
                options,
                mountpoint.as_posix(),
            ],
            check=True,
        )
    spotlight_basedir = mountpoint / config.SCITAS_EXPORT_RELATIVE_WORKDIR
    spotlight_basedir.mkdir(parents=True, exist_ok=True)


def main():
    tyro.cli(mount_scitas_export_share)


if __name__ == "__main__":
    main()
