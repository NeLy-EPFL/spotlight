import os
import subprocess
import warnings
from pathlib import Path


def _resolve_n_workers(n_workers: int) -> int:
    if "SLURM_CPUS_PER_TASK" in os.environ:
        n_cores_total = int(os.environ["SLURM_CPUS_PER_TASK"])
    else:
        n_cores_total = os.cpu_count()

    if n_workers > n_cores_total:
        warnings.warn(
            f"Requested {n_workers} workers, but only {n_cores_total} cores are "
            f"available. Using {n_cores_total} workers instead."
        )
        return n_cores_total

    if n_workers == 0:
        return 1

    if n_workers <= 0:
        # if n_workers is -1, use all cores. if -2, use all but one cores, etc.
        return max(1, n_cores_total + n_workers + 1)

    return n_workers


def zip_dir(input_dir: Path, output_path: Path, n_workers: int) -> None:
    """Zip data using multiple threads.

    This does the equivalent of `tar cf - input_dir | pigz > output_path`.
    """
    if not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory {input_dir} does not exist.")
    n_workers = _resolve_n_workers(n_workers)

    with open(output_path, "wb") as output_file:
        tar = subprocess.Popen(
            ["tar", "cf", "-", "-C", str(input_dir.parent), input_dir.name],
            stdout=subprocess.PIPE,
        )
        pigz = subprocess.Popen(
            ["pigz", "-p", str(n_workers)], stdin=tar.stdout, stdout=output_file
        )
        tar.stdout.close()
        pigz.communicate()
        tar.wait()

    if tar.returncode != 0:
        raise subprocess.CalledProcessError(tar.returncode, tar.args)
    if pigz.returncode != 0:
        raise subprocess.CalledProcessError(pigz.returncode, pigz.args)


def unzip_dir(input_path: Path, output_dir: Path, n_workers: int = -1) -> None:
    """Unpack zipped data using multiple threads (the opposite of `zip_dir`)."""
    if not input_path.is_file():
        raise FileNotFoundError(f"Input archive {input_path} does not exist.")
    n_workers = _resolve_n_workers(n_workers)
    output_dir.mkdir(parents=True, exist_ok=True)

    with open(input_path, "rb") as input_file:
        pigz = subprocess.Popen(
            ["pigz", "-dc", "-p", str(n_workers)],
            stdin=input_file,
            stdout=subprocess.PIPE,
        )
        tar = subprocess.Popen(
            ["tar", "xf", "-", "-C", str(output_dir)], stdin=pigz.stdout
        )
        pigz.stdout.close()
        tar.communicate()
        pigz.wait()

    if pigz.returncode != 0:
        raise subprocess.CalledProcessError(pigz.returncode, pigz.args)
    if tar.returncode != 0:
        raise subprocess.CalledProcessError(tar.returncode, tar.args)
