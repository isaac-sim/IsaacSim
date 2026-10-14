"""Check a short benchmark through its installed command-line entry point."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path


def main() -> None:
    """Verify validation, retained sample count, and default simulation timestep."""
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory) / "result.json"
        subprocess.run(
            [
                sys.executable,
                str(Path(__file__).with_name("main.py")),
                "--output",
                str(output),
                "--warmup-frames",
                "2",
                "--measured-frames",
                "20",
            ],
            check=True,
        )
        result = json.loads(output.read_text())
        if not result["status"]["valid"]:
            raise AssertionError(result["status"])
        if result["benchmark"]["parameters"]["time_step"] != 0.005:
            raise AssertionError("Expected a 0.005 s physics timestep")
        if len(result["metrics"]["duration_ms"]["samples"]) != 20:
            raise AssertionError("Unexpected measured sample count")
        if [sample["value"] for sample in result["metrics"]["simulated_time_s"]["samples"]] != [0.005] * 20:
            raise AssertionError("Unexpected simulated time per frame")
    print("Benchmark smoke test passed.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
