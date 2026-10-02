"""Patch repoman.py to use OMNI_REPO_ROOT instead of realpath resolution.

repo.bat sets OMNI_REPO_ROOT with embedded quotes ("%~dp0"), and
os.path.realpath() follows junctions back to the long path. This script
patches repoman.py to prefer a clean OMNI_REPO_ROOT value.
"""
import pathlib
import sys

REPOMAN = pathlib.Path(r"C:\b\tools\repoman\repoman.py")

OLD = 'REPO_ROOT = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..")'

# Strip embedded quotes and trailing backslashes from OMNI_REPO_ROOT
NEW = """REPO_ROOT = os.path.normpath(os.environ.get("OMNI_REPO_ROOT", "").strip('"\\\\')) if os.environ.get("OMNI_REPO_ROOT", "").strip('"\\\\') else os.path.join(os.path.dirname(os.path.realpath(__file__)), "../..")"""

src = REPOMAN.read_text(encoding="utf-8")
if OLD not in src:
    print(f"ERROR: Could not find REPO_ROOT pattern in {REPOMAN}", file=sys.stderr)
    sys.exit(1)

REPOMAN.write_text(src.replace(OLD, NEW), encoding="utf-8")
print(f"Patched {REPOMAN} to prefer OMNI_REPO_ROOT")
