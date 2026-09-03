"""Make the ament Python package importable from a source checkout."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
