"""Make the ament Python package importable from a source checkout."""

import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))
