import sys
from pathlib import Path

# Ajoute la racine du repository au PYTHONPATH pour que "pentograph_core" soit importable
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

