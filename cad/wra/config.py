"""Load params.yaml into an attribute-accessible structure (P.arm.L2 ...).

All CAD modules, the calculation script, the URDF generator and the firmware-header
generator read the SAME file so the design cannot drift between them.
"""
from __future__ import annotations
import copy
import pathlib
import yaml

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT = HERE / "params.yaml"


class AttrDict(dict):
    """dict with attribute access, recursive."""

    def __getattr__(self, k):
        try:
            return self[k]
        except KeyError as e:
            raise AttributeError(k) from e

    def __setattr__(self, k, v):
        self[k] = v

    @staticmethod
    def wrap(obj):
        if isinstance(obj, dict):
            return AttrDict({k: AttrDict.wrap(v) for k, v in obj.items()})
        if isinstance(obj, list):
            return [AttrDict.wrap(v) for v in obj]
        return obj


def load(path: str | pathlib.Path | None = None, overrides: dict | None = None) -> AttrDict:
    data = yaml.safe_load(pathlib.Path(path or DEFAULT).read_text())
    if overrides:
        data = copy.deepcopy(data)
        for dotted, val in overrides.items():
            d = data
            keys = dotted.split(".")
            for k in keys[:-1]:
                d = d[k]
            d[keys[-1]] = val
    return AttrDict.wrap(data)


P = load()
