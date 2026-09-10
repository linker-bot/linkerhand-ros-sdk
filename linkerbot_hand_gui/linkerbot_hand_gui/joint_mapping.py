"""Pure O6 capability-to-URDF joint mapping and conversion logic."""

from __future__ import annotations

import math
from dataclasses import dataclass
from xml.etree import ElementTree

from .asset_resolver import O6Assets, load_o6_decoder

_CAPABILITY_TO_DECODER_NAME = {
    "thumb_flex": "thumb_cmc_pitch",
    "thumb_abd": "thumb_cmc_yaw",
    "index": "index_mcp_pitch",
    "middle": "middle_mcp_pitch",
    "ring": "ring_mcp_pitch",
    "pinky": "pinky_mcp_pitch",
}


@dataclass(frozen=True, slots=True)
class JointLimit:
    """URDF radians range for one active joint."""

    lower: float
    upper: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.lower) or not math.isfinite(self.upper):
            raise ValueError("joint limits must be finite")
        if self.upper <= self.lower:
            raise ValueError("joint upper limit must exceed lower limit")


@dataclass(frozen=True, slots=True)
class O6JointMapping:
    """Mapping between ROS capability order and URDF active-joint order."""

    capability_names: tuple[str, ...]
    urdf_names: tuple[str, ...]
    limits: dict[str, JointLimit]
    capability_to_urdf: tuple[str, ...]
    side: str

    @classmethod
    def from_assets(cls, assets: O6Assets) -> O6JointMapping:
        """Build a mapping from official URDF and decoder metadata."""

        root = ElementTree.parse(assets.urdf_path).getroot()
        prefix = "l" if assets.side == "left" else "r"
        active: list[str] = []
        limits: dict[str, JointLimit] = {}
        for joint in root.findall("joint"):
            name = joint.get("name")
            if (
                not name
                or joint.get("type") == "fixed"
                or joint.find("mimic") is not None
            ):
                continue
            limit = joint.find("limit")
            if limit is None:
                raise ValueError(
                    f"{assets.urdf_path}: active joint {name!r} has no limit"
                )
            active.append(name)
            limits[name] = JointLimit(
                lower=float(limit.get("lower", "nan")),
                upper=float(limit.get("upper", "nan")),
            )

        decoder_channels = load_o6_decoder(assets)
        expected = tuple(
            f"{prefix}h_{suffix}" for suffix in _CAPABILITY_TO_DECODER_NAME.values()
        )
        if decoder_channels != expected:
            raise ValueError(
                f"{assets.decoder_path}: O6 decoder order is {decoder_channels}, "
                f"expected {expected}"
            )
        if set(active) != set(decoder_channels):
            raise ValueError(
                f"{assets.urdf_path}: active joints do not match decoder channels"
            )
        capability_names = tuple(_CAPABILITY_TO_DECODER_NAME)
        return cls(
            capability_names=capability_names,
            urdf_names=tuple(active),
            limits=limits,
            capability_to_urdf=decoder_channels,
            side=assets.side,
        )

    def validate_capabilities(
        self, names: tuple[str, ...], model: str, side: str
    ) -> None:
        """Reject a runtime profile that cannot be mapped safely."""

        if model.upper() != "O6":
            raise ValueError(f"O6 GUI cannot control model {model!r}")
        if side != self.side:
            raise ValueError(
                f"runtime O6 side {side!r} does not match GUI side {self.side!r}"
            )
        if names != self.capability_names:
            raise ValueError(
                f"runtime O6 joint order {names!r} does not match "
                f"{self.capability_names!r}"
            )

    def percent_from_urdf(self, values: dict[str, float]) -> tuple[float, ...]:
        """Convert active-joint radians to O6 percent in capability order."""

        result: list[float] = []
        for name in self.capability_to_urdf:
            if name not in values:
                raise ValueError(f"target joint state is missing {name!r}")
            value = float(values[name])
            if not math.isfinite(value):
                raise ValueError(f"target joint {name!r} is not finite")
            limit = self.limits[name]
            bounded = min(limit.upper, max(limit.lower, value))
            result.append(100.0 * (limit.upper - bounded) / (limit.upper - limit.lower))
        return tuple(result)

    def urdf_from_percent(self, percent: tuple[float, ...]) -> dict[str, float]:
        """Convert capability percent feedback to active-joint radians."""

        if len(percent) != len(self.capability_names):
            raise ValueError("percent feedback length does not match O6 capabilities")
        result: dict[str, float] = {}
        for urdf_name, value in zip(self.capability_to_urdf, percent, strict=True):
            if not math.isfinite(value):
                continue
            bounded = min(100.0, max(0.0, float(value)))
            limit = self.limits[urdf_name]
            result[urdf_name] = limit.lower + (100.0 - bounded) / 100.0 * (
                limit.upper - limit.lower
            )
        return result


__all__ = ["JointLimit", "O6JointMapping"]
