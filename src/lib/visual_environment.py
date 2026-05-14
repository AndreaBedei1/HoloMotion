from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class WaterFogConfig:
    """Visual water-fog parameters supported by HoloOcean 2.2.2."""

    fog_density: float = 10.0
    fog_depth: float = 0.0
    color_r: float = 0.4
    color_g: float = 0.6
    color_b: float = 1.0

    def to_dict(self) -> dict:
        return {
            "fog_density": float(self.fog_density),
            "fog_depth": float(self.fog_depth),
            "color_r": float(self.color_r),
            "color_g": float(self.color_g),
            "color_b": float(self.color_b),
            "api_method": "HoloOceanEnvironment.water_fog",
            "command": "WaterFogCommand",
            "effect_scope": "visual only",
        }


def strongest_supported_water_fog_config() -> WaterFogConfig:
    """Return the strongest documented HoloOcean water-fog setting."""

    return WaterFogConfig(fog_density=10.0, fog_depth=0.0)


def apply_water_fog(env, config: WaterFogConfig) -> None:
    """Apply visual water fog through HoloOcean's supported runtime API."""

    env.water_fog(
        config.fog_density,
        config.fog_depth,
        config.color_r,
        config.color_g,
        config.color_b,
    )
