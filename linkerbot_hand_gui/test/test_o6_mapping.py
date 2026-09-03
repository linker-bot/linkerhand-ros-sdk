import json
import os
import re
import struct
from pathlib import Path

import pytest

from linkerbot_hand_gui.asset_resolver import (
    O6Assets,
    load_o6_decoder,
    load_o6_urdf,
    resolve_o6_assets,
)
from linkerbot_hand_gui.joint_mapping import O6JointMapping


ASSET_ROOT = (
    Path(os.environ["LINKERBOT_HAND_ASSETS_ROOT"]).expanduser()
    if os.environ.get("LINKERBOT_HAND_ASSETS_ROOT")
    else None
)


@pytest.mark.skipif(
    ASSET_ROOT is None or not ASSET_ROOT.is_dir(),
    reason="set LINKERBOT_HAND_ASSETS_ROOT for asset tests",
)
@pytest.mark.parametrize("side", ["left", "right"])
def test_o6_assets_and_decoder_match_urdf(side: str) -> None:
    assert ASSET_ROOT is not None
    assets = resolve_o6_assets(side, ASSET_ROOT)
    mapping = O6JointMapping.from_assets(assets)
    assert len(mapping.urdf_names) == 6
    assert load_o6_decoder(assets) == mapping.capability_to_urdf
    assert all("joint" not in name for name in mapping.capability_names)


@pytest.mark.skipif(
    ASSET_ROOT is None or not ASSET_ROOT.is_dir(),
    reason="set LINKERBOT_HAND_ASSETS_ROOT for asset tests",
)
def test_o6_percent_mapping_is_reversible_for_right_hand() -> None:
    assert ASSET_ROOT is not None
    assets = resolve_o6_assets("right", ASSET_ROOT)
    mapping = O6JointMapping.from_assets(assets)
    values = {name: limit.lower for name, limit in mapping.limits.items()}
    percent = mapping.percent_from_urdf(values)
    assert percent == pytest.approx((100.0,) * 6)
    reconstructed = mapping.urdf_from_percent(percent)
    assert reconstructed.keys() == values.keys()
    for name, value in values.items():
        assert reconstructed[name] == pytest.approx(value)


@pytest.mark.skipif(
    ASSET_ROOT is None or not ASSET_ROOT.is_dir(),
    reason="set LINKERBOT_HAND_ASSETS_ROOT for asset tests",
)
def test_o6_preview_uses_materialized_visual_glb() -> None:
    """Complete official GLBs must render through the colour-corrected GLB cache."""

    assert ASSET_ROOT is not None
    assets = resolve_o6_assets("right", ASSET_ROOT)
    description = load_o6_urdf(assets, require_complete_visual=True)
    assert all(
        Path(filename).suffix == ".glb" and Path(filename).is_file()
        for filename in re.findall(r"file://([^\"]+)", description)
        if "_vis." in filename
    )


def test_unknown_side_is_rejected() -> None:
    with pytest.raises(ValueError, match="left.*right|right.*left"):
        resolve_o6_assets("middle")


def test_urdf_prefers_materialized_visual_glb(tmp_path: Path) -> None:
    """Do not replace a complete visual mesh with an OBJ companion."""

    component = tmp_path / "linkerhand_o6"
    (component / "meshes" / "visual").mkdir(parents=True)
    (component / "meshes" / "collision").mkdir()
    urdf = component / "linkerhand_o6_right.urdf"
    urdf.write_text(
        """<robot name=\"o6\"><link name=\"base\"><visual><geometry>
        <mesh filename=\"meshes/visual/palm_vis.glb\"/>
        </geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    glb = component / "meshes" / "visual" / "palm_vis.glb"
    glb.write_bytes(struct.pack("<4sII", b"glTF", 2, 304) + b"\0" * 292)
    (component / "meshes" / "visual" / "palm_vis.obj").write_bytes(b"o" * 400)
    assets = O6Assets("right", component, urdf, component / "decoder.yaml")

    description = load_o6_urdf(assets)

    assert "palm_vis.glb" in description
    assert "palm_vis.obj" not in description


def test_strict_preview_rejects_visual_obj_fallback(tmp_path: Path) -> None:
    """Do not display a partly uncoloured model when a GLB is missing."""

    component = tmp_path / "linkerhand_o6"
    (component / "meshes" / "visual").mkdir(parents=True)
    urdf = component / "linkerhand_o6_right.urdf"
    urdf.write_text(
        """<robot name="o6"><link name="base"><visual><geometry>
        <mesh filename="meshes/visual/finger_vis.glb"/>
        </geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    (component / "meshes" / "visual" / "finger_vis.glb").write_text(
        "version https://git-lfs.github.com/spec/v1\n",
        encoding="utf-8",
    )
    (component / "meshes" / "visual" / "finger_vis.obj").write_bytes(b"o" * 400)
    assets = O6Assets("right", component, urdf, component / "decoder.yaml")

    with pytest.raises(FileNotFoundError, match="complete visual GLB"):
        load_o6_urdf(assets, require_complete_visual=True)


def test_thumb_joint_missing_material_is_rendered_grey(tmp_path: Path) -> None:
    """A missing thumb material must not fall back to glTF's white default."""

    component = tmp_path / "linkerhand_o6"
    visual_dir = component / "meshes" / "visual"
    visual_dir.mkdir(parents=True)
    urdf = component / "linkerhand_o6_right.urdf"
    urdf.write_text(
        """<robot name="o6"><link name="thumb"><visual><geometry>
        <mesh filename="meshes/visual/thumb_vis.glb"/>
        </geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    document = {
        "asset": {"version": "2.0"},
        "materials": [{"name": "mat2-thumb_metacarpals", "doubleSided": True}],
    }
    json_chunk = json.dumps(document, separators=(",", ":")).encode()
    json_chunk += b" " * ((-len(json_chunk)) % 4)
    # ``_is_mesh_file`` intentionally rejects tiny files; pad the synthetic
    # binary chunk to the same minimum size used by real GLB assets.
    json_chunk += b" " * max(0, 256 - len(json_chunk))
    glb = struct.pack("<4sII", b"glTF", 2, 20 + len(json_chunk))
    glb += struct.pack("<II", len(json_chunk), 0x4E4F534A) + json_chunk
    source = visual_dir / "thumb_vis.glb"
    source.write_bytes(glb)
    assets = O6Assets("right", component, urdf, component / "decoder.yaml")

    description = load_o6_urdf(assets, require_complete_visual=True)
    cached = Path(re.search(r"file://([^\"]+)", description).group(1))
    data = cached.read_bytes()
    chunk_length = struct.unpack_from("<I", data, 12)[0]
    patched = json.loads(data[20 : 20 + chunk_length].rstrip(b" \x00"))
    pbr = patched["materials"][0]["pbrMetallicRoughness"]

    assert pbr["baseColorFactor"] == pytest.approx(
        [0.4225910306, 0.4269180596, 0.4026749134, 1.0]
    )
    assert pbr["metallicFactor"] == pytest.approx(0.393)
    assert pbr["roughnessFactor"] == pytest.approx(0.302)


def test_rviz_material_extensions_are_removed_without_changing_base_colors(
    tmp_path: Path,
) -> None:
    """The cache rewrite must keep PBR colours but remove RViz-buggy extras."""

    component = tmp_path / "linkerhand_o6"
    visual_dir = component / "meshes" / "visual"
    visual_dir.mkdir(parents=True)
    urdf = component / "linkerhand_o6_right.urdf"
    urdf.write_text(
        """<robot name=\"o6\"><link name=\"base\"><visual><geometry>
        <mesh filename=\"meshes/visual/base_vis.glb\"/>
        </geometry></visual></link></robot>""",
        encoding="utf-8",
    )
    document = {
        "asset": {"version": "2.0"},
        "extensionsUsed": ["KHR_materials_specular", "KHR_materials_ior"],
        "extensionsRequired": ["KHR_materials_specular"],
        "materials": [
            {
                "name": "blue",
                "extensions": {
                    "KHR_materials_specular": {"specularColorFactor": [1.4, 1.4, 1.4]},
                    "KHR_materials_ior": {"ior": 1.0},
                },
                "pbrMetallicRoughness": {
                    "baseColorFactor": [0.1, 0.2, 0.8, 1.0]
                },
            }
        ],
    }
    payload = json.dumps(document, separators=(",", ":")).encode()
    payload += b" " * ((-len(payload)) % 4)
    glb = struct.pack("<4sII", b"glTF", 2, 20 + len(payload))
    glb += struct.pack("<II", len(payload), 0x4E4F534A) + payload
    source = visual_dir / "base_vis.glb"
    source.write_bytes(glb)
    assets = O6Assets("right", component, urdf, component / "decoder.yaml")

    description = load_o6_urdf(assets, require_complete_visual=True)
    cached = Path(re.search(r"file://([^\"]+)", description).group(1))
    data = cached.read_bytes()
    chunk_length = struct.unpack_from("<I", data, 12)[0]
    patched = json.loads(data[20 : 20 + chunk_length].rstrip(b" \x00"))

    assert patched["materials"][0]["pbrMetallicRoughness"]["baseColorFactor"] == [
        0.1,
        0.2,
        0.8,
        1.0,
    ]
    assert "extensionsUsed" not in patched
    assert "extensionsRequired" not in patched
    assert "extensions" not in patched["materials"][0]
