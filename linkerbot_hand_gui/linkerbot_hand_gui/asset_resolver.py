"""Resolve O6 URDF resources from the shared linker-sim-assets package."""

from __future__ import annotations

import hashlib
import importlib
import json
import os
import struct
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Literal
from xml.etree import ElementTree

import yaml

Side = Literal["left", "right"]
_ASSET_ENV = "LINKERBOT_HAND_ASSETS_ROOT"
_COMPONENT = "linkerhand_o6"
_MATERIAL_PATCH_VERSION = "o6-rviz-glb-v4"
_JOINT_GREY = (0.4225910306, 0.4269180596, 0.4026749134, 1.0)
_JOINT_METALLIC = 0.393
_JOINT_ROUGHNESS = 0.302


@dataclass(frozen=True, slots=True)
class O6Assets:
    """Paths and source metadata for one O6 variant."""

    side: Side
    component_dir: Path
    urdf_path: Path
    decoder_path: Path


def resolve_o6_assets(
    side: str, assets_root: str | os.PathLike[str] | None = None
) -> O6Assets:
    """Find the official O6 URDF and decoder for ``side``.

    Resolution order is explicit path, ``LINKERBOT_HAND_ASSETS_ROOT``, then
    the installed ``linker_robot_assets`` package.  The resolver accepts both
    the package's ``asset_root()`` directory and a repository checkout.

    The mesh files are intentionally not copied into this ROS package.  The
    launch description rewrites their relative paths against the resolved
    component directory, so a checkout with Git LFS files works unchanged.
    """

    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    candidates: list[Path] = []
    if assets_root is not None:
        candidates.append(Path(assets_root).expanduser())
    env_root = os.environ.get(_ASSET_ENV)
    if env_root:
        candidates.append(Path(env_root).expanduser())
    try:
        assets_module = importlib.import_module("linker_robot_assets")
        package_root = assets_module.asset_root()
    except (ImportError, AttributeError):
        package_root = None
    if package_root is not None:
        candidates.append(Path(package_root))

    checked: list[Path] = []
    for candidate in candidates:
        for component_dir in _component_candidates(candidate):
            if component_dir in checked:
                continue
            checked.append(component_dir)
            urdf = component_dir / f"linkerhand_o6_{side}.urdf"
            decoder = component_dir / "decoder.yaml"
            if urdf.is_file() and decoder.is_file():
                return O6Assets(side, component_dir, urdf, decoder)

    searched = ", ".join(str(path) for path in checked) or "<no asset roots>"
    raise FileNotFoundError(
        "O6 URDF assets were not found. Install linker-robot-assets or set "
        f"{_ASSET_ENV} to linker-sim-assets/src/linker_robot_assets/assets; "
        f"searched: {searched}"
    )


def load_o6_urdf(assets: O6Assets, *, require_complete_visual: bool = False) -> str:
    """Load URDF XML and make every relative mesh path absolute.

    ``robot_state_publisher`` receives a string parameter and has no reliable
    notion of the source URDF's directory.  Resolving paths here avoids the
    common failure where the model's joints work but RViz cannot load meshes.

    Args:
        assets: Resolved O6 asset directory and URDF.
        require_complete_visual: Require every visual mesh to be a complete
            official GLB.  This is used by the standalone preview so one
            missing Git LFS object cannot silently turn part of the model
            into an uncoloured OBJ fallback.  Collision meshes retain the
            historical fallback behaviour when this is false.

    Raises:
        FileNotFoundError: If strict visual mode finds a missing, partial, or
            non-GLB visual mesh.
    """

    root = ElementTree.parse(assets.urdf_path).getroot()
    visual_meshes = {id(mesh) for mesh in root.findall(".//visual//mesh")}
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if not filename or filename.startswith(("file://", "package://")):
            continue
        resolved = (assets.urdf_path.parent / filename).resolve()
        if require_complete_visual and id(mesh) in visual_meshes:
            if resolved.suffix.lower() != ".glb" or not _is_mesh_file(resolved):
                raise FileNotFoundError(
                    "O6 preview requires complete visual GLB "
                    f"{resolved}; download the Git LFS asset instead of using "
                    "an OBJ fallback"
                )
            mesh.set("filename", f"file://{_materialize_visual_materials(resolved)}")
            continue
        # Keep a materialized visual GLB whenever possible.  GLB and OBJ
        # companions are exported from the same source, but they are not safe
        # fallbacks when one file is a partial Git LFS download.  In
        # particular, preferring a partial palm OBJ makes the complete hand
        # disappear while the finger meshes remain visible.
        if resolved.suffix.lower() == ".glb":
            if _is_mesh_file(resolved):
                pass
            elif _is_mesh_file(resolved.with_suffix(".obj")):
                resolved = resolved.with_suffix(".obj")
            elif _is_mesh_file(_collision_companion(resolved)):
                resolved = _collision_companion(resolved)
        elif not _is_mesh_file(resolved):
            collision = _collision_companion(resolved)
            if _is_mesh_file(collision):
                resolved = collision
        if id(mesh) in visual_meshes and resolved.suffix.lower() == ".glb":
            resolved = _materialize_visual_materials(resolved)
        mesh.set("filename", f"file://{resolved}")
    return ElementTree.tostring(root, encoding="unicode")


def _collision_companion(path: Path) -> Path:
    """Return the conventional ``visual/*_vis.*`` to ``collision/*_col.obj`` path."""

    visual_name = path.name
    if "_vis." not in visual_name:
        return path
    collision_name = visual_name.replace("_vis.", "_col.")
    return (
        path.parent.parent / "collision" / Path(collision_name).with_suffix(".obj").name
    )


def _is_mesh_file(path: Path) -> bool:
    """Check that a mesh exists and is not an unmaterialized Git LFS pointer."""

    try:
        size = path.stat().st_size
        if size < 256:
            return False
        with path.open("rb") as stream:
            prefix = stream.read(128)
            if b"git-lfs.github.com/spec" in prefix:
                return False
            if path.suffix.lower() == ".glb":
                # A partially resumed LFS download still has a valid glTF
                # magic and can be several megabytes long.  RViz silently
                # drops it unless the header's declared length is present.
                stream.seek(0)
                header = stream.read(12)
                if len(header) != 12 or header[:4] != b"glTF":
                    return False
                declared_length = struct.unpack_from("<I", header, 8)[0]
                return declared_length == size
            if path.suffix.lower() == ".obj":
                # Detect the common truncated visual OBJ pattern: all vertex
                # records arrived but only a small prefix of faces did.  A
                # complete visual mesh has a face count of the same order as
                # its vertex count; collision meshes are intentionally small
                # and are allowed through this check.
                stream.seek(0)
                vertices = faces = 0
                for line in stream:
                    if line.startswith(b"v "):
                        vertices += 1
                    elif line.startswith(b"f "):
                        faces += 1
                if vertices > 1000 and faces < vertices / 20:
                    return False
    except OSError:
        return False
    return True


def _materialize_visual_materials(path: Path) -> Path:
    """Make an O6 visual GLB consumable by the Assimp/RViz renderer.

    The official meshes use Blender's ``KHR_materials_specular`` and, for a
    few files, ``KHR_materials_ior`` extensions.  The Assimp version shipped
    with some ROS 2 distributions accepts the files but maps those extensions
    incorrectly, which produces the characteristic black/cyan striping in
    RViz.  RViz only needs the standard PBR base colour for this preview, so
    the unsupported material extensions are removed from a cache copy.  One
    thumb mesh also has a material slot without a base colour; it is assigned
    the same joint grey used by the companion mesh.

    The source asset is external and remains untouched.  A deterministic
    cache copy is used only when a rewrite is necessary.
    """

    try:
        data = path.read_bytes()
        if len(data) < 20 or data[:4] != b"glTF":
            return path
        version, declared_length = struct.unpack_from("<II", data, 4)
        if declared_length != len(data):
            return path
        offset = 12
        chunks: list[tuple[int, bytes]] = []
        document: dict[str, object] | None = None
        json_index = -1
        while offset + 8 <= len(data):
            chunk_length, chunk_type = struct.unpack_from("<II", data, offset)
            offset += 8
            chunk_end = offset + chunk_length
            if chunk_end > len(data):
                return path
            payload = data[offset:chunk_end]
            offset = chunk_end
            chunks.append((chunk_type, payload))
            if chunk_type == 0x4E4F534A:
                json_index = len(chunks) - 1
                document = json.loads(payload.rstrip(b"\x00 \t\r\n"))
        if document is None or json_index < 0:
            return path

        changed = False
        for material in document.get("materials", []):
            if not isinstance(material, dict):
                continue
            # ``KHR_materials_specular`` and ``KHR_materials_ior`` are valid
            # glTF, but the Assimp/RViz combination in ROS Jazzy can render
            # their values as saturated strips.  Keep standard PBR fields and
            # drop only these optional extensions.
            extensions = material.get("extensions")
            if isinstance(extensions, dict):
                for extension in ("KHR_materials_specular", "KHR_materials_ior"):
                    if extension in extensions:
                        del extensions[extension]
                        changed = True
                if not extensions:
                    material.pop("extensions", None)

            pbr = material.setdefault("pbrMetallicRoughness", {})
            if not isinstance(pbr, dict):
                pbr = {}
                material["pbrMetallicRoughness"] = pbr
            if "baseColorFactor" not in pbr:
                # The O6 thumb metacarpal is the only known mesh with an
                # uncoloured material slot.  Keep this conservative fallback
                # for any exporter that leaves a slot unnamed as well: a
                # neutral joint grey is preferable to RViz's bright white.
                pbr["baseColorFactor"] = list(_JOINT_GREY)
                pbr["metallicFactor"] = _JOINT_METALLIC
                pbr["roughnessFactor"] = _JOINT_ROUGHNESS
                changed = True

        # Remove extension declarations only after material extensions have
        # been removed.  Other future extensions remain declared and are not
        # silently altered by this compatibility path.
        for key in ("extensionsUsed", "extensionsRequired"):
            values = document.get(key)
            if isinstance(values, list):
                filtered = [
                    value
                    for value in values
                    if value not in {"KHR_materials_specular", "KHR_materials_ior"}
                ]
                if filtered != values:
                    changed = True
                    if filtered:
                        document[key] = filtered
                    else:
                        document.pop(key, None)
        if not changed:
            return path

        encoded = json.dumps(
            document, separators=(",", ":"), ensure_ascii=False
        ).encode("utf-8")
        encoded += b" " * ((-len(encoded)) % 4)
        chunks[json_index] = (0x4E4F534A, encoded)
        rebuilt = bytearray(struct.pack("<4sII", b"glTF", version, 0))
        for chunk_type, payload in chunks:
            rebuilt.extend(struct.pack("<II", len(payload), chunk_type))
            rebuilt.extend(payload)
        struct.pack_into("<I", rebuilt, 8, len(rebuilt))

        cache_root = Path(os.environ.get("XDG_CACHE_HOME", tempfile.gettempdir())) / (
            "linkerbot_hand_gui"
            if "XDG_CACHE_HOME" in os.environ
            else "linkerbot_hand_gui_meshes"
        )
        cache_root.mkdir(parents=True, exist_ok=True)
        digest = hashlib.sha256(data + _MATERIAL_PATCH_VERSION.encode()).hexdigest()[
            :16
        ]
        cached = cache_root / f"{path.stem}.{digest}.glb"
        if not cached.is_file() or not _is_mesh_file(cached):
            temporary = cached.with_suffix(cached.suffix + ".tmp")
            temporary.write_bytes(rebuilt)
            os.replace(temporary, cached)
        return cached
    except (OSError, ValueError, TypeError, json.JSONDecodeError, struct.error):
        # Rendering can still proceed with the original complete GLB if a
        # user cache is read-only or an unfamiliar exporter is encountered.
        return path


def load_o6_decoder(assets: O6Assets) -> tuple[str, ...]:
    """Return decoder channel names in SDK command order."""

    with assets.decoder_path.open(encoding="utf-8") as stream:
        spec = yaml.safe_load(stream) or {}
    channels = spec.get("channels")
    if (
        not isinstance(channels, list)
        or not channels
        or not all(isinstance(value, str) and value for value in channels)
    ):
        raise ValueError(f"{assets.decoder_path}: channels must be a non-empty list")
    # ``{S}h_...`` expands to ``lh_...`` or ``rh_...``; only the
    # one-character side prefix is substituted here.
    prefix = "l" if assets.side == "left" else "r"
    return tuple(value.replace("{S}", prefix, 1) for value in channels)


def _component_candidates(root: Path) -> tuple[Path, ...]:
    """Return supported roots without assuming a particular checkout layout."""

    return (
        root / "components" / "hands" / _COMPONENT,
        root
        / "src"
        / "linker_robot_assets"
        / "assets"
        / "components"
        / "hands"
        / _COMPONENT,
        root / _COMPONENT,
        root,
    )


__all__ = ["O6Assets", "Side", "load_o6_decoder", "load_o6_urdf", "resolve_o6_assets"]
