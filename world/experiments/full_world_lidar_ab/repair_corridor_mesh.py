#!/usr/bin/env python3
"""Add inward-facing copies of the full-world corridor faces.

Gazebo's GPU LiDAR detects the reversed faces in the experiment but misses the
inside of the original outward-only corridor cover.  This tool retains the
existing triangles for exterior rendering/collision and adds reversed triangles
as a second primitive for each corridor cover.  It makes a backup before an
in-place change and is deliberately idempotent.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import struct
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / "models/models/miss2_env/miss2.glb"
BACKUP = TARGET.with_name(TARGET.name + ".before_two_sided_corridor_repair")
MARKER = "sae_inward_face_copy"


def parse_glb(path: Path) -> tuple[dict, bytes]:
    raw = path.read_bytes()
    if len(raw) < 20:
        raise ValueError("GLB is too short")
    magic, version, total = struct.unpack_from("<III", raw, 0)
    if magic != 0x46546C67 or version != 2 or total != len(raw):
        raise ValueError("Expected a complete GLB version 2 file")
    json_size, json_kind = struct.unpack_from("<II", raw, 12)
    if json_kind != 0x4E4F534A:
        raise ValueError("First GLB chunk is not JSON")
    json_end = 20 + json_size
    bin_size, bin_kind = struct.unpack_from("<II", raw, json_end)
    if bin_kind != 0x004E4942 or json_end + 8 + bin_size != len(raw):
        raise ValueError("Expected exactly one BIN chunk")
    return json.loads(raw[20:json_end]), raw[json_end + 8:]


def build_glb(document: dict, binary: bytes) -> bytes:
    encoded = json.dumps(document, separators=(",", ":")).encode("utf-8")
    encoded += b" " * (-len(encoded) % 4)
    binary += b"\0" * (-len(binary) % 4)
    total = 12 + 8 + len(encoded) + 8 + len(binary)
    return (struct.pack("<III", 0x46546C67, 2, total)
            + struct.pack("<II", len(encoded), 0x4E4F534A) + encoded
            + struct.pack("<II", len(binary), 0x004E4942) + binary)


def corridor_primitives(document: dict) -> list[dict]:
    result: list[dict] = []
    for node in document["nodes"]:
        if node.get("name", "").startswith("corridor cover"):
            result.extend(document["meshes"][node["mesh"]]["primitives"])
    if len(result) not in (2, 4):
        raise ValueError(f"Expected two or four corridor primitives; found {len(result)}")
    return result


def report(document: dict) -> dict:
    primitives = corridor_primitives(document)
    marked = [p for p in primitives if p.get("extras", {}).get(MARKER)]
    return {
        "corridor_primitives": len(primitives),
        "inward_face_copies": len(marked),
        "already_repaired": len(marked) == 2,
    }


def repair(document: dict, binary: bytes) -> tuple[dict, bytes]:
    before = report(document)
    if before["already_repaired"]:
        return document, binary
    if before["inward_face_copies"]:
        raise ValueError("Found only one inward-face copy; refusing a partial repair")

    primitives = [p for p in corridor_primitives(document)
                  if not p.get("extras", {}).get(MARKER)]
    # The original covers share their index accessor. Validate that deliberately
    # so the same reversed triangle list can be safely used by both meshes.
    source_index = primitives[0]["indices"]
    if primitives[1]["indices"] != source_index:
        raise ValueError("Corridor covers no longer share their original index accessor")
    accessor = document["accessors"][source_index]
    if accessor["componentType"] != 5123 or accessor["type"] != "SCALAR" or accessor["count"] % 3:
        raise ValueError("Unexpected corridor index accessor layout")
    view = document["bufferViews"][accessor["bufferView"]]
    if view.get("buffer", 0) != 0:
        raise ValueError("Only buffer 0 is supported")
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    indices = np.frombuffer(binary, dtype="<u2", count=accessor["count"], offset=start).copy().reshape(-1, 3)
    indices[:, [1, 2]] = indices[:, [2, 1]]

    append_offset = len(binary)
    if append_offset % 4:
        binary += b"\0" * (-append_offset % 4)
        append_offset = len(binary)
    reversed_bytes = indices.astype("<u2", copy=False).tobytes()
    binary += reversed_bytes
    document["buffers"][0]["byteLength"] = len(binary)
    view_index = len(document["bufferViews"])
    document["bufferViews"].append({"buffer": 0, "byteOffset": append_offset,
                                    "byteLength": len(reversed_bytes), "target": 34963})
    accessor_index = len(document["accessors"])
    document["accessors"].append({"bufferView": view_index, "componentType": 5123,
                                  "count": accessor["count"], "type": "SCALAR"})

    # Add one reversed primitive per mesh, with original attributes/material.
    # This produces coincident inside/outside surfaces without moving walls,
    # changing corridor width, or removing the outward collision-facing faces.
    for node in document["nodes"]:
        if not node.get("name", "").startswith("corridor cover"):
            continue
        mesh = document["meshes"][node["mesh"]]
        original = mesh["primitives"][0]
        inward = copy.deepcopy(original)
        inward["indices"] = accessor_index
        inward.setdefault("extras", {})[MARKER] = True
        mesh["primitives"].append(inward)
    return document, binary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true", help="write the repaired production GLB")
    args = parser.parse_args()
    document, binary = parse_glb(TARGET)
    prior = report(document)
    print(json.dumps({"target": str(TARGET), "sha256_before": hashlib.sha256(TARGET.read_bytes()).hexdigest(),
                      "before": prior}, indent=2))
    document, binary = repair(document, binary)
    after = report(document)
    if not args.apply:
        print(json.dumps({"would_write": True, "after": after,
                          "instruction": "Run again with --apply to create the backup and repair the production GLB."}, indent=2))
        return 0
    if prior["already_repaired"]:
        print("Production GLB already has both inward face copies; no write made.")
        return 0
    if BACKUP.exists():
        raise FileExistsError(f"Refusing to overwrite backup: {BACKUP}")
    shutil.copy2(TARGET, BACKUP)
    TARGET.write_bytes(build_glb(document, binary))
    reparsed, _ = parse_glb(TARGET)
    if not report(reparsed)["already_repaired"]:
        raise RuntimeError("Post-write validation failed; restore from backup before proceeding")
    print(json.dumps({"backup": str(BACKUP), "sha256_after": hashlib.sha256(TARGET.read_bytes()).hexdigest(),
                      "after": report(reparsed)}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
