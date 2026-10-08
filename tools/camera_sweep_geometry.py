#!/usr/bin/env python3
"""Apply the 20 m camera sweep geometry update to coverage_mission_pipeline.

Changes:
- Inset every exported partition by 9 m before clipping it to the global
  operational route space. This creates an 18 m nominal gap between route
  centerlines on opposite sides of a shared partition boundary.
- Keep coverage-contract validation against the original, un-inset partitions.
- Configure the Stage 21 camera footprint and overlap for exactly 18 m nominal
  spacing between parallel sweeps.
- Add focused regression tests.

Run from anywhere:
    python3 tools/camera_sweep_geometry.py ~/coverage_ws/src/coverage_mission_pipeline
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def replace_once(text: str, old: str, new: str, path: Path, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise RuntimeError(
            f"{path}: expected exactly one occurrence for {label!r}, found {count}"
        )
    return text.replace(old, new, 1)


def replace_count(
    text: str,
    old: str,
    new: str,
    expected: int,
    path: Path,
    label: str,
) -> str:
    count = text.count(old)
    if count != expected:
        raise RuntimeError(
            f"{path}: expected {expected} occurrences for {label!r}, found {count}"
        )
    return text.replace(old, new)


def patch_adapter(text: str, path: Path) -> str:
    text = replace_once(
        text,
        'SWARM_PARTITIONS_ADAPTER_ALGORITHM = "swarm_partitions_json_adapter_v2"',
        'SWARM_PARTITIONS_ADAPTER_ALGORITHM = "swarm_partitions_json_adapter_v3"',
        path,
        "adapter algorithm version",
    )

    text = replace_once(
        text,
        '''    clearance_m: float = 0.0
    tracking_margin_m: float = 0.0
    min_component_area_m2: float = 0.0
''',
        '''    clearance_m: float = 0.0
    tracking_margin_m: float = 0.0
    partition_inset_m: float = 0.0
    min_component_area_m2: float = 0.0
''',
        path,
        "adapter partition inset field",
    )

    text = replace_once(
        text,
        '''            "clearance_m",
            "tracking_margin_m",
            "min_component_area_m2",
''',
        '''            "clearance_m",
            "tracking_margin_m",
            "partition_inset_m",
            "min_component_area_m2",
''',
        path,
        "adapter scalar validation",
    )

    text = replace_once(
        text,
        '''    route_space_projected: BaseGeometry
    route_space_local: BaseGeometry
    tracking_margin_m: float
    component_ids_by_partition_id: Mapping[int, tuple[str, ...]]
''',
        '''    route_space_projected: BaseGeometry
    route_space_local: BaseGeometry
    tracking_margin_m: float
    partition_inset_m: float
    component_ids_by_partition_id: Mapping[int, tuple[str, ...]]
''',
        path,
        "adapter result partition inset field",
    )

    text = replace_once(
        text,
        '''        object.__setattr__(
            self,
            "tracking_margin_m",
            _finite_nonnegative(self.tracking_margin_m, "tracking_margin_m"),
        )
        if not self.safe_area_projected.buffer(
''',
        '''        object.__setattr__(
            self,
            "tracking_margin_m",
            _finite_nonnegative(self.tracking_margin_m, "tracking_margin_m"),
        )
        object.__setattr__(
            self,
            "partition_inset_m",
            _finite_nonnegative(self.partition_inset_m, "partition_inset_m"),
        )
        if not self.safe_area_projected.buffer(
''',
        path,
        "adapter result inset validation",
    )

    text = replace_once(
        text,
        '''            "route_space_m2": float(self.route_space_projected.area),
            "tracking_margin_m": self.tracking_margin_m,
            "frame": self.frame.to_dict(),
''',
        '''            "route_space_m2": float(self.route_space_projected.area),
            "tracking_margin_m": self.tracking_margin_m,
            "partition_inset_m": self.partition_inset_m,
            "frame": self.frame.to_dict(),
''',
        path,
        "adapter summary inset",
    )

    text = replace_once(
        text,
        '''    clipped_projected_components: list[Polygon] = []
    local_component_geometries: list[BaseGeometry] = []

    for partition_id in sorted(partitions_projected):
        projected_components = clip_partition_to_safe_area(
            partitions_projected[partition_id],
            route_space_projected,
            min_component_area_m2=config.min_component_area_m2,
        )
''',
        '''    coverage_validation_components: list[Polygon] = []
    local_component_geometries: list[BaseGeometry] = []

    for partition_id in sorted(partitions_projected):
        original_partition = partitions_projected[partition_id]

        # Validate the exporter contract using the original partition geometry.
        # The deliberate route-centerline inset must not be mistaken for an
        # uncovered source-partition gap.
        validation_components = clip_partition_to_safe_area(
            original_partition,
            route_space_projected,
            min_component_area_m2=0.0,
        )
        coverage_validation_components.extend(validation_components)

        # Apply the camera-derived inset to the partition before intersecting it
        # with the global route space. At shared borders, neighbouring partitions
        # each retreat by this amount. At the outer boundary and exclusions, the
        # final inset is the stricter of this value and the existing global
        # clearance/tracking geometry; the values are not added together.
        planning_partition = original_partition
        if config.partition_inset_m > 0.0:
            planning_partition = planning_partition.buffer(
                -config.partition_inset_m,
                resolution=16,
                join_style=2,
                mitre_limit=5.0,
            )
            if planning_partition.is_empty:
                raise SwarmPartitionsAdapterError(
                    f"partition {partition_id} is empty after applying "
                    f"{config.partition_inset_m:.3f} m partition inset"
                )
            if not planning_partition.is_valid:
                raise SwarmPartitionsAdapterError(
                    f"partition {partition_id} became invalid after partition "
                    f"inset: {explain_validity(planning_partition)}"
                )

        projected_components = clip_partition_to_safe_area(
            planning_partition,
            route_space_projected,
            min_component_area_m2=config.min_component_area_m2,
        )
''',
        path,
        "partition planning inset",
    )

    text = replace_once(
        text,
        '''        clipped_projected_components.extend(projected_components)
        local_components = [
''',
        '''        local_components = [
''',
        path,
        "remove planned components from source coverage validation",
    )

    text = replace_once(
        text,
        '''                f"partition {partition_id} has no plannable component after "
                "clearance and tracking margin"
''',
        '''                f"partition {partition_id} has no plannable component after "
                "clearance, tracking margin and partition inset"
''',
        path,
        "inset failure message",
    )

    text = replace_once(
        text,
        '''    covered = unary_union(clipped_projected_components)
''',
        '''    covered = unary_union(coverage_validation_components)
''',
        path,
        "source coverage validation geometry",
    )

    text = replace_once(
        text,
        '''        tracking_margin_m=config.tracking_margin_m,
        component_ids_by_partition_id=component_ids_by_partition,
''',
        '''        tracking_margin_m=config.tracking_margin_m,
        partition_inset_m=config.partition_inset_m,
        component_ids_by_partition_id=component_ids_by_partition,
''',
        path,
        "adapter result inset value",
    )
    return text


def patch_config_parser(text: str, path: Path) -> str:
    text = replace_once(
        text,
        '''def _strict_keys(value: Mapping[str, Any], required: set[str], path: str) -> None:
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - required)
''',
        '''def _strict_keys(
    value: Mapping[str, Any],
    required: set[str],
    path: str,
    *,
    optional: set[str] | None = None,
) -> None:
    allowed = required | (optional or set())
    actual = set(value.keys())
    missing = sorted(required - actual)
    unknown = sorted(actual - allowed)
''',
        path,
        "optional strict config keys",
    )

    text = replace_once(
        text,
        '''            "partition_overlap_tolerance_m2",
        },
        "adapter",
    )
''',
        '''            "partition_overlap_tolerance_m2",
        },
        "adapter",
        optional={"partition_inset_m"},
    )
''',
        path,
        "optional partition inset config key",
    )

    text = replace_once(
        text,
        '''            clearance_m=adapter["clearance_m"],
            tracking_margin_m=adapter["tracking_margin_m"],
            min_component_area_m2=adapter["min_component_area_m2"],
''',
        '''            clearance_m=adapter["clearance_m"],
            tracking_margin_m=adapter["tracking_margin_m"],
            partition_inset_m=adapter.get("partition_inset_m", 0.0),
            min_component_area_m2=adapter["min_component_area_m2"],
''',
        path,
        "construct adapter partition inset",
    )

    text = replace_once(
        text,
        '''                "clearance_m": self.adapter.clearance_m,
                "tracking_margin_m": self.adapter.tracking_margin_m,
                "min_component_area_m2": self.adapter.min_component_area_m2,
''',
        '''                "clearance_m": self.adapter.clearance_m,
                "tracking_margin_m": self.adapter.tracking_margin_m,
                "partition_inset_m": self.adapter.partition_inset_m,
                "min_component_area_m2": self.adapter.min_component_area_m2,
''',
        path,
        "serialize adapter partition inset",
    )
    return text


def patch_demo_config(text: str, path: Path) -> str:
    text = replace_once(
        text,
        '''  clearance_m: 10.0
  tracking_margin_m: 2.0
  min_component_area_m2: 250.0
''',
        '''  clearance_m: 10.0
  tracking_margin_m: 2.0
  partition_inset_m: 9.0
  min_component_area_m2: 250.0
''',
        path,
        "Stage 21 partition inset",
    )
    text = replace_count(
        text,
        "    lateral_footprint_m: 14.9553871794\n",
        "    lateral_footprint_m: 25.9763037279\n",
        5,
        path,
        "Stage 21 camera footprint",
    )
    text = replace_count(
        text,
        "    lateral_overlap: 0.2\n",
        "    lateral_overlap: 0.3070607663\n",
        5,
        path,
        "Stage 21 exact 18 m sweep spacing",
    )
    return text


def patch_example_config(text: str, path: Path) -> str:
    return replace_once(
        text,
        '''  clearance_m: 5.0
  tracking_margin_m: 2.0
  min_component_area_m2: 0.0
''',
        '''  clearance_m: 5.0
  tracking_margin_m: 2.0
  partition_inset_m: 0.0
  min_component_area_m2: 0.0
''',
        path,
        "example partition inset",
    )


def patch_adapter_tests(text: str, path: Path) -> str:
    anchor = '''def test_components_and_connectors_use_route_space_not_authoritative_safe_area():
'''
    test = '''def test_partition_inset_creates_18_m_shared_gap_without_adding_to_outer_clearance():
    result = adapt_swarm_partitions_payload(
        _payload(),
        _config(
            clearance_m=10.0,
            tracking_margin_m=2.0,
            partition_inset_m=9.0,
        ),
    )
    left, right = result.definition.components
    assert left.polygon.bounds == pytest.approx(
        (12.0, 12.0, 491.0, 988.0),
        abs=1.0e-4,
    )
    assert right.polygon.bounds == pytest.approx(
        (509.0, 12.0, 988.0, 988.0),
        abs=1.0e-4,
    )
    assert right.polygon.bounds[0] - left.polygon.bounds[2] == pytest.approx(
        18.0,
        abs=1.0e-4,
    )
    assert result.partition_inset_m == 9.0
    assert result.route_space_projected.bounds == pytest.approx(
        (300012.0, 3200012.0, 300988.0, 3200988.0)
    )


def test_negative_partition_inset_is_rejected():
    with pytest.raises(SwarmPartitionsAdapterError, match="partition_inset_m"):
        _config(partition_inset_m=-1.0)


'''
    return replace_once(
        text,
        anchor,
        test + anchor,
        path,
        "partition inset regression tests",
    )


def patch_config_tests(text: str, path: Path) -> str:
    text = replace_once(
        text,
        '''            "clearance_m": 5.0,
            "tracking_margin_m": 2.0,
            "min_component_area_m2": 0.0,
''',
        '''            "clearance_m": 5.0,
            "tracking_margin_m": 2.0,
            "partition_inset_m": 9.0,
            "min_component_area_m2": 0.0,
''',
        path,
        "valid config partition inset",
    )
    text = replace_once(
        text,
        '''        assert config.clearance_m == 5.0
        assert config.tracking_margin_m == 2.0
        assert config.min_component_area_m2 == 0.0
''',
        '''        assert config.clearance_m == 5.0
        assert config.tracking_margin_m == 2.0
        assert config.partition_inset_m == 9.0
        assert config.min_component_area_m2 == 0.0
''',
        path,
        "config inset scalar assertion",
    )
    anchor = '''@pytest.mark.parametrize("value", [-1.0, float("nan"), True, "2"])
def test_invalid_tracking_margin_rejected(value):
'''
    addition = '''@pytest.mark.parametrize("value", [-1.0, float("nan"), True, "2"])
def test_invalid_partition_inset_rejected(value):
    payload = valid_payload()
    payload["adapter"]["partition_inset_m"] = value
    with pytest.raises(SwarmMissionConfigError, match="partition_inset_m"):
        parse(payload)


def test_partition_inset_is_optional_for_existing_schema_v2_files():
    payload = valid_payload()
    del payload["adapter"]["partition_inset_m"]
    assert parse(payload).adapter.partition_inset_m == 0.0


'''
    return replace_once(
        text,
        anchor,
        addition + anchor,
        path,
        "config partition inset validation tests",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "repository",
        nargs="?",
        default=".",
        help="Path to the coverage_mission_pipeline repository",
    )
    args = parser.parse_args()
    root = Path(args.repository).expanduser().resolve()

    required = {
        root / "coverage_mission_pipeline/swarm_partitions_adapter.py": patch_adapter,
        root / "coverage_mission_pipeline/swarm_mission_config.py": patch_config_parser,
        root / "demo/noida_stage21/input/swarm_mission.yaml": patch_demo_config,
        root / "config/swarm_mission.example.yaml": patch_example_config,
        root / "test/test_swarm_partitions_adapter.py": patch_adapter_tests,
        root / "test/test_swarm_mission_config.py": patch_config_tests,
    }

    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise RuntimeError(
            "Repository layout does not match the expected main branch:\n  "
            + "\n  ".join(missing)
        )

    updated: dict[Path, str] = {}
    for path, transform in required.items():
        original = path.read_text(encoding="utf-8")
        changed = transform(original, path)
        if changed == original:
            raise RuntimeError(f"{path}: no change produced")
        updated[path] = changed

    for path, changed in updated.items():
        path.write_text(changed, encoding="utf-8")
        print(f"updated {path.relative_to(root)}")

    print()
    print("Configured Stage 21 values:")
    print("  partition inset:       9.0 m")
    print("  camera footprint:      25.9763037279 m")
    print("  lateral overlap:       0.3070607663")
    print("  nominal sweep spacing: 18.0 m")
    print()
    print("Run:")
    print(
        "  python3 -m pytest -q "
        "test/test_swarm_partitions_adapter.py "
        "test/test_swarm_mission_config.py"
    )
    print("  git diff --check")
    print("  git diff")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
