"""Static contracts at the proven-corridor -> coverage boundary."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from corridor_altitude import AltitudeController
from experimental_corridor_manager import (
    Telemetry, coverage_entry_registered, north_velocity_in_body, field_advance_health)
from coverage_mission.config import Config
from coverage_mission.engine import Engine
from coverage_mission.geometry import Pose
from coverage_mission.planning import CoveragePlan
from coverage_mission.planning import GroundMap, route


ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config/full_mission_coverage.json"


def test_field_advance_command_and_progress_use_same_north_axis():
    for yaw in (0., math.pi/2, -math.pi/2, math.pi, .3):
        forward, right = north_velocity_in_body(.15, yaw)
        north = forward * math.cos(yaw) - right * math.sin(yaw)
        east = forward * math.sin(yaw) + right * math.cos(yaw)
        assert abs(north - .15) < 1e-12
        assert abs(east) < 1e-12


def test_field_advance_health_checks_both_axes_and_heading():
    cfg = Config.load(CONFIG)
    pose = Telemetry(x_m=-19.4, y_m=-4., z_m=-2., vx_m_s=0., vy_m_s=0.,
                     yaw_rad=0., position_time=10., attitude_time=10.)
    assert field_advance_health(pose, cfg, 10.2)
    assert not field_advance_health(pose, cfg, 10.6)
    pose.y_m = 100.
    assert not field_advance_health(pose, cfg, 10.2)
    pose.y_m = -4.
    pose.vy_m_s = math.nan
    assert not field_advance_health(pose, cfg, 10.2)


def test_exit_guard_accepts_registered_pose_and_rejects_stale_or_wrong_origin():
    cfg = Config.load(CONFIG)
    pose = Telemetry(x_m=-19.4, y_m=-4.0, position_time=10.0)
    assert coverage_entry_registered(pose, cfg, 10.2)
    assert not coverage_entry_registered(pose, cfg, 11.1)
    pose.x_m = -30.0
    assert not coverage_entry_registered(pose, cfg, 10.2)
    pose.x_m = math.nan
    assert not coverage_entry_registered(pose, cfg, 10.2)


def test_coverage_climb_commands_upward_and_waits_for_settled_altitude():
    climb = AltitudeController(target=10.0, max_speed=.5, tolerance=.15,
                               dwell=1.0, timeout=90.0, telemetry_max_age=1.5, gain=.8)
    climb.start(10.0)
    first = climb.update(10.0, -1.3, 0.0, 10.0, mission_time=2.0)
    assert first.vz_down == -.5 and not first.ready
    for t in (11.0, 11.6, 12.1):
        result = climb.update(t, -10.0, 0.0, t, mission_time=t-8.0)
    assert result.ready


def test_full_world_progress_budget_allows_long_checked_connector_but_remains_finite():
    cfg = Config.load(CONFIG)
    assert cfg.no_progress == 120.0
    engine = Engine(cfg)
    assert engine.replan_period > 5.0
    engine.ground.observed[:] = True
    engine.ground.refresh()
    for t in (0.0, 80.0, 120.1):
        engine.ground.last_t = t
        decision = engine.step(Pose(t, -10.0, 10.0, cfg.altitude), camera_t=t)
        if t < cfg.no_progress:
            assert decision.state != "BLOCKED"
        else:
            assert decision.state == "BLOCKED"


def test_full_world_near_pass_is_creditable_without_changing_red_clearance():
    cfg = Config.load(CONFIG)
    assert cfg.arrival == cfg.uncertainty == 0.30
    ground = GroundMap(cfg)
    ground.observed[:] = True
    ground.refresh()
    plan = CoveragePlan(cfg)
    plan.done[:] = True
    plan.done[-1] = False
    target = plan.points[-1]
    plan.update(Pose(1, target[0]+0.28, target[1], cfg.altitude), ground)
    assert plan.done[-1]
    assert cfg.body_radius == 0.40
    assert cfg.clearance == 0.70


def test_reachable_coverage_precedes_unknown_ordered_point():
    cfg = Config.load(CONFIG)
    ground = GroundMap(cfg)
    plan = CoveragePlan(cfg)
    ground.observed[:] = True
    first, reachable = 0, len(plan.points)-1
    # A single unreadable pixel is now an explicitly bounded interior
    # print/texture hole; use a genuinely unknown patch for this contract.
    cell=plan.cells[first]
    ground.observed[cell[0]-4:cell[0]+5,cell[1]-4:cell[1]+5]=False
    ground.refresh()
    plan.done[:] = True
    plan.done[[first, reachable]] = False
    current = plan.points[reachable] + [0.1, 0.1]
    selected = plan.reachable_target(ground, current)
    assert selected is not None
    assert tuple(selected) == tuple(plan.points[reachable])
    # The first obligation is retained for later observations, not credited.
    assert first in plan.pending()


def test_unknown_ordered_frontier_does_not_displace_known_reachable_work():
    cfg = Config.load(CONFIG)
    ground = GroundMap(cfg)
    plan = CoveragePlan(cfg)
    ground.observed[:] = True
    first, later = 0, len(plan.points)-1
    cell=plan.cells[first]
    ground.observed[cell[0]-4:cell[0]+5,cell[1]-4:cell[1]+5]=False
    ground.refresh()
    plan.done[:] = True
    plan.done[[first, later]] = False
    current = plan.points[first] + [1.0, 1.0]
    assert tuple(plan.reachable_target(ground, current)) == tuple(plan.points[later])
    assert first in plan.pending()


def test_unknown_distant_row_uses_local_observable_frontier():
    cfg = Config.load(CONFIG)
    ground = GroundMap(cfg)
    ground.observed[20:380, 20:80] = True
    ground.refresh()
    current = (15.0, -10.0)
    distant_unknown = (-19.0, -10.0)
    path = route(ground, current, distant_unknown)
    assert path
    assert math.dist(current, path[-1]) < 5.0


def test_only_confirmed_red_enclosure_exempts_unreachable_ground():
    cfg = Config.load(CONFIG)
    ground = GroundMap(cfg)
    ground.observed[:] = True
    ground.observed[184:256, 114:186] = False
    ring = ground.observed.copy()
    ring[:] = False
    ring[180:260, 110:190] = True
    ring[184:256, 114:186] = False
    ground.red[:] = ring
    ground.refresh()
    assert not ground.enclosed[210, 150]  # One-frame colour is not proof.

    ground.confirmed[:] = ring
    ground.refresh()
    assert ground.enclosed[210, 150]
    assert not ((~ground.observed & ~ground.red & ~ground.enclosed).any())
    plan = CoveragePlan(cfg)
    plan.done[:] = True
    trapped = next(i for i, point in enumerate(plan.points)
                   if ground.enclosed[ground.cell(point)])
    outside = len(plan.points)-1
    plan.done[[trapped, outside]] = False
    plan.update(Pose(1, -10, -10, cfg.altitude), ground)
    assert trapped not in plan.pending()
    assert outside in plan.pending()  # A clear reachable miss still matters.
    assert not plan.done[trapped]  # Exemption is not traversal credit.

    engine = Engine(cfg)
    engine.ground = ground
    engine.plan.done[:] = True
    engine.plan.done[trapped] = False
    ground.last_t = 1.0
    engine.last_progress_t = -130.0
    engine.progress_count = int(engine.plan.done.sum()+ground.observed.sum())
    decision = engine.step(Pose(1, -10, -10, cfg.altitude), camera_t=1.0)
    assert decision.state == "SETTLING"
    assert decision.pending == decision.unseen == 0

    ground.confirmed[180:184, 150] = False  # Raw gap is narrower than clearance.
    ground.red[180:184, 150] = False
    ground.refresh()
    assert ground.enclosed[210, 150]
    ground.confirmed[180:184, 138:163] = False  # Open a clearance-sized corridor.
    ground.red[180:184, 138:163] = False
    ground.refresh()
    assert not ground.enclosed[210, 150]


def test_full_world_camera_field_and_red_zones_share_one_geometry():
    cfg = Config.load(CONFIG)
    assert (cfg.n_max-cfg.n_min, cfg.e_max-cfg.e_min) == (40, 30)
    assert len(set(CoveragePlan(cfg).lanes)) > 1
    world = ET.parse(ROOT / "world/worlds/miss2_full_world.sdf").getroot()
    assert world.find("world").attrib["name"] == "miss2_world"
    drone = next(i for i in world.findall("world/include")
                 if i.findtext("uri") == "model://iris_miss2_full")
    x0, y0, *_ = map(float, drone.findtext("pose").split())
    assert (x0, y0) == (0.0, -36.5)
    for name in ("coverage_red_zone_1", "coverage_red_zone_2"):
        red = world.find(f"world/model[@name='{name}']")
        e, world_y, *_ = map(float, red.findtext("pose").split())
        n = world_y  # Verified against LOCAL_POSITION_NED in the full SITL world.
        assert cfg.n_min+cfg.clearance < n < cfg.n_max-cfg.clearance
        assert cfg.e_min+cfg.clearance < e < cfg.e_max-cfg.clearance
    model = ET.parse(ROOT / "world/models/models/iris_miss2_full/model.sdf").getroot()
    camera = model.find(".//sensor[@name='camera_downward']")
    assert math.isclose(float(camera.findtext("camera/horizontal_fov")), cfg.camera.hfov)
    assert camera.findtext("topic") == cfg.downward_topic
    assert cfg.clock_topic == "/world/miss2_world/clock"
