#!/usr/bin/env python3
"""Read-only mission diagnosis; modified meshes exist only in generated artifacts.

Compare an unchanged mesh with a copy whose corridor cover triangle winding is
reversed. Keep pose, material, scanner, and native geometry settings identical.
This is a static sensor experiment, not a fix to the mission or a flight test.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

os.environ.setdefault('PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION', 'python')
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'experiments/corridor_only'))
from replay import geometry_report
from native.common.types import NativeScan
from gz.transport13 import Node
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from google.protobuf.json_format import MessageToDict


def prepare(variant, directory, bare=False, no_camera=False, no_joint=False):
    original = ROOT / 'models/models/miss2_env/miss2.glb'
    data = bytearray(original.read_bytes())
    json_size, _ = struct.unpack_from('<II', data, 12)
    document = json.loads(data[20:20 + json_size])
    binary_start = 20 + json_size + 8
    changed = []
    if variant in ('inward', 'inward_normals'):
        flipped_indices = set()
        flipped_normals = set()
        for node in document['nodes']:
            if not node.get('name', '').startswith('corridor cover'):
                continue
            for primitive in document['meshes'][node['mesh']]['primitives']:
                accessor = document['accessors'][primitive['indices']]
                view = document['bufferViews'][accessor['bufferView']]
                offset = binary_start + view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
                dtype = {5121: 'u1', 5123: '<u2', 5125: '<u4'}[accessor['componentType']]
                indices = np.frombuffer(data, dtype=dtype, count=accessor['count'], offset=offset).reshape(-1, 3)
                # Both corridor meshes share the same index accessor. Flip it
                # once; flipping once per node silently cancels the experiment.
                if primitive['indices'] not in flipped_indices:
                    indices[:, [1, 2]] = indices[:, [2, 1]]
                    flipped_indices.add(primitive['indices'])
                if variant == 'inward_normals' and primitive['attributes']['NORMAL'] not in flipped_normals:
                    a = document['accessors'][primitive['attributes']['NORMAL']]
                    v = document['bufferViews'][a['bufferView']]
                    normals = np.ndarray((a['count'], 3), dtype='<f4', buffer=data,
                        offset=binary_start + v.get('byteOffset', 0) + a.get('byteOffset', 0),
                        strides=(v.get('byteStride', 12), 4))
                    normals *= -1
                    flipped_normals.add(primitive['attributes']['NORMAL'])
                changed.append(node['name'])
    mesh_path = directory / 'environment.glb'
    if variant == 'replace_cover':
        removed = {i for i, node in enumerate(document['nodes'])
                   if node.get('name', '').startswith('corridor cover')}
        for scene in document['scenes']:
            scene['nodes'] = [i for i in scene['nodes'] if i not in removed]
        encoded = json.dumps(document, separators=(',', ':')).encode()
        encoded += b' ' * (-len(encoded) % 4)
        tail = data[20 + json_size:]
        data = bytearray(struct.pack('<III', 0x46546c67, 2, 20 + len(encoded) + len(tail))
                         + struct.pack('<II', len(encoded), 0x4e4f534a) + encoded + tail)
        changed = ['cover nodes removed from scene; visual boxes substituted']
    mesh_path.write_bytes(data)

    # Mesh bounds establish corridor X=[-5.87,-2.12], Y=[-30.109,-20.109].
    # XY/yaw inferred from saved NED telemetry and the configured ENU/NED
    # transform; Z is a controlled test height, not asserted Gazebo ground truth.
    capture = json.loads((ROOT / 'integration/artifacts/preentry_capture/preentry_geometry.json').read_text())
    pose = capture['local_position_ned']
    height = 1.3 if variant == 'low_control' else 2.61
    probe_pose = [pose['y'], pose['x'], height, 0, 0, math.pi / 2 - pose['yaw_rad']]
    sdf = ET.Element('sdf', version='1.9')
    world = ET.SubElement(sdf, 'world', name='winding_diagnosis')
    for filename, name in [('physics', 'Physics'), ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    environment = ET.SubElement(world, 'model', name='environment')
    ET.SubElement(environment, 'static').text = 'true'
    link = ET.SubElement(environment, 'link', name='link')
    ET.SubElement(link, 'pose').text = '0 0 0 1.5707963267948966 0 0'
    for kind in ('visual', 'collision'):
        element = ET.SubElement(link, kind, name=kind)
        mesh = ET.SubElement(ET.SubElement(element, 'geometry'), 'mesh')
        ET.SubElement(mesh, 'uri').text = str(mesh_path)
    if variant in ('boxes_only', 'boxes_collision'):
        world.remove(environment)
    if variant in ('box_control', 'low_control', 'replace_cover', 'boxes_only', 'boxes_collision'):
        # Diagnostic visual surfaces only: preserve the mesh collision geometry.
        for index, x in enumerate((-5.91963272, -2.06963272)):
            wall = ET.SubElement(world, 'model', name=f'wall_control_{index}')
            ET.SubElement(wall, 'static').text = 'true'
            ET.SubElement(wall, 'pose').text = f'{x} -25.10943413 1.52261853 0 0 0'
            wall_link = ET.SubElement(wall, 'link', name='link')
            visual = ET.SubElement(wall_link, 'visual', name='visual')
            box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = '.1 10 3.05476284'
            material = ET.SubElement(visual, 'material')
            ET.SubElement(material, 'ambient').text = '.7 .7 .7 1'
            ET.SubElement(material, 'diffuse').text = '.7 .7 .7 1'
            if variant == 'boxes_collision':
                collision = ET.SubElement(wall_link, 'collision', name='collision')
                collision.append(copy.deepcopy(visual.find('geometry')))
    # Use a unique topic as well as partition to verify publisher provenance.
    probe = ET.parse(Path(__file__).parent / 'models/lidar_probe_bottom/model.sdf').getroot().find('model')
    if bare:
        probe.remove(probe.find('include'))
        probe.remove(probe.find('joint'))
    elif no_camera:
        # Preserve the complete vehicle and its joints/visuals, removing only
        # the camera sensor inherited from the full-world base model.
        body = ET.parse(ROOT / 'models/models/iris_with_standoffs/model.sdf').getroot().find('model')
        for body_link in body.findall('link'):
            for sensor in body_link.findall('sensor'):
                if sensor.get('type') == 'camera':
                    body_link.remove(sensor)
        probe.remove(probe.find('include'))
        probe.append(body)
    if no_joint and probe.find('joint') is not None:
        probe.remove(probe.find('joint'))
    ET.SubElement(probe, 'static').text = 'true'
    ET.SubElement(probe, 'pose').text = ' '.join(map(str, probe_pose))
    topic = f'/winding_diagnosis_{os.getpid()}/scan'
    probe.find('link/sensor/topic').text = topic
    world.append(probe)
    world_path = directory / 'world.sdf'
    ET.indent(sdf)
    ET.ElementTree(sdf).write(world_path, encoding='unicode', xml_declaration=True)
    return world_path, {'variant': variant, 'changed_meshes': changed, 'probe_pose': probe_pose,
                        'lidar_z_world': height - .25, 'pose_note': 'XY/yaw inferred; Z controlled', 'topic': topic, 'bare_sensor': bare, 'no_camera': no_camera, 'no_joint': no_joint}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('variant', choices=('original', 'inward', 'inward_normals', 'box_control', 'low_control', 'replace_cover', 'boxes_only', 'boxes_collision'))
    parser.add_argument('--bare', action='store_true')
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--no-joint', action='store_true')
    args = parser.parse_args()
    directory = Path(__file__).parent / 'artifacts/winding_diagnosis' / (args.variant + ('_bare' if args.bare else '') + ('_no_camera' if args.no_camera else '') + ('_no_joint' if args.no_joint else ''))
    directory.mkdir(parents=True, exist_ok=True)
    world, details = prepare(args.variant, directory, args.bare, args.no_camera, args.no_joint)
    os.environ['GZ_PARTITION'] = f'sae_winding_{args.variant}_{os.getpid()}'
    state = {'count': 0, 'scan': None}
    lock = threading.Lock()

    def receive(msg):
        ranges = np.asarray(msg.ranges, dtype=float)
        if ranges.size != 500 or msg.vertical_count > 1:
            return
        scan = NativeScan(msg.angle_min + np.arange(ranges.size) * msg.angle_step,
                          ranges, np.zeros_like(ranges), time.monotonic(), msg.range_min, msg.range_max)
        with lock:
            state['scan'] = scan
            state['count'] += 1

    node = Node()
    assert node.subscribe(LaserScan, details['topic'], receive)
    with (directory / 'gazebo.log').open('w') as log:
        process = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', str(world)],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                with lock:
                    scan, count = state['scan'], state['count']
                if count >= 15 and scan.age_s < .5:
                    break
                if process.poll() is not None:
                    raise RuntimeError(f'Gazebo exited: {process.returncode}; see {log.name}')
                time.sleep(.1)
            else:
                raise RuntimeError('Timed out waiting for 15 fresh scans')
            report, _ = geometry_report(scan)
            ok, scene = node.request('/world/winding_diagnosis/scene/info', Empty(), Empty, Scene, 3000)
            if ok:
                (directory / 'scene.json').write_text(json.dumps(MessageToDict(scene), indent=2))
            details.update(report)
            details['scans_received'] = count
            np.savez(directory / 'scan.npz', angles_rad=scan.angles_rad, ranges_m=scan.ranges_m,
                     range_min_m=scan.range_min_m, range_max_m=scan.range_max_m)
            (directory / 'report.json').write_text(json.dumps(details, indent=2))
            print(json.dumps(details, indent=2), flush=True)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()


if __name__ == '__main__':
    main()
