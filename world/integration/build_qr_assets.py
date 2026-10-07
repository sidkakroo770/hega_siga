"""Deterministic QR visuals. Collision mesh is deliberately left untouched."""
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import qrcode
from pyzbar.pyzbar import decode, ZBarSymbol

ROOT = Path(__file__).resolve().parents[2]


def texture(payload, path):
    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_M,
                       box_size=32, border=4)
    qr.add_data(payload); qr.make(fit=True)
    image = qr.make_image(fill_color='black', back_color='white').convert('RGB')
    image.save(path)
    decoded = decode(image, symbols=[ZBarSymbol.QRCODE])
    if len(decoded) != 1 or decoded[0].data.decode('utf-8') != payload:
        raise RuntimeError('Generated QR failed decoder self-check')
    return image.size[0]


def strip_visuals():
    folder = ROOT / 'world/models/models/miss2_env'
    original = folder / 'miss2.glb'
    raw = original.read_bytes()
    length, kind = struct.unpack_from('<II', raw, 12)
    assert kind == 0x4e4f534a
    doc = json.loads(raw[20:20+length])
    removed = {i for i, n in enumerate(doc['nodes'])
               if n.get('name', '').lower().startswith('initial qr code')}
    if len(removed) != 6:
        raise RuntimeError(f'Unexpected QR mesh set: {removed}')
    for scene in doc['scenes']:
        scene['nodes'] = [i for i in scene.get('nodes', []) if i not in removed]
    for node in doc['nodes']:
        if 'children' in node:
            node['children'] = [i for i in node['children'] if i not in removed]
    encoded = json.dumps(doc, separators=(',', ':')).encode()
    encoded += b' ' * ((-len(encoded)) % 4)
    tail = raw[20+length:]
    result = struct.pack('<III', 0x46546c67, 2, 20+len(encoded)+len(tail))
    result += struct.pack('<II', len(encoded), 0x4e4f534a) + encoded + tail
    (folder / 'miss2_no_qr.glb').write_bytes(result)


def add_marker(parent, name, payload, x, y, z=.12, size=1.):
    folder = ROOT / 'world/models/models/qr_markers'
    maps = folder / 'materials/textures'; maps.mkdir(parents=True, exist_ok=True)
    path = maps / (name+'.png')
    pixels = texture(payload, path)
    model = ET.SubElement(parent, 'model', name=name)
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 0'
    link = ET.SubElement(model, 'link', name='print')
    visual = ET.SubElement(link, 'visual', name='qr')
    plane = ET.SubElement(ET.SubElement(visual, 'geometry'), 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = f'{size} {size}'
    mat = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'): ET.SubElement(mat, tag).text = '1 1 1 1'
    ET.SubElement(mat, 'specular').text = '0 0 0 1'
    ET.SubElement(mat, 'lighting').text = 'false'
    metal = ET.SubElement(ET.SubElement(mat, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = 'model://qr_markers/materials/textures/'+path.name
    ET.SubElement(metal, 'roughness').text = '1'
    ET.SubElement(metal, 'metalness').text = '0'
    return dict(name=name, payload=payload, world_enu=[x, y, z], size=size,
                texture_pixels=pixels)


def main():
    strip_visuals()
    folder = ROOT / 'world/models/models/qr_markers'; folder.mkdir(parents=True, exist_ok=True)
    root = ET.Element('sdf', version='1.9')
    # Frame-independent mission input never receives this truth manifest.
    container = ET.SubElement(root, 'model', name='qr_markers')
    ET.SubElement(container, 'static').text = 'true'
    markers = [
        ('qr_start', 'REF-001', 0., -35.5),
        ('qr_nonmatch_early', 'OTHER-001', -4., -17.),
        # 1.20 m centre clearance from red-zone south edge N=-9;
        # same reachable near-boundary placement as the targeted flight.
        ('qr_target_early', 'REF-001', -3., -10.2),
        ('qr_nonmatch_west', 'OTHER-002', -8.5, 9.6),
        ('qr_nonmatch_east', 'OTHER-003', 9.2, 8.4),
        ('qr_red_excluded', 'REF-001', 0., 2.),
    ]
    truth = [add_marker(container, *args) for args in markers]
    ET.indent(root)
    ET.ElementTree(root).write(folder/'model.sdf', encoding='unicode', xml_declaration=True)
    (folder/'model.config').write_text('<model><name>qr_markers</name><version>1</version>'
        '<sdf version="1.9">model.sdf</sdf></model>\n')
    (folder/'truth.json').write_text(json.dumps(truth, indent=2)+'\n')
    print('Verified', len(truth), 'black-and-white QR assets')


if __name__ == '__main__': main()
