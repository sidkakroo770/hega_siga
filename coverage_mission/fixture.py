"""Generate isolated Gazebo assets from the copied, working Iris physics model.

Outputs are generated test artifacts, never edits to source mission models.
Gazebo world ENU: X=east, Y=north. Spawn yaw pi/2 points body X north.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from .config import Config

ROOT=Path(__file__).resolve().parents[1]


def element(parent,tag,text=None,**attrs):
    e=ET.SubElement(parent,tag,attrs)
    if text is not None: e.text=str(text)
    return e


def camera(model,name,pose,topic,fov,width,height):
    link=element(model,'link',name=name+'_link')
    element(link,'pose',pose)
    inertial=element(link,'inertial'); element(inertial,'mass',.001)
    inertia=element(inertial,'inertia')
    for k in ('ixx','iyy','izz'): element(inertia,k,.000001)
    sensor=element(link,'sensor',name=name,type='camera')
    element(sensor,'always_on','true'); element(sensor,'update_rate',15)
    element(sensor,'topic',topic)
    c=element(sensor,'camera'); element(c,'horizontal_fov',fov)
    im=element(c,'image'); element(im,'width',width); element(im,'height',height); element(im,'format','R8G8B8')
    clip=element(c,'clip'); element(clip,'near',.05); element(clip,'far',100)
    j=element(model,'joint',name=name+'_joint',type='fixed')
    element(j,'parent','iris_with_standoffs::base_link'); element(j,'child',name+'_link')


def box(world,name,n0,n1,e0,e1,color,z=0,collision=False):
    model=element(world,'model',name=name); element(model,'static','true')
    element(model,'pose',f'{(e0+e1)/2} {(n0+n1)/2} {z-.01} 0 0 0')
    link=element(model,'link',name='link')
    for kind in (('collision','visual') if collision else ('visual',)):
        item=element(link,kind,name=kind); geometry=element(item,'geometry')
        b=element(geometry,'box'); element(b,'size',f'{e1-e0} {n1-n0} .02')
        if kind=='visual':
            mat=element(item,'material'); element(mat,'ambient',color); element(mat,'diffuse',color)


def build(config,output,scenario='central',spawn_yaw=0,real_time_factor=1,instance=0):
    cfg=Config.load(config)
    c=cfg.camera
    nominal_fx=c.width/(2*math.tan(c.hfov/2))
    if (any(c.distortion) or any((c.mount_yaw,c.mount_pitch,c.mount_roll,c.down_offset)) or
            not math.isclose(c.fx,nominal_fx) or not math.isclose(c.fy,nominal_fx) or
            c.cx!=c.width/2 or c.cy!=c.height/2):
        raise ValueError('This fixture supports centred ideal nadir cameras only; do not silently simulate a different calibration')
    source=ROOT/'world/experiments/corridor_only/models/iris_corridor_test/model.sdf'
    modeltree=ET.parse(source); model=modeltree.getroot().find('model'); model.set('name','iris_coverage')
    model.find("plugin/fdm_port_in").text=str(9002+10*instance)
    for tag in ('link','joint'):
        for item in list(model.findall(tag)):
            if item.get('name','').startswith('lidar'): model.remove(item)
    camera(model,'down','0 0 0 0 1.5707963267948966 0',cfg.downward_topic,cfg.camera.hfov,cfg.camera.width,cfg.camera.height)
    camera(model,'front','0.12 0 0 0 0 0','/coverage/front/image',1.047,640,480)
    folder=output/'models/iris_coverage'; folder.mkdir(parents=True,exist_ok=True)
    ET.indent(modeltree)
    modeltree.write(folder/'model.sdf',encoding='unicode',xml_declaration=True)
    configroot=ET.Element('model'); element(configroot,'name','iris_coverage'); element(configroot,'version','1')
    sdf=element(configroot,'sdf','model.sdf',version='1.9')
    ET.ElementTree(configroot).write(folder/'model.config',encoding='unicode')
    root=ET.Element('sdf',version='1.9'); world=element(root,'world',name='coverage_test')
    physics=element(world,'physics',name='physics',type='ignored')
    element(physics,'max_step_size',.001); element(physics,'real_time_factor',real_time_factor)
    for key in ('Physics','UserCommands','SceneBroadcaster','Imu'):
        slug={'Physics':'physics','UserCommands':'user-commands','SceneBroadcaster':'scene-broadcaster','Imu':'imu'}[key]
        element(world,'plugin',filename='gz-sim-'+slug+'-system',name='gz::sim::systems::'+key)
    sensors=element(world,'plugin',filename='gz-sim-sensors-system',name='gz::sim::systems::Sensors')
    element(sensors,'render_engine','ogre2')
    light=element(world,'light',name='sun',type='directional')
    element(light,'pose','0 0 20 0 0 0'); element(light,'diffuse','.9 .9 .9 1'); element(light,'direction','0 0 -1'); element(light,'cast_shadows','false')
    box(world,'ground',-100,100,-100,100,'.18 .25 .18 1',collision=True)
    box(world,'field',cfg.n_min,cfg.n_max,cfg.e_min,cfg.e_max,'.3 .65 .3 1',z=.003)
    zones={'clear':[], 'central':[(4,6,-.5,2)],'multiple':[(3,4.5,-.5,1),(6,8,3,4.5)],
           'incursion':[(2,3,-.6,.6)],'edge':[(3,5,6.5,7.7)],
           'partial':[(1.7,3.2,1.5,3.2),(5,6,-.6,.8)],
           'row_end':[(8,9.5,-.4,1.2)],'row_end_open':[(8,9.5,0,1.2)]}[scenario]
    for i,(n0,n1,e0,e1) in enumerate(zones): box(world,'red_'+str(i),n0,n1,e0,e1,'1 0 0 1',z=.007)
    # Bind the generated model explicitly; another fixture on SDF_PATH must not
    # silently supply its camera profile or physics port.
    include=element(world,'include'); element(include,'uri',str(folder.resolve()))
    element(include,'pose',f'0 0 .25 0 0 {1.5707963267948966-spawn_yaw}')
    tree=ET.ElementTree(root); ET.indent(tree); tree.write(output/'coverage.sdf',encoding='unicode',xml_declaration=True)
    (output/'truth.json').write_text(json.dumps({'frame':'local_north_east','zones':zones,'config':cfg.as_dict()},indent=2)+'\n')
    print(output/'coverage.sdf')


def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',type=Path,default=ROOT/'config/coverage.json')
    p.add_argument('--output',type=Path,default=ROOT/'artifacts/gazebo_fixture')
    p.add_argument('--scenario',choices=['clear','central','multiple','row_end','row_end_open','incursion','edge','partial'],default='central')
    a=p.parse_args(); build(a.config,a.output,a.scenario)


if __name__=='__main__': main()
