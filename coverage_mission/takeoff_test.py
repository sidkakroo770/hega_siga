"""Explicit standalone SITL test takeoff; never part of the coverage controller."""
import argparse
import math
import time
from pymavlink import mavutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mavlink',default='udpin:127.0.0.1:14650')
    parser.add_argument('--altitude',type=float,default=10.)
    parser.add_argument('--entry-north',type=float,default=0.)
    parser.add_argument('--entry-east',type=float,default=0.)
    args=parser.parse_args()
    if args.altitude<=0 or args.altitude>15: raise SystemExit('Test altitude must be in (0,15]')
    if not math.hypot(args.entry_north,args.entry_east)<=.5:
        raise SystemExit('Test entry offset must be within the known 0.5 m start patch')
    master=mavutil.mavlink_connection(args.mavlink,source_system=244)
    if master.wait_heartbeat(timeout=30) is None: raise SystemExit('No SITL heartbeat')
    mode=master.mode_mapping()['GUIDED']
    master.set_mode('GUIDED')
    deadline=time.monotonic()+120
    armed=False; sent=False; last_request=0; last_print=0
    climb_ready=False; last_setpoint=0.; settled=None
    offset=bool(args.entry_north or args.entry_east)
    while time.monotonic()<deadline:
        msg=master.recv_match(blocking=True,timeout=.5)
        now=time.monotonic()
        if msg is None: continue
        if msg.get_srcSystem()!=master.target_system: continue
        kind=msg.get_type()
        if climb_ready and offset and now-last_setpoint>.2:
            master.mav.set_position_target_local_ned_send(0,master.target_system,master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,3576,args.entry_north,args.entry_east,-args.altitude,
                0,0,0,0,0,0,0,0)
            last_setpoint=now
        if climb_ready and offset and kind=='LOCAL_POSITION_NED':
            ready=(math.hypot(msg.x-args.entry_north,msg.y-args.entry_east)<.1 and
                   math.hypot(msg.vx,msg.vy)<.1 and abs(msg.vz)<.1 and abs(msg.z+args.altitude)<.3)
            if not ready: settled=None
            elif settled is None: settled=msg.time_boot_ms
            elif msg.time_boot_ms-settled>=2000:
                print('[READY] Offset airborne entry established in unchanged local origin',flush=True)
                master.close(); return 0
        if kind=='HEARTBEAT':
            armed=bool(msg.base_mode&128)
            if msg.custom_mode!=mode:
                if now-last_request>2: master.set_mode('GUIDED'); last_request=now
                continue
            if not armed and now-last_request>3:
                master.arducopter_arm(); last_request=now
            if armed and not sent:
                master.mav.command_long_send(master.target_system,master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,args.altitude)
                sent=True; print('[TAKEOFF] accepted armed GUIDED; requesting climb',flush=True)
        if kind=='COMMAND_ACK': print('[ACK]',msg.command,msg.result,flush=True)
        if kind=='STATUSTEXT': print('[FC]',msg.text,flush=True)
        if kind=='GLOBAL_POSITION_INT':
            alt=msg.relative_alt*.001
            if now-last_print>2: print(f'[ALT] {alt:.2f}',flush=True); last_print=now
            if sent and abs(alt-args.altitude)<.3 and abs(msg.vz)<15:
                climb_ready=True
                if offset: continue
                print('[READY] Airborne entry established; no coverage commands sent',flush=True)
                master.close(); return 0
    master.close()
    raise SystemExit('Takeoff verification timed out; inspect SITL state')


if __name__=='__main__': main()
