import dataclasses
import math
import time
from typing import List

import numpy as np
from navigation.local_planner import LocalPlanner

from carlaSetupUtils import set_weather, set_sync, setup_actors, delete_actors
import carla
from carla import Location, Vehicle, Rotation

import re


@dataclasses.dataclass
class VehicleStat:
    location: Location
    dist_travelled: float
    wheel_speeds: List[float]


@dataclasses.dataclass
class TelemInfo:
    speed: float
    wheel_speeds: List[float]
    wheel_torques: List[float]
    tire_frictions: List[float]
    normalized_long_forces: List[float]
    normalized_lat_forces: List[float]
    drag: float
    engine_rpm: float


def wheels_stopped(wheel_speeds: List[float]) -> bool:
    eps = 1e-10
    return np.all(np.abs(wheel_speeds) <= eps)


wheel_speed_rx = re.compile("omega=([^\s,]*)")
wheel_torque_rx = re.compile("torque=([^\s,]*)")
speed_rx = re.compile("speed=([^\s,]*)")
engine_rx = re.compile("engine_rpm=([^\s,]*)")
drag_rx = re.compile("omega=([^\s,]*)")
tire_friction_rx = re.compile("tire_friction=([^\s,]*)")
norm_longf_rx = re.compile("normalized_long_force=([^\s,\(\)]*)")
norm_latf_rx = re.compile("normalized_lat_force=([^\s,\(\)]*)")



def parsed_telem_info(v: Vehicle) -> TelemInfo:
    telem_str = str(v.get_telemetry_data())
    wheel_speeds = [float(ws) for ws in wheel_speed_rx.findall(telem_str)]
    wheel_torques = [float(ws) for ws in wheel_torque_rx.findall(telem_str)]
    speed = float(speed_rx.findall(telem_str)[0])
    engine_rpm = float(engine_rx.findall(telem_str)[0])
    drag = float(drag_rx.findall(telem_str)[0])
    tire_frictions = [float(ws) for ws in tire_friction_rx.findall(telem_str)]
    norm_longfs = [float(ws) for ws in norm_longf_rx.findall(telem_str)]
    norm_latfs = [float(ws) for ws in norm_latf_rx.findall(telem_str)]
    return TelemInfo(speed=speed,
                     wheel_torques=wheel_torques,
                     drag=drag,
                     tire_frictions=tire_frictions,
                     normalized_long_forces=norm_longfs,
                     normalized_lat_forces=norm_latfs,
                     wheel_speeds=wheel_speeds,
                     engine_rpm=engine_rpm)


# def get_wheel_speeds(v: Vehicle) -> List[float]:
#     wheel_speed_strs = wheel_rx.findall(str(v.get_telemetry_data()))
#     return [float(ws) for ws in wheel_speed_strs]


def get_telem_speed(v: Vehicle) -> float:
    t_string = str(v.get_telemetry_data())
    m = speed_rx.findall(t_string)
    return float(m[0])


def run():
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # Load desired map
        client.load_world("Town01")
        set_sync(world, client, 0.05)
        set_weather(world, 0, 0, 0, 0, 0, 75)

        bpl = world.get_blueprint_library()
        ego_bp = bpl.find('vehicle.mercedes.coupe_2020')
        ego_bp.set_attribute('role_name', 'ego')
        ego_start_trans = carla.Transform(Location(207, 133, 0.1), Rotation(0, 0, 0))

        bps = [ego_bp]
        transforms = [ego_start_trans]

        # # Find Trigger Friction Blueprint
        friction_bp = world.get_blueprint_library().find('static.trigger.friction')
        friction_values = [1.0, 0.5, 0.0]

        # Create Camera to follow them
        spectator = world.get_spectator()

        num_episodes = 10
        num_timesteps = 250
        for f_val in friction_values:
            print(f"Friction Value: {f_val}")

            extent = carla.Location(800, 500.0, 5.0)
            friction_bp.set_attribute('friction', str(f_val))
            friction_bp.set_attribute('extent_x', str(extent.x))
            friction_bp.set_attribute('extent_y', str(extent.y))
            friction_bp.set_attribute('extent_z', str(extent.z))

            #Spawn Trigger Friction
            friction_transform = carla.Transform()
            friction_transform.location = ego_start_trans.location + Location(x=30)
            friction_surf = world.spawn_actor(friction_bp, friction_transform)

            actor_list = setup_actors(world, bps, transforms)
            local_planners = [LocalPlanner(actor) for actor in actor_list]
            vehicle_stats = [VehicleStat(actor.get_location(), 0.0, 0.0) for actor in actor_list]

            for i in range(num_timesteps):
                # time.sleep(0.05)
                world.tick()

                world.debug.draw_box(box=carla.BoundingBox(friction_transform.location, extent * 1e-2),
                                     rotation=friction_transform.rotation,
                                     thickness=0.5, color=carla.Color(r=255, g=0, b=0))

                new_vehicle_stats = []
                for actor, vs in zip(actor_list, vehicle_stats):
                    telem = actor.get_telemetry_data()
                    new_loc = actor.get_location()
                    new_dist = vs.dist_travelled + new_loc.distance(vs.location)
                    # new_wheel_speeds = get_wheel_speeds(actor)
                    new_vehicle_stats.append(VehicleStat(new_loc, new_dist, 0.0))

                vehicle_stats = new_vehicle_stats

                spectator.set_transform(carla.Transform(actor_list[0].get_transform().location + Location(z=30),
                                                        Rotation(pitch=-90)))

                # Move both cars forward for 50m, then have them stop for a couple seconds
                # print([vs.dist_travelled for vs in vehicle_stats])
                for actor, lp, vs in zip(actor_list, local_planners, vehicle_stats):
                    if vs.dist_travelled < 20:
                        control = lp.run_step()
                    else:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 0.5
                        control.hand_brake = False

                        # print(vs.wheel_speeds)
                        telem_info = parsed_telem_info(actor)

                        if wheels_stopped(telem_info.wheel_speeds):
                            actor.open_door(carla.VehicleDoor.All)
                            # actor_vel = actor.get_velocity().length()
                            if telem_info.speed > 0.01:
                                # print(telem_info)
                                actor_telem = actor.get_telemetry_data()
                                print(f"Wheels Speeds: {telem_info.wheel_speeds}, but Speed = {telem_info.speed}")

                    actor.apply_control(control)

            delete_actors(client, actor_list)
            friction_surf.destroy()
            # world.tick()
    finally:
        delete_actors(client, actor_list)
        # if not friction_surf.is_alive:
        #     friction_surf.destroy()
        print("Done")


if __name__ == '__main__':
    run()
