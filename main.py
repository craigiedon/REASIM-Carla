import dataclasses
import time

from navigation.local_planner import LocalPlanner

from carlaSetupUtils import set_weather, set_sync, setup_actors, delete_actors
import carla
from carla import Location, Vehicle


@dataclasses
class VehicleStat:
    location: Location
    dist_travelled: float


def car_stopped(actor: Vehicle) -> bool:
    eps = 1e-10
    v = actor.get_velocity()
    return v.length() <= eps


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

        extent = carla.Location(500, 500.0, 5.0)

        friction_bp.set_attribute('friction', str(0.0))
        friction_bp.set_attribute('extent_x', str(extent.x))
        friction_bp.set_attribute('extent_y', str(extent.y))
        friction_bp.set_attribute('extent_z', str(extent.z))

        # Spawn Trigger Friction
        friction_transform = carla.Transform()
        friction_transform.location = ego_start_trans.location + Location(x=15)
        world.spawn_actor(friction_bp, friction_transform)

        # Create Camera to follow them
        spectator = world.get_spectator()

        num_episodes = 10
        num_timesteps = 250
        for ep_id in range(num_episodes):
            actor_list = setup_actors(world, bps, transforms)
            local_planners = [LocalPlanner(actor) for actor in actor_list]
            vehicle_stats = [VehicleStat(actor.get_location(), 0.0) for actor in actor_list]

            for i in range(num_timesteps):
                print(f"Time Step {i}")
                world.tick()

                world.debug.draw_box(box=carla.BoundingBox(friction_transform.location, extent * 1e-2),
                                     rotation=friction_transform.rotation,
                                     thickness=0.5, color=carla.Color(r=255, g=0, b=0))

                new_vehicle_stats = []
                for actor, vs in zip(actor_list, vehicle_stats):
                    new_loc = actor.get_location()
                    new_dist = vs.dist_travelled + new_loc.distance(vs.location)
                    new_vehicle_stats.append(VehicleStat(new_loc, new_dist))

                vehicle_stats = new_vehicle_stats

                spectator.set_transform(carla.Transform(actor_list[0].get_transform().location + Location(z=30),
                                                        Rotation(pitch=-90)))

                # Move both cars forward for 50m, then have them stop for a couple seconds
                # print([vs.dist_travelled for vs in vehicle_stats])
                for actor, lp, vs in zip(actor_list, local_planners, vehicle_stats):
                    if vs.dist_travelled < 30:
                        control = lp.run_step()
                    else:
                        control = carla.VehicleControl()
                        control.throttle = 0.0
                        control.brake = 0.5
                        control.hand_brake = False
                    actor.apply_control(control)

                    if car_stopped(actor):
                        actor.open_door(carla.VehicleDoor.All)

            delete_actors(client, actor_list)
    finally:
        delete_actors(client, actor_list)
        print("Done")


if __name__ == '__main__':
    run()
