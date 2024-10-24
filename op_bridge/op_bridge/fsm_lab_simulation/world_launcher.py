import os
import carla


# Create the world launcher.
class WorldLauncher(object):
    def __init__(self):
        self.local_host = os.environ["SIMULATOR_LOCAL_HOST"]
        self.port = int(os.environ["SIMULATOR_PORT"])
        self.frame_rate = float(os.environ["AGENT_FRAME_RATE"])
        self.bridge_mode = os.environ["OP_BRIDGE_MODE"]
        self.map_name = os.environ["FREE_MAP_NAME"]
        self.client = None
        self.world = None
    
    def load_world(self):
        self.client = carla.Client(self.local_host, self.port)
        self.client.set_timeout(20)

        # Check whether the world simulator has been initialized or not.
        exist_world_name = self.client.get_world().get_map().name
        exist_world_name = os.path.basename(exist_world_name)
        if exist_world_name == self.map_name:
            self.world = self.client.get_world()
            return True
        else:
            # Get all available maps.
            maps_list = self.client.get_available_maps()
            for map in maps_list:
                temp_map_name = os.path.basename(map)
                if temp_map_name == self.map_name:
                    self.map_name = map
                    break
            
            if self.bridge_mode == "free" or self.bridge_mode == '':
                try:
                    self.client.load_world(self.map_name)
                    self.world = self.client.get_world()
                    if self.world is not None:
                        settings = self.world.get_settings()
                        settings.fixed_delta_seconds = 1.0 / self.frame_rate
                        # settings.synchronous_mode = True
                        self.world.apply_settings(settings)
                except RuntimeError:
                    raise RuntimeError(f"The current map list is: {[os.path.basename(map) for map in maps_list]}, '{self.map_name}' is not supported!")
            else:
                raise NotImplementedError("Waiting for Implemnetation...")
            
            return self.world.get_map().name


if __name__ == "__main__":
    world_launcher = WorldLauncher()
    world_name = world_launcher.load_world()
    
    if world_name is not None and type(world_name) is not bool:
        print(f"Launch world '{world_name}' successful, please enjoy!")
    elif type(world_name) is bool:
        print(f"World {os.environ['FREE_MAP_NAME']} has been successfully launched before, please enjoy!")
    else:
        print("Launch world failed!")
