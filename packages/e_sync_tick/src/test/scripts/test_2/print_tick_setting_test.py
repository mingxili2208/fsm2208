import carla

import pprint
# Connect to the CARLA simulator
client = carla.Client('localhost', 2000)  # Adjust host and port if needed
client.set_timeout(10.0)

# Get the world object
world = client.get_world()

settings = world.get_settings()

# 使用 pprint 格式化打印
pprint.pprint({attr: getattr(settings, attr) for attr in dir(settings) if not attr.startswith("__")})






# # Print the fixed delta seconds (tick time)
# print(f"Fixed Delta Seconds: {settings.fixed_delta_seconds}")

# # Calculate the tick rate (frequency)
# tick_rate = 1.0 / settings.fixed_delta_seconds
# print(f"Tick Rate (Hz): {tick_rate}")