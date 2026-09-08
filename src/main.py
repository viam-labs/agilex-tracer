import asyncio
from viam.module.module import Module

try:
    from models.tracer_base import TracerBase
    from models.tracer_lights import TracerLights
    from models.tracer_odometry import TracerOdometry
    from models.tracer_power import TracerPower
except ModuleNotFoundError:
    from .models.tracer_base import TracerBase
    from .models.tracer_lights import TracerLights
    from .models.tracer_odometry import TracerOdometry
    from .models.tracer_power import TracerPower


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
