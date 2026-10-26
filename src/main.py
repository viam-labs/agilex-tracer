import asyncio

from viam.module.module import Module

# Imported for their side effect: defining each model registers it with the
# registry that run_from_registry serves.
try:
    from models.tracer_base import TracerBase  # noqa: F401
    from models.tracer_lights import TracerLights  # noqa: F401
    from models.tracer_odometry import TracerOdometry  # noqa: F401
    from models.tracer_power import TracerPower  # noqa: F401
except ModuleNotFoundError:
    from .models.tracer_base import TracerBase  # noqa: F401
    from .models.tracer_lights import TracerLights  # noqa: F401
    from .models.tracer_odometry import TracerOdometry  # noqa: F401
    from .models.tracer_power import TracerPower  # noqa: F401


if __name__ == "__main__":
    asyncio.run(Module.run_from_registry())
