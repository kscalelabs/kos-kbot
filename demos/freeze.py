import asyncio
from pykos import KOS

type_04 = [41, 44, 31, 34]
type_03 = [42, 43, 32, 33]
type_02 = [45, 35]


async def read_actuator(kos: KOS):
    # actuator_ids = list(range(41, 46)) + list(range(31, 36))  # Creates [41,42,43,44,45,31,32,33,34,35]
    
    for actuator_id in type_04:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=120,
            kd=5,
            max_torque=80.0,
            torque_enabled=True,
        )

    for actuator_id in type_03:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=80,
            kd=4,
            max_torque=50.0,
            torque_enabled=True,
        )

    for actuator_id in type_02:
        await kos.actuator.configure_actuator(
            actuator_id=actuator_id,
            kp=50,
            kd=1,
            max_torque=17.0,
            torque_enabled=True,
        )
    # # Configure all actuators
    # for actuator_id in actuator_ids:
    #     await kos.actuator.configure_actuator(
    #         actuator_id=actuator_id,
    #         kp=50,
    #         kd=2,
    #         max_torque=20.0,
    #         torque_enabled=True,
    #     )

    actuator_ids = [11, 12, 13, 14, 15, 21, 22, 23, 24, 25]
    
    state = await kos.actuator.get_actuators_state(actuator_ids)
    print(state)

async def move_actuator(kos: KOS, d=5):
    actuator_ids = list(range(41, 46)) + list(range(31, 36))
    
    # Get current positions for all actuators
    states = (await kos.actuator.get_actuators_state(actuator_ids)).states
    
    # Create command list for all actuators
    commands = [
        {
            "actuator_id": state.actuator_id,
            "position": state.position + d,
        }
        for state in states
    ]
    
    await kos.actuator.command_actuators(commands)

async def main():
    async with KOS(ip="localhost", port=50051) as real_kos:
        await read_actuator(real_kos)
        #await move_actuator(real_kos, 0)


if __name__ == "__main__":
    asyncio.run(main())

