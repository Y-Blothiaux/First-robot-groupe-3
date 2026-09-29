import pypot.dynamixel

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1])
dxl_io.set_moving_speed({1: 360}) # Degrees / s