from dsr2.gripper_client import GripperClient

g = GripperClient("127.0.0.1", 9999)  # 컨테이너 안이면 host.docker.internal
assert g.connect()
assert g.initialize()

g.set_force(60)  # 20~100
g.set_position(700)  # 0=닫힘, 1000=열림
print(g.get_state())

g.open()
print(g.get_state())

g.close(100)
print(g.get_state())

g.disconnect()
