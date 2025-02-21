map_id = 2
map_world = "NVCC.world"
map_width, map_height = 131.0, 75.0
gs_map_pos_list = [
    (30.0, 46.0, 0),
    (93.0, 20.0, 1.57)
]
lionel_map_x_pos, lionel_map_y_pos = 4.0, 21.0

webserver_port = "8000"
use_gzweb = True

# Have to set at least one of below two to be `True`
use_dtu = True
use_twisted = True

# When running in docker, both may different, it's based on how created container
gs_pty = "/tmp/fake_gs"
cb_pty = "/tmp/fake_cb"

# Please make sure boothnumber map is your target MAP: `map_id`
boothnumber = False

# Override exsiting settings to test code simpler.
try:
    from local_settings import *
except ImportError as e:
    pass
