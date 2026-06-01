# author: wangye(Wayne)
# license: Apache Licence
# file: update_cam-cam-config.py
# time: 2024-04-15-15:39:05
# contact: wang121ye@hotmail.com
# site:  wangyendt@github.com
# software: PyCharm
# code is far away from bugs.


import sys
from pywayne.tools import wayne_print
import shutil
from Utils import Helper

if len(sys.argv) < 2:
	wayne_print('Usage: python3 update_cam-cam-config.py /path/to/main-config')
	exit(1)
config_yaml_file = sys.argv[1]

helper = Helper(config_yaml_file)

cam_cam_result_yaml_path = helper.get_module_path('cam_cam', 'group_0_result_yaml', use_global_config=True, max_waiting_time=1000)
shutil.copy2(cam_cam_result_yaml_path, './configs/cam-camchain-taurus-640x400_cam0.yaml')
wayne_print('successfully copy cam-cam config from common root to target root', 'green', True)

