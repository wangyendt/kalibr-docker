# author: wangye(Wayne)
# license: Apache Licence
# file: view_SE3_to_rotvec.py
# time: 2024-03-19-11:05:12
# contact: wang121ye@hotmail.com
# site:  wangyendt@github.com
# software: PyCharm
# code is far away from bugs.


from pywayne.vio.tools import *
import qmt

T_1 = np.array([[-0.99946076, 0.01069697, 0.03104453, -0.03445609],
                [0.02327823, 0.89763029, 0.44013405, 0.08565758],
                [-0.02315841, 0.44061937, -0.89739526, -0.04207174],
                [0., 0., 0., 1.]]
               )

T_2 = np.array([[-0.99935596, 0.01231044, 0.03370627, -0.03444406],
                [0.02616213, 0.89285402, 0.44958564, 0.08631018],
                [-0.02456018, 0.45017792, -0.89260105, -0.04242683],
                [0., 0., 0., 1.]])

T_3 = np.array([[0.99996877, -0.00324292, 0.00720768, 0.00109975],
                [-0.00104324, -0.95812486, -0.28634883, 0.02745724],
                [0.00783446, 0.28633237, -0.95809832, -0.02440745],
                [0., 0., 0., 1.]])
T_4 = np.array([[0.9999681, -0.00698819, 0.00386811, 0.0033599],
                [-0.0056029, -0.95885421, -0.28384364, 0.03203962],
                [0.0056925, 0.28381291, -0.95886278, -0.01064046],
                [0., 0., 0., 1.]])


def print_rot_vec(T):
    pose = SE3_to_pose(T).squeeze()
    rot = np.round(np.degrees(qmt.quatToRotVec(pose[3:])), 4).tolist()
    pos = np.round(pose[:3], 4).tolist()
    print(f'{rot=}(deg), {pos=}(m)')


print('2月标定数据：')
print('T_ci (uncalibrated imu)')
print_rot_vec(T_1)
print('T_ci (imu)')
print_rot_vec(T_2)
print('\n3月15日标定数据：')
print('T_ci (uncalibrated imu)')
print_rot_vec(T_3)
print('T_ci (imu)')
print_rot_vec(T_4)
