import numpy as np
from scipy.spatial.transform import Rotation as R


def random_point_in_circle(radius):
    """
    在半径为 radius 的圆内均匀随机生成一个二维点。

    注意这里使用 sqrt(random) 修正半径分布，避免点过度集中在圆心附近。
    """
    angle = np.random.uniform(0, 2 * np.pi)
    r = radius * np.sqrt(np.random.uniform(0, 1))
    x = r * np.cos(angle)
    y = r * np.sin(angle)
    return np.array([x, y])

def generate_camera_pose(cylinder_radius, cylinder_height, upper_cylinder_height, obj_xy, obj_radius = 0.05):
    """
    生成一帧采集用的随机相机位姿。

    采样规则：
    - 目标点在物体中心 obj_xy 附近的小圆内，z=0。
    - 相机 XY 在半径 cylinder_radius 的圆内随机采样。
    - 相机 Z 在 [cylinder_height, cylinder_height + upper_cylinder_height] 内随机采样。
    - 相机朝向目标点，返回位置、目标点、四元数和欧拉角。

    这个函数只负责数学采样，不直接修改 Isaac 场景；ui_builder.generate_frames()
    会把返回的 position/rotation 应用到 Replicator camera。
    """
    # 在物体附近采样一个注视点，避免所有相机都严格看向同一个中心点。
    target_xy = random_point_in_circle(obj_radius)
    target_xy = target_xy + obj_xy
    target_point = np.array([target_xy[0], target_xy[1], 0.0])

    # 相机位置在上方圆柱体空间内随机分布。
    camera_xy = random_point_in_circle(cylinder_radius)
    camera_z = np.random.uniform(cylinder_height, cylinder_height + upper_cylinder_height)
    camera_position = np.array([camera_xy[0], camera_xy[1], camera_z])

    # 计算方向向量：从相机指向目标点。
    direction = target_point - camera_position
    direction_normalized = direction / np.linalg.norm(direction)

    # 构造相机旋转矩阵：局部 Z 轴指向目标点，再由叉乘补齐 X/Y 轴。
    z_axis = direction_normalized
    x_axis = np.array([1.0, 0.0, 0.0])  # 假设X轴作为初始参考方向
    if np.allclose(z_axis, x_axis) or np.allclose(z_axis, -x_axis):
        # 如果z_axis与x_axis平行，则使用Y轴作为参考
        x_axis = np.array([0.0, 1.0, 0.0])
    y_axis = np.cross(z_axis, x_axis)
    x_axis = np.cross(y_axis, z_axis)

    # 归一化坐标轴
    x_axis /= np.linalg.norm(x_axis)
    y_axis /= np.linalg.norm(y_axis)
    z_axis /= np.linalg.norm(z_axis)

    # scipy Rotation.from_matrix 期望 3x3 旋转矩阵，列向量为局部坐标轴在世界系中的方向。
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    # Isaac/Replicator modify.pose 这里使用欧拉角；四元数保留给调试或后处理。
    rotation = R.from_matrix(rotation_matrix)
    quaternion = rotation.as_quat()  # [x, y, z, w]
    euler_angles = rotation.as_euler('zyx', degrees=True)  # [pitch, yaw, roll]

    return {
        "camera_position": camera_position,
        "target_point": target_point,
        "quaternion": quaternion,
        "euler_angles": euler_angles
    }
