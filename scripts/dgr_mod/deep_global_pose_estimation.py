import copy

import numpy as np
import open3d as o3d
from core.deep_global_registration import DeepGlobalRegistration
from config import get_config
from scipy.spatial.transform import Rotation


MODEL_WEIGHTS = "my_models/ResUNetBN2C-feat32-kitti-v0.3.pth"


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries(
        [source_temp, target_temp],
        zoom=0.4559,
        front=[0.6452, -0.3036, -0.7011],
        lookat=[1.9892, 2.0208, 1.8945],
        up=[-0.2779, -0.9482, 0.1556],
    )


if __name__ == "__main__":
    config = get_config()

    # Setup the model weights and
    config.weights = MODEL_WEIGHTS
    config.pcd0 = ""
    config.pcd1 = ""

    # preprocessing
    pcd0 = o3d.io.read_point_cloud("my_models/enviroment.pcd")
    pcd0.estimate_normals()
    pcd1 = o3d.io.read_point_cloud("my_models/scan.pcd")
    pcd1.estimate_normals()

    # registration
    dgr = DeepGlobalRegistration(config)
    T01 = dgr.register(pcd0, pcd1)

    # Print the transforms individually
    back_transform = np.linalg.inv(T01)
    rot_mat = Rotation.from_matrix(back_transform[0:3, 0:3].copy())
    print(f"=> Translation: {back_transform[:3, 3]}")
    print(f"=> Rotation: {rot_mat.as_euler('xyz')}")

    draw_registration_result(pcd1, pcd0, np.identity(4))
    draw_registration_result(pcd1, pcd0, back_transform)
    # o3d.visualization.draw_geometries([pcd0, pcd1])

    # pcd0.transform(T01)
    # o3d.visualization.draw_geometries([pcd0, pcd1])
