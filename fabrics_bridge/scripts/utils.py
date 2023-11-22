from fabrics_msgs.msg import FabricsObstacleArray, FabricsObstacle
from geometry_msgs.msg import Point, Quaternion, Vector3
import numpy as np

# Helpers
def _it(self):
    yield self.x
    yield self.y
    yield self.z


Point.__iter__ = _it
Vector3.__iter__ = _it


def _it(self):
    yield self.x
    yield self.y
    yield self.z
    yield self.w


Quaternion.__iter__ = _it


def sphere_obs(
    x: float, y: float, z: float, r: float = 0.05
) -> FabricsObstacle:
    o = FabricsObstacle()
    o.header.frame_id = "panda_link0"
    o.radius = r
    o.position = Point(x, y, z)
    o.obstacle_type = "sphere"
    return o

def quaternion_to_rotation_matrix(
    quaternion: np.ndarray, ordering: str = "wxyz"
) -> np.ndarray:
    # Normalize the quaternion if needed
    quaternion /= np.linalg.norm(quaternion)

    if ordering == "wxyz":
        w, x, y, z = quaternion
    elif ordering == "xyzw":
        x, y, z, w = quaternion
    else:
        raise InvalidQuaternionOrderError(
            f"Order {ordering} is not permitted, options are 'xyzw', and 'wxyz'"
        )
    rotation_matrix = np.array(
        [
            [
                1 - 2 * y**2 - 2 * z**2,
                2 * x * y - 2 * w * z,
                2 * x * z + 2 * w * y,
            ],
            [
                2 * x * y + 2 * w * z,
                1 - 2 * x**2 - 2 * z**2,
                2 * y * z - 2 * w * x,
            ],
            [
                2 * x * z - 2 * w * y,
                2 * y * z + 2 * w * x,
                1 - 2 * x**2 - 2 * y**2,
            ],
        ]
    )

    return rotation_matrix

def pose_to_transformation_matrix(
    position: np.ndarray,
    quaternion: np.ndarray,
    ordering: str = "wxyz"
) -> np.ndarray:
    transform_matrix = np.identity(4)
    transform_matrix[0:3,3] = position
    transform_matrix[0:3,0:3] = quaternion_to_rotation_matrix(quaternion, ordering=ordering)
    return transform_matrix
