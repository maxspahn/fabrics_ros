import math
import numpy as np
import rospy
import hashlib

from fabrics_msgs.msg import FabricsObstacle
from geometry_msgs.msg import Point, Quaternion, Vector3

from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from fabrics.planner.non_holonomic_parameterized_planner import NonHolonomicParameterizedFabricPlanner

def create_planner(planner_type: str, forward_kinematics) -> ParameterizedFabricPlanner:
    base_energy: str = (
        "0.5 * 0.2 * ca.dot(xdot, xdot)"
    )
    base_metric = "0.1 * np.array([ \
        [sym('m_base'), 0, 0, 0, 0, 0, 0, 0],  \
        [0, 1, 0, 0, 0, 0, 0, 0],  \
        [0, 0, 1, 0, 0, 0, 0, 0],  \
        [0, 0, 0, 1, 0, 0, 0, 0],  \
        [0, 0, 0, 0, 1, 0, 0, 0],  \
        [0, 0, 0, 0, 0, 1, 0, 0],  \
        [0, 0, 0, 0, 0, 0, 1, 0],  \
        [0, 0, 0, 0, 0, 0, 0, 1],  \
        ])"
    base_energy: str = (
        f"ca.dot(xdot, ca.mtimes({base_metric}, xdot))"
    )
    collision_geometry: str = (
        "-0.5 / (x ** 2) * xdot ** 2"
    )
    collision_finsler: str = (
        "0.1/(x**2) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    limit_geometry: str = (
        "-0.50 / (x ** 2) * xdot ** 2"
    )
    limit_finsler: str = (
        "0.4/(x**1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    self_collision_geometry: str = (
        "-0.5 / (x ** 1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot ** 2"
    )
    self_collision_finsler: str = (
        "0.1/(x**2) * xdot**2"
    )
    attractor_potential: str = (
        "5.0 * (ca.norm_2(x) + 1 / 10 * ca.log(1 + ca.exp(-2 * 10 * ca.norm_2(x))))"
    )
    attractor_metric: str = (
        "((2.0 - 0.3) * ca.exp(-1 * (0.75 * ca.norm_2(x))**2) + 0.3) * ca.SX(np.identity(x.size()[0]))"
    )
    degrees_of_freedom = rospy.get_param("/degrees_of_freedom")
    print("dof: ", degrees_of_freedom)
    if planner_type == "holonomic":
        print("hereeeeeee")
        print("degrees of freedom: ", degrees_of_freedom)
        print("forward_kinematics: ", forward_kinematics)
        print("base_energy: ", base_energy)
        print("collision_geometry: ", collision_geometry)
        print("collision_finsler: ", collision_finsler)
        print("self collision finsler: ", self_collision_finsler)
        print("limit geometry: ", limit_geometry)
        print("limit finsler: ", limit_finsler)
        return ParameterizedFabricPlanner(
            degrees_of_freedom,
            forward_kinematics,
            # base_energy=base_energy,
            # collision_geometry=collision_geometry,
            # collision_finsler=collision_finsler,
            # self_collision_finsler=self_collision_finsler,
            # limit_geometry=limit_geometry,
            # limit_finsler=limit_finsler,
        )
        print("print in utils ")
    if planner_type == "nonholonomic":
        collision_geometry: str = (
            "-sym('k_geo_col') / (x ** 1) * xdot ** 2"
        )
        collision_finsler: str = (
            "sym('k_fin_col')/(x**1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
        )
        return NonHolonomicParameterizedFabricPlanner(
            degrees_of_freedom,
            forward_kinematics,
            collision_geometry=collision_geometry,
            collision_finsler=collision_finsler,
            self_collision_finsler=self_collision_finsler,
            limit_geometry=limit_geometry,
            limit_finsler=limit_finsler,
            l_offset=rospy.get_param("/l_offset"),
        )


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

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])

def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])

def Rz(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def rectangle_spheres(origin, w, h, r, extra_s = 0):
    # Method to determine a rectangle of sphere with a certain radius qaually distributes on the perimeter
    perimeter = 2*h+2*w
    # n_spheres = int((perimeter + 2*r)/(2*r) - 1 + extra_s)
    n_spheres = 10
    rot_matrix = Rz(math.pi/2)

    # Initialize matrix containing sphere points
    sphere_loc = np.zeros((3, n_spheres))
    dist = perimeter/float(n_spheres)
    for i in range(n_spheres):
        # x location of the sphere, unwrapped rectangle into a line
        sphere_loc[0,i] = dist*i

    for i in range(n_spheres):
        if sphere_loc[0,i] > w:
            # Translate to origin
            sphere_loc[0,i] -= w
            # Rotate
            sphere_loc[:,i] =  np.matmul(rot_matrix, sphere_loc[:,i])
            # Translate back
            sphere_loc[0,i] += w

    for i in range(n_spheres):
        if sphere_loc[1,i] > h:
            sphere_loc[0,i] -= w
            sphere_loc[1,i] -= h
            sphere_loc[:,i] =  np.matmul(rot_matrix, sphere_loc[:,i])
            sphere_loc[0,i] += w
            sphere_loc[1,i] += h

    for i in range(n_spheres):
        if sphere_loc[0,i] < 0:
            sphere_loc[1,i] -= h
            # Rotate
            sphere_loc[:,i] =  np.matmul(rot_matrix, sphere_loc[:,i])
            # Translate back
            sphere_loc[1,i] += h

    # Translate origin in the center of rectangle
    sphere_loc[0,:] = sphere_loc[0,:] - w/2 + origin[0]
    sphere_loc[1,:] = sphere_loc[1,:] - h/2 + origin[1]
    sphere_loc[2,:] = sphere_loc[2,:] + origin[2]

    return sphere_loc.T

def pose_to_transformation_matrix(
    position: np.ndarray,
    quaternion: np.ndarray,
    ordering: str = "wxyz"
) -> np.ndarray:
    transform_matrix = np.identity(4)
    transform_matrix[0:3,3] = position
    transform_matrix[0:3,0:3] = quaternion_to_rotation_matrix(quaternion, ordering=ordering)
    return transform_matrix




def list_to_unique_hash(a: list) -> str:
    string = ""
    for value in sorted(a):
        string = string.join(value)
    encoded_string = string.encode("utf-8")
    return hashlib.md5(encoded_string).hexdigest()



