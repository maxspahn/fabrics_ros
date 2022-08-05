import rospy
from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner
from fabrics.planner.non_holonomic_parameterized_planner import NonHolonomicParameterizedFabricPlanner

def PlannerCreationFunctionNotFound(Exception):
    pass

def create_planner():
    planner_type = rospy.get_param("/planner_type")
    kinematic_type = rospy.get_param("/kinematic_type")
    urdf = rospy.get_param(rospy.get_param("/urdf_source"))
    robot_type = rospy.get_param("/robot_type")
    degrees_of_freedom = rospy.get_param("/degrees_of_freedom")

    if planner_type == 'static':
        planner_arguments = create_static_arguments()
    elif planner_type == 'symbolic':
        planner_arguments = create_symbolic_arguments()
    elif planner_type == 'default':
        planner_arguments = create_default_planner()
    else:
        raise PlannerCreationFunctionNotFound(f"Planner creation for type {planner_type} does not exist.")

    if kinematic_type == "holonomic":
        return ParameterizedFabricPlanner(
            degrees_of_freedom,
            robot_type,
            urdf=urdf,
            root_link=rospy.get_param("/root_link"),
            end_link=rospy.get_param("/end_links"),
            **planner_arguments,
        )
    elif kinematic_type == "nonholonomic":
        return NonHolonomicParameterizedFabricPlanner(
            degrees_of_freedom,
            robot_type,
            urdf=urdf,
            root_link=rospy.get_param("/root_link"),
            end_link=rospy.get_param("/end_links"),
            l_offset=rospy.get_param("/l_offset"),
            **planner_arguments,
        )


def create_symbolic_arguments():
    collision_geometry: str = (
        "-sym('k_geo') / (x ** sym('exp_geo')) * xdot ** 2"
    )
    collision_finsler: str = (
        "sym('k_fin')/(x**sym('exp_fin')) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    limit_geometry: str = (
        "-sym('k_limit_geo') / (x ** sym('exp_limit_geo')) * xdot ** 2"
    )
    limit_finsler: str = (
        "sym('k_limit_fin')/(x**sym('exp_limit_fin')) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    self_collision_geometry: str = (
        "-0.5 / (x ** 1) * xdot ** 2"
    )
    self_collision_finsler: str = (
        "0.1/(x**2) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    """
    collision_geometry: str = (
        "-0.5 / (x ** 2) * xdot ** 2"
    )
    collision_finsler: str = (
        "0.1/(x**2) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    self_collision_geometry: str = (
        "-0.5 / (x ** 1) * xdot ** 2"
    )
    self_collision_finsler: str = (
        "0.1/(x**2) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    limit_geometry: str = (
        "-0.5/ (x ** 1) * xdot ** 2"
    )
    limit_finsler: str = (
        "0.4/(x**1) * (-0.5 * (ca.sign(xdot) - 1)) * xdot**2"
    )
    """
    base_energy: str = (
        "0.5 * sym('base_inertia') * ca.dot(xdot, xdot)"
    )
    damper_beta: str = (
        "0.5 * (ca.tanh(-sym('alpha_b') * (ca.norm_2(x) - sym('radius_shift'))) + 1) * sym('beta_close') + sym('beta_distant') + ca.fmax(0, sym('a_ex') - sym('a_le'))"
    )
    damper_eta: str = (
        "0.5 * (ca.tanh(-0.5 * sym('ex_lag') * (1 - sym('ex_factor')) - 0.5) + 1)"
    )
    kwargs = {}
    kwargs['base_energy'] = base_energy
    kwargs['damper_beta'] = damper_beta
    kwargs['damper_eta'] = damper_eta
    kwargs['collision_finsler'] = collision_finsler
    kwargs['collision_geometry'] = collision_geometry
    kwargs['self_collision_finsler'] = self_collision_finsler
    kwargs['self_collision_geometry'] = self_collision_geometry
    kwargs['limit_geometry'] = limit_geometry
    kwargs['limit_finsler'] = limit_finsler
    return kwargs

def create_default_planner():
    kwargs = {}
    return kwargs

def create_static_arguments():
    base_energy: str = (
        "0.5 * 0.2 * ca.dot(xdot, xdot)"
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
    kwargs = {}
    kwargs['base_energy'] = base_energy
    kwargs['collision_finsler'] = collision_finsler
    kwargs['collision_geometry'] = collision_geometry
    kwargs['self_collision_finsler'] = self_collision_finsler
    kwargs['self_collision_geometry'] = self_collision_geometry
    kwargs['limit_geometry'] = limit_geometry
    kwargs['limit_finsler'] = limit_finsler
    kwargs["attractor_metric"] = attractor_metric
    kwargs["attractor_potential"] = attractor_potential
    return kwargs

