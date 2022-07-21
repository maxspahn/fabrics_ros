from fabrics.planner.parameterized_planner import ParameterizedFabricPlanner

def PlannerCreationFunctionNotFound(Exception):
    pass

def create_planner(urdf, planner_type: str):
    if planner_type == 'static':
        return create_static_planner(urdf)
    elif planner_type == 'symbolic':
        return create_symbolic_planner(urdf)
    elif planner_type == 'default':
        return create_default_planner(urdf)
    else:
        raise PlannerCreationFunctionNotFound(f"Planner creation for type {planner_type} does not exist.")

def create_symbolic_planner(urdf):
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

    kwargs = {}
    kwargs['base_energy'] = base_energy
    kwargs['damper_beta'] = damper_beta
    kwargs['collision_finsler'] = collision_finsler
    kwargs['collision_geometry'] = collision_geometry
    kwargs['self_collision_finsler'] = self_collision_finsler
    kwargs['self_collision_geometry'] = self_collision_geometry
    kwargs['limit_geometry'] = limit_geometry
    kwargs['limit_finsler'] = limit_finsler
    robot_type = "panda"
    degrees_of_freedom = 7
    return ParameterizedFabricPlanner(
        degrees_of_freedom,
        robot_type,
        urdf=urdf,
        root_link="panda_link0",
        end_link=["panda_link5_offset", "panda_vacuum_2", "panda_vacuum", "camera_link"],
        **kwargs
    )

def create_default_planner(urdf):
    robot_type = "panda"
    degrees_of_freedom = 7
    return ParameterizedFabricPlanner(
        degrees_of_freedom,
        robot_type,
        urdf=urdf,
        root_link="panda_link0",
        end_link=["panda_link5_offset", "panda_vacuum_2", "panda_vacuum", "camera_link"],
    )

def create_static_planner(urdf):
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
    robot_type = "panda"
    degrees_of_freedom = 7
    return ParameterizedFabricPlanner(
        degrees_of_freedom,
        robot_type,
        urdf=urdf,
        root_link="panda_link0",
        end_link=["panda_link5_offset", "panda_vacuum_2", "panda_vacuum", "camera_link"],
        collision_geometry=collision_geometry,
        collision_finsler=collision_finsler,
        self_collision_finsler=self_collision_finsler,
        self_collision_geometry=self_collision_geometry,
        limit_geometry=limit_geometry,
        limit_finsler=limit_finsler,
        base_energy=base_energy,
        attractor_metric=attractor_metric,
        attractor_potential=attractor_potential,
    )

