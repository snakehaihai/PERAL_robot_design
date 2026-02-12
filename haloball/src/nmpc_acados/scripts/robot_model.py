from acados_template import AcadosModel
from casadi import SX, vertcat, sin, cos

def export_robot_model() -> AcadosModel:
    model_name = 'haloball'

    # State variables
    x = SX.sym("x")
    y = SX.sym("y")
    yaw = SX.sym("yaw")

    x = vertcat(x, y, yaw)

    # State derivatives
    x_dot = SX.sym("x_dot")
    y_dot = SX.sym("y_dot")
    yaw_dot = SX.sym("yaw_dot")

    xdot = vertcat(x_dot, y_dot, yaw_dot)

    # Control input
    right_wheel_angular_vel = SX.sym("right_wheel_angular_vel")
    left_wheel_angular_vel = SX.sym("left_wheel_angular_vel")
    
    u = vertcat(right_wheel_angular_vel, left_wheel_angular_vel)

    # Kinematics
    f_expl = vertcat(0.027835412 * (right_wheel_angular_vel + left_wheel_angular_vel) * cos(yaw), 
                     0.027835412 * (right_wheel_angular_vel + left_wheel_angular_vel) * sin(yaw), 
                     0.161031312* (right_wheel_angular_vel - left_wheel_angular_vel))

    f_impl = xdot - f_expl
    
    # Algebraic variables
    z = []

    # Parameters
    p = []
    
    # Acados definition
    model = AcadosModel()
    model.name = model_name
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p # Add parameters to model    

    return model
