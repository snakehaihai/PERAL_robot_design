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
    xdot = SX.sym('xdot', x.rows())

    # Control input
    linear_vel = SX.sym("linear_vel")
    angular_vel = SX.sym("angular_vel")
    
    u = vertcat(linear_vel, angular_vel)

    # Kinematics
    f_expl = vertcat(1.712063 * linear_vel * cos(yaw), 
                     1.712063 * linear_vel * sin(yaw), 
                     angular_vel)

    f_impl = xdot - f_expl
    
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
    model.p = p # Add parameters to model    

    return model