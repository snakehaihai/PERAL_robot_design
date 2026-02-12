from acados_template import AcadosOcp, AcadosOcpSolver
from robot_model import export_robot_model
from casadi import vertcat
import numpy as np
import scipy.linalg

# Intital state
X0 = np.array([0.0, 0.0, 0.0])

# Max control input allowed (rad/s)
max_wheel_angular_vel = 15  

# Samping Interval: T_s = T_horizon / N_horizon
# T_s should be 10-25% of system closed-loop response time

# Prediction horizon (seconds)
T_horizon = 5.0  

def Solver() -> AcadosOcpSolver:
    N_horizon = 25  # prediction horizon steps [dt]

    # Create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model
    nx = model.x.rows()
    nu = model.u.rows()

    # Set dimensions
    ocp.solver_options.N_horizon = N_horizon

    # Set cost weight
    Q = np.diag([100, 100, 0.01])  # [x,y,yaw]
    R = np.diag([0.1, 0.1]) # [right_wheel_angular_vel, left_wheel_angular_vel]

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    ny = nx + nu
    ny_e = nx

    ocp.cost.W = scipy.linalg.block_diag(Q, R)  # stage cost weight matrix
    ocp.cost.W_e = Q    # terminal cost weight matrix

    ocp.cost.Vx = np.zeros((ny, nx))
    ocp.cost.Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = np.eye(nx)

    Vu = np.zeros((ny, nu))
    Vu[nx : nx + nu, 0:nu] = np.eye(nu)
    ocp.cost.Vu = Vu

    # For reference trajectory
    ocp.cost.yref = np.zeros((ny,)) # stage coost reference
    ocp.cost.yref_e = np.zeros((ny_e,)) # terminal cost reference

    # Set variables 
    x, y, yaw  = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
    right_wheel_angular_vel, left_wheel_angular_vel = ocp.model.u[0], ocp.model.u[1]

    # Cost presentation
    ocp.model.cost_y_expr = vertcat(x, y, yaw, right_wheel_angular_vel, left_wheel_angular_vel)
    ocp.model.cost_y_expr_e = vertcat(x, y, yaw)
                                     
    # Set hard constraints
    ocp.constraints.lbu = np.array([-max_wheel_angular_vel,-max_wheel_angular_vel])  # control lower bound
    ocp.constraints.ubu = np.array([+max_wheel_angular_vel,+max_wheel_angular_vel]) # control upper bound
    ocp.constraints.idxbu = np.array([0, 1]) # apply the lower/upper bound constraint to right_wheel_angular_vel, left_wheel_angular_vel

    ocp.constraints.x0 = X0
    
    # Set prediction horizon
    ocp.solver_options.tf = T_horizon
    
    # Set options
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI" # SQP_RTI or SQP
    
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.qp_solver_iter_max = 100
    ocp.solver_options.tol = 1e-3
    ocp.solver_options.print_level = 0
    
    # Create solver
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_solver.json")

    return acados_solver

Solver()
print("Acados solver for MPC problem is ready")