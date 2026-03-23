from acados_template import AcadosOcp, AcadosOcpSolver
from robot_model import export_robot_model
from casadi import vertcat
import numpy as np
import scipy.linalg

# Max control input allowed
max_linear_vel = 0.25 # (m/s)
max_angular_vel = 2.0  # (rad/s)

# Samping Interval: T_s = T_horizon / N_horizon
# T_s should be 10-25% of system closed-loop response time

N_horizon = 25  # prediction horizon steps [dt]
T_horizon = 5.0 # Prediction horizon (seconds)

def Solver() -> AcadosOcpSolver:

    # Create ocp object to formulate the OCP
    ocp = AcadosOcp()

    model = export_robot_model()
    ocp.model = model

    # Set dimensions
    nx = model.x.rows() # state dimension
    nu = model.u.rows() # control dimension
    ny = nx + nu    # output dimension for cost
    ny_e = nx   # output dimension for terminal cost

    # Discretisation
    ocp.solver_options.N_horizon = N_horizon
    ocp.solver_options.tf = T_horizon

    # Cost function type
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'

    # Set cost weight
    Q = np.diag([100, 100, 0.01])  # [x,y,yaw]
    R = np.diag([0.1, 0.1]) # [linear_vel, angular_vel]

    ocp.cost.W = scipy.linalg.block_diag(Q, R)  # stage cost weight matrix
    ocp.cost.W_e = Q    # terminal cost weight matrix

    # For reference trajectory
    ocp.cost.yref = np.zeros(ny) # stage cost reference
    ocp.cost.yref_e = np.zeros(ny_e) # terminal cost reference

    # Cost presentation
    ocp.model.cost_y_expr = vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = vertcat(model.x)
                                     
    # Control bounds (hard constraints)
    ocp.constraints.lbu = np.array([0,-max_angular_vel])  # control lower bound
    ocp.constraints.ubu = np.array([+max_linear_vel,+max_angular_vel]) # control upper bound
    ocp.constraints.idxbu = np.array([0, 1]) # apply the lower/upper bound constraint to linear_vel, angular_vel

    # Initial state constraint
    ocp.constraints.x0 = np.zeros(nx)
    
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