export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"$HOME/acados/lib"
export ACADOS_SOURCE_DIR="$HOME/acados"

cd ~/haloball/src/mpc_controller/scripts

source ~/env/bin/activate
python3 create_solver.py
