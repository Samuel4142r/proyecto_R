#!/bin/bash
# Corre las simulaciones del experimento en modo headless y guarda los CSV en results/.
#   3 corridas con odometria de ruedas (fig8_run1..3)
#   1 corrida de comparacion con la pose real como realimentacion (fig8_gt)
# Uso (desde la raiz del repo, con el workspace compilado):
#   bash scripts/run_experiments.sh

cd "$(dirname "$0")/.."
source install/setup.bash
mkdir -p results/logs

run() {
  local name=$1; shift
  echo ">>> $name"
  timeout 90 ros2 launch ackermann_controller run_all.launch.py \
      headless:=true rviz:=false run_name:="$name" "$@" > "results/logs/$name.log" 2>&1
  pkill -f "gz sim -r" 2>/dev/null
  pkill -f parameter_bridge 2>/dev/null
  sleep 3
  grep -h "CSV guardado" "results/logs/$name.log" | sed 's/.*\]: //'
}

run fig8_run1
run fig8_run2
run fig8_run3
run fig8_gt odom_topic:=/ground_truth/odom

python3 scripts/plot_results.py
