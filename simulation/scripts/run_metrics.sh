# ------------------------------------------------------------------
# RUN METRICS SCRIPT
#
# This script runs the metrics for both ICBS and RHCR algorithms 
# on the successful scenarios.
# ------------------------------------------------------------------

QNT_BOX=8

# Select the successful scenarios for ICBS and RHCR to run the metrics (those that found a solution within the timeout)
#SUCESSFUL_SCENARIOS_ICBS="1 2 3 4 5 6 7 8 9 10 11 12"
#SUCESSFUL_SCENARIOS_RHCR="1 2 3 4 5 6 7 8 9 10 11 12"
SUCESSFUL_SCENARIOS_ICBS="1 2 3 4 5 6"
SUCESSFUL_SCENARIOS_RHCR="1 2 3 4 5 6"

# METRIC: PLANNING TIME - Running ICBS 3x for each scenario
echo -e "\n=== Running ICBS 3x for each scenario ==="
for i in $SUCESSFUL_SCENARIOS_ICBS
do
  for j in {1..3}
  do
     echo -e "\nScenario $i - Iteration $j"
     python3 MAPF_ICBS/code/run_experiments.py $i $QNT_BOX
     sleep 5
  done
done

# METRIC: PLANNING TIME - Running RHCR 3x for each scenario
echo -e "\n=== Running RHCR 3x for each scenario ==="
for i in $SUCESSFUL_SCENARIOS_RHCR
do
  for j in {1..3}
  do
     echo -e "\nScenario $i - Iteration $j"
     python3 RHCR/run_rhcr.py $i $QNT_BOX
     sleep 5
  done
done

# MAKESPAN - Running ICBS for each scenario
echo -e "\n=== Running ICBS for each scenario ==="
for i in $SUCESSFUL_SCENARIOS_ICBS
do
  python3 MAPF_ICBS/code/visualize.py $i $QNT_BOX --not-show
  sleep 3
done

# MAKESPAN - Running RHCR for each scenario
echo -e "\n=== Running RHCR for each scenario ==="
for i in $SUCESSFUL_SCENARIOS_RHCR
do
  python3 RHCR/visualize_rhcr.py $i $QNT_BOX --not-show
  sleep 3
done

echo -e "\n=== Done ==="