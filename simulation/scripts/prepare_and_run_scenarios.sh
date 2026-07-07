# ------------------------------------------------------------------
# RUN SCENARIOS SCRIPT
#
# This script executes the full experimental workflow for a specified number 
# of scenarios and boxes. It generates the scenarios, plots each generated 
# instance, runs the VRPPD solver, converts the generated maps and paths into 
# the formats required by ICBS and RHCR, executes both ICBS and RHCR experiments 
# for every scenario, and finally converts the ICBS output files into 
# visualization-ready formats. The number of scenarios and boxes can be 
# configured through the AMOUNT_INST and QNT_BOX variables.
# ------------------------------------------------------------------

# Modify the number of instances to be generated in the scenarios
AMOUNT_INST=6

# Modify the number of boxes to be generated in the scenarios
QNT_BOX=8

# Generating scenarios
echo -e "\n=== Generating scenarios ==="
python3 VRPPD_solver/generate_scenarios.py $AMOUNT_INST $QNT_BOX

# Plotting scenarios
sleep 3
echo -e "\n=== Plotting scenarios ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo -e "Scenario $i"
  python3 VRPPD_solver/plot_scenarios.py $i $QNT_BOX
  sleep 1
done

# Running VRPPD VRPPD_solver for each scenario
sleep 3
echo -e "\n=== Running VRPPD VRPPD_solver for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo -e "\nScenario $i"
  python3.13 VRPPD_solver/vrppd_VRPPD_solver.py $i $QNT_BOX
  sleep 5
done

# Converting ICBS maps for each scenario
sleep 3
echo -e "\n=== Converting ICBS maps for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo "Scenario $i"
  python3 MAPF-ICBS/code/convert_map.py $i $QNT_BOX
done

# Converting RHCR maps for each scenario
sleep 3
echo -e "\n=== Converting RHCR maps for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo "Scenario $i"
  python3 RHCR/convert_map.py $i $QNT_BOX
done

# Converting VRPPD paths to node format for each scenario
sleep 3
echo -e "\n=== Converting VRPPD paths to RHCR node format for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo "Scenario $i"
  python3 RHCR/convert_vrppd_path_to_node.py $i $QNT_BOX
done

# Running ICBS experiments for each scenario
echo "=== Running ICBS experiments for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo "Scenario $i"
  python3 MAPF-ICBS/code/run_experiments.py $i $QNT_BOX
  sleep 10
done

# Running RHCR experiments for each scenario
echo "=== Running RHCR experiments for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo "Scenario $i"
  python3 RHCR/run_rhcr.py $i $QNT_BOX
  sleep 10
done

# Converting ICBS visualization files for each scenario
echo "=== Converting ICBS visualization files for each scenario ==="
for i in $(seq 1 $AMOUNT_INST)
do
  echo "Scenario $i"
  python3 file_conversion/icbs_conversion_visualize.py $i $QNT_BOX
  sleep 1
done

# Converting ICBS and RHCR files for the MAPF simulation and experiments
echo -e "\n=== Converting files for each scenario ==="
for i in $SUCESSFUL_SCENARIOS_ICBS
do
  python3 file_conversion/icbs_conversion.py $i $QNT_BOX
  sleep 1
done

sleep 2

for i in $SUCESSFUL_SCENARIOS_RHCR
do
  python3 file_conversion/rhcr_conversion.py $i $QNT_BOX
  sleep 1
done

echo -e "\n=== Done ==="