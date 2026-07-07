# Experiment Automation Scripts

This repository includes two shell scripts designed to automate the complete experimental workflow. Together, these scripts handle scenario generation, solver execution, file conversion, planner evaluation, and metric collection. The scripts are divided into:

- `prepare_and_run_scenarios.sh`: responsible for generating the scenarios and running the path planning files;
- `run_metrics.sh`: responsible for generating the metrics necessary for evaluation.

## `prepare_and_run_scenarios.sh`

Runs the full pipeline for a configurable number of scenarios and boxes. It is intended to generate all experimental instances and produce the outputs required by the MAPF planners (ICBS and RHCR).

### Configuration

Before running the script, configure the desired number of scenarios and boxes at the beginning of the script:

```bash
AMOUNT_INST=6   # Number of scenarios to generate
QNT_BOX=8       # Number of boxes per scenario
```

### Workflow

The script automatically performs the following steps:

1. Scenario Generation: creates warehouse scenarios with the specified number of boxes, with different box weights and initial positions.

2. Scenario Visualization: generates plots of each scenario for verification and inspection.

3. VRPPD Solving: runs the VRPPD solver to compute pickup and delivery routes.

4. Map Conversion: converts the generated maps into the formats required by ICBS and RHCR.

5. Path Conversion: converts VRPPD solutions into the node-based format expected by RHCR.

6. MAPF Planning: executes ICBS and RHCR for every scenario.

7. Output Processing: converts ICBS outputs into visualization-ready files and converts planner outputs into formats suitable for MAPF simulations and further analysis.

### Usage

```bash
chmod +x prepare_and_run_scenarios.sh
./prepare_and_run_scenarios.sh
```

### Output

After execution, the script produces:

Generated scenarios and plots.
VRPPD solutions.
ICBS and RHCR input files.
Planner solutions.
Visualization files.
Converted outputs for simulations and experiments.

---

## `run_metrics.sh`

This script is used after the scenario generation pipeline has completed. Its purpose is to collect performance metrics from the planners, including planning time and makespan, by running additional experiments.

Only scenarios for which solutions were found by the planners should be included in the execution lists.

### Configuration

Set the number of boxes and specify the successful scenarios for each planner:

```bash
QNT_BOX=8

SUCESSFUL_SCENARIOS_ICBS="1 2 3 4 5 6"
SUCESSFUL_SCENARIOS_RHCR="1 2 4 5 6"
```

### Metrics

#### Planning Time

To reduce variability and obtain more reliable measurements, for each successful scenario:

* ICBS is executed 3 times.
* RHCR is executed 3 times.

The resulting execution times can be used to compute average planning times and standard deviations.

#### Makespan

For each successful scenario:

* ICBS visualization is executed to extract makespan information.
* RHCR visualization is executed to extract makespan information.

These values represent the total execution length of the generated multi-agent plans.

### Usage

```bash
chmod +x run_metrics.sh
./run_metrics.sh
```

### Notes

* Run prepare_and_run_scenarios.sh before executing this script.
* Only include scenarios that successfully produced solutions within the configured timeout.
* Ensure all dependencies and required components (VRPPD, ICBS, RHCR, and conversion scripts) are properly installed and accessible.
* Generated metrics can be used for planner comparison and experimental evaluation.