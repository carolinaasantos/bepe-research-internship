# README: VRPPD Solver

## Overview

This module solves a **Vehicle Routing Problem with Pickup and Delivery (VRPPD)** using **Google OR-Tools**. Given a set of pickup–delivery requests, the solver assigns tasks to robots while respecting capacity and routing constraints. The generated solution is later used by the path-planning stage.

The solver outputs:

* `*_vrppd_paths.txt`: ordered routes for each robot.
* `*_map_input.txt`: grid map used by the path planner.

## Input Data

The solver loads a `data` dictionary containing:

* Robot start locations
* Robot capacities
* Pickup and delivery locations
* Transportation requests

Each request is defined as:

```python
{
    "pickup": pickup_node_id,
    "delivery": delivery_node_id,
    "weight": weight_in_kg
}
```

## Constraints

### Capacity Constraint

Robot loads must never exceed their carrying capacities. Pickup nodes increase the current load, while delivery nodes decrease it.

### One-Shot Constraint

Robots are restricted to carrying a single box at a time. This is enforced through an additional dimension with maximum capacity equal to 1.

### Pickup and Delivery Constraint

For each request:

* Pickup and delivery must be assigned to the same robot.
* Pickup must occur before delivery.

## Objective Function

Route costs are computed using **Manhattan distance**:

```text
|x1 - x2| + |y1 - y2|
```

The solver minimizes total travel cost while also encouraging balanced route lengths across robots.

## Search Configuration

* Initial solution: `PARALLEL_CHEAPEST_INSERTION`
* Local search: `GUIDED_LOCAL_SEARCH`
* Time limit: 2 seconds

## Workflow

```text
Load instance
    ↓
Create data model
    ↓
Build routing model
    ↓
Add constraints
    ↓
Run OR-Tools solver
    ↓
Export routes and map files
```

### Output example:
```
=== SOLUTION FOUND ===

ROBOT 1 (Capacity: 1.5 kg)
  (Starts in node (5, 6))
  -> [PICKUP Box 4 (1.0 kg)]
  -> [DELIVERY Box 4]
  -> [PICKUP Box 5 (1.0 kg)]
  -> [DELIVERY Box 5]
  -> [PICKUP Box 2 (1.5 kg)]
  -> [DELIVERY Box 2]
  -> [PICKUP Box 3 (0.5 kg)]
  -> [DELIVERY Box 3]
  -> [END OF ROUTE]
----------------------------------------
ROBOT 2 (Capacity: 2 kg)
  (Starts in node (6, 6))
  -> [PICKUP Box 7 (2.0 kg)]
  -> [DELIVERY Box 7]
  -> [PICKUP Box 8 (2.0 kg)]
  -> [DELIVERY Box 8]
  -> [PICKUP Box 6 (2.0 kg)]
  -> [DELIVERY Box 6]
  -> [END OF ROUTE]
----------------------------------------
ROBOT 3 (Capacity: 1.5 kg)
  (Starts in node (1, 7))
  -> [PICKUP Box 1 (1.0 kg)]
  -> [DELIVERY Box 1]
  -> [END OF ROUTE]
----------------------------------------
Files generated successfully!
```

