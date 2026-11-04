# Programmable Multi-Robot Consensus & Formation Control Playground

A static, GitHub-Pages-ready teaching environment for **multi-robot consensus, formation control, communication graphs, programmable distributed protocols, and noisy robot sensing**.

The second version extends the original playground from a fixed controller demo into a small browser-based **multi-agent control laboratory**.

## Main capabilities

### 1. Interactive robot simulator

The simulator supports:

- 3–15 robots;
- 2-D single-integrator dynamics;
- 2-D double-integrator dynamics;
- configurable update rate;
- configurable actuator speed limits;
- draggable robots;
- injected team disturbances;
- robot trajectory visualization;
- desired formation visualization;
- live error histories.

### 2. Editable distributed control protocol

Students can edit the protocol itself.

The page includes an in-browser Python editor with starter protocols for:

- rendezvous/average-style consensus;
- offset-based formation control;
- leader–follower formation;
- sensor-aware control using distance and bearing measurements.

Python is executed locally in the browser with **Pyodide**. No backend is required.

The required Python function is:

```python
def control(robot_id, observations, params):
    # Return planar command [ux, uy]
    return [0.0, 0.0]
```

The controller receives a per-robot observation object rather than the full privileged world state.

A typical observation looks like:

```python
{
    "self": {
        "id": 0,
        "position": [x, y],
        "velocity": [vx, vy]
    },
    "neighbors": [
        {
            "id": 1,
            "weight": 1.0,
            "relative_position": [dx, dy],
            "distance": 84.2,
            "bearing": 0.31,
            "rf_rssi": -57.4,
            "radar": {
                "range": 83.9,
                "bearing": 0.30
            },
            "desired_offset": [70, 0]
        }
    ],
    "desired_offset": [0, 0],
    "leader": True,
    "dt": 0.0333
}
```

Fields can be `None` when a corresponding sensor is disabled, out of range, or drops a measurement.

### 3. Editable graph structure

The communication graph can be generated from presets or edited manually.

Presets:

- complete graph;
- ring;
- path;
- star;
- distance-limited graph;
- custom weighted graph.

The adjacency matrix is editable directly in the browser.

For matrix entry

```text
A[i][j]
```

a nonzero value means robot `i` uses information from robot `j`, and the value is passed to the controller as the edge weight.

The graph can be directed or undirected.

This makes it possible to teach:

- graph sparsity;
- weighted consensus;
- directed information flow;
- switching graphs;
- state-dependent range graphs;
- graph disconnection;
- leader reachability.

## Sensor simulation

Each robot can be equipped with any combination of the following sensors.

### Absolute position

Model:

```text
p_measured = p_true + bias + Gaussian noise
```

The playground applies independent x/y noise and a per-run random bias.

Useful for approximating localization systems such as motion capture, GPS-like position, UWB positioning, visual localization, or SLAM output at an abstract level.

### Relative distance

Model:

```text
d_measured = d_true + bias + Gaussian noise
```

Parameters:

- noise standard deviation;
- bias standard deviation;
- maximum range;
- measurement dropout probability.

### Relative bearing

Model:

```text
theta_measured = theta_true + bias + Gaussian angular noise
```

The UI specifies angular noise in degrees. The Python protocol receives radians.

### RF / RSSI

A simple log-distance model is used:

```text
RSSI(d) = P0 - 10 n log10(d/d0) + shadowing + bias
```

with `d0 = 1` simulator distance unit.

Editable parameters include:

- `P0`;
- path-loss exponent `n`;
- Gaussian shadowing/noise;
- bias;
- maximum communication/sensing range;
- dropout probability.

This is intended for teaching **RF-informed coordination and localization**, not for reproducing any one radio chipset or environment.

### Radar

Radar returns a noisy range-bearing pair:

```python
{
    "range": measured_range,
    "bearing": measured_bearing
}
```

It includes configurable:

- range noise;
- bias;
- finite detection range;
- dropout.

The model is intentionally lightweight so students can focus on estimation/control coupling rather than radar signal processing.

## About the noise defaults

The values preloaded in the UI are **illustrative classroom defaults**, not calibrated specifications for a particular sensor.

Students should change them to match their own hardware, experiment, or literature values.

This is especially important for:

- RF RSSI, which is highly environment dependent;
- camera/SLAM position estimates;
- low-cost ranging sensors;
- radar systems with different bandwidths and processing stacks.

## Protocol parameters

The Simulation tab exposes common parameters such as:

```text
k_position
k_velocity
max_speed
```

There is also a free-form JSON field for arbitrary user-defined parameters:

```json
{
  "leader_gain": 1.0,
  "avoid_gain": 1.8,
  "my_parameter": 4.2
}
```

These values become entries in the Python `params` dictionary.

This lets students write experiments without modifying the simulator internals.

## Example: distance/bearing formation controller

```python
import math

def control(robot_id, observations, params):
    k = params["k_position"]
    di = observations["desired_offset"]

    ux = 0.0
    uy = 0.0

    for neighbor in observations["neighbors"]:
        rp = neighbor.get("relative_position")

        if rp is None:
            d = neighbor.get("distance")
            b = neighbor.get("bearing")

            if d is None or b is None:
                continue

            rp = [
                d * math.cos(b),
                d * math.sin(b)
            ]

        dj = neighbor["desired_offset"]

        desired_relative = [
            dj[0] - di[0],
            dj[1] - di[1]
        ]

        ux += k * (rp[0] - desired_relative[0])
        uy += k * (rp[1] - desired_relative[1])

    return [ux, uy]
```

This is useful for showing that a controller need not receive global x/y positions if suitable relative measurements are available.

## Example: RF-aware interaction weight

Students can use RSSI to modify interaction weights:

```python
def control(robot_id, observations, params):
    k = params["k_position"]

    ux = 0.0
    uy = 0.0

    for n in observations["neighbors"]:
        rp = n.get("relative_position")
        rssi = n.get("rf_rssi")

        if rp is None or rssi is None:
            continue

        # Simple illustrative confidence rule.
        confidence = max(0.0, min(1.0, (rssi + 90.0) / 45.0))

        ux += k * confidence * rp[0]
        uy += k * confidence * rp[1]

    return [ux, uy]
```

This is not claimed as an optimal RF control law; it is intended as an editable teaching starting point.

## Suggested experiments

### Experiment 1 — graph structure vs convergence

1. Use the same initial robot positions.
2. Compare complete, ring, path, and custom sparse graphs.
3. Keep the protocol gain constant.
4. Compare error histories.

Questions:

- Why does topology affect convergence rate?
- What happens when the graph disconnects?
- What changes for directed graphs?

### Experiment 2 — weighted consensus

1. Start with a ring.
2. Open the graph editor.
3. Change selected weights from `1.0` to smaller or larger values.
4. Observe which interactions dominate.

### Experiment 3 — sensing assumptions

1. Disable absolute position.
2. Keep distance and bearing enabled.
3. Use the sensor-aware controller.
4. Increase bearing noise.

Questions:

- Which controller terms are most sensitive to angular error?
- How could a filter improve the observation?
- When is relative sensing preferable to global localization?

### Experiment 4 — RF-based coordination

1. Enable RF/RSSI.
2. Increase the path-loss exponent.
3. Add RSSI-dependent weighting in Python.
4. Move robots apart.

Questions:

- How does communication quality affect effective graph structure?
- What happens when RF measurements and graph adjacency disagree?
- How would fading or correlated shadowing alter the model?

### Experiment 5 — delayed / multi-rate sensing

Use the sensor update divider to conceptually separate control and sensor update rates.

Possible extension exercises:

- hold the previous measurement between sensor updates;
- add an explicit estimator;
- use different update rates for different sensors.

### Experiment 6 — leader–follower control

Load the leader–follower Python preset and drag robot 1.

Then edit the graph so only selected robots can observe the leader.

Discuss leader reachability and rooted spanning structures.

## GitHub Pages deployment

This project is still static.

Repository layout:

```text
multi-robot-control-playground-v2/
├── index.html
├── README.md
└── LICENSE
```

To deploy:

1. Create a GitHub repository.
2. Copy these files to the repository root.
3. Push to the `main` branch.
4. Go to **Settings → Pages**.
5. Choose **Deploy from a branch**.
6. Select `main` and `/ (root)`.
7. Save.

## Python runtime and Internet access

The simulator itself is self-contained.

The optional Python protocol editor loads Pyodide from:

```text
https://cdn.jsdelivr.net/pyodide/
```

Therefore:

- normal JavaScript simulation works offline after loading `index.html`;
- Python execution requires network access the first time Pyodide is loaded unless you vendor Pyodide yourself.

For a completely offline classroom deployment, download a Pyodide distribution into the repository and change the script URL in `index.html` to the local copy.

## Security model

Python code is executed in the student's browser through WebAssembly/Pyodide.

There is no application backend and student protocols are not sent to a server by this project.

For public deployments, standard browser security still applies. Do not extend the playground with privileged API keys or secrets embedded in client-side JavaScript.

## Limitations

The application is an educational simulator.

Important simplifications include:

- planar point robots;
- idealized kinematics/dynamics;
- no rigid-body orientation state;
- simple actuator saturation;
- abstract sensor models;
- no explicit packet scheduler;
- no realistic RF multipath;
- no radar signal processing;
- no SLAM estimator;
- no collision geometry;
- no safety controller;
- no formal verification of arbitrary user-written protocols.

It should not be used directly as a controller for physical robots.

## Good student extensions

1. Add exact graph Laplacian eigenvalues.
2. Implement push-sum for directed graphs.
3. Add dynamic average consensus.
4. Add consensus with communication delay.
5. Add asynchronous updates.
6. Add packet queues and bandwidth limits.
7. Add correlated RF shadowing.
8. Add UWB range measurements.
9. Add camera field-of-view constraints.
10. Add lidar range/bearing observations.
11. Add EKF or information-filter estimation.
12. Add bearing-only formation control.
13. Add rigidity-matrix metrics.
14. Add multi-leader containment control.
15. Add obstacle fields.
16. Add differential-drive/unicycle dynamics.
17. Add ROS 2 / rosbridge connectivity.
18. Reproduce the same controller in Gazebo or Isaac Sim.
19. Export simulation logs to CSV.
20. Add automated grading scenarios for a robotics course.


## Control Barrier Function safety layer

Version 3 adds an explicit **safety layer** between the nominal task controller and the robot dynamics.

The execution pipeline is:

```text
sensors
  ↓
nominal distributed controller
  ↓
built-in Control Barrier Function filter
  ↓
optional user-programmable Python safety_filter()
  ↓
speed saturation
  ↓
robot dynamics
```

This architecture is useful pedagogically because it separates:

- **performance / task objectives**, such as rendezvous or formation maintenance;
- **safety constraints**, such as collision avoidance and workspace containment.

### Pairwise collision CBF

For robots `i` and `j`, the default pairwise safety function is

```text
h_ij(p) = ||p_i - p_j||² - d_safe².
```

The safe set is

```text
h_ij >= 0.
```

For single-integrator dynamics, the playground enforces the standard zeroing-CBF condition

```text
h_dot + alpha h >= 0,
```

which becomes

```text
2 (p_i-p_j)^T (u_i-u_j) + alpha h_ij >= 0.
```

When the nominal controller violates this inequality, the browser safety filter modifies the robot commands.

### Boundary CBFs

The workspace uses four barrier functions, for example

```text
h_left   = x - margin
h_right  = (width-margin) - x
h_top    = y - margin
h_bottom = (height-margin) - y.
```

Each is enforced with the same form:

```text
h_dot + alpha h >= 0.
```

This keeps robots away from the configured boundary margin.

### Lightweight projection solver

A production CBF controller is often written as a quadratic program:

```text
minimize_u   ||u - u_nom||²

subject to   A_cbf u >= b_cbf.
```

To keep this project build-free and dependency-light, the browser implementation uses repeated projections onto the CBF half-spaces. This is suitable for teaching and interactive visualization, but it is **not a replacement for a robust QP solver** in physical-robot deployments.

Editable CBF parameters include:

- safe robot separation;
- workspace boundary margin;
- class-K / CBF gain `alpha`;
- number of projection iterations;
- relaxation `epsilon`;
- collision CBF enable/disable;
- boundary CBF enable/disable.

Active barrier constraints are highlighted in the simulation.

### Programmable Python safety hook

Students can add a second programmable safety stage:

```python
def safety_filter(robot_id, state, nominal_u, params):
    return nominal_u
```

The `state` object includes:

```python
{
    "id": robot_id,
    "position": [x, y],
    "velocity": [vx, vy],
    "all_positions": [[x0,y0], [x1,y1], ...],
    "dt": timestep
}
```

This can be used to explore:

- custom barrier functions;
- obstacle avoidance;
- speed-dependent safety envelopes;
- heterogeneous robot sizes;
- keep-in / keep-out regions;
- leader-specific safety logic;
- higher-order CBF ideas.

### Suggested CBF experiment

1. Load the formation controller.
2. Enable the pairwise CBF.
3. Use a very small desired formation spacing.
4. Set the CBF safe separation larger than that spacing.
5. Run the simulation.

Students should observe a direct conflict between the nominal formation objective and the safety constraint.

Discussion questions:

- Why can the formation controller no longer reach zero error?
- Which objective has priority?
- What does CBF relaxation mean physically?
- What happens when the safety constraints are mutually infeasible?
- How would a QP expose infeasibility?
- How should the controller change for second-order or unicycle dynamics?

### Important CBF limitation

The built-in collision barrier is derived for the single-integrator model. The playground will still apply the filter when double-integrator dynamics are selected so students can experiment, but that is not a formally correct high-order CBF treatment. A rigorous double-integrator implementation should use a relative-degree-two / higher-order CBF formulation.


## Fully programmable experiment pipeline

The playground now exposes four independent Python hooks:

```text
graph_neighbors()
       ↓
built-in sensor models
       ↓
sensor_filter()
       ↓
control()
       ↓
built-in CBF filter
       ↓
safety_filter()
       ↓
robot dynamics
```

### Dynamic graph hook

```python
def graph_neighbors(robot_id, world, params):
    return [[neighbor_id, weight], ...]
```

This can create distance-dependent, switching, directed, weighted, leader-rooted, or communication-quality-aware graphs.

### Sensor hook

```python
def sensor_filter(robot_id, observation, params):
    return observation
```

Use it for filtering, quantization, sensor fusion, custom dropout, outliers, spoofing, confidence weighting, or custom measurement models.

### Nominal control hook

```python
def control(robot_id, observations, params):
    return [ux, uy]
```

### Safety hook

```python
def safety_filter(robot_id, state, nominal_u, params):
    return [safe_ux, safe_uy]
```

## Arbitrary runtime parameters

The Protocol Editor contains a typed parameter table supporting:

- numbers;
- booleans;
- strings;
- JSON objects/arrays.

Students can introduce new controller or graph parameters without editing the simulator source.

## Heterogeneous robot sensing

Global sensor settings can be overridden per robot.

Example:

```json
{
  "0": {"pos": {"noise": 0.5}},
  "2": {"rf": {"on": true, "noise": 4.0, "drop": 0.1}}
}
```

The Sensor Lab provides editable illustrative presets for ideal sensing, motion-capture-like position, UWB-like ranging, Wi-Fi RSSI-heavy sensing, and short-range radar-heavy sensing.

The presets are not manufacturer specifications; all noise, bias, range, and dropout values remain editable.

## Multi-rate sensing

The sensor update divider is now functional. A value of `N` updates sensor measurements every `N` control steps and holds the last observation between updates.

This allows students to study mismatched sensor/control update rates.

## Scenario JSON import/export

A scenario can save:

- team size and dynamics;
- robot initial states;
- formation;
- typed controller parameters;
- weighted adjacency matrix;
- Python graph code;
- Python sensor code;
- Python controller code;
- Python safety code;
- sensor settings and per-robot overrides;
- CBF settings.

This makes classroom experiments reproducible and easy to distribute.

## License

MIT.
