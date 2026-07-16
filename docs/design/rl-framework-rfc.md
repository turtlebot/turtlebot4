# RFC: backend-independent reinforcement learning for TurtleBot 4

- Status: proposed for maintainer discussion
- Target: ROS 2 Jazzy
- Scope: architecture and repository ownership

## Summary

This RFC proposes a small, backend-independent reinforcement-learning (RL) layer for
TurtleBot 4. It separates task and algorithm code from ROS communication, simulator
control, and physical-robot safety. The immediate request is maintainer guidance on
whether the shared interfaces and core abstractions belong in this repository, while
simulator and learning-algorithm implementations live elsewhere.

A working community-fork prototype exists to make the proposal reviewable, but this RFC
does not ask to merge that implementation.

## Problem

An RL experiment that combines neural networks, rewards, ROS callbacks, and simulator
reset calls is difficult to test deterministically and difficult to move safely between
a simulator and a robot. The TurtleBot 4 common repository also should not require every
user to install a machine-learning framework or Gazebo.

The proposal therefore treats algorithms, task semantics, robot I/O, and world control
as separate components connected through typed ports.

## Goals

- Provide a Gymnasium-style `reset`/`step` environment with separate `terminated`
  and `truncated` results.
- Keep the algorithm and task core independent of ROS, Gazebo, and deep-learning
  frameworks.
- Support deterministic no-ROS tests, standard ROS topics, and Gazebo Harmonic through
  replaceable adapters.
- Define explicit command watchdog, sensor-freshness, and zero-stop behavior for
  physical-robot evaluation.
- Preserve the behavior and dependency footprint of existing TurtleBot 4 packages.
- Record reproducible seeds, configuration, revision, outcomes, and timing metrics.

## Non-goals

This proposal does not promise an optimal policy, distribute a trained model, enable
unsupervised physical-robot training, or claim simulator-to-real parity. PPO, SAC,
distributed training, perception models, and broad domain randomization are future
work. DQN is only a small reference algorithm used to validate the boundaries.

## User scenarios

A contributor should be able to:

- test task observations, rewards, and termination in milliseconds with a deterministic
  Mock backend;
- train against Gazebo without changing the algorithm;
- evaluate a frozen policy over multiple seeds;
- add another simulator by implementing the world and robot ports; and
- connect the standard ROS adapter to a physical robot under human supervision.

## Proposed architecture

Dependencies point inward: algorithms depend on the pure-Python core, while Mock, ROS,
and Gazebo packages implement ports owned by that core. An algorithm never imports a
backend.

The main abstractions are:

- `RobotBackend`: obtains sensor samples, executes velocity commands, stops, and
  closes;
- `WorldBackend`: resets episode state and manages a goal;
- task environment: builds observations, rewards, and termination results from the
  backends;
- algorithm: consumes the environment API without knowing which backends are active;
  and
- configuration/registry: validates dimensions and selects backend pairs without
  hard-coded imports.

The API follows current Gymnasium semantics:

- `reset(seed, options) -> (observation, info)`
- `step(action) -> (observation, reward, terminated, truncated, info)`

Goal and collision are task termination. Time limits, stale sensors, and backend faults
are truncation. Observation and action dimensions are derived from validated
configuration instead of constants.

## Proposed package ownership

The following split is intentionally open for maintainer direction:

| Concern | Proposed home | Required dependency profile |
| --- | --- | --- |
| Typed ports and environment core | this repository or a dedicated RL repository | Python and NumPy only |
| Optional ROS interfaces | this repository if generally useful | ROS interface packages only |
| Standard ROS robot adapter | this repository or a dedicated RL repository | standard ROS messages |
| Gazebo world adapter | `turtlebot4_simulator` | Gazebo Harmonic integration |
| Mock backend and contract tests | alongside the core | no ROS runtime |
| DQN reference | community examples or a dedicated RL repository | optional learning dependency |

Keeping the core here would make the TurtleBot 4 interfaces discoverable, but a
dedicated repository would isolate release cadence and experimental dependencies. The
proposal does not assume either decision.

## ROS 2 integration

Robot I/O consumes `sensor_msgs/LaserScan` and `nav_msgs/Odometry`, and publishes
`geometry_msgs/TwistStamped`. Topic names, frames, freshness limits, and command
watchdogs are configuration. Optional episode-management interfaces must describe
integration state without encoding an algorithm such as DQN.

Gazebo owns pause, reset, and entity operations. The standard ROS adapter owns robot
topics and safe stopping. Mock implements both port sets over one deterministic state.

Reset and step must reject observations whose source timestamps predate the reset or
action boundary. This prevents a simulator restart or delayed DDS message from being
mistaken for a new transition.

## Physical-robot safety boundary

Physical mode never teleports the robot or resets physics. Stale sensors, command
timeouts, clock jumps, exceptions, episode completion, and adapter shutdown all produce
a zero-velocity command.

Physical validation is staged and supervised:

1. sensor-only validation;
2. zero-command validation;
3. one low-speed action;
4. short supervised episodes; and
5. fixed-goal evaluation with an accessible emergency stop.

Simulator or Mock success cannot be reported as physical-robot validation.

## Dependencies and configuration

The core should require only Python and NumPy. A reference NumPy network is sufficient
for contract testing; larger learning frameworks must remain optional and lazily
loaded. ROS and Gazebo dependencies stay in adapter packages.

Observation bins and ranges, commands, thresholds, topics, entity names, timeouts,
goals, randomization allow-lists, and algorithm hyperparameters are validated
configuration values. Checkpoints include a versioned configuration fingerprint and
reject incompatible observation or action shapes.

## Test and evidence strategy

Pure unit tests cover geometry, scan processing, actions, rewards, termination, replay,
epsilon schedules, learning, and checkpoints. Contract-style Mock tests cover reset,
stop, faults, close, repeated episodes, and equal-seed determinism. ROS conversions are
tested as pure functions where possible.

Environment-dependent gates remain explicit:

- headless Gazebo repeated resets require fresh scan and pose samples;
- low-speed command motion must end with a confirmed zero-stop; and
- physical-robot tests require hardware and human supervision.

The community prototype has exercised 1,000 deterministic Mock episodes and 100
timestamp-valid Gazebo Harmonic resets. Those results demonstrate feasibility only;
they are not upstream test evidence and do not replace maintainer review.

Evaluation records repository revision, configuration hash, environment version, seed
set, reward, success, collision, timeout, steps, elapsed time, path length, SPL,
training duration, and inference latency.

## Licensing and provenance

Proposed files use Apache-2.0 and preserve existing notices. TurtleBot3 examples informed
the problem statement, but no TurtleBot3 machine-learning source is copied. Any future
derived file must retain its original copyright and license and record its source
revision and modifications.

## Suggested pull-request sequence

If maintainers support the direction, follow-up changes should stay independently
reviewable:

1. accepted RFC and architecture decision records;
2. interfaces, core, Mock, and contract tests;
3. standard ROS adapter and safety behavior;
4. optional DQN reference;
5. Gazebo adapter in the simulator repository, if requested; and
6. tutorials, metrics, and environment-dependent evidence.

## Alternatives considered

A monolithic ROS/Gazebo training node is initially smaller but cannot be tested without
its runtime. A simulator-specific Gym environment duplicates algorithm and task
behavior. Making a deep-learning framework mandatory increases installation and CI
cost. A generic robotics RL framework expands scope beyond TurtleBot 4.

## Questions for maintainers

- Do shared core abstractions and optional ROS interfaces belong in this repository or
  in a dedicated TurtleBot 4 RL repository?
- Should all Gazebo integration live exclusively in `turtlebot4_simulator`?
- Is a small reference algorithm useful upstream, or should algorithms remain entirely
  in community examples?
- Which plugin-discovery mechanism and metrics schema would be acceptable for a stable
  public API?
- Which reference world, configuration, and seed set should define simulator
  regressions?
