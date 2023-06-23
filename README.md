# ros2_benchmark

A reference implementation of **non-functional black-box performance benchmarking** :black_circle: for ROS 2 used in [RobotPerf project](https://robotperf.org). See https://github.com/robotperf for all the source code.

### Approaches to benchmark performance

```
             Probe      Probe
             +            +
             |            |
    +--------|------------|-------+     +-----------------------------+
    |        |            |       |     |                             |
    |     +--|------------|-+     |     |                             |
    |     |  v            v |     |     |        - latency   <--------------+ Probe
    |     |                 |     |     |        - throughput<--------------+ Probe
    |     |     Function    |     |     |        - memory    <--------------+ Probe
    |     |                 |     |     |        - power     <--------------+ Probe
    |     +-----------------+     |     |                             |
    |      System under test      |     |       System under test     |
    +-----------------------------+     +-----------------------------+


              Functional                            Non-functional


    +-------------+                     +----------------------------+
    | Test App.   |                     |  +-----------------------+ |
    |  + +  +  +  |                     |  |    Application        | |
    +--|-|--|--|--+---------------+     |  |                   <------------+ Probe
    |  | |  |  |                  |     |  +-----------------------+ |
    |  v v  v  v                  |     |                            |
    |     Probes                  |     |                      <------------+ Probe
    |                             |     |                            |
    |       System under test     |     |   System under test        |
    |                             |     |                      <------------+ Probe
    |                             |     |                            |
    |                             |     |                            |
    +-----------------------------+     +----------------------------+


             Black-Box                            Grey-box


```

### Other past approaches to non-functional black-box performance benchmarking

- [ros2_benchmarking](https://github.com/piappl/ros2_benchmarking/) : First implementation available for ROS 2, aimed to provide a framework to compare ROS and ROS 2 communications.
- [performance_test](https://gitlab.com/ApexAI/performance_test/): Tool is designed to measure inter and intra-process communications. Runs at least one publisher and at least one subscriber, each one in one independent thread or process and records different performance metrics. It also provides a way to generate a report with the results through a different package.
- [reference_system](https://github.com/ros-realtime/reference-system/): Tool designed to provide a framework for creating reference systems that can represent real-world distributed systems in order to more fairly compare various configurations of each system (e.g. measuring performance of different ROS 2 executors). It also provides a way to generate reports as well.
- [ros2-performance](https://github.com/irobot-ros/ros2-performance/): Another framework to evaluate ROS communications and inspired on `performance_test`. There's a decent rationale in the form of a proposal, a good evaluation of prior work and a well documented set of experiments.
- [system_metrics_collector](https://github.com/ros-tooling/system_metrics_collector/): A lightweight and *real-time* metrics collector for ROS 2. Automatically collects and aggregates *CPU* % used and *memory* % performance metrics used by both system and ROS 2 processes. Data is aggregated in order to provide constant time average, min, max, sample count, and standard deviation values for each collected metric. *Deprecated*.
- [ros2_latency_evaluation](https://github.com/Barkhausen-Institut/ros2_latency_evaluation/): A tool to benchmarking performance of a ROS 2 Node system in separate processses (initially focused on both inter-process and intra-process interactions, later focused). Forked from `ros2-performance`.
- [ros2_timer_latency_measurement](https://github.com/hsgwa/ros2_timer_latency_measurement/):  A minimal *real-time safe* testing utility for measuring jitter and latency.  Measures nanosleep latency between ROS child threads and latency of timer callbacks (also within ROS) across two different Linux kernel setups (`vanilla` and a `RT_PREEMPT`` patched kernel).
- [buildfarm_perf_tests](https://github.com/ros2/buildfarm_perf_tests/): Tests which run regularly on the official ROS 2 buildfarm. Formally, extends `performance_test` with additional tests that measure additional metrics including CPU usage, memory, resident anonymous memory or virtual memory.