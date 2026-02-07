# Autonomous Network Self-Healing with Drones

A self-healing protocol for drone swarms that autonomously detects loss of connectivity with a base station and reorganizes to form multi-hop relay chains, restoring network integrity without centralized control.

## Overview

This project implements a decentralized self-healing protocol where drones in a swarm can detect communication failures and cooperatively reposition themselves to maintain connectivity with a base station. The system uses:

- **ACK-based reachability detection** to identify connectivity loss
- **Flooding-based hop discovery** to establish distance metrics to the base station
- **Periodic neighbor broadcasts** for local situational awareness
- **Virtual spring-damper formation control** for coordinated repositioning

When a drone loses direct communication with the base station (detected via missing acknowledgments), it broadcasts a `HELP_PROXY` request. Neighboring drones then activate their formation controllers, repositioning to form a multi-hop relay chain that restores end-to-end connectivity.

## Architecture

### System Components

The architecture is designed around two node types:

| Component | Description |
|-----------|-------------|
| **Base Station** | Stationary node that tracks drone positions, sends acknowledgments, and triggers periodic floods |
| **Drones** | Mobile agents running the same control stack with unicast (to base) and broadcast (swarm) communication |

### Module Structure

```
├── apps/                          # Executable applications
│   ├── help_proxy_sim.cpp         # Main simulation scenario
│   └── controller_tuner.cpp       # Parameter tuning grid search
├── common/                        # Shared data structures
│   ├── messages.h                 # Protocol message definitions
│   ├── packet.h                   # Packet envelope format
│   └── vector3D.h                 # 3D vector utilities
├── interfaces/                    # Abstract interfaces for modularity
├── modules/
│   ├── communication/             # Packet packing/unpacking & transport delegation
│   ├── controller/                # Virtual spring-damper formation control
│   ├── dispatch/                  # Message routing to protocol handlers
│   ├── flood/                     # Hop discovery via flooding protocol
│   └── neighbor/                  # Local neighbor state management
└── platform/ns3/                  # NS-3 specific implementations
    ├── base_station/              # NS-3 base station node logic
    ├── custom_mobility/           # Kinematic mobility model
    ├── drone/                     # NS-3 drone node logic
    ├── position/                  # NS-3 position interface
    ├── radio_environment/         # WiFi ad-hoc network configuration
    ├── transport/                 # UDP socket transport
    └── velocity_actuator/         # Velocity command application
```

### Communication Flow

1. **CommunicationManager** handles packet serialization and transport
2. **DispatchManager** routes packets to appropriate handlers:
   - `FloodManager` → Hop discovery floods
   - `NeighborManager` → Neighbor state updates
   - Node Logic → Position updates, ACKs, and help requests
3. **Controller** computes motion commands using the virtual spring-damper model

## Self-Healing Protocol

### Formation Control

The controller uses a virtual spring-damper model with two force components:

**Attractive Forces** (toward relay neighbors):
```
F_att = K_att × (p_neighbor - p_self)   for neighbors with different hop counts
```

**Repulsive Forces** (collision avoidance):
```
F_rep = K_rep × (1/dist²) × unit_vector   when dist < D_safe
```

This drives drones toward the geometric midpoint between their preceding and following neighbors in the relay chain, maximizing SNR for both links.

### Key Parameters

| Parameter | Symbol | Description | Default |
|-----------|--------|-------------|---------|
| Coverage radius | R_max | Base station radio range | 50 m |
| Attractive gain | K_att | Spring constant for attraction | 1.0 |
| Repulsive gain | K_rep | Repulsion strength | 8.0 |
| Safety distance | D_safe | Minimum separation threshold | 2.0 m |
| Max velocity | V_max | Drone speed limit | 1.0 m/s |
| Drone mass | m | Based on Crazyflie 2.1 | 29 g |

## Simulation

The simulation runs in **NS-3** (Network Simulator 3) with:
- IEEE 802.11b WiFi in ad-hoc mode
- UDP transport on a single port
- Range-based propagation loss model
- Custom kinematic mobility model

### Default Scenario

- 1 base station at origin (0, 0, 0)
- 3 drones at (40, 15, 0), (60, 20, 0), and (30, 25, 0)
- Drone 2 starts outside coverage (>50m) and triggers self-healing
- Simulation duration: 150 seconds

## Building and Running

### Using Docker (Recommended)

Build the Docker image:

```bash
docker build -t simulator-ns3:dev -f Dockerfile .
```

Run the simulation:

```bash
docker run --rm -v $(pwd)/output:/output simulator-ns3:dev sim
```

This produces simulation output files in the `output` directory, including a NetAnim XML trace.

### Simulation Parameters

Pass custom parameters via command line:

```bash
docker run --rm -v $(pwd)/output:/output simulator-ns3:dev sim \
  --kAtt=1.0 --kRep=8.0 --dSafe=2.0 --vMax=1.0 \
  --maxRangeMeters=50.0 --simSeconds=150.0
```

### Parameter Tuner

Run the grid search tuner to optimize controller parameters:

```bash
docker run --rm -v $(pwd)/output:/output simulator-ns3:dev tuner
```

Tuner options:

```bash
docker run --rm -v $(pwd)/output:/output simulator-ns3:dev tuner \
  --kAttMin=0.5 --kAttMax=2.0 --kAttStep=0.5 \
  --kRepMin=4.0 --kRepMax=12.0 --kRepStep=2.0 \
  --dSafeMin=1.0 --dSafeMax=3.0 --dSafeStep=0.5 \
  --simSeconds=150.0
```

## Visualization

To visualize the simulation, use the **NetAnim** tool included in NS-3:

1. The simulation generates `output/drone-simulation.xml`
2. Open with NetAnim following the [NS-3 animation documentation](https://www.nsnam.org/docs/release/3.46/models/html/animation.html)

## Results

With default parameters, the system achieves:

| Metric | Value |
|--------|-------|
| Average oscillation | ~0.09 m |
| Average minimum inter-drone distance | 1.46 m |
| Average time to target position | 13.12 s |
| Connection restoration time | 1.67 s |

## Authors

- Gianluca Bresolin (University of Padua)
- Riccardo Fabbian (University of Padua)

### Supervisor

- Federico Corò (University of Padua)

## References

This work is inspired by research on virtual spring-damper formation control and the Crazyflie 2.1 drone platform. See the technical report in `report/` for detailed methodology and analysis.

## License

University of Padua - Advanced Topics in Communication Networks and Systems