# Autonomous Network Self-Healing with Drones
To build and run the NS-3 simulator with the autonomous network self-healing
drones application, use the following commands:

```bash
docker build -t simulator-ns3:dev -f simulator/Dockerfile .
docker run --rm -v $(pwd)/output:/output simulator-ns3:dev
```

This will produce the simulation output files in the `output` directory.
To visualize the simulation, you can use the NetAnim tool included in NS-3
(follow the instructions in the NS-3 documentation available at [https://www.nsnam.org/docs/release/3.46/models/html/animation.html](https://www.nsnam.org/docs/release/3.46/models/html/animation.html)).