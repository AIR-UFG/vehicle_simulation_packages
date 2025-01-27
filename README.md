# Vehicle Simulation Packages

This repository contains all the ROS packages related to simulation, description, and models of the world for vehicle simulations.

## Repository Structure

```
vehicle_simulation_packages
├── air_description
├── air_docs
├── air_sim
└── vehicle_control_plugin
```

### Packages

- **air_description**: Contains the URDF files, meshes, and resources for describing the air vehicle model.
- **air_docs**: Documentation related to the air vehicle simulation and description.
- **air_sim**: Contains the configuration, launch files, models, and worlds for simulating the air vehicle in a virtual environment.
- **vehicle_control_plugin**: Contains the control plugins for managing vehicle behavior in the simulation.

### 3D-Reconstructed Simulation World

Introduces a new Gazebo simulation world created from 3D reconstructions of real-world environments, using [SpectacularAI](https://github.com/SpectacularAI/sdk-examples/tree/main/python/mapping) and [Nerfstudio](https://github.com/nerfstudio-project/nerfstudio#1-installation-setup-the-environment).

The new rescaled models are:
- estacionamento_2
- estacionamento_3
- uno
- arvore_1
- arvore_5
- carro_branco
- carro_cinza

To launch the simulation in the newly reconstructed world, run the following command:

```bash
./utils/run.sh ROBOT_POSE="6.0,-1.0,0.3,0.0,0.0,0.0" GPU=true RVIZ=true WORLD_NAME=size_test_03.world
```

---
---

