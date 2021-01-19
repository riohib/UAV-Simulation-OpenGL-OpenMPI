# UAV-Simulation-OpenGL-OpenMPI
A model UAV simulation using OpenGL and OpenMPI, which utilizes and implements some basic laws of Kinetic Physics to demonstrate an UAV simulation.

Description:
- Each UAV is instantiated on a separate node on our cluster. All kinds of physical calculation is handled by that particular node associated with the uav.
- Equations from physics are implemented for each UAV, such as speed, velocity, acceleration etc.
- The uav models start from different positions in a football field and rise up towards the surface of a imaginary sphere mid-air. 
- Hooke's law is used to mathematically create an attraction to the surface of the sphere.
- Once the UAV settles on the surface of the imaginary sphere, thrust is applied from the uavs.
- Cetripetal and centrifugal forces are calculated to keep the uavs on the surface of the sphere.

Small Demonstration:

![](uav_sim.gif)
