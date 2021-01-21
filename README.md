# UAV-Simulation-OpenGL-OpenMPI
A model UAV simulation using OpenGL and OpenMPI, which utilizes and implements some basic laws of Kinetic Physics to demonstrate an UAV simulation.

Description:
- Each UAV is instantiated on a separate node on our cluster. All kinds of physical calculation is handled by that particular node associated with the uav.
- OpenGL is used to render the graphics (UAV positions, movement, football field etc.) and openMPI is used to handle the distributed calculations.
- Equations from physics are implemented for each UAV, such as speed, velocity, acceleration etc. In each time-step all these properties of motion are calculated for the UAV, and they are updated each time-step to generate motion.
- The UAV models start from different positions in a football field and rise up towards the surface of an imaginary sphere mid-air. 
- Hooke's law is used to mathematically create an attraction to the surface of the sphere. As a result we can observe a 'spring-like' behavior from the UAVs in relation to the surface of the imaginary sphere.
- Once the UAV settles on the surface of the sphere, thrust is applied from the uavs.
- Cetripetal and centrifugal forces are calculated to keep the uavs on the surface of the sphere.  

Small Demonstration:

![](uav_sim.gif)
