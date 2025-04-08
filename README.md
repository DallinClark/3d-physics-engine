# Realtime 3D Physics Engine

![Banner Image](![image](https://github.com/user-attachments/assets/26add229-bdab-4f4f-baab-092169b9dd7f))  

## Overview

This project is a simulation tool real-time rigid body dynamics. Developed in C++ and rendered with OpenGL.

## Features

- **Rigid Body Dynamics:**  
  Simulates 3D objects with properties like mass, velocity, and rotation.

- **Dynamic Gravity:**  
  Customize gravity strength and direction.

- **Impulse-Based Collision Resolution:**  
  Realistic collision responses computed via impulse methods ensure objects behave naturally upon impact.

- **Bounciness and Density:**  
  Fine-tune restitution (bounciness) and density for each object.

- **Convex Shape Support:**  
  The engine supports collision detection for any convex shape including spheres, polygons, and custom geometries.

- **Broad and Narrow Phase Collision Detection:**  
  Axis allgned boudning boxes are used for the narrow phase, and the Seperating Axis Therom is implemented for the narrow phase. Sutherman-Hodgeman clipping is used to detect colisions points.

## Screenshots

## Video Demo

Experience the engine in motion:

[![Video Demo](![image](https://github.com/user-attachments/assets/f1f3a720-3e13-48c1-9373-dfe694f22643)
)]((https://www.youtube.com/watch?v=YO55TEJMftM))  

## Acknolowdgements

Thanks to "Two-Bit Coding" and "blackedout01" on Youtube for physics tutorials.
Thanks to Chris Heckler for his artiles on RBD simulation. https://www.chrishecker.com/Rigid_Body_Dynamics 
