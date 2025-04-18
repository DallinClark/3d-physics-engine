# Realtime 3D Physics Engine

<div style="display: flex; justify-content: space-around;">
  <img src="https://github.com/user-attachments/assets/9faffb4e-b64d-4a63-9541-43514fab38b5" style="width: 45%;" alt="Image 1" />
  <img src="https://github.com/user-attachments/assets/299fafda-a7f6-4675-a165-bc8c83f3e575" style="width: 45%;" alt="Image 2" />
  <img src="https://github.com/user-attachments/assets/891e4c9e-72bf-465c-b6cf-0616051abdd6" style="width: 45%;" alt="Image 3" />
  <img src="https://github.com/user-attachments/assets/11276ce2-53d7-4060-983f-8140b2075ea9" style="width: 45%;" alt="Image 4" />
</div>



## Overview



This project is a simulation tool real-time rigid body dynamics. Developed in C++ and rendered with OpenGL.

Experience the engine in motion:

<a href="https://www.youtube.com/watch?v=51z4WZ5UAGE" style="font-size: 48px;">Full Video Demo</a>


## Features

- **Rigid Body Dynamics:**  
  Simulates 3D objects with variable properties\
  Adjustable density, dimensions, resitution(bounciness), initial velocity, and initial transforms per object

- **Dynamic Gravity:**  
  Customizable gravity strength and direction.

- **Static Objects:**  
  Supports unmovable objects with infinite mass

- **Impulse-Based Collision Resolution:**  
  Realistic collision responses computed via impulse methods with explicit Euler integration

- **Convex Shape Support:**  
  The engine supports collision detection for any convex shape including spheres, polygons, and custom geometries

- **Broad and Narrow Phase Collision Detection:**   
  Axis allgned boudning boxes are used for the narrow phase, and the Seperating Axis Therom is implemented for the narrow phase.\
  Sutherman-Hodgeman clipping is used to detect colisions points

## Acknolowdgements

Thanks to "Two-Bit Coding" and "blackedout01" on Youtube for physics tutorials.\
Thanks to Chris Heckler for his artiles on RBD simulation. https://www.chrishecker.com/Rigid_Body_Dynamics 




