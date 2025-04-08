# Realtime 3D Physics Engine

![Banner Image](path/to/banner-image.png)  
*Insert a banner image representing the physics engine (optional)*

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

Showcase your engine in action with images:

- **Example Screenshot:**  
  ![Screenshot](path/to/screenshot.png)  
  *Replace with relevant images displaying the simulation environment.*

## Video Demo

Experience the engine in motion:

[![Video Demo](path/to/video_thumbnail.png)](https://www.youtube.com/watch?v=your_demo_video_link)  
*Replace the thumbnail image and video link with your actual demo details.*

## Acknolowdgements

Thanks to "Two-Bit Coding" and "blackedout01" on Youtube for physics tutorials.
Thanks to Chris Heckler for his artiles on RBD simulation. https://www.chrishecker.com/Rigid_Body_Dynamics 
