<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>

[![LinkedIn][linkedin-shield]][linkedin-url]


<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://healthtech.unistra.fr/">
    <img src="images/logo.JPG" alt="Logo" width="720" height="120">
  </a>

  <h1 align="center">Design and Simulation of a Virtual Environment and Collision Model for the Haptic Device Pantograph</h1>

  <p align="center">
    <a href="https://github.com/Jesse-Alves?tab=repositories"><strong>View all repositories  »</strong></a>
    <br />
    <br />
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
<!--         <li><a href="#installation">Installation</a></li> -->
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
<!--     <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li> -->
    <li><a href="#contact">Contact</a></li>
<!--     <li><a href="#acknowledgments">Acknowledgments</a></li> -->
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

A project has been developed in ROS2 Humble environment that converges robotics and sensory perception through the lens of haptic technology. At the heart of this exploration lies the haptic device, particularly the Pantograph robot model, serving as a bridge between human interaction and virtual environments. Haptic devices facilitate tactile feedback, enabling users to interact with digital spaces in ways that simulate real-world sensations. This project involves the creation of a collision model and a meticulously designed virtual environment within RVIZ to facilitate the Pantograph's tactile exploration. Utilizing forward and inverse kinematics (FKM and IKM), the research investigates the detection of the robot's end-effector, a crucial aspect in understanding its spatial presence. The project's zenith lies in torque computation, orchestrating motor forces to simulate haptic feedback. This endeavor aims to transform digital interaction into a palpable, immersive experience by amalgamating algorithms and sensory perception.

Keywords: ROS2 Humble, Robotics, Haptic Technology, Pantograph Robot Model, Virtual Environments, RVIZ.

[(Project Presentation)](https://github.com/Jesse-Alves/Design-and-Simulation-of-a-Virtual-Environment-and-Collision-Model-for-the-Haptic-Device-Pantograph/blob/main/Haptic%20Loop%20-%20Final%20Presentation.pdf)



## Collision Model Simulation usign a Maze

<div align="center">
    <img width="600" src="images/gif1.gif" alt="color picker" />
</div>




## Robot Model in RVIZ

<div align="center">
  <a href=" ">
    <img src="images/img1.png" height="180" />
    <img src="images/img2.png" height="180" />
    <img src="images/img3.png" height="180" />
  </a>
</div>

## Torque Analysis

<div align="center">
  <a href=" ">
    <img src="images/img4.png" height="280" />
  </a>
</div>


<p align="right">(<a href="#readme-top">back to top</a>)</p>



### Built With
* ![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white) 
* ![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white) 
* ![ROS2](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
* ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
* ![LINUX](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black)


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- GETTING STARTED -->
## Getting Started

The Haptic loop is a collision model developed to be used in Haptic Device. It was developed in ROS2 Humble. 

### Prerequisites

All this project was built in ROS2 Humble environment, using ROS2 Control.



## The Project files

* The haptic_description 

This package describe the Pantograph robot in URDF file, following the dimentions used for High Cost group and the base frame from article: ["The Pantograph Mk-II: A Haptic Instrument, Gianni Campion, Student Member, IEEE, Qi Wang, Member, IEEE, and Vincent Hayward, Senior Member, IEEE"](https://github.com/Jesse-Alves/Design-and-Simulation-of-a-Virtual-Environment-and-Collision-Model-for-the-Haptic-Device-Pantograph/blob/main/The%20Pantograph%20Mk-II%20A%20Haptic%20Instrument.pdf)
 
  
* The haptic_bringup

This package lauch all the packages in ROS2. 
 
* The haptic_nodes

In this packages, inside of Scripts folder will be find:
 
   I - "pseudo_trajectory.py" - A publisher to simulate the movement of the robot from the user. This node publish 
   	the angles theta1 and theta5, in order to simulate what really happend with the encoders sensors read by 
   	a microcontroller.
   	
   II - "markers.py" - A node to publish markers message to show the maze spheres in RVIZ.
   
   III - "create_random_maze.py" 
   	- It is a simple python code that generate a random maze in collision model. 
   	- It is possible change this maze changing the variables: "w" - width, "h" - height, 
   	  "workspace" - the cordinates of the robot workspace corner.
   	- Also, it is possible, increase or decrease the number of sphere, by changing the factor that multiply the radius in the 
   	  loops (convert_maze function).
   	- This file generate a csv file.
   	  
   IV - "haptic_loop.py" This is the haptic loop itself. It is a node that read the encoder angles topics, import a csv file with the 
   	spheres information (radius, center x, center y), compute the distance between the robot end-effector and the spheres, in case of
   	collision, compute the force of the collision (based on spring damper model), compute the motors torque necessary to produce this force and publish this torque via topic.
 
 
* Simulations in ROS bag
 
 - Some important simulations were recorded using ROS bag, i.e., the topics in the simulation. It is possible to run this simulation using Plot Juggler.


<!-- USAGE EXAMPLES -->
## Usage

1 - Unzip the file in ubuntu system.

2 - Open the terminal in ubuntu inside of /src directory.

3 - Run the code below to test.

`colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install && source install/setup.bash && ros2 launch haptic_bringup haptic.launch.py`


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Jessé de Oliveira Santana Alves - [Linkedin - jessealves11](https://linkedin.com/in/jessealves11) - jessalves2@gmail.com

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/jessealves11
