A Collaborative Robot Cell for Unstructured Bin-Picking
======
By Albert Sonne Olesen, Benedek Benjamin Gergaly, Emil Albin Ryberg and Mads Riis Thomsen.

GitHub repository for group 663 of the 6th semester 2020 on the Robotics education of Aalborg University.
------
This repository contains all the **_simulation_** code related to the Bachelor project of group 663: A Collaborative Robot Cell for Unstructured Bin-Picking:
Based on Deep Learning Policies Designed for Assembly of Phones. 
![alt text](https://github.com/EmilRyberg/P6BinPickingSimulation/blob/master/textures/setup_img.jpg "Virtual environment setup")

The GitHub repository with documentation for the controller and bin-picking logic can be found in [this repository](https://github.com/EmilRyberg/P6BinPicking "P6 - Bin-picking repository").

The project is based on the previous work, done by the group in the 5th semester, of [performing assembly of dummy phones from 2D table-picking](https://www.youtube.com/watch?v=oPsAurclCmY "P5 - 2D Table-picking") ([5th semester GitHub](https://github.com/EmilRyberg/P5BinPicking)).

The project aims to use a UR-5 collaborative robot to perform bin-picking of the dummy phone parts by Festo, to then perform an assembly of the phone.
The project was carried out during Covid-19 and thus access to a physical robot environment was restricted and the project had to be conducted in a virtual environment.

The simulation software used is [Webots](https://cyberbotics.com/).

### Dependencies
The libraries required to run all code in the repository are listed below:
```
pillow
numpy
opencv-python
opencv-contrib-python
imutils
scipy
pytorch
detectron2
terminaltables
matplotlib
pyrealsense2 (not needed from simulation)
urx (not needed from simulation)
```
