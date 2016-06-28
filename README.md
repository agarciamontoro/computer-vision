# Computer Vision

Assignments and projects related to the Computer Vision course, Double degree of Computer Science and Mathematics, University of Granada.

## What is this?

This repo contains some assignments and exercises (all of them in spanish) on Computer Vision stuff. Its mainly divided into three parts:

* **Project 1**: The first project is an introduction to the OpenCV world, its functions and the very first definitions and behaviour of basic Computer Vision concepts. You will find cool stuff like the following images in the documentation of the assignment:
<p align="center">
<img src="https://cloud.githubusercontent.com/assets/3924815/16422808/ffe26bd8-3d59-11e6-9a7f-8a9fb4930fca.png" width="61%" alt="Gaussian pyramid"/> <img src="https://cloud.githubusercontent.com/assets/3924815/16422826/0d16369a-3d5a-11e6-8fa8-f07dbe868aca.png" width="38%" alt="Border detection"/>
</p>

* **Project 2**: The second project make a deeper study in the world of homographies and detection of features, obtaining applications such as an automatic mosaic composer:
<p align="center">
<img src="https://cloud.githubusercontent.com/assets/3924815/16422931/669f061a-3d5a-11e6-904d-64079f2f0eea.png" width="40.5%" alt="Detection of features"/> <img src="https://cloud.githubusercontent.com/assets/3924815/16422946/74479d72-3d5a-11e6-8ef9-b52becc48394.png" width="59%" alt="Mosaic"/>
</p>

* **Project 3**: This final assignment is centered in camera related stuff, geometry and [image rectification](https://github.com/agarciamontoro/image-rectification). This project is quite more theoretical, but its results are also quite cool :)
<p align="center">
<img src="https://cloud.githubusercontent.com/assets/3924815/16423087/f6ab4cfa-3d5a-11e6-8ada-d7d55c1849e2.png" width="39%" alt="Detection of features"/> <img src="https://cloud.githubusercontent.com/assets/3924815/16423113/0d2ea698-3d5b-11e6-9676-f268341b2c94.png" width="60%" alt="Mosaic"/>
</p>

## How is this done?

The magic in the code of this repo relies on the awesome OpenCV library, that provides the user with really cool tools to develop computer vision algorithms.

The code itself follows an object-oriented pattern, with two main classes: [Camera](https://github.com/agarciamontoro/computer-vision/blob/master/inc/camera.hpp) and [Image](https://github.com/agarciamontoro/computer-vision/blob/master/inc/image.hpp). The main file uses the objects and methods defined in those headers to make the magic happen :)
