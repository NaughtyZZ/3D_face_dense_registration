# Towards Fine-Grained Optimal 3D Face Dense Registration: An Iterative Dividing and Diffusing Method (IJCV2023)

by Zhenfeng Fan, Silong Peng, and Shihong Xia

## Introduction

This repository is built for the official implementation of the paper "Towards Fine-Grained Optimal 3D Face Dense Registration: An Iterative Dividing and Diffusing Method" published at *International Journal of Computer Vision* in June, 2023.
To view the full version of this paper, please visit the website [https://doi.org/10.1007/s11263-023-01825-7](https://doi.org/10.1007/s11263-023-01825-7), which provides the online version.

## Abstract
Dense vertex-to-vertex correspondence (i.e. registration) between 3D faces is a fundamental and challenging issue for 3D &2D face analysis. While the sparse landmarks are definite with anatomically ground-truth correspondence, the dense vertex correspondences on most facial regions are unknown. In this view, the current methods commonly result in reasonable but diverse solutions, which deviate from the optimum to the dense registration problem. In this paper, we revisit dense registration by a dimension-degraded problem, i.e. proportional segmentation of a line, and employ an iterative dividing and diffusing method to reach an optimum solution that is robust to different initializations. We formulate a local registration problem for dividing and a linear least-square problem for diffusing, with constraints on fixed features on a 3D facial surface. We further propose a multi-resolution algorithm to accelerate the computational process. The proposed method is linked to a novel local scaling metric, where we illustrate the physical significance as smooth adaptions for local cells of 3D facial shapes. Extensive experiments on public datasets demonstrate the effectiveness of the proposed method in various aspects. Generally, the proposed method leads to not only significantly better representations of 3D facial data, but also coherent local deformations with elegant grid architecture for fine-grained registrations.

### Some significant advantages over the existing methods are: 
1）The gridded cells of the registered target are distributed **as uniformly as possible** referring to a certain template.

2）The landmarks of the deformed template and the target are matched **exactly**. 

3）The surfaces of the deformed template and the target are also adhered to each other **tightly** to any desired extent.

4）The implementation of the algorithm is  **very fast** and **robust to partial target data**.


## Usage

### C++ Single Resolution Version from Scratch (**Recommended**)
We provide the original **C/C++** codes which cover the implementations of the proposed method in its single-resolution version **from scratch**. The inputs are the template and the raw scanning target data. I suggest that these codes can be compiled with the newest C/C++ environments.

An executable demo of registration process is achieved by running the code:
```
.\VC++(from scratch)\demo\faceDenseRegistration.exe
```
A challenging example with **large expressions** and **partial data** is shown as follows: 

<img src="figures\Registration_example.png" alt="registration_example" style="zoom: 50%;width:50%;height:auto;" />

### C++ Single-and-Multiple-Resolution Version with initializations by the NICP Method
We provide the original verison with **C/C++** code, which was compiled with **Visual Studio 2015** in our experiments. These codes cover the implementations of the proposed method in both its single-resolution and MR versions. The input targets are initialized by the NICP method (Amberg et al. 2007). I suggest that these codes can be compiled with the newest C/C++ environments.

An executable demo of registration process is achieved by running the code:
```
.\VC++(with initializations by NICP)\demo\faceDenseRegistration.exe .\plyInput\ .\plyOutput\ 
```
#### Dependencies for C++ Versions

[the libigl Library](https://libigl.github.io/)

[the Intel Math Kernal Library](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-math-kernel-library-intel-mkl-compiling-and-linking-with-microsoft-visual-cc.html)

[the Eigen Library (a copy already pasted here)](https://eigen.tuxfamily.org/index.php?title=Main_Page)


### Matlab Version 
We also provide a **simple** and **stand-alone** implementation with **matlab** for a fast understanding of the methods in this paper. The code is tested to be compatible with matlab versions after **matlab2016b**.
Note that the matlab version is less efficient than the original version.

An example of registration process is achieved by running the code:
```
.\matlab\demo_dense_registration.m
```
The registration process is shown as follows: 

<img src="figures\step_example.png" alt="registration_process" style="zoom: 67%;" />

It generally leads to coherent local deformations with elegant grid architecture for fine-grained registrations, as

<img src="figures\mesh_detailed_structure.png" alt="grid_structures" style="zoom: 67%;" />

We suggest that it can also be applied to other data that are not limited to faces.

<img src="figures\hand_example.png" alt="hand_example" style="zoom: 67%;" />


## Sponsorships

This work is supported in part by the National Key Research and Development Program of China (No. 2022YFF0902302), the National Science Foundation of China (No. 62106250), and China Postdoctoral Science Foundation (No. 2021M703272).

## Bibtex
If you find this project helpful to your research, please consider citing:

```
@article{fan2023towards,
  title={Towards Fine-Grained Optimal 3D Face Dense Registration: An Iterative Dividing and Diffusing Method},
  author={Fan, Zhenfeng and Peng, Silong and Xia, Shihong},
  journal={International Journal of Computer Vision},
  pages={1--21},
  month = {June},
  year={2023},
  publisher={Springer}
}
```
