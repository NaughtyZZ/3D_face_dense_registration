# Towards Fine-Grained Optimal 3D Face Dense Registration: An Iterative Dividing and Diffusing Method (IJCV2023)

by Zhenfeng Fan, Silong Penng, and Shihong Xia

## Introduction

This repository is built for the official implementation of the paper "Towards Fine-Grained Optimal 3D Face Dense Registration: An Iterative Dividing and Diffusing Method  **(IJCV)** ".
To view the full version of this paper, please visit the website [https://rdcu.be/ddHfp](https://rdcu.be/ddHfp), which provides the online version.

## Abstract
Dense vertex-to-vertex correspondence (i.e. registration) between 3D faces is a fundamental and challenging issue for 3D &2D face analysis. While the sparse landmarks are definite with anatomically ground-truth correspondence, the dense vertex correspondences on most facial regions are unknown. In this view, the current methods commonly result in reasonable but diverse solutions, which deviate from the optimum to the dense registration problem. In this paper, we revisit dense registration by a dimension-degraded problem, i.e. proportional segmentation of a line, and employ an iterative dividing and diffusing method to reach an optimum solution that is robust to different initializations. We formulate a local registration problem for dividing and a linear least-square problem for diffusing, with constraints on fixed features on a 3D facial surface. We further propose a multi-resolution algorithm to accelerate the computational process. The proposed method is linked to a novel local scaling metric, where we illustrate the physical significance as smooth adaptions for local cells of 3D facial shapes. Extensive experiments on public datasets demonstrate the effectiveness of the proposed method in various aspects. Generally, the proposed method leads to not only significantly better representations of 3D facial data, but also coherent local deformations with elegant grid architecture for fine-grained registrations.

## Usage

### C++ Version
We provide the original verison with **C/C++** code, which was compiled with **Visual Studio 2015** in our experiments. I suggest that these codes can be compiled with the newest C/C++ environments.

An executable demo of registration process is achieved by running the code:
```
.\VC++\demo\faceDenseRegistration.exe .\plyInput\ .\plyOutput\ 
```
### Matlab Version
We also provide a **simple** and **stand-alone** implementation with **matlab** for a fast understanding of the methods in this paper. The code is tested to be compatible with matlab version after matlab2016b.
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