# **LoRePoSE** :tanker:  
**Precise Pose Estimation for Loading and Unloading Ports of Liquid Chemical Tankers**  

## Project Overview :bulb:
This project focuses on developing a robust system for precise pose estimation of loading and unloading ports in liquid chemical tankers. The goal is to automate the loading process of various chemical products, such as alcohols, benzene, and LNG, by accurately estimating the 3D coordinates and normal vectors of the ports using RGB-D cameras.

### Key Objectives :dart:
- **Precise Estimation**: Accurately determine the 3D coordinates and normal vectors of the loading and unloading ports' circular surfaces.
- **Robustness**: Design a solution that handles different tanker layouts and adapts to varying port positions.
- **Efficiency**: Ensure the entire processing time does not exceed 60 seconds.
- **Visualization**: (Optional) Visualize the estimated poses for better clarity and validation.

### Materials and Tools :toolbox:
- **Programming Languages**: Python/C++
- **Point Cloud Data**: Provided datasets including experimental and real tanker point clouds.
- **Software**: CloudCompare, Point Cloud Library (PCL)
- **Additional Resources**: RANSAC model fitting, ICP-based point cloud registration, etc.
