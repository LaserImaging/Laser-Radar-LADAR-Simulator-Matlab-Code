# LADAR Simulator MATLAB Code

This repository contains a Matlab implementation of the Development of a 3D LADAR Simulator Based on a Fast Target Impulse Response Generation Approach.

This (Laser Radar) LADAR simulator code has been developed, using MATLAB to simulate LADAR systems, and to produce 3D simulated scanning images under a wide variety of conditions. It can be considered as an efficient simulation code to use when developing LADAR systems and their data processing algorithms.

The simulator is designed to display the results after the scanning process in two different formats, one in spherical coordinates (spherical image), the other in Cartesian coordinates (3D point cloud). 


<p align="center"><img src="Ladar Images.png" width="600"></p>


## Implementations

This code is implemented in the MATLAB programming language.

## Getting Started

The repository contains three MATLAB files.

* LADAR.m : Implements the LADAR scanning Process based on the 'Fast Target Impulse Response Generation Approach'.
* 3D CAD Model.mat : 3D Matrix file, contains [faces & vertices] for 3D CAD model.
* Demo.m :  This file demonstrates the simulator's capability to simulate the direct detection time-of-flight LADAR systems and to produce 3D simulated scanning images under a wide variety of conditions. It reads the [faces & vertices] 3D information for the CAD model from the “3D CAD Model.mat” file, scanning it according to the scanning parameters using the “LADAR.m” function, and displays the results in two different formats: spherical image and 3D point cloud.


## Paper Abstract

A new laser detection and ranging (LADAR) simulator has been developed, using MATLAB and its graphical user interface, to simulate direct detection time of flight LADAR systems, and to produce 3D simulated scanning images under a wide variety of conditions. This simulator models each stage from the laser source to data generation and can be considered as an efficient simulation tool to use when developing LADAR systems and their data processing algorithms. The novel approach proposed for this simulator is to generate the actual target impulse response. This approach is fast and able to deal with high scanning requirements without losing the fidelity that accompanies increments in speed. This leads to a more efficient LADAR simulator and opens up the possibility for simulating LADAR beam propagation more accurately by using a large number of laser footprint samples. The approach is to select only the parts of the target that lie in the laser beam angular field by mathematically deriving the required equations and calculating the target angular ranges. The performance of the new simulator has been evaluated under different scanning conditions, the results showing significant increments in processing speeds in comparison to conventional approaches, which are also used in this study as a point of comparison for the results. The results also show the simulator’s ability to simulate phenomena related to the scanning process, for example, type of noise, scanning resolution and laser beam width.



In return for making this code available, I would appreciate that you cite the following publications:

* Ali Adnan Al-Temeemy; "The Development of a 3D LADAR Simulator Based on a Fast Target Impulse Response Generation Approach," 3D Research, Springer (2017), volume 8, 31, Aug 2017, ISSN2092-6731, doi:10.1007/s13319-017-0142-y.

* Ali A. Al-Temeemy and J. W. Spencer. "Simulation of 3D Ladar Imaging System using Fast Target Response Generation Approach," In Proceeding of SPIE - Optical Design and Engineering IV, Marseille, France,  volume 8167, pages 816720-(1-9), Sep 2011, doi: 10.1117/12.902309.


How to cite this repository:

see "LADAR.bib" file \[BiBTeX format]

```bibtex
@article{ALTEMEEMY2017,
title = {The Development of a 3D LADAR Simulator Based on a Fast Target Impulse Response Generation Approach},
author = {Ali A. Al-Temeemy},
journal = {3D Research},
volume = {8},
article-number={31},
year = {2017},
issn = {2092-6731},
doi = {10.1007/s13319-017-0142-y},
url = {https://doi.org/10.1007/s13319-017-0142-y}
}
```

```bibtex
@inproceedings{ALTEMEEMY2011,
author = {Ali A. Al-Temeemy and J. W. Spencer},
title = {Simulation of 3D ladar imaging system using fast target response generation approach},
volume = {8167},
booktitle = {Optical Design and Engineering IV},
organization = {International Society for Optics and Photonics},
publisher = {SPIE},
pages = {816720},
keywords = {LADAR Simulator, 3D Laser Imaging, Laser Beam Propagation},
year = {2011},
doi = {10.1117/12.902309},
URL = {https://doi.org/10.1117/12.902309}
}
```

## Author

Ali A. Al-Temeemy is a Professor in the Department of Laser and Optoelectronics Engineering at Al-Nahrain University and an honorary research fellow in the Department of Electrical Engineering and Electronics at the University of Liverpool.

## License
See the License files for details.

- MATLAB code: (see LICENSE).
- "3D CAD Model": (see 3D CAD Model LICENSE).

