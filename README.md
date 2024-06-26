# Adaptive Terminal Sliding Mode Control for Rigid Robotic Manipulators

## Overview

This repository contains the technical report, original paper, and MATLAB code associated with the project on Adaptive Terminal Sliding Mode Control (ATSMC) for Rigid Robotic Manipulators. The project addresses the limitations of conventional Terminal Sliding Mode Control (TSMC) by introducing an adaptive approach that eliminates the need for prior knowledge of uncertainties and disturbances in robotic manipulator dynamics.

## Contents

- **Report**: A detailed technical report explaining the ATSMC method, its implementation, and simulation results.
- **MATLAB Code**: MATLAB scripts used for simulating the ATSMC method on a two-degree-of-freedom robotic manipulator.

## Files

### Reports

- `Technical Report on Adaptive Terminal Sliding Mode Control.pdf`: Technical report detailing the ATSMC approach.

### MATLAB Code

- `graph1.m`: Script for generating Error vs Time graph.
- `graph2.m`: Script for generating Torque vs Time graph.
- `graph3.m`: Script for generating Surface vs Time graph.
- `graph4.m`: Script for generating de/dt vs e graph.
- `graph6.m`: Script for generating the sixth set of simulation graphs.
- `ATSMC.m`: Main script implementing the Adaptive Terminal Sliding Mode Control.
- `CTSMC.m`: Script implementing the Continuous Terminal Sliding Mode Control for comparison.

## Results

- The MATLAB scripts will generate simulation results demonstrating the effectiveness of the ATSMC approach compared to the conventional TSMC. The results include error convergence graphs, control torque plots, and surface tracking illustrations.
- [Graphs](/Results): The results obtained are not same as the original paper, the report uses different parameters i.e. constants.
- Torque vs Time
<div align=left>
<table>
  <tr>
    <td><img src="https://github.com/akoushik2k/Adaptive-Terminal-Sliding/blob/0587b16051af36dd5e911db6043c8c2cf7d21cf4/Results/Torque%201%20vs%20time.jpg" alt="Torqu1vsTime" width="400"/></a></td>
    <td><img src="https://github.com/akoushik2k/Adaptive-Terminal-Sliding/blob/0587b16051af36dd5e911db6043c8c2cf7d21cf4/Results/Torque%202%20vs%20time.jpg" alt="Torque2vsTime" width="400"/></a></td>
  </tr>
</table>
</div>
- Error(Rad) vs Time
<div align=left>
<table>
  <tr>
    <td><img src="https://github.com/akoushik2k/Adaptive-Terminal-Sliding/blob/c7d83aca8ecb40e9c1c7ba6ab0353dc2390e3242/Results/e1%20vs%20time.jpg" alt="error1vsTime" width="400"/></a></td>
    <td><img src="https://github.com/akoushik2k/Adaptive-Terminal-Sliding/blob/c7d83aca8ecb40e9c1c7ba6ab0353dc2390e3242/Results/e2%20vs%20time.jpg" alt="error2vsTime" width="400"/></a></td>
  </tr>
</table>
</div>
- Surface vs Time
<div align=left>
<table>
  <tr>
    <td><img src="https://github.com/akoushik2k/Adaptive-Terminal-Sliding/blob/c7d83aca8ecb40e9c1c7ba6ab0353dc2390e3242/Results/S1%20vs%20Time.png" alt="surface1vsTime" width="400"/></a></td>
    <td><img src="https://github.com/akoushik2k/Adaptive-Terminal-Sliding/blob/c7d83aca8ecb40e9c1c7ba6ab0353dc2390e3242/Results/S2%20vs%20Time.png" alt="surface2vsTime" width="400"/></a></td>
  </tr>
</table>
</div>

## References
[Adaptive terminal sliding mode control for rigid robotic manipulators](https://link.springer.com/article/10.1007/s11633-011-0576-2)

## Acknowledgements 
- Thanks to the authors of the original paper for their foundational research.
- Special thanks to the ENPM Control of Robotic Systems course for the guidance and resources provided.
