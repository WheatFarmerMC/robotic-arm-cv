# Geometric vs. Jacobian Inverse Kinematics
### A Technical Comparison for a Planar Robotic Arm
*Simon Dudek · July 2026*

---

## Abstract
In the field of robotics, Inverse Kinematics is frequently used, however there are multiple approaches to using Inverse Kinematics. This paper will investigate the differences between 2 approaches solving for the angles needed to place the end effector of a kinematic chain, that being the Jacobian Pseudoinverse and Geometric Inverse Kinematic. This research was conducted by creating a simulation of a 2 joint articulated structure and using different approaches inverse kinematics to find the angles needed for the structure's end effector to reach the point. Data was then collected concerning the time, error, and iterations used in both approaches following the same points, under 2 different conditions, one having completely random points and the other having points that were more predictably stepped. This data displayed a clear trend of Jacobian Pseudoinverse Kinematics freezing when encountering 2 points far from each other, and otherwise showed a slower mean time and error that that of Geometric Kinematics. From this study, we were able to conclude that Geometric Inverse Kinematics is more reliable and faster when working with 2 joints, and that Jacobian Pseudoinverse Kinematics often yields slightly less accurate results, and can be unreliable; however, it is far easier to expand with more joints and can be accurate if used outside of stress testing.

---

## 1. Background and Motivation
Inverse Kinematics is the study of the motion required for a articulated structure to reach a position. Inverse Kinematics is most important to fields such as robotics, as it is the main process behind structures such as robotic arms being able to reach designated positions. Inverse Kinematics therefore works backward from Forward Kinematics by using points to find joint angles. Inverse Kinematics is therefore a foundational process to robotics, but due to its nature, it can be difficult to find the position of the joints necessary to reach points. For 2 joints, it is possible to use a geometric approach using the law of cosines, however when working with more, a different approach is necessary because geometric approach cannot apply to a 3 or more joint arm. Therefore, a Jacobian approach is often used, which uses a Jacobian Matrix to iteratively approximate what angles of the joints would allow an end effector to reach a point. Using these 2 approaches, we can compare them and their use with a 2 joint arm.

## 2. Geometric Inverse Kinematics
### 2.1 Derivation
[Law of cosines, equations, diagram]

### 2.2 Limitations
[Where it breaks down]

## 3. Jacobian-Based Inverse Kinematics
### 3.1 The Jacobian Matrix
[Physical interpretation, how to compute it]

### 3.2 The Pseudoinverse Update Rule
[Δθ = J⁺ · Δx, walk through numerically]

### 3.3 Convergence and Stability
[What happens near singularities]

## 4. Comparison
[Table + discussion]

## 5. Results
[Plots from your implementation — embed as images]

## 6. Conclusion
[What you learned, real-world implications]

## 7. References
https://www.mathworks.com/discovery/inverse-kinematics.html
https://www.mdpi.com/2218-6581/13/6/91
https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
https://medium.com/@manuelmort/inverse-kinematics-of-two-link-planar-arm-geometric-approach-5f3ffdfde16d