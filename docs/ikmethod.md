# Geometric vs. Jacobian Inverse Kinematics
### A Technical Comparison for a Planar Robotic Arm
*Simon Dudek · July 2026*

---

## Abstract
In the field of robotics, Inverse Kinematics is frequently used, however there are multiple approaches to using Inverse Kinematics. This paper will investigate the differences between 2 approaches solving for the angles needed to place the end effector of a kinematic chain, that being the Jacobian Pseudoinverse and Geometric Inverse Kinematic. This research was conducted by creating a simulation of a 2 joint articulated structure and using different approaches inverse kinematics to find the angles needed for the structure's end effector to reach the point. Data was then collected concerning the time, error, and iterations used in both approaches following the same points, under 2 different conditions, one having completely random points and the other having points that were more predictably stepped. This data displayed a clear trend of Jacobian Pseudoinverse Kinematics freezing when encountering 2 points far from each other, and otherwise showed a slower mean time and error that that of Geometric Kinematics. From this study, we were able to conclude that Geometric Inverse Kinematics is more reliable and faster when working with 2 joints, and that Jacobian Pseudoinverse Kinematics often yields slightly less accurate results, and can be unreliable; however, it is far easier to expand with more joints and can be accurate if used outside of stress testing.

---

## 1. Introduction
Inverse Kinematics is the study of the motion required for an articulated structure to reach a position. Inverse Kinematics is most important to fields such as robotics, as it is the main process behind structures such as robotic arms being able to reach designated positions. Inverse Kinematics therefore works backward from Forward Kinematics by using points to find joint angles. Inverse Kinematics is therefore a foundational process to robotics, but due to its nature, it can be difficult to find the position of the joints necessary to reach points. For 2 joints on a 2d plane, it is possible to use a closed-form approach such as a geometric approach; however, when working with more joints, a different approach is necessary because the geometric approach cannot apply to a structure with 3 or more joints. Therefore, a Numerical approach, such as the Jacobian approach is often used, which uses a Jacobian Matrix to iteratively approximate what angles of the joints would allow an end effector to reach a point. This study will focus on comparing these 2 approaches to inverse kinematics when working with a 2 joint arm.

## 2. Geometric Closed-Form Inverse Kinematics
### 2.1 Method
Using Geometric Closed-Form Inverse Kinematics, we can find the joint angles for a 2 joint arm using multiple formulas, and find 2 different possible angle outcomes. Due to the nature of being closed-form, this will lead to an exact outcome but will only work with 2 joints. To begin, we can define our joint angles as $\theta_1$ and $\theta_2$ for joints 1 and 2 respectively, and use $\phi$ as the angle of the elbow between the 2 links. We can also use $c$ to represent the length from the end effector to the base, and $a1$ and $a2$ to represent the lengths of links 1 and 2 respectively. Because we know that $\phi$ and $\theta_2$ are supplementary, we can also state that $\theta_2 = \pi - \phi$. Using the cosine angle difference identity $\cos(\pi - x) = -\cos(x)$, we can state that $-\cos(\phi) = \cos(\theta_2)$. We can then use the Law of Cosines, represented by the formula $c^2 = (a1)^2 + (a2)^2 - 2(a1)(a2)\cos(\phi)$, and simplify it to solve for $-\cos(\phi)$ with the formula $-\cos(\phi) = \frac{x^2 + y^2 - (a1)^2-(a2)^2}{2a1(a2)}$. We can now use our cosine angle difference identity to plug $cos(\theta_2)$ in for $-\cos(\phi)$ and solve for $\theta_2$, giving us the equation $\theta_2 = \cos^{-1}\left(\frac{x^2+y^2-(a1)^2-(a2)^2}{2a1(a2)}\right)$ We can also easily find the angle used for the elbow up outcome by multiplying the solution by $-1$

<img src="figures/arm_schematic2.png" width="500" alt="Arm schematic">

*Figure 1: Geometric IK setup to find $\theta_2$ for a 2-joint planar arm.*

In order to find $\theta_1$, we must use the formula $\theta_1 = \gamma - \beta$. In this case, $\gamma$ is the angle between the horizontal line running through the base and the line stretching from the base to the end-effector. In the following formula, we  will also use $x$ and $y$, which can be associated with c with the assertion that $c^2 = x^2 + y^2$, or in other words, $x$ is the horizontal distance of the end-effector from the base, and $y$ is the vertical distance of the end-effector from the base. Using these variables, we can state $\gamma$ is found using the equation $\gamma = \tan^{-1}(\frac{y}{x})$.

<img src="figures/arm_schematic3.png" width="500" alt="Arm schematic">

*Figure 2: Geometric IK setup to find $\gamma$ for a 2-joint planar arm.*

Next, we must find the $\beta$ angle, which is the angle between the 1st link and the line stretching from the base to the end effector. This angle can be found by using $\theta_2$ and $a2$ to create a new right triangle, and then using the tangent ratio of $\tan(\beta) = \frac{a2\sin(\theta_2)}{a1+a2\cos(\theta_2)}$, which, when solving for $\beta$, can be simplified to $\beta = \tan^{-1}(\frac{a2\sin(\theta_2)}{a1+a2\cos(\theta_2)})$.

<img src="figures/arm_schematic4.png" width="500" alt="Arm schematic">

*Figure 3: Geometric IK setup to find $\beta$ for a 2-joint planar arm.* 

Now that $\beta$ and $\gamma$ are known, $\theta_1$ is easy to find. If we use the formula from before, we can simply plug our numbers into $\theta_1 = \gamma - \beta$, we can get the formula for finding $\theta_1$ in relation to $\theta_2$, which is $\theta_1 = \tan^{-1}(\frac{y}{x}) - \tan^{-1}(\frac{a2\sin(\theta_2)}{a1+a2\cos(\theta_2)})$. Using this formula, paired with the formula $\theta_2 = \cos^{-1}\left(\frac{x^2+y^2-(a1)^2-(a2)^2}{2a1(a2)}\right)$, we can easily find the joint angles for most in range end-effector positions for a 2 joint articulated structure.

### 2.2 Usage
Geometric Closed-Form Inverse Kinematics works best when there are only 2 joints, no irregular shapes, and no constraints. This is because Geometric Inverse Kinematics cannot be expanded past 2 joints, and cannot account for movement constraints past being able to select whether the elbow is above or below the end-effector. One of the greatest benefits of Geometric Closed-Form Inverse Kinematics is the reliability and minimal computational cost of using it, as the time and cost is nearly minimal whilst still achieving a moderate reliability.

Geometric Closed-Form Inverse Kinematics does suffer from a few limitations, though; it cannot derive angles from any points that cause the $\phi$ angle to measure at $180\degree$, or in other words a location that would lead both links to be collinear. Geometric Closed-Form Inverse Kinematics also cannot handle any locations where the $\phi$ would measure at $0\degree$, or when the arm would fold in on itself. These locations are called singularities. By far the largest disadvantage is its lack of generalization, as it can only be used on 2 joint planar structures, and is unable to account for the paths to get to locations. Therefore, any constraints that might be used are usually neglected, which may cause issues in certain applications.

Due to all these factors, we can see that Geometric Closed-Form Inverse Kinematics are useful where a low cost, deterministic method for Inverse Kinematics on a 2 joint planar arm without constraints is necessary. Geometric Inverse Kinematics can provide exact and precise solutions to such systems, but will likely be unable to function if any expansion or generalization is done. Because of this, we can see that Geometric Closed-Form Inverse Kinematics is usually not useful outside of very specific application.

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