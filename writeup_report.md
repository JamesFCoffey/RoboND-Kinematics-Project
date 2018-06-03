## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/Kuka_1.jpg
[image2]: ./misc_images/Kuka_2.jpg
[image3]: ./misc_images/Kuka_3.jpg
[image4]: ./misc_images/Kuka_4.jpg
[image5]: ./misc_images/Kuka_5.jpg
[image6]: ./misc_images/dh-transform.png
[image7]: ./misc_images/dh-transform-matrix.png
[image8]: ./misc_images/T0_1.png
[image9]: ./misc_images/T1_2.png
[image10]: ./misc_images/T2_3.png
[image11]: ./misc_images/T3_4.png
[image12]: ./misc_images/T4_5.png
[image13]: ./misc_images/T5_6.png
[image14]: ./misc_images/T6_7.png
[image15]: ./misc_images/T0_7.png
[image16]: ./misc_images/Roll_Pitch_Yaw.png
[image17]: ./misc_images/correction_matrix.png
[image18]: ./misc_images/R0_6.png
[image19]: ./misc_images/WC.png
[image20]: ./misc_images/theta_1_trig.jpg
[image21]: ./misc_images/theta_1.png
[image22]: ./misc_images/sides_abc.png
[image23]: ./misc_images/angles_abc.png
[image24]: ./misc_images/theta_23_trig.jpg
[image25]: ./misc_images/theta_3_trig.jpg
[image26]: ./misc_images/theta_2.png
[image27]: ./misc_images/theta_3.png
[image28]: ./misc_images/extrinsicxyz.png
[image29]: ./misc_images/beta.png
[image30]: ./misc_images/gamma.gif
[image31]: ./misc_images/alpha.gif
[image32]: ./misc_images/R3_6.png
[image33]: ./misc_images/theta_4.png
[image34]: ./misc_images/theta_5.png
[image35]: ./misc_images/theta_6.png
[image36]: ./misc_images/pick_and_place.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I started with a drawn schematic of the Kuka KR210 robot.

![KR210 drawn schematic][image1]

Using the following DH convention from Udacity's "DH Parameter Assignment Algorithm" class notes:
1. Label all joints from {1, 2, … , n}.
2. Label all links from {0, 1, …, n} starting with the fixed base link as 0.
3. Draw lines through all joints, defining the joint axes.
4. Assign the Z-axis of each frame to point along its joint axis.
![KR210 with assigned Z-axes][image2]
5. Identify the common normal between each frame Z_hat<sub>i-1</sub> and frame Z_hat<sub>i</sub>.
6. The endpoints of "intermediate links" (i.e., not the base link or the end effector) are associated with two joint axes, {i} and {i+1}. For i from 1 to n-1, assign the X_hat<sub>i</sub> to be
    * For skew axes, along the normal between Z_hat<sub>i</sub> and Z_hat<sub>i+1</sub> and pointing from {i} to {i+1}.
    * For intersecting axes, normal to the plane containing Z_hat<sub>i</sub> and Z_hat<sub>i+1</sub>.
    * For parallel or coincident axes, the assignment is arbitrary; look for ways to make other DH parameters equal to zero.
7. For the base link, always choose frame {0} to be coincident with frame {1} when the first joint variable (theta<sub>1</sub> or d<sub>1</sub>) is equal to zero. This will guarantee that alpha<sub>0</sub> = a<sub>1</sub> = 0, and, if joint 1 is a revolute, d<sub>1</sub> = 0. If joint 1 is prismatic, then theta<sub>1</sub> = 0.
8. For the end effector frame, if joint n is revolute, choose X<sub>n</sub> to be in the direction of X<sub>n-1</sub> when theta<sub>n</sub> = 0 and the origin of frame {n} such that d<sub>n</sub> = 0.
![KR210 with assigned origins and X-axes][image3]

Given the following angles between the Z-axes and using the right-hand rule sign convention:

|           Relationship Between Z-Axes           | Twist Angle (degrees) |
|-------------------------------------------------|-----------------------|
|      Z<sup>0</sup> is parallel to Z<sub>1</sub> |                     0 |
| Z<sup>1</sup> is perpendicular to Z<sub>2</sub> |                   -90 |
|      Z<sup>2</sup> is parallel to Z<sub>3</sub> |                     0 |
| Z<sup>3</sup> is perpendicular to Z<sub>4</sub> |                   -90 |
| Z<sup>4</sup> is perpendicular to Z<sub>5</sub> |                    90 |
| Z<sup>5</sup> is perpendicular to Z<sub>6</sub> |                   -90 |
|      Z<sup>6</sup> is parallel to Z<sub>7</sub> |                     0 |

The values for alpha can be filled out in radians:

|             i             | alpha(i-1) | a(i-1) | d(i-1) |  theta(i) |
|---------------------------|------------|--------|--------|-----------|
| T<sup>0</sup><sub>1</sub> |          0 |        |        |           |
| T<sup>1</sup><sub>2</sub> |      -pi/2 |        |        |           |
| T<sup>2</sup><sub>3</sub> |          0 |        |        |           |
| T<sup>3</sup><sub>4</sub> |      -pi/2 |        |        |           |
| T<sup>4</sup><sub>5</sub> |       pi/2 |        |        |           |
| T<sup>5</sup><sub>6</sub> |      -pi/2 |        |        |           |
| T<sup>6</sup><sub>7</sub> |          0 |        |        |           |

From the kr210.urdf.xacro file, the joint displacements can be summarized below:

|               Joints               | delta_x | delta_y | delta_z |
|------------------------------------|---------|---------|---------|
|  Fixed Base Joint to J<sub>1</sub> |       0 |       0 |    0.33 |
|     J<sup>1</sup> to J<sub>2</sub> |    0.35 |       0 |    0.42 |
|     J<sup>2</sup> to J<sub>3</sub> |       0 |       0 |    1.25 |
|     J<sup>3</sup> to J<sub>4</sub> |    0.96 |       0 |  -0.054 |
|     J<sup>4</sup> to J<sub>5</sub> |    0.54 |       0 |       0 |
|     J<sup>5</sup> to J<sub>6</sub> |   0.193 |       0 |       0 |
|     J<sup>6</sup> to J<sub>7</sub> |    0.11 |       0 |       0 |

![DH Parameters from Displacements for Joints 1 to 3][image4]

|             i             | alpha(i-1) | a(i-1) |  d(i)  |  theta(i) |
|---------------------------|------------|--------|--------|-----------|
| T<sup>0</sup><sub>1</sub> |          0 |      0 |   0.75 |           |
| T<sup>1</sup><sub>2</sub> |      -pi/2 |   0.35 |      0 |           |
| T<sup>2</sup><sub>3</sub> |          0 |   1.25 |      0 |           |
| T<sup>3</sup><sub>4</sub> |      -pi/2 |        |        |           |
| T<sup>4</sup><sub>5</sub> |       pi/2 |        |        |           |
| T<sup>5</sup><sub>6</sub> |      -pi/2 |        |        |           |
| T<sup>6</sup><sub>7</sub> |          0 |        |        |           |

![DH Parameters from Displacements for Joints 4 to 6][image5]

|             i             | alpha(i-1) | a(i-1) |  d(i)  |  theta(i) |
|---------------------------|------------|--------|--------|-----------|
| T<sup>0</sup><sub>1</sub> |          0 |      0 |   0.75 |           |
| T<sup>1</sup><sub>2</sub> |      -pi/2 |   0.35 |      0 |           |
| T<sup>2</sup><sub>3</sub> |          0 |   1.25 |      0 |           |
| T<sup>3</sup><sub>4</sub> |      -pi/2 | -0.054 |   1.50 |           |
| T<sup>4</sup><sub>5</sub> |       pi/2 |      0 |      0 |           |
| T<sup>5</sup><sub>6</sub> |      -pi/2 |      0 |      0 |           |
| T<sup>6</sup><sub>7</sub> |          0 |      0 |  0.303 |           |

The joint parameters (q) can be filled in for theta. As X<sub>1</sub> and X<sub>2</sub> must be separated by 90 degrees, theta(2) must be q2 - pi/2. There is no joint paramter q7, so theta(7) is set to 0.

The resulting DH paramter table is:

|             i             | alpha(i-1) | a(i-1) |  d(i)  |  theta(i) |
|---------------------------|------------|--------|--------|-----------|
| T<sup>0</sup><sub>1</sub> |          0 |      0 |   0.75 |        q1 |
| T<sup>1</sup><sub>2</sub> |      -pi/2 |   0.35 |      0 | q2 - pi/2 |
| T<sup>2</sup><sub>3</sub> |          0 |   1.25 |      0 |        q3 |
| T<sup>3</sup><sub>4</sub> |      -pi/2 | -0.054 |   1.50 |        q4 |
| T<sup>4</sup><sub>5</sub> |       pi/2 |      0 |      0 |        q5 |
| T<sup>5</sup><sub>6</sub> |      -pi/2 |      0 |      0 |        q6 |
| T<sup>6</sup><sub>7</sub> |          0 |      0 |  0.303 |         0 |

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

From Udacity's "Forward Kinematics with Kuka KR210" class notes, as "the DH convention uses four individual transforms,

![dh-transform][image6]

to describe the relative translation and orientation of link (i-1) to link (i). In matrix form, this transform is,"

![dh-transform-matrix][image7]

Filling the matrix out using the DH parameter table, the individual transformation matrices are:

![T0_1][image8]
![T1_2][image9]
![T2_3][image10]
![T3_4][image11]
![T4_5][image12]
![T5_6][image13]
![T6_7][image14]

Multiplying the individual transformation matrices together generates a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose:

![T0_7][image15]

T<sup>0</sup><sub>7</sub> = `Matrix([[((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*cos(q6) - (-sin(q1)*cos(q4) + sin(q4)*sin(q2 + q3)*cos(q1))*sin(q6), -((sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*cos(q5) + sin(q5)*cos(q1)*cos(q2 + q3))*sin(q6) + (sin(q1)*cos(q4) - sin(q4)*sin(q2 + q3)*cos(q1))*cos(q6), -(sin(q1)*sin(q4) + sin(q2 + q3)*cos(q1)*cos(q4))*sin(q5) + cos(q1)*cos(q5)*cos(q2 + q3), -0.303*sin(q1)*sin(q4)*sin(q5) + 1.25*sin(q2)*cos(q1) - 0.303*sin(q5)*sin(q2 + q3)*cos(q1)*cos(q4) - 0.054*sin(q2 + q3)*cos(q1) + 0.303*cos(q1)*cos(q5)*cos(q2 + q3) + 1.5*cos(q1)*cos(q2 + q3) + 0.35*cos(q1)],
[((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*cos(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*sin(q6), -((sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*cos(q5) + sin(q1)*sin(q5)*cos(q2 + q3))*sin(q6) - (sin(q1)*sin(q4)*sin(q2 + q3) + cos(q1)*cos(q4))*cos(q6), -(sin(q1)*sin(q2 + q3)*cos(q4) - sin(q4)*cos(q1))*sin(q5) + sin(q1)*cos(q5)*cos(q2 + q3),  1.25*sin(q1)*sin(q2) - 0.303*sin(q1)*sin(q5)*sin(q2 + q3)*cos(q4) - 0.054*sin(q1)*sin(q2 + q3) + 0.303*sin(q1)*cos(q5)*cos(q2 + q3) + 1.5*sin(q1)*cos(q2 + q3) + 0.35*sin(q1) + 0.303*sin(q4)*sin(q5)*cos(q1)],
[-(sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*cos(q6) - sin(q4)*sin(q6)*cos(q2 + q3), (sin(q5)*sin(q2 + q3) - cos(q4)*cos(q5)*cos(q2 + q3))*sin(q6) - sin(q4)*cos(q6)*cos(q2 + q3), -sin(q5)*cos(q4)*cos(q2 + q3) - sin(q2 + q3)*cos(q5), -0.303*sin(q5)*cos(q4)*cos(q2 + q3) - 0.303*sin(q2 + q3)*cos(q5) - 1.5*sin(q2 + q3) + 1.25*cos(q2) - 0.054*cos(q2 + q3) + 0.75],
[0, 0, 0, 1]])`


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Define matrices for roll (x), pitch (y), and yaw (z):

![Roll_Pitch_Yaw][image16]

Calculate correction matrix between DH parameters and Gazebo:

![correction_matrix][image17]

Define Rotation matrix from frames 0 to 6:

![R0_6][image18]

Calculate the wrist center position (Inverse Position Kinematics):

![WC][image19]

![theta_1_trig][image20]

![theta_1][image21]

![sides_abc][image22]

![angles_abc][image23]

![theta_23_trig][image24]

![theta_3_trig][image25]

![theta_2][image26]

![theta_3][image27]

From Udacity class notes "Euler Angles from a Rotation Matrix" (Inverse Orientation Kinematics):

![extrinsicxyz][image28]

![beta][image29]

![gamma][image30]

![alpha][image31]

![R3_6][image32]

![theta_4][image33]

![theta_5][image34]

![theta_6][image35]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

I implemented the a basic forward kinetics outside the joint trajectory forward loop to keep the calculation time down. For the inverse kinetics code, I inserted the final value of fixed calculations to reduce the calculation time there too. The implementation might fail in that it did not seem to penalize the number of rotations or tortuosity of the path selected. This might cause the gripped cylinder to fall, whereas more gentle paths would not.  If I were going to pursue this project further, I might improve the code by using the faster NumPy matrix multiplication or writing in C++ which is much faster.

Below is a screenshot of the completed pick and place process.

![pick_and_place][image36]
