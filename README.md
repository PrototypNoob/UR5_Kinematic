# Documentation for UR5Kinematic

In this document an class to solve an forward and inverse kinematic is shown. The software solution is written in C++ and was tested under linux/WSL.

last edited:   21.08.2022

- [Softwarearchitektur](#soft)
   - [Class-Diagram](#class)
   - [Used Librarys](#lib)
   - [How-To-Build](#build)
- [Tests](#test)
   - [Forward Kinematic](#forward_test)
   - [Inverse Kinematic](#inverse_test)
- [Mathematical Calculation](#calc)
   - [Denavit–Hartenberg Parameters](#dh)
   - [Forward (Direct) Kinematic](#for)
   - [Inverse Kineamtic](#inv)
- [Improvments](#improve)

<a name="soft"></a>
## Softwarearchitektur

Here the Softwarearchitektur for the UR5Kinematic calss is explained and shown. This should allow other users to create easily changes and use the class, without the need to check the source code. The source code is still documented, to make working with the code easy.

<a name="class"></a>
### Class-Diagram

The following diagram shows the class diagramm of the developed UR5Kinematic.

![UR5Kinematic class diagram](assets/robotkinematik_class_diagram.png?raw=true "UR5Kinematic class diagram")

In this class only the needed methods can be accessed, while the variables are only accessable via methodes or in the class itself.

<a name="lib"></a>
### Used Librarys

The software solution was written with Visual Studio Code on the WSL Ubuntu 20.04 LTS and is build and tested with WSL. 

The following libraries where used to create the software solution.

| Library   | Link to Library                                                                | Version      |
|---        |---                                                                             |---           |
| Eigen     | [https://gitlab.com/libeigen/eigen.git](https://gitlab.com/libeigen/eigen.git) | Eigen 3.4.0  |

The Library Eigen was installed via the cmake file contained in the git. Changes could be needed if the library is in another directory. If prebuild executables are needed please contact the git 

<a name="build"></a>
### How-To-Build

The examples/tests can be build via the cmake. To build the classes please enter the following lines:
```
 mkdir build
 cd build
 cmake ..
 make
```

The Tests decribed inside [Test](#test) should be executabale, for execution of the executables please see [Forward-Kinematik Test](#forward-kinematik-test) and [Inverse-Kinematik Test](#inverse-kinematik-test).

<a name="test"></a>
## Tests

<a name="forward_test"></a>
### Forward-Kinematik Test

To be able to check the forward kinematic one simple test was created. In this test the implemented forward kinematic is compared with the forward kinematic out of the excel from:

[https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)

The Excel needs to be edited with the Denavit-Hartenberg values, to be able to compare the developed code. Afterwards an angle is configurated and tested with the implemented code. It is expected that the position values deviate from the excel sheet because of rounding error. Because of that a margin should be build in. This should be tested with multiple angles, for this the `forward_test` should be used. The Test should be executed via the following line:

```
   ./forward_test  [theta_1] [theta_2] [theta_3] [theta_4] [theta_5] [theta_6] [x_value_ee] [y_value_ee] [z_value_ee]
```

Where `[theta_1]` to `[theta_6]` are the angles for each joint. `[x_value_ee]`, `[y_value_ee]` and `[z_value_ee]` are the position values of the end effektor. One possible way to start the code would be with:

```
   ./forward_test  2.77507351 4.76474886 0.95993109 3.42084533 1.60570291 2.44346095 0.27120 0.00974 0.78972
```

This allows to check multiple values without rebuilding the code inbetween test. The output of this test looks as follows:

![forward kinematic single test](assets/forward_test.png?raw=true "forward kinematic single test")

<a name="inverse_test"></a>
### Inverse-Kinematik Test

The forward kinematic can be used to evaluate the developed inverse kinematic. The reason why the forward kinematic needs to be used is that the inverse kinematic has multiple solutions, which allows different angle position to reach the same positon. Because of that the forward kinematic is first calculated. The values of the forward kinematic is then used to calclate the angles. After the inverse kinematic calculated the angle values, the angle values are used to calculate the forward kinematic again. If the values bevor and after the calculation of the inverse kinematic fit (with an allowed error margin) then the test was successfull. 

To be able to calculate first the forward kinematic, the code needs to be started with the following parameters:

```
   ./inverse_test  [theta_1] [theta_2] [theta_3] [theta_4] [theta_5] [theta_6]
```

where `[theta_1]` to `[theta_6]` are the angle values for the forward kinematic. An Example how to start the code is shown with:

```
   ./inverse_test  2.77507351 4.76474886 0.95993109 3.42084533 1.60570291 2.44346095
```

The execution of this test allows to check different values without the need to rebuild the inverse kineamtic test. The output of the test is shown below:

![inverse kinematic single test](assets/inverse_test.png?raw=true "inverse kinematic single test")

<a name="calc"></a>
## Mathematical Calculation

The mathematical calculations can be used to evaluate the given solution and give inside into the thought prozess while developing the software solution. This part is not used to increase the readability of the given code, but gives an overview for variable names.

<a name="dh"></a>
### Denavit–Hartenberg Parameters

The Denavit-Hartenberg Parameters of the UR5 where taken from:

[https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/) - last veiwed: 21.08.2022

To be able to evaluate the Software Solution the Table is transfered to this Document. This also means that the Program is only tested with the values of the following table, please check the software solution if the values are changed.

| Kinematic       | $\theta$ [rad]     | $a$ [m]            | $d$ [m]            | $\alpha$ [rad]     |
| --------------  | -----------------  | -----------------  | -----------------  | -----------------  |
| Joint 1         | $0$                | $0$                | $0.089159$         | $\frac{\pi}{2}$    |
| Joint 2         | $0$                | $-0.425$           | $0$                | $0$                |
| Joint 3         | $0$                | $-0.39225$         | $0$                | $0$                |
| Joint 4         | $0$                | $0$                | $0.10915$          | $\frac{\pi}{2}$    |
| Joint 5         | $0$                | $0$                | $0.09465$          | $-\frac{\pi}{2}$   |
| Joint 6         | $0$                | $0$                | $0.0823$           | $0$                |

<a name="for"></a>
### Forward Kineamtic

The Denavit-Hartenberg Parameters can be used to calculate the forward kinematic of the UR5. For this the transforamtion matrix need to be created. The transformation matrix does show the position of the robotic system based of the position from the previous joint. Fot this the Denavit-Hartenberg Parameters can be calculated like described in,

$$
   ^{n-1}T_n = Trans_{z_{n-1}}(d_n) \cdot Rot_{z_{n-1}}(\theta_n) \cdot Trans_{z_{n-1}}(a_n) \cdot Rot_{z_{n-1}}(\alpha_n)
$$

where the translation Matrix can be described with,

$$
Trans_{z_{n-1}}(d_n) = 
\begin{bmatrix}
   1 & 0 & 0 & 0 \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & d_n \\
   0 & 0 & 0 & 1
\end{bmatrix};\ Trans_{z_{n-1}}(a_n) = 
\begin{bmatrix}
   1 & 0 & 0 & a_n \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1
\end{bmatrix}
$$

and the rotation Matrix can be described with,

$$
Rot_{z_{n-1}}(\theta_n) = 
\begin{bmatrix}
   \cos(\theta_n) & -\sin(\theta_n) & 0 & 0 \\
   \sin(\theta_n) & \cos(\theta_n) & 0 & 0 \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1
\end{bmatrix};\ Rot_{z_{n-1}}(\alpha_n) = 
\begin{bmatrix}
   1 & 0 & 0 & 0 \\
   0 & \cos(\alpha_n) & -\sin(\alpha_n) & 0 \\
   0 & \sin(\alpha_n) & \cos(\alpha_n) & 0 \\
   0 & 0 & 0 & 1
\end{bmatrix}
$$

The final transformation Matrix between joint $n$ and joint $n-1$ can be defined as followed, 

$$
   ^{n-1}T_n = 
   \begin{bmatrix}
      \cos(\theta_n) & -\sin(\theta_n)\cos{\alpha_n} & \sin{\theta_n}\sin{\alpha_n} & a_n\cos{\theta_n} \\
      \sin(\theta_n) & \cos(\theta_n)\cos{\alpha_n} & -\cos{\theta_n}\sin{\alpha_n} & a_n\cos{\theta_n} \\
      0 & \sin{\alpha_n} & \cos{\alpha_n} & d_n \\
      0 & 0 & 0 & 1
   \end{bmatrix}
$$

The final Denavit-Hartenberg transformation Matrix is a 4x4 Matrix and consists of one part that is realted to the rotation ( $R_n^{n-1}$ ), the displacement ( $D_n^{n-1}$ ) and the perspective ( $1$ ).

$$
^{n-1}T_n = 
\left[
\begin{array}{c c c | c}
   & & &  \\
   & R_n^{n-1} & &  D_n^{n-1} \\
   & & &  \\
   0 & 0 & 0 & 1  
\end{array}
\right]
$$

To be able to calculate the psotion of the end effector the complete chain of joints needs to be calulated, fot this the following equation is used.

$$
   ^0T_6 = ^0T_1 \cdot ^1T_2 \cdot ^2T_3 \cdot ^3T_4 \cdot ^4T_5 \cdot ^5T_6 
$$

The complete Matrix can be calculated via software and is not calculated manually. To be able to get the position of the end effector only the values of $D_6^0$ need to be calculated.

<a name="inv"></a>
### Inverse Kineamtic

The transformation matrix from the forward kinematic, can be used to calculate the jacobian of the system and finaly the inverse kinematic. To be able to calculate the jacobian each (revolute or prismatic) joint can be used to calculate an "joint" jacobian. For this the following eqautions can be used,


|           | Prismatic             | Revolute                                            |
|---        |---                    |---                                                  |
|Linear     | $R^0_{n-1} \cdot M_1$ | $R^0_{n-1} \cdot M_1 \times (D^0_{n_{max}} - D^0_{n-1})$    |
|Rotational | $M_0$                 | $M_1$                                               |

   
where:

$$
   M_1 = 
   \begin{bmatrix}
   0 \\
   0 \\
   1
   \end{bmatrix}
   ;\   
   M_0 = 
   \begin{bmatrix}
   0 \\
   0 \\
   0
   \end{bmatrix}
   ;\ 
   R^0_0 = 
   \begin{bmatrix}
   1 & 0 & 0 \\
   0 & 1 & 0\\
   0 & 0 & 1
   \end{bmatrix}
$$

To be able to calculate the jacobian of the UR5 Robot, six revolute joints are used and can be specified as follows,

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{z}
\end{bmatrix}=\begin{bmatrix}
R_0^0\cdot M_1 \times (D_6^0-D_0^0) & R_1^0\cdot M_1 \times (D_6^0-D_1^0) & R_2^0\cdot M_1 \times (D_6^0-D_2^0) & R_3^0\cdot M_1 \times (D_6^0-D_3^0) & R_4^0\cdot M_1 \times (D_6^0-D_4^0) & R_5^0\cdot M_1 \times (D_6^0-D_5^0)\\
M_1 & M_1 & M_1 & M_1 & M_1 & M_1
\end{bmatrix}
\cdot
\begin{bmatrix}
\dot{\theta_1} \\
\dot{\theta_2} \\
\dot{\theta_3} \\
\dot{\theta_4} \\
\dot{\theta_5} \\
\dot{\theta_6}
\end{bmatrix}
$$

The orientation of the jacobian matrix can be ignored, because only the velocities in x,y and z direction are used in the forward kineamtic and can be used to calculate the inverse kinematik with the IK solver algorithm as shown in: [https://en.wikipedia.org/wiki/Inverse_kinematics](https://en.wikipedia.org/wiki/Inverse_kinematics)

The calculation of the inverse kinematic is done in the software solution.

<a name="improve"></a>
## Possible Improvments

This should be used to list some possible improvments for the class:
- create config file for denavit hartenberg parameter, this would allow to change the values without the need to rebuild the code
- check if there are other solver methods, the solver explained on the wikipedia page was used
- create a test for the forward kinematic that does not resolve around the use of the extern excel document
- create a test for the inverse kinematic code that computs multiple inverse kinematics and check average time and worst time, to be able to create a better runtime estimate for the inverse kinematic 
- check optimation potential, in regards to runtime, ram usage and maybe also cpu usage
