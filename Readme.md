# DH参数与机械臂运动学

## DH参数->转换矩阵

本文不作DH参数的详细介绍，若不了解DH参数请参考CSDN博客：
[一文带你完全掌握机器人DH参数建模(详细步骤+实例+代码)\_六轴dh c语言-CSDN博客](https://blog.csdn.net/maple_2014/article/details/105612912)

注意，以下的所有计算都是以轴向指向纸面外时绕轴逆时针旋转为正方向。

$$
R =\left[\begin{matrix}
	\cos{\theta}&-\sin{\theta} &0\\
	\cos{\alpha}\sin{\theta} & \cos{\alpha}\cos{\theta} & -\sin{\alpha}\\
	\sin{\alpha}\sin{\theta}&\sin{\alpha}\cos{\theta}&\cos{\alpha}\\
\end{matrix}\right]\\
T = \left[\begin{matrix} a & -d\sin{\alpha} & d\cos{\alpha} \end{matrix}\right]^T\\
 H = \left[\begin{matrix}
	R & T\\
	0 & 1
\end{matrix}\right]\\
H^{-1} = \left[\begin{matrix}
	R^T & -R^TT\\
	0&1
\end{matrix}\right]
$$

## FK（正向运动学）

设机械臂原点为 $^0P$ ， i-1 轴到 i 轴的转换矩阵为 $_i^{i-1}H$ ，末端位姿为 $^EP$ ，机械臂共有 n 个轴。因此有：

$$
^EP = _{n-1}^nH..._1^2H_0^1H^0P
$$

由此得到末端位置。

## IK（逆向运动学）

以6轴转动机械臂为例。
首先建立DH参数：

|i|a|$\alpha$|d|$\theta$|
|:---:|:---:|:---:|:---:|:---:|
|0|$a_0$|$\alpha_0$|$d_0$|$\theta_0$
|1|$a_1$|$\alpha_1$|$d_1$|$\theta_1$
|2|$a_2$|$\alpha_2$|$d_2$|$\theta_2$
|3|$a_3$|$\alpha_3$|$d_3$|$\theta_3$
|4|$a_4$|$\alpha_4$|$d_4$|$\theta_4$
|5|$a_5$|$\alpha_5$|$d_5$|$\theta_5$
|6|$a_6$|$\alpha_6$|$d_6$|$\theta_6$

以及工具的DH参数:

|a|$\alpha$|d|$\theta$|
|:---:|:---:|:---:|:---:|
|$a_T$|$\alpha_T$|$d_T$|$\theta_T$|

大多数的6轴转动机械臂都是由前三轴控制位置，后三轴控制姿态变换。
一般的，令轴4、轴5、轴6的转动轴交于一点，使轴2、轴3、轴4的中心点共面（共面不一定需要，但在计算时可以通过使三者共面来方便计算。下列计算默认三者共面，且三者组成的平面距离$^0P$为$d_1$。

为方便理解，这里使用实际的机械臂DH参数：

|i|a|$\alpha$|d|$\theta$|
|:---:|:---:|:---:|:---:|:---:|
|1|0|0|155.5|0|
|2|75.95|90| 7.05| 90|
|3|390| 0| 0| 0|
|4|117.5| 90|394|0|
|5|0|90|0|0|
|6|0|-90|0|0|

并带有一个工具：

|a|$\alpha$|d|$\theta$|
|:---:|:---:|:---:|:---:|
|0|0|119|0|

### 首先确定目标点的位置与姿态。

通过三个关键点就能够生成相应的变换矩阵 $_0^AH$ ，改变换等同于工具最终的姿态 $_0^TH$ ，步骤如下：
设目标点为 $A(X_A,Y_A,Z_A)^T$ ，存在另外两个点 $B(X_B,Y_B,Z_B)^T$ 、 $C(X_C,Y_C,Z_C)^T$ ，得到:

$$
\overrightarrow{AB} = B - A,
\overrightarrow{AC} = C - A,
\overrightarrow{Z'_A} = \overrightarrow{AB}\times\overrightarrow{AC},
\overrightarrow{Z_A} = Normalize(\overrightarrow{Z'_A}),
\overrightarrow{X_A} = Normalize(\overrightarrow{AB}),
\overrightarrow{Y_A} = Normalize(\overrightarrow{Z_A}\times\overrightarrow{X_A}),
_0^TH = _0^AH = \left[\begin{matrix}
			\overrightarrow{X_A}&\overrightarrow{Y_A}&\overrightarrow{Z_A}&A \\ 
			0&0&0&1 \\ 
			\end{matrix}\right]
$$

上述式子中Normalize代表归一化。

### 获取前三轴的姿态

得到目标点的位置与姿态后，使用工具与轴6的逆变换得到轴4、5、6的坐标系原点 $^4P$ ，即：

$$
^4P = _0^TH _6^TH^{-1} {}^0P
$$

得到 $^4P$ 后，可以通过 $^4P$ 与 $^0P$ 得到 $\theta_1$ ，具体如下:
由于轴2、3、4共面，且平面距离 $^0P$ 为 $d_1$ ，我们记 $^4P$ 在轴1 X-Y 坐标系平面上与 $^0P$ 的距离为 $d_{14}$ ，且 $^4P$ 在轴1 X-Y平面上的投影为  $^4P'$ 。

想象在轴1 X-Y平面上存在一个以 $^0P$ 为圆心，半径为 $d_1$ 的圆，那么有过 $^4P'$ 的切线 $L_{^4P'}$（显然有两条，根据实际情况取其一），该切线与 $^0P$ 距离为 $d_1$ 且与轴2、3、4坐标系原点组成的平面共面。设 $^4P'$ 与 $^0P$ 的连线与所在平面坐标系X轴的夹角为 $\theta_{04}$ ，$^0P$ 与 $L_{^4P'}$ 的切点的连线与 $^4P'$ 与 $^0P$ 的连线形成的夹角为 $\alpha_{04}$，则有：

$$
\alpha_{04} = \arccos\frac{d_{14}}{\sqrt{(X_{^4P'} - X_{^0P})^2 +(Y_{^4P'} - Y_{^0P})^2 }}\\

\theta_{04} = \arctan2(Y_{^4P'},X_{^4P'})\\

\theta_1 = \alpha_{04} + \theta_{04} - \frac{\pi}{2}
$$

得到 $\theta_1$ 。更新轴1对应的变换矩阵 $_0^1H'$ ，得到轴2的坐标系原点：

$$
P = _1^2H_0^1H'^0P
$$

由于轴2、3转动轴朝向一致，且轴2、3、4坐标系原点共面，因此将其作为平面几何问题求解。
过程如下：

令:

$$
d_{24} = ||^4P - ^2P||，\overrightarrow{^2P^4P} = ^4P - ^2P，l_{23} = a_3，l_{34} = \sqrt{d^2_4 + a^2_4}\\
$$

则有：

$$
\theta_{offset3} = \arccos{\frac{d^2_4 + l^2_{34} - a^2_4}{2d_4l_{34}}}\\\theta_3 = \arccos{\frac{l^2_{23} + l^2_{34} -d^2_{24}}{2l_{23}l_{34}}} + \theta_{offset3} - \frac{\pi}{2}
$$

为什么出现了一个 $\theta_{offset3}$ ？我们观察式子，发现 $l_{34} = \sqrt{d^2_4 + a^2_4}$ ，这是因为在轴2、3、4所在的平面内，轴3与轴4的投影的连线并不是 $a_4$ ，一个直角三角形，斜边为 $\sqrt{d^2_4 + a^2_4}$ ，邻边为 $d_4$ ，对边为 $a_4$ ，而这个三角形在轴3处的顶角角度为 $\theta_{offset3}$ ，这个角度是不变的，由于初始时 $\theta_3$ 为0，而实际上轴4原点与轴3原点的连线与轴2的夹角为 $\theta_{\angle234} = (\theta_{3})_0 + \frac{\pi}{2} - \theta_{offset3} = \pi - \theta_{offset3}$ ，而根据实际的几何关系，$\theta_{\angle234} = \theta_3 + \frac{\pi}{2} - \theta_{offset3}$ ，从而得到： $\theta_3 = \theta_{\angle234} + \theta_{offset3} - \frac{\pi}{2}$ 。
同理， $\theta_2$ 的计算过程如下：
令：

$$
l_{h24} = \sqrt{(X_{^4P} - X_{^2P})^2 + (Y_{^4P} - Y_{^2P})^2}\\
l_{v24} = Z_{^4P} - Z_{^2P}\\
$$

故有：

$$
\alpha_{\angle423} = \arccos{\frac{l^2_{23} + d^2_{24} -l^2_{34}}{2l_{23}d_{24}}}\\
\theta_{24} = \arctan2(l_{h24},l_{v24})\\
\theta_2 = \alpha_{\angle423} + \theta_{24}
$$

由此我们得到了前三轴的逆运动学角度。

### 获取后三轴的姿态

通过前三轴我们可以得到轴4的初始姿态 $_0^4H$ ，即：

$$
_0^4H = _3^4H_2^3{H'}_1^2{H'}_0^1{H'}
$$

注意，此时 $_3^4H$ 还未更新。得到轴4的姿态后，通过 $_0^4H$ 的第三列与第二列即可获取当前坐标系下的Z基底 $\overrightarrow{Z_{temp4}}$ 与Y基底 $\overrightarrow{Y_{temp4}}$ 。通过 $\overrightarrow{Z_{temp4}}$ 与 $\overrightarrow{Z_A}$ 以及 $\overrightarrow{Y_{temp4}}$ 我们可以得到 $\theta_4$ 。过程如下:

$$
\overrightarrow{Y_{46}} = \overrightarrow{Z_{temp4}}\times\overrightarrow{Z_A} \\ 
\hat{\theta_{4}} = \arccos{\frac{\overrightarrow{Y_{46}}\bullet\overrightarrow{Y_{temp_4}}}{||\overrightarrow{Y_{46}}||||\overrightarrow{Y_{temp4}}||}}
$$

通过 $\hat{\theta_{4}}$ 得到临时变量 $\hat{^4_0H}$ ，通过临时变换所得到的坐标系X轴与 $\overrightarrow{Y_{46}}$ 是否正交得到 $\theta_{4}$ 的符号。当前二者正交时，角度为正， $\theta_4 = \hat{\theta_4}$ ，否则 $\theta_4 = -\hat{\theta_4}$ 。

同理，我们可以获取轴5的角度。过程如下：
首先得到轴5当前的变换矩阵，通过其第二列获取其Y基底 $\overrightarrow{Y_{temp5}}$ 

$$
_0^5H = _4^5H_3^4{H'}_2^3{H'}_1^2{H'}_0^1{H'}
$$

上述式子中使用了更新后的变换 $_3^4{H'}$ ，实际上，不使用更新后的变换不影响计算 $\theta_5$ 。这是因为Z轴为旋转轴，而计算 $\theta_5$ 时考虑的是轴4的Z轴与轴6的Z轴（亦是目标点的Z轴 $\overrightarrow{Z_A}$ ）。

随后有:

$$
\theta_5 = \arccos{\frac{-\overrightarrow{Y_{temp5}}\bullet\overrightarrow{Z_A}}{||\overrightarrow{Y_{temp5}}||||\overrightarrow{Z_A}||}}
$$

由于 $\theta_4$ 已经限制了轴5在 $[0,\pi]$ 之间便可以指向目标，因此不需要再判别象限。之所以使用轴5的-Y轴是更具当前使用的机械臂DH参数决定的，在当前参数下，轴5的Y轴正方向为目标Z轴的负方向。

最后计算 $\theta_6$ ，过程与前二者类似。首先得到轴6当前的变换矩阵，通过其第一列获取其X基底 $\overrightarrow{X_{temp6}}$ 。

$$
_0^6H = _5^6H_4^5{H'}_3^4{H'}_2^3{H'}_1^2{H'}_0^1{H'}
$$

随后得：

$$
\hat{\theta_{6}} = \arccos{\frac{\overrightarrow{X_A}\bullet\overrightarrow{X_{temp6}}}{||\overrightarrow{X_A}||||\overrightarrow{X_{temp6}}||}}\\
$$

和轴4一样，轴6也需要在得到 $\hat{\theta_6}$ 后得到临时变换 $\hat{^6_0H}$ ，并通过临时变换的到的坐标系X轴与目标坐标系Y轴内积判断是否内积来得到 $\theta_6$ 的正负性。当前二者正交时， $\theta_6 = \hat{\theta_6}$ ，否则 $\theta_6 = -\hat{\theta_6}$ 。

至此，后三轴的角度也计算完成。

## 演示

这里使用C++与PCL库实现逆解运算中的DH参数对应的机械臂。

### 测试一

首先，通过正向运动学得到末端姿态，再通过逆向运动学测试是否能够完成逆向运动学：


![](https://github.com/ClamJom/Six-Axis-Robot-Simulate/blob/master/imgs/57bc66d4bd16416eab1880024c1b91cc.png "测试一")
结果：

![](https://github.com/ClamJom/Six-Axis-Robot-Simulate/blob/master/imgs/b63b45497bc448ce8cfdcd24c787eb6f.png "测试一的结果")
以及输出的结果：

```
0      0      1 588.95
0     -1      0   7.05
1      0     -0    663
0      0      0      1
588.95
7.05
663
1
```

前四行是目标姿态，后四行是IK之后FK得到的结果向量。

视图正中央的小球为工具末端，白线为工具，末端所在的坐标轴其实是在目标点建立的坐标系，红色为X，蓝色为Z，绿色为Y，黄色是辅助点和目标点的连线，这里与Y轴重合。

接下来的测试与测试一类似。

### 测试二

给定某三个点，计算IK。
给定的三个点为 $A = (400,400,400), B = (400,405,405), C = (400,410,400)$ 。
测试代码段：

![](https://github.com/ClamJom/Six-Axis-Robot-Simulate/blob/master/imgs/3d1aa2b701ec42bf87d57e4abd844f2a.png "测试二")
结果：

![](https://github.com/ClamJom/Six-Axis-Robot-Simulate/blob/master/imgs/79c0371845194f5baa6de3b8912f06c6.png "测试二的结果")
输出：

```
0         0         1       400
0.707107 -0.707107        -0       400
0.707107  0.707107        -0       400
0         0         0         1
400
400
400
  1
```

## Q&A

1. Q：为什么程序与推导过程有地方不符合？以及为什么角度要取反？
   A：主要的不符合推导过程的地方在于计算轴3的角度，关于 $\theta_{offset3}$ 的符号问题，这一点不清楚，在程序实现时按照公式写得到的是错误的结果，可能是由于角度全部取反后导致的过程变化，这其中的疏忽造成了这一结果。至于角度为什么取反，因为一开始调试时写错了一个正负号导致角度反转而我自己没意识到，以为PCL库是顺时针为正，因此全部取反了角度并编写了程序，但意外得到了正确的结果。
2. Q： 你这过程可靠吗？
   A：不可靠，本人是初学者，两周时间学习并得到的自己的想法并误打误撞实现了目的，因此还需要读者自己更加严谨的推导以及自己思考，我只是分享我学习的一些总结与想法，实际是否如此本人并不清除。由于本人研究这些内容本身就已经超出本人的专业方向，因此再次申明：仅供参考，请依据实际情况自行研究思考。
