# 三维空间刚体运动

想描述三维空间中的物体运动，就必须先确定坐标系的概念，因为运动都是相对的，我们无法描述一个物体的绝对运动情况，只能描述一个物体的相对运动情况

## 点与坐标系

对于我们所关心的对象，都会有一个与之相关的坐标系，比如说一个激光雷达就会有一个雷达坐标系，有一个相机就会有一个相机坐标系，一般我们关心多少物体就会有多少坐标系

### 坐标系

在机器人领域，一般使用右手系，机器人的运动也都是在右手系里面进行讨论

![image-20230328135941278](/home/robot/文档/Notes/img/image-20230328135941278.png)

如我们在激光坐标系下观测到某些情况，在相机坐标系下拍摄到某些情况

### 向量运算

对于向量$a,b$

定义内积（也就是点乘）：
$$
a \cdot b=a^Tb=\sum\limits^3_{i=1}a_ib_i=\vert a \vert \vert b\vert \cos <a,b>
$$
定义外积（结果是一个与两个向量都垂直的向量）：
$$
\begin{align}
a\times b&=
\left|
\begin{matrix}
\bold{i}&\bold{j}&\bold{k}\\
a_1&a_2&a_3\\
b_1&b_2&b_3\\
\end{matrix}
\right|\\
&=
(a_2b_3-a_3b_2)\bold{i}+(a_3b_1-a_1b_3)\bold{j}+(a_1b_2-a_2b_1)\bold{k}\\

&=
\left[
\begin{matrix}
a_2b_3-a_3b_2\\
a_3b_1-a_1b_3\\
a_1b_2-a_2b_1\\
\end{matrix}
\right]\\
&=
\left[
\begin{matrix}
0&-a_3&a_2\\
a_3&0&-a_1\\
-a_2&a_1&0\\
\end{matrix}
\right]b\\
&\triangleq
a^\land b
\end{align}
$$

### 坐标变换

**问题**：如果我们在相机或者雷达坐标系下观测到一个对象，那么这个对象在世界坐标系下或者机器人坐标系下的位置是如何表示的呢？

这里就需要用数学公式来表述了，也就是坐标变换的问题：如何计算同一个向量在不同坐标系里的坐标？

实际上，两个坐标系之间的关系，只有旋转和平移两种，或者说，两个坐标系之间的欧式变换由旋转和平移组成

## 旋转矩阵

### 齐次坐标

$\tilde{a}=\left[\begin{matrix}a\\1\end{matrix}\right]，在摄影几何里面非常常见，乘以非零常数的时候仍然表达同一个坐标$

即：
$$
\tilde{a}=\left[\begin{matrix}a\\1\end{matrix}\right]=k\left[\begin{matrix}a\\1\end{matrix}\right]
$$

### 欧式群

变换矩阵的集合称为特殊欧式群$SE(3)$（Special Euclidean Group）
$$
SE(3)=
\left\{
T=
\left[
\begin{matrix}
R&t\\
0^T&1\\
\end{matrix}
\right]
\in
\mathbb{R}^{4\times 4}
\vert
R\in SO(3),t\in \mathbb{R}^3
\right.
$$
逆形式：
$$
T^{-1}=
\left[
\begin{matrix}
R^T&-R^Tt\\
0^T&1\\
\end{matrix}
\right]
$$

### 欧拉旋转定理（Euler's rotation theorem）

刚体在三维空间里的一般运动，可以分解为刚体上某一点的平移，以及绕经过此点的旋转轴的转动

即两个坐标系直接的运动可以完全用$R，t$描述
$$
a^{\prime}=Ra+t
$$
但是，如果使用旋转矩阵加平移向量，在复合情况下会有不便之处

如果有$b=R_1a+t_1$，$c=R_2b+t_2$

那么复合形式就是
$$
c=R_2(R_1a+t_1)+t_2
$$
如果使用齐次形式，那么就比较方便
$$
\left[
\begin{matrix}
\tilde{a}\\1
\end{matrix}
\right]
=
\left[
\begin{matrix}
R&t\\
0^T&1\\
\end{matrix}
\right]
\left[\begin{matrix}a\\1\end{matrix}\right]
\triangleq
T\left[\begin{matrix}a\\1\end{matrix}\right]
\\
\tilde{b}=T_1\tilde{a},\tilde{c}=T_2\tilde{b}\Rightarrow \tilde{c}=T_2T_1\tilde{b}
$$

## 旋转向量与欧拉角

#### 旋转向量

#### 欧拉角

实际上，欧拉角的定义方式比较多（XYZ三轴不同的先后顺序），而且会存在奇异性问题（万向锁这种），所以一般不会直接使用，在SLAM中也很少使用欧拉角表示姿态

#### 万向锁

这是欧拉角奇异性问题的一种体现，即旋转角在特定值的时候，会有两个旋转轴重合，这种情况下旋转自由度减一

正常情况下

![image-20230329201433675](/home/robot/文档/Notes/img/image-20230329201433675.png)

奇异情况下

![image-20230329201447449](/home/robot/文档/Notes/img/image-20230329201447449.png)

在这种时候，两轴重合，会存在这种情况

可以证明，无法在仅用三个实数表达旋转的时候避免奇异性问题

但是，如果使用四个数来表达旋转，则不会出现这种情况，这也就是四元数的用处

### 四元数

这是一种节省空间而且没有奇异性的表达形式，可以用来描述旋转

2D 情况下，可用单位复数表达旋转
$$
z=x+iy=\rho e^{i\theta}
$$
三维情况下，四元数就是复数的扩充

四元数（Quaternion）

**如何使用四元数描述旋转**

设有点$p$，在经历了一次四元数$q$表示的旋转之后，得到了$p^\prime$，他们的关系如何表述
$$
p^\prime=qpq^{-1}
$$

## 程序设计

使用动态矩阵的时候，运算会比较慢

# 李群和李代数

## 背景

当我们去估计相机位姿的时候，当我们的估计不同准确的时候，如何对旋转和平移进行微调呢？

但是我们可以发现，旋转矩阵是无法相加的，相加之后就不是一个旋转矩阵了，所以我们需要用一种新理论去完成这个操作。

### 问题

首先我们知道，运动$x$是可以被观测的，也就是说，我们可以使用一个观测模型$R,t$来描述，或者用一个变换矩阵$T$来描述

但是，在SLAM中有一个问题，值是估计出来的，如果我们发现不准确，那么就应该进行调整，得到新的估计，那么我们设调整后的旋转平移为$R^\prime,t^\prime$，设他们直接相差一个微小量$\Delta R,\Delta t$，即有
$$
t^\prime=t+\Delta t\\
R^\prime =R+\Delta R
$$
但是，我们知道$R$对乘法封闭，但是对加减是不封闭的，自然也没办法进行求导，因为优化是必须基于导数的，但是如果有一个函数$u(R)$，那么有
$$
\frac{du}{dR}=\lim\limits_{\Delta R\to 0}\frac{u(R+\Delta R)-u(R)}{\Delta R}
$$
但是由于其性质，我们无法完成这个求导操作，自然无法完成优化

## 群

三维旋转矩阵构成了特殊正交群$SO(3)$（Special Orthogonal Group）
$$
SO(3)=
\left\{
R
\in
\mathbb{R}^{3\times 3}
\vert
RR^T=I,det(R)=1
\right\}
$$


三维变换矩阵的集合称为特殊欧式群$SE(3)$（Special Euclidean Group）
$$
SE(3)=
\left\{
T=
\left[
\begin{matrix}
R&t\\
0^T&1\\
\end{matrix}
\right]
\in
\mathbb{R}^{4\times 4}
\vert
R\in SO(3),t\in \mathbb{R}^3
\right\}
$$
那么什么是群？

群是一种集合加上一种运算的代数结构（容易验证，旋转或者变换矩阵集合与矩阵乘法构成群，因此称为旋转矩阵群和变换矩阵群）

记集合为$A$，运算为$\cdot$，那么当运算满足以下性质时，称二元组$(A,\cdot)$构成群

1. 封闭性：$$

## 李群

李群是具有连续性质的群，或者说这个群是光滑的

既是群也是流形

直观上看，一个刚体能够连续地在空间中运动，故$SO(3)$和$SE(3)$都是李群。

但是，$SO(3)$和$SE(3)$只有定义良好的乘法，没有加法，所以难以进行取极限、求导等操作。

## 李代数