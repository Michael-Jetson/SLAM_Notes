# 机器人运动学
## 两轮差速底盘运动学模型
## 三轮全向底盘运动学模型

### 全向模型

好处：
- 任何方向平移
- 结构简单
- 全驱动系统（在$xy\theta$三个方向上的运动是可以完全解耦的，可以分别进行分析，更容易分析）
- 可以进行运动学分解
以车辆中心为原点，以一个轮子的方向为X正方向建立坐标系
### 运动分解——平移X
- 我们设$v_x\neq 0,V_y=\omega=0$
- $V_1=0*V_x$
- $V_2=-sin{60}*V_x$
- $V_3=sin{60}*V_x$
### 运动分解——平移Y
- 我们设
- $V_1=0*V_x$
- $V_2=-sin{60}*V_x$
- $V_3=sin{60}*V_x$

### 运动分解——旋转

- 旋转的时候，明显各点角速度一致
- 我们设$V_x=0,V_y=0,V_\theta\neq 0$
- $V_1=0*V_x$
- $V_2=-sin{60}*V_x$
- $V_3=sin{60}*V_x$

### 运动合成

因为每种情况下只有一种分量不为0，所以我们将运动进行合成
$$
\begin{align}
&V_{1}  =0*V_{x} +1*V_{y}  +&d*V_{\theta}& \\
&V_{1}  =0*V_{x} +1*V_{y} +&d*V_{\theta} &
\end{align}
$$

$$
\begin{align}
	y & =d & z & =1\\
	y & =cx+d & z & =x+1\\
	y_{12} & =bx^{2}+cx+d & z & =x_{2}+x+1\nonumber \\
	y(x) & =ax^{3}+bx^{2}+cx+d & z & =x^{3}+x^{2}+x+1
\end{align}
$$

## 正向/逆向运动学

- 正向运动学是通过各个轮子的速度合成为坐标系下运动
- 逆向运动学是通过期望的运动速度反向求解各个轮子的速度

- 这里各个轮子的速度可以通过编码器求得

## 航迹推算

### 示意图

### 递推公式

- $(x,y,\theta)$为底盘当前位姿

- $(dx,dy,d\theta)$为底盘运动学结算增量

- 我们假设一个机器人底盘到世界坐标系的变换
  $$
  T_{V2W}=\left[
  \begin{matrix}
  cos\theta&-sin\theta &x\\
  sin\theta & cos\theta &0\\
  0&0&1
  \end{matrix}
  \right]
  $$

- 但是实际上所测得的增量是有误差的，误差来源：系统误差（比如说轮子测量误差）和随机误差（比如说打滑）

- 为了减小系统误差，我们需要进行标定

# 里程计标定

## 线性最小二乘法的基本原理

可以不关心底层的具体细节直接使用，但是最好基于自己的模型进行修改

线性方程组$Ax=b$（工程里面最常见的情况）

- $A$为$m\times n$的矩阵
- $x$为$n\times 1$的向量是我们要求解的状态量，可以是：机器人的位姿，机器人特征点的位置
- 每一行表示一个约束，每一列表示一个自由度或者说未知数的维数
- 当$m=n$的时候，约束正好，称为适定方程组，有唯一解（A可以有广义逆，所以必定有解）
- 当$m<n$时，约束不足，称为欠定方程组，方程组有无穷多解
- 当$m>n$时，约束过多，称为超定方程组，通常无解，也是实际情况中最经常碰到的情况
- 因为实际上我们一次采样的数量会非常多（每次采样就是一个约束），所以往往会得到一个超定方程组，因为噪声原因，他们彼此之间都是矛盾的，所以是无法得到一个解的

### 最小二乘解

- 绝大多数情况都是$m>n$，是超定方程组
- 无解，但是我们可以寻找最接近真实解的解
- 无解但是有最小二乘解
- 通解：$x^*=(A^TA)^{-1}A^Tb$
- 注意，在这里$Ax=b$是一个病态的方程，这是因为给$b$加一点微小的扰动，就会造成方程组解的巨大变化，所以在实际工程里面，我们会对$A^TA$进行一个QR分解来让解更稳定

### 线性空间角度看最小二乘法

- $Ax$表示$A$的列向量空间$S$，这是因为当$x$取尽每一个值的时候，$Ax$就表示$A$的列向量张成的组合，即列向量空间，或者叫$A$的列空间

- 方程组无解就意味着向量$b$不在$S$中

- 显然，最近的解即为：向量$b$在$S$中的投影

- 我们设$Ax^*$为向量$b$在空间$S$中的投影，显然$(b-Ax^*)$垂直于空间$S$

- 显然$(b-Ax^*)$与矩阵$A$的每一个列向量都垂直，那么显然有
  $$
  a_1^T(b-Ax^*)=0\\
  a_2^T(b-Ax^*)=0\\
  \cdots \\
  a_n^T(b-Ax^*)=0
  $$
  即有
  $$
  A^T(b-Ax^*)=0\\
  A^Tb=A^TAx^*\\
  x^*=(A^TA)^{-1}A^Tb
  $$
  

## 直线拟合

我们假设有这种理想情况：数据完美符合直线$y=5x+2$

![image-20230327182708046](/home/robot/.config/Typora/typora-user-images/image-20230327182708046.png)

但是实际上采样会混入噪声，所以我们有了采样数据

![./img/image-20230327183816465.png)
$$
x=(1,2,3,4,5,6,7,8,9,10)\\
y=(6.9918,14.2987,16.2019,22.4263,25.6191,33.2563,35.7755,42.0298,47.9954,53.9545)
$$
我们构建方程组：
$$
\left [ 
\begin{matrix}
x_1 &1\\
x_2 &1\\
\vdots & \vdots \\
x_n &1
\end{matrix}
\right]
*
\left [ 
\begin{matrix}
a\\
b
\end{matrix}
\right]
=
\left [ 
\begin{matrix}
y_1 \\
y_2 \\
\vdots \\
y_n
\end{matrix}
\right]
$$
即有
$$
Ax=b
$$
于是我们进行拟合
$$
A^TA=
\left [ 
\begin{matrix}
x_1 &x_2& \cdots &x_n\\
1&1&\cdots&1
\end{matrix}
\right]
*
\left [ 
\begin{matrix}
x_1 &1\\
x_2 &1\\
\vdots & \vdots \\
x_n &1\\
\end{matrix}
\right]
=
\left [ 
\begin{matrix}
\sum \limits ^n_{i=1}x^2_i &\sum \limits ^n_{i=1}x_i \\
\sum \limits ^n_{i=1}x_i&n\\ 
\end{matrix}
\right]
\\
(A^TA)^{-1}=
\left [ 
\begin{matrix}
\frac{-n}{B} &\frac{\sum \limits ^n_{i=1}x^2_i }{B}  \\
\frac{\sum \limits ^n_{i=1}x_i}{B}&\frac{\sum \limits ^n_{i=1}x^2_i}{B}\\ 
\end{matrix}
\right]，其中B=
(\sum \limits ^n_{i=1}x_i)^2-n\sum \limits ^n_{i=1}x_i^2
\\
解得：x=
\left [ 
\begin{matrix}
\frac{n\sum x_iy_i-\sum x_i \sum y_i}{n\sum x_i^2-\sum x_i\sum x_i}\\
\frac{\sum x_i^2\sum y_i-\sum x_iy_i\sum x_i}{n\sum x_i^2-\sum x_i\sum x_i}
\end{matrix}
\right]
$$
代入数据有：
$$
\sum x_i^2=385\\
\sum x_i=55\\
\sum x_iy_i=2059.7039\\
\sum y_i=298.5494
$$
解得：
$$
a=\frac{2059.7039*10-298.5494*55}{385*10-55*55}=5.063\\
b=\frac{385*298.5494-2059.7039*55}{385*10-55*55}=2.009
$$

拟合效果如图

![拟合对比](/home/robot/.config/Typora/typora-user-images/image-20230327182820209.png)

- 迭代最小二乘法

在实际过程中，数据不是一次性出现的，而是不断出现的

迭代最小二乘法的用处是可以不断进行标定，用于数据不断过来的情况

最小二乘法与卡尔曼滤波是有联系的，或者可以认为最小二乘法是一个滤波的过程（当卡尔曼滤波的过程噪声是0的时候，卡尔曼就是迭代最小二乘）

## 直接线性方法标定里程计

- 我们使用激光雷达的scan-match数据作为真值$u^*_i$，因为我们认为激光雷达的观测值比里程计要准确，这里$u_i^*$表示第$i$帧和第$j$帧之间的相对位姿关系，这里用$P_i$表示第$i$帧的位姿，于是有$u_i=P^{-1}_iP_j$

- 里程计测量得到的数据为$u_i$

- 假设他们之间成线性关系$u_i^*=X*u_i$

  其中：因为我们是在二维平面上进行标定，所以$u_i$这些都是三维向量，$u_i=(u_{ix},u_{iy},u_{i\theta})$
  $$
  X=
  \left[
  \begin{matrix}
  x_{11}&x_{12}&x_{13}\\
  x_{21}&x_{22}&x_{23}\\
  x_{31}&x_{32}&x_{33}\\
  \end{matrix}
  \right]
  $$
  

（因为我们认为里程计是有累计误差的，是不准确的，所以我们需要一种更准确的方式来校准里程计，我们这里使用scan-match也就是使用激光雷达的扫描数据计算里程计，即激光里程计，显然这种方式没有累计误差，会更准确。在这里我们不考虑如何计算激光里程计或者激光里程计是怎么来的，我们只需要知道有这种里程计可以使用即可）

对于每一组数据，可得
$$
u_{ix}*x_{11}+u_{iy}*x_{12}+u_{i\theta}*x_{13}=u_{ix}^*\\
u_{ix}*x_{21}+u_{iy}*x_{22}+u_{i\theta}*x_{23}=u_{iy}^*\\
u_{ix}*x_{31}+u_{iy}*x_{32}+u_{i\theta}*x_{33}=u_{i\theta}^*\\
\left[
\begin{matrix}
u_{ix}&u_{iy}&u_{i\theta}&0&0&0&0&0&0\\
0&0&0&u_{ix}&u_{iy}&u_{i\theta}&0&0&0\\
0&0&0&0&0&0&u_{ix}&u_{iy}&u_{i\theta}\\
\end{matrix}
\right]
\left[
\begin{matrix}
x_{11}\\
\vdots\\
x_{33}
\end{matrix}
\right]
=
\left[
\begin{matrix}
u_{ix}^*\\
u_{iy}^*\\
u_{i\theta}^*\\
\end{matrix}
\right]\\
A=
\left[
\begin{matrix}
A_1\\
\vdots\\
A_n\\
\end{matrix}
\right]
\hspace{3em}
b=
\left[
\begin{matrix}
b_1\\
\vdots\\
b_n\\
\end{matrix}
\right]
\hspace{3em}
\vec{X}=(A^TA)^{-1}A^Tb
$$
