# 概述

这种机器人的自主运动过程中，有两大基本问题——我在什么地方？周围环境是什么样子的？也就是定位和建图的问题，而想完成定位与建图问题，就需要依靠传感器完成对外界环境的感知

传感器主要是有两种：

(1) 携带于机器人本体上的传感器，例：机器人的轮式编码器、相机、激光传感器、惯性测量单元(IMU)等

(2) 安装于环境之中的传感器，例：导轨、二维码标志等

但是环境传感器是有限制的，比如说GPS需要在能接收到卫星信号的环境下才可以工作

而本体上传感器具有以下优点：激光、相机等携带式传感器测量的通常都是一些间接的物理量而不是直接的位置数据，所以更加自由使用携带式传感器来完成SLAM也是我们重点关注的问题

## slam介绍

slam是什么呢？slam就是同时建图与定位，我们先不用学术的说法去介绍它，而是先用实际生活中的例子来介绍。

大家知道，如果你在一个地方生活久了，就会很熟悉这个地方，可能随便把你扔在其中某一个地点，你就可以知道你在什么位置，你应该怎么走回家，但是如果你出现在一个陌生的没去过的地方，你可能就一时半会不知道自己的位置了。

这个过程其实就是一个slam的过程，你在熟悉的环境下走路，相当于已经有一个地图在你大脑里面了，你需要根据眼睛看到的信息（相当于传感器感知环境）来判断自己在这个地图的什么地方，然后就可以进行自主导航了。

## 视觉传感器

视觉SLAM是本书的主题，所以我们非常关心小萝卜的眼睛能够做些什么事。即如何用相机解决定位和建图的问题。

- SLAM中使用的相机更加简单，以一定速率采集图像、形成视频,相机的特点就是以二维投影形式记录了三维世界的信息
- 但是该过程丢掉了一个维度：距离(或深度)

相机分类

- 单目相机 Monocular
- 双目相机(立体相机)Stereo
- 深度相机 RGB-D
- 其他 全景、Event Camera

## 单目相机

单目相机(只使用一个摄像头的相机)通过相机的运动形成视差，可以测量物体相对深度。

优点:结构简单，成本低，便于标定和识别

缺点:在单张图片里，无法确定一个物体的真实大小。它可能是一个很大但很远的物体，也可能是一个很近很小的物体。即单目SLAM估计的轨迹和地图将与真实的轨迹和地图相差一个因子也就是尺度(scale)，单凭图像无法确定这个真实尺度，所以称尺度不确定性。

## 双目相机

双目相机(由两个单目相机组成)通过基线来估计每个像素的空间位置。(类似于人眼)

优点:基线距离越大，能够测量的距离就越远;并且可以运用到室内和室外。

缺点:配置与标定较为复杂，深度量程和精度受到双目基线与分辨率限制，计算非常消耗计算资源，需要GPU(图形处理器)/FPGA设备(现场可编程门阵列)加速用两部相机来定位。

## 深度相机

深度相机(RGB-D)通过红外结构光或ToF(time of fly)的物理方法测量物体深度信息。

优点:相比于双目相机可节省大量的计算资源。

缺点:是测量范围窄，噪声大，视野小，易受日光干扰，无法测量透射材质等问题，主要用在室内，室外很难应用。
深度相机主要用来三维成像，和距离的测量。

## SLAM流程

1、传感器信息读取:在视觉SLAM中主要为相机图像信息的读取和预处理。

2、前端视觉里程计(Visual Odometry，VO)视觉里程计的任务是估计相邻图像间相机的运动，以及局部地图的样子。(VO又称为前端)

3、回环检测(Loop Closure Detection)用于判断机器人是否到达过先前的位置。

4、后端(非线性)优化(optimization)。对不同时刻的视觉里程计测量的相机位姿及回环检测的信息进行优化，得到全局一致的轨迹和地图。

5、建图(Mapping)。根据估计的轨迹，建立任务要求对应的地图。

### 前端视觉里程计

视觉里程计通过相邻帧间的图像估计相机运动，并恢复场景的空间结构，但只计算相邻时刻的运动，不关心再往前的信息。但前端过程中必然存在误差，误差会不断累积，形成累积漂移(会发现原本直的走廊变成了斜的，而原本90°的直角变成一歪的)。为消除漂移，我们需要回环检测和后端优化。

### 后端优化

定义:如何处理前端所传噪声的数据，从带有噪声的数据中估计整个系统的状态，以及这个状态估计的不确定性有多大--称为最大后验概率估计

通常来说，前端给后端提供待优化的数据，以及这些数据的初始值。后端负责整体的优化过程，它往往面对的就只有数据。

反映了SLAM问题的本质:对运动主体自身和周围环境空间不确定性的估计。(状态估计理论一估计状态的均值和不确定性)

### 回环检测

回环检测的作用:主要解决位置估计随时间漂移的问题(通俗的理解就是，假设机器人经过一段时间又回到了原点(事实)，但是我们的位置估计值没有回到原点，怎么解决)

回环检测要达到的目标:通过某种手段，让机器人知道“回到原点”这件事情，让机器人具有识别到过的场景的能力。再把位置估计值“拉”过去

相机检测手段:判断与之前位置的差异，计算图像间相似性。

回环检测后:可将所得的信息告诉后端优化算法，把轨迹和地图调整到符合回环检测结果的样子。

### 地图

地图大体上可分为以下两类:

1. 度量地图(强调精确的表示地图中的位置关系)，常用稀疏与稠密进行分类
   稀疏地图:即由路标组成的地图
   稠密地图:着重于建模所有看到的东西(可用于导航)(耗费大量的储存空间)

2. 拓扑地图(更加强调元素之间的关系)

   是一个图:由节点和边组成

   例:只关注A、B点是连通的，而不考虑如何从A点到达B点，不适用于表达较为复杂结构的地图

## 传感器对比

激光雷达与相机对比

- 激光雷达有效距离远，可以达到百米级别（可达300m），相机有效距离相对近
- 雷达贵，动辄上万，相机较为便宜，高速相机不过上千
- 激光雷达受天气影响较大，雨雪都会成为巨大干扰，因为雨雪也会反射激光
- 雷达重，激光较轻
- 在纹理不清楚或者光线过强过弱的地方，相机几乎无法运作

## SLAM数学表述

关于运动模型:
机器人会携带一个测量自身运动的传感器，这个传感器可以测量有关运动的读数，但不一定直接就是位置之差，还可能是加速度、角速度这些信息。我们可以用一个抽象的数学模型来描述

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bx_k%7D%3Df%28%5Coverrightarrow%7Bx_%7Bk-1%7D%7D%2C%5Coverrightarrow%7Bu_k%7D%2C%5Coverrightarrow%7Bw_k%7D%29%0A)

u是运动传感器的读数，w是运动过程中加入的噪声，上面公式即运动方程

关于观测模型:

机器人在 ![](https://www.zhihu.com/equation?tex=%5Coverrightarrow%7Bx_k%7D) 位置上看到某个路标点 ![](https://www.zhihu.com/equation?tex=%5Coverrightarrow%7By_j%7D) ,产生了一个观测数据 ![](https://www.zhihu.com/equation?tex=%5Coverrightarrow%7Bz_%7Bk%2Cj%7D%7D) ，同样可以用一个抽象的数学模型描述:

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bz_%7Bk%2Cj%7D%7D%3Dh%28%5Coverrightarrow%7By_j%7D%2C%5Coverrightarrow%7Bx_k%7D%2C%5Coverrightarrow%7Bv_%7Bk%2Cj%7D%7D%29%0A)

其中 v 是可观测的噪声

实际上，这只是一个简化的方程，描述了一种形式，在真实世界中，根据机器人的真实运动和传感器的种类，存在着若干种参数化形式。而考虑视觉SLAM时，传感器是相机，则观测方程就是“对路标点拍摄后，得到图像中的像素”的过程

举个例子解释:假设机器人在平面运动，那么位姿(位置+姿态)由两个位置的坐标和一个转角来描述

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bx_k%7D%3D%28x%2Cy%2C%5Ctheta%29%5ET%0A)

同时，运动传感器能够测量到机器人在任意两个时间间隔位置和转角的变化量

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bu_k%7D%3D%28%5CDelta%20x%2C%5CDelta%20y%2C%5CDelta%20%5Ctheta%29%5ET%0A)

于是，此时运动方程就可以写成

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bx_k%7D%3D%5Coverrightarrow%7Bx_%7Bk-1%7D%7D%2B%5Coverrightarrow%7Bu_k%7D%2B%5Coverrightarrow%7Bw_k%7D%0A)

关于观测方程，比如机器人携带着一个二维激光传感器(当激光传感器观测一个2D路标点时，可以测到路标点与机器人之间的距离 ![](https://www.zhihu.com/equation?tex=r) 和夹角 ![](https://www.zhihu.com/equation?tex=%5Cphi)​)，记路标点

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7By%7D%3D%28y_1%2Cy_2%29%5ET%0A)

位姿为

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bx_k%7D%3D%28x_1%2Cx_2%29%5ET%0A)

观测数据为

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bz_%7Bk%2Cj%7D%7D%3D%28r_%7Bk%2Cj%7D%2C%5Cphi_%7Bk%2Cj%7D%29%5ET%0A)

于是观测方程就可以写成:

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Bbmatrix%7D%0Ar_%7Bk%2Cj%7D%5C%5C%0A%5Cphi_%7Bk%2Cj%7D%0A%5Cend%7Bbmatrix%7D%0A%3D%0A%5Cbegin%7Bbmatrix%7D%0A%5Csqrt%7B%28%5Coverrightarrow%7By_%7B1%2Cj%7D%7D-%5Coverrightarrow%7Bx_%7B1%2Ck%7D%7D%29%5E2%2B%28%5Coverrightarrow%7By_%7B2%2Cj%7D%7D-%5Coverrightarrow%7Bx_%7B2%2Ck%7D%7D%29%5E2%7D%5C%5C%0A%5Carctan%28%5Cfrac%7B%5Coverrightarrow%7By_%7B2%2Cj%7D%7D-%5Coverrightarrow%7Bx_%7B2%2Ck%7D%7D%7D%7B%5Coverrightarrow%7By_%7B1%2Cj%7D%7D-%5Coverrightarrow%7Bx_%7B1%2Ck%7D%7D%7D%29%0A%5Cend%7Bbmatrix%7D%0A)


# 三维空间刚体运动

想描述三维空间中的物体运动，就必须先确定坐标系的概念，因为运动都是相对的，我们无法描述一个物体的绝对运动情况，只能描述一个物体的相对运动情况

## 点与坐标系

三维空间由3个轴组成，则一个空间点的位置可以由3个坐标指定

对于一个刚体(在运动和受力作用后，形状和大小不变，而且内部各点的相对位置不变的物体)，不光有位置，还有自身的姿态，如相机可以看成三维空间中的刚体，则位置就是说相机在空间中的哪个地方，姿态是指相机的朝向(相机处于空间(0，0，0)处，朝向正前方)，这些情况都需要使用数学语言进行描述

点没有长度，没有体积，但是点和点可以组成向量

向量是带指向性的箭头(有方向性)，可以进行加法、减法等运算

### 坐标系

当我们指定坐标系后，才可以谈论该向量在此坐标系下的坐标

在机器人领域，一般使用右手系，机器人的运动也都是在右手系里面进行讨论

![image-20230328135941278](./img/image-20230328135941278.png)

如我们在激光坐标系下观测到某些情况，在相机坐标系下拍摄到某些情况

### 向量运算

对于向量![](https://www.zhihu.com/equation?tex=a%2Cb) 可以定义内积（也就是点乘)

然后定义外积（结果是一个与两个向量都垂直的向量，外积向量长度就是两向量为临边的平行四边形面积）：

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Balign%7D%0Aa%5Ctimes%20b%26%3D%0A%5Cleft%7C%0A%5Cbegin%7Bmatrix%7D%0A%5Cbold%7Bi%7D%26%5Cbold%7Bj%7D%26%5Cbold%7Bk%7D%5C%5C%0Aa_1%26a_2%26a_3%5C%5C%0Ab_1%26b_2%26b_3%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%7C%5C%5C%0A%26%3D%0A%28a_2b_3-a_3b_2%29%5Cbold%7Bi%7D%2B%28a_3b_1-a_1b_3%29%5Cbold%7Bj%7D%2B%28a_1b_2-a_2b_1%29%5Cbold%7Bk%7D%5C%5C%0A%0A%26%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0Aa_2b_3-a_3b_2%5C%5C%0Aa_3b_1-a_1b_3%5C%5C%0Aa_1b_2-a_2b_1%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%5C%5C%0A%26%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0A0%26-a_3%26a_2%5C%5C%0Aa_3%260%26-a_1%5C%5C%0A-a_2%26a_1%260%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5Db%5C%5C%0A%26%5Ctriangleq%0%0Aa%5E%5Cland%20b%0A%5Cend%7Balign%7D%0A)


### 坐标变换

**问题**：如果我们在相机或者雷达坐标系下观测到一个对象，那么这个对象在世界坐标系下或者机器人坐标系下的位置是如何表示的呢？

这里就需要用数学公式来表述了，也就是坐标变换的问题：如何计算同一个向量在不同坐标系里的坐标？

实际上，两个坐标系之间的关系，只有旋转和平移两种，或者说，两个坐标系之间的欧式变换由旋转和平移组成

## 旋转矩阵

实际上，某个坐标系进行旋转，一个固定向量 ![](https://www.zhihu.com/equation?tex=a) 在旋转前后的坐标变化可以使用旋转矩阵 ![](https://www.zhihu.com/equation?tex=R)​ 计算

![](https://www.zhihu.com/equation?tex=%0Aa%3DRa%5E%5Cprime%0A)

中间的矩阵R为旋转矩阵，描述了两个坐标系的变换关系，可以证明：

R是一个正交矩阵，行列式为+1

取行列式为1的正交矩阵称为旋转矩阵

![](https://www.zhihu.com/equation?tex=%0ASO%28n%29%3D%5C%7BR%5Cin%20%5Cmathbb%7BR%7D%5E%7Bn%5Ctimes%20n%7D%7CRR%5ET%3DI%2Cdet%28R%29%3D1%5C%7D%0A)

其中 ![](https://www.zhihu.com/equation?tex=SO%28n%29)​ 是特殊正交群

如果考虑平移关系，则有

![](https://www.zhihu.com/equation?tex=%0Aa%3DRa%5E%5Cprime%2Bt%0A)

这样就可以完整的描述旋转和平移的关系

## 旋转矩阵的左乘/右乘

旋转矩阵的左乘和右乘在数学和物理上有不同的意义，主要体现在旋转的顺序和方向上。

实际上，旋转矩阵有两种物理意义

1. 坐标系的旋转，其应用场景有SLAM，机械臂运动学等，也就是其中的点本质上是没有移动的，只是坐标系发生了改变，导致点在两个坐标系下有了不同的位姿表示

   ![](https://pic3.zhimg.com/v2-aa06a0cd65ed3438c4a81417b56c0c0e_1440w.jpg)

   其中有：![](https://www.zhihu.com/equation?tex=P%3DR_z%28%5Calpha%29P%5E%5Cprime)

2. 向量的旋转，其应用场景有机器人的姿态估计等，在这个情况下，点本身是发生了移动的，点围绕一个坐标系原点进行旋转

   ![](https://pic3.zhimg.com/v2-b6cc20098234793618dd553bb7e3621e_1440w.jpg)

   其中有：![](https://www.zhihu.com/equation?tex=P%3DR_z%28%5Calpha%29P%5E%5Cprime)

实际上，上面两种物理意义下，矩阵乘法是一致的，也就是在这种矩阵乘法下，两种物理意义不冲突

那么旋转矩阵的左乘和右乘有什么区别呢

1. 左乘旋转矩阵：这种表示方式通常用于描述绕固定坐标系进行旋转，或者说是主动旋转（active rotation），旋转矩阵可以连续左乘

   左乘即旋转向量本身。这意味着我们将向量 ![](https://www.zhihu.com/equation?tex=v) 看作是固定在一个坐标系中，然后对这个向量应用旋转矩阵 ![](https://www.zhihu.com/equation?tex=R) 来得到旋转后的向量 ![](https://www.zhihu.com/equation?tex=v%5E%5Cprime)。

   例如，如果 ![](https://www.zhihu.com/equation?tex=R) 是一个绕 z 轴旋转的矩阵，左乘表示向量 ![](https://www.zhihu.com/equation?tex=v) 围绕 z 轴旋转。

2. 右乘旋转矩阵：这种表示方式通常用于绕动态的坐标系旋转，也就是以前一时刻的位置为旋转中心进行旋转，是被动旋转（passive rotation），即旋转坐标系而不是向量

   这意味着我们将向量 ![](https://www.zhihu.com/equation?tex=v) 看作是固定的，然后对坐标系应用旋转矩阵 ![](https://www.zhihu.com/equation?tex=R) 来得到旋转后的向量 ![](https://www.zhihu.com/equation?tex=v%5E%5Cprime) （可以理解为在新坐标系下的坐标)。右乘通常用于描述坐标系的旋转，而不是旋转向量本身。

   例如，如果 ![](https://www.zhihu.com/equation?tex=R) 是一个绕 z 轴旋转的矩阵，右乘表示坐标系围绕 z 轴旋转。

以自动驾驶场景为例，有车身的自车坐标系和全局的地面坐标系

### 齐次坐标

虽然旋转矩阵可以表达欧式空间的旋转和平移，但是不是一种线性关系，多次变换的复合计算是多次叠加，很复杂，则可以使用齐次坐标改写变换矩阵

即：


![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Bbmatrix%7D%0Aa%5E%5Cprime%20%5C%5C%0A1%0A%5Cend%7Bbmatrix%7D%0A%3D%5Cbegin%7Bbmatrix%7D%0AR%20%26t%20%5C%5C%0A0%5ET%20%261%0A%5Cend%7Bbmatrix%7D%0A%5Cbegin%7Bbmatrix%7D%0Aa%5C%5C%0A1%0A%5Cend%7Bbmatrix%7D%0A%5Ctriangleq%0%0AT%0A%5Cbegin%7Bbmatrix%7D%0Aa%5C%5C%0A1%0A%5Cend%7Bbmatrix%7D%0A)

这样就简化了记忆和计算，多次变换可以直接进行乘法计算

这种用四个数表达三维向量的做法称为齐次坐标，引入齐次坐标后，旋转和平移可以放在同一个矩阵中，称为变换矩阵，变化矩阵的集合定义为

![](https://www.zhihu.com/equation?tex=%0ASE%283%29%3D%5C%7B%0AT%3D%0A%5Cbegin%7Bbmatrix%7D%0AR%20%26t%20%5C%5C%0A0%5ET%20%261%0A%5Cend%7Bbmatrix%7D%0A%5Cin%20%5Cmathbb%7BR%7D%5E%7B4%5Ctimes%204%7D%7CR%5Cin%20SO%283%29%2Ct%20%5Cin%20%5Cmathbb%7BR%7D%5E3%5C%7D%0A)

称为特殊欧式群，并且可以定义反向的变换

![](https://www.zhihu.com/equation?tex=%0AT%5E%7B-1%7D%3D%0A%5Cbegin%7Bbmatrix%7D%0AR%5ET%20%26-R%5ETt%20%5C%5C%0A0%5ET%20%261%0A%5Cend%7Bbmatrix%7D%0A)

但是这种变换矩阵的方法是有缺陷的：

1. SO(3)的旋转矩阵有九个量，但一次旋转只有三个自由度。因此这种表达方式是冗余的。同理，变换矩阵用十六个量表达了六自由度的变换。那么，是否有更紧凑的表示呢?
2. 旋转矩阵自身带有约束:它必须是个正交矩阵,且行列式为 1。变换矩阵也是如此。当我们想要估计或优化一个旋转矩阵,变换矩阵时，这些约束会使得求解变得更困难。

## 旋转向量与欧拉角

#### 旋转向量——除了旋转矩阵之外的旋转表示

旋转矩阵表示旋转是冗杂的(旋转矩阵有9个量，但一次旋转只有3个自由度并且旋转矩阵自身带有约束)，所以有了新的表示方法——旋转向量，不过要注意一下，旋转向量与旋转矩阵只是表达方式不同，但是表达的内容是相同的
旋转向量：三自由度，用 ![](https://www.zhihu.com/equation?tex=R%5E3)​ 向量表示，方向为旋转轴，长度为转过的角度，定义为

![](https://www.zhihu.com/equation?tex=%0A%5Coverrightarrow%7Bw%7D%3D%5Ctheta%20%5Coverrightarrow%7Bn%7D%0A)

其与旋转矩阵的转换方式为罗德里格斯公式

![](https://www.zhihu.com/equation?tex=%0A%5Cmathbf%7BR%7D%20%3D%20%5Ccos%20%5Ctheta%20%5Cmathbf%7BI%7D%20%2B%20%281%20-%20%5Ccos%5Ctheta%29%20nn%5ET%20%2B%5Csin%5Ctheta%20n%5E%5Cwedge%0A)

而反过来，旋转矩阵转旋转向量为

![](https://www.zhihu.com/equation?tex=%0A%E8%A7%92%E5%BA%A6%EF%BC%9A%5Ctheta%3D%5Carccos%28%5Cfrac%7Btr%28R%29-1%7D%7B2%7D%29%5C%5C%0A%E8%BD%B4%EF%BC%9ARn%3Dn%0A)

其中转轴是矩阵 R 特征值1对应的特征向量

实际上这个计算过程是这样的：

对于转角 ![](https://www.zhihu.com/equation?tex=%5Ctheta) ，取两边的迹，有

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Baligned%7D%0Atr%28R%29%26%3D%5Ccos%20%5Ctheta%20tr%28%5Cmathbf%7BI%7D%29%20%2B%20%281%20-%20%5Ccos%5Ctheta%29%20tr%28nn%5ET%29%20%2B%5Csin%5Ctheta%20tr%28n%5E%5Cwedge%29%5C%5C%0A%26%3D3%5Ccos%5Ctheta%2B%281-%5Ccos%5Ctheta%29%5C%5C%0A%26%3D1%2B2%5Ccos%5Ctheta%0A%5Cend%7Baligned%7D%0A)

关于转轴，旋转轴上的向量在旋转后不发生改变，说明

![](https://www.zhihu.com/equation?tex=%0ARn%3Dn%0A)

说明转轴是矩阵 R 特征值1对应的特征向量

具体的罗德里格斯公式的证明过程，可以考虑将向量分解为垂直于旋转向量和沿着旋转向量的分量，然后分别计算旋转之后的情况

#### 欧拉角

无论是旋转矩阵、旋转向量，虽然它们能描述旋转，但对我们人类是非常不直观的。当我们看到一个旋转矩阵或旋转向量时很难想象出来这个旋转究竟是什么样的。当它们变换时，我们也不知道物体是向哪个方向在转动。

而欧拉角提供了一种非常直观的方式
欧拉角将旋转分解为三次不同轴上的转动，以便理解

- 例如按照Z-Y-X转动
- 轴顺序亦可不同，因此存在许多种定义方式不同的欧拉角
- 其中ZYX 顺序（航向-俯仰-滚转）是常用的一种，顺序上首先围绕 z 轴旋转，接着围绕新的 y 轴旋转，最后围绕新的 x 轴旋转。广泛应用于航空航天和机器人学中，用来描述航向（yaw）、俯仰（pitch）和滚转（roll）。

实际上，欧拉角的定义方式比较多（XYZ三轴不同的先后顺序），而且会存在奇异性问题（万向锁这种），所以一般不会直接使用，在SLAM中也很少使用欧拉角表示姿态，因为欧拉角存在不连续和奇异点问题

#### 万向锁

这是欧拉角奇异性问题的一种体现，即旋转角在特定值的时候，会有两个旋转轴重合，这种情况下旋转自由度减一

由于万向锁问题，欧拉角不适合插值和迭代，往往用于人机交互中，并且可以证明，用三个实数来表达三维旋转时，会不可避免地碰到奇异性问题。所以SLAM程序中很少直接用欧拉角表示姿态

正常情况下

![image-20230329201433675](https://raw.githubusercontent.com/Michael-Jetson/SLAM_Notes/main/img/image-20230329201433675.png)

奇异情况下

![image-20230329201447449](https://raw.githubusercontent.com/Michael-Jetson/SLAM_Notes/main/img/image-20230329201447449.png)

在这种时候，两轴重合，会存在这种情况

但是，如果使用四个数来表达旋转，则不会出现这种情况，这也就是四元数的用处

### 四元数

这是一种节省空间（紧凑）而且没有奇异性的表达形式，可以用来描述旋转

2D 情况下，可用单位复数表达旋转

![](https://www.zhihu.com/equation?tex=%0Az%3Dx%2Biy%3D%5Crho%20e%5E%7Bi%5Ctheta%7D%0A)

三维情况下，四元数就是复数的扩充

四元数（Quaternion）

**如何使用四元数描述旋转**

设有点![](https://www.zhihu.com/equation?tex=p)，在经历了一次四元数![](https://www.zhihu.com/equation?tex=q)表示的旋转之后，得到了![](https://www.zhihu.com/equation?tex=p%5E%5Cprime)，他们的关系如何表述

![](https://www.zhihu.com/equation?tex=%0Ap%5E%5Cprime%3Dqpq%5E%7B-1%7D%0A)

而如何使用四元数表示旋转呢？

任意单位向量v，沿着以单位向量定义的旋转轴u旋转 ![](https://www.zhihu.com/equation?tex=%5Ctheta) 度之后的 v 可以使用四元数乘法来得到，此时可以令 ![](https://www.zhihu.com/equation?tex=v%3D%5B0%2Cv%5D)，令 ![](https://www.zhihu.com/equation?tex=p%3D%5B%5Ccos%20%5Cfrac12%5Ctheta%2Cu%5Csin%20%5Cfrac12%5Ctheta%5D)

则有：![](https://www.zhihu.com/equation?tex=v%5E%5Cprime%3Dpvp%5E%2A%3Dpvp%5E%7B-1%7D)​

并且，变换后的四元数实部为0，虚部是罗德里格斯公式的结果


## 程序设计

使用动态矩阵的时候，运算会比较慢

# 李群和李代数

## 背景

在SLAM中，除了表示之外，还要对它们进行估计和优化，因为SLAM整个过程就是在不断地估计机器人的位姿与地图，该位姿是由旋转矩阵或变换矩阵描述的。为了优化位姿，需要对变换矩阵进行插值、求导、迭代等操作，比如说当我们去估计相机位姿的时候，当估计不准确的时候，要对旋转和平移进行微调。

比如说某个时刻，机器人的位姿为 ![](https://www.zhihu.com/equation?tex=T_%7Bcw%7D)，它观测到一个世界坐标位于 ![](https://www.zhihu.com/equation?tex=P_w) 的点，产生了一个观测数据 ![](https://www.zhihu.com/equation?tex=Z_c) ，根据坐标  变换，有

![](https://www.zhihu.com/equation?tex=%0A%5Cforall%20i%2CZ_c%3DT_%7Bcw%7DP_%7Bwi%7D%2Bw%0A)

那我们实际要做的事情是求一个欧氏变换 ![](https://www.zhihu.com/equation?tex=T_%7Bcw%7D)，使得其满足上式。

然而，由于观测噪声 ![](https://www.zhihu.com/equation?tex=w) 的存在，![](https://www.zhihu.com/equation?tex=z) 往往不可能精确地满足 ![](https://www.zhihu.com/equation?tex=z%3D%20Tp) 的关系。所以，我们通常会计算理想的观测与实际数据的误差：

![](https://www.zhihu.com/equation?tex=%0Ae%3Dz-Tp%0A)

假设一共有N个这样的路标点和观测，则就有N个上式，则对于机器人的位姿估计，相当于寻找一个最优的T，使得整体误差最小化。

但是我们可以发现，旋转矩阵对乘法封闭，但是对加减是不封闭的，相加之后就不是一个旋转矩阵了，如果有一个函数![](https://www.zhihu.com/equation?tex=u%28R%29)，那么有

![](https://www.zhihu.com/equation?tex=%0A%5Cfrac%7Bdu%7D%7BdR%7D%3D%5Clim%5Climits_%7B%5CDelta%20R%5Cto%200%7D%5Cfrac%7Bu%28R%2B%5CDelta%20R%29-u%28R%29%7D%7B%5CDelta%20R%7D%0A)

但是由于其性质，我们无法完成这个求导操作，自然无法完成优化，所以我们需要用一种新理论去完成这个操作

## 群

群是一种集合加上一种运算的代数结构，记集合为 ![](https://www.zhihu.com/equation?tex=A)，运算为 ![](https://www.zhihu.com/equation?tex=%5Ccdot)，那么当运算满足以下性质时，称二元组 ![](https://www.zhihu.com/equation?tex=%28A%2C%5Ccdot%29)​ 成群

1. 封闭性：![](https://www.zhihu.com/equation?tex=%5Cforall%20a_1%2Ca_2%5Cin%20A%2Ca_1%5Ccdot%20a_2%20%5Cin%20A)
2. 结合律：![](https://www.zhihu.com/equation?tex=%5Cforall%20a_1%2Ca_2%2Ca_3%5Cin%20A%2C%28a_1%5Ccdot%20a_2%29%5Ccdot%20a_3%3D%20a_1%5Ccdot%20%28a_2%5Ccdot%20a_3%29)
3. 幺元：![](https://www.zhihu.com/equation?tex=%5Cexists%20a_0%5Cin%20A%2C%5Cforall%20a%20%5Cin%20A%2Ca_0%5Ccdot%20a%3Da%5Ccdot%20a_0%3Da)
4. 逆：![](https://www.zhihu.com/equation?tex=%5Cforall%20a%5Cin%20A%2C%5Cexists%20a%5E%7B-1%7D%5Cin%20A%2Ca%5Ccdot%20a%5E%7B-1%7D%3Da_0)

实际上旋转矩阵和变换矩阵就可以构成群，因为满足旋转矩阵和变换矩阵要求的所有矩阵就是一个集合，乘法就是所定义的运算（旋转矩阵和变换矩阵的乘积还是旋转矩阵和变换矩阵）

三维旋转矩阵构成了特殊正交群![](https://www.zhihu.com/equation?tex=SO%283%29)（Special Orthogonal Group）

三维变换矩阵的集合称为特殊欧式群![](https://www.zhihu.com/equation?tex=SE%283%29)（Special Euclidean Group)

一般线性群 ![](https://www.zhihu.com/equation?tex=GL%28n%29) 指 ![](https://www.zhihu.com/equation?tex=n%5Ctimes%20n) 的可逆矩阵，它们对矩阵乘法成群。

## 李群概念

李群是具有连续性质的群，或者说这个群是光滑可微的，所以既是群也是流形（Manifold），直观上看，一个刚体能够连续地在空间中运动，也就有连续的位姿，相应的旋转矩阵和变换矩阵也是连续的，故![](https://www.zhihu.com/equation?tex=SO%283%29)和![](https://www.zhihu.com/equation?tex=SE%283%29)都是李群。

具体来说，流形可以被定义为一个空间（可以想成一个曲面），它在每个点处都有一个“切空间（切线或者切平面）”，切空间是一个局部欧几里得空间，它可以用欧几里得几何的方法来描述，然后我们可以使用切空间的一些性质来近似表示局部曲面的性质（类似于函数可以使用若干阶导数的多项式近似表示，甚至二者之间可以形成一个双射关系)，这种性质可以用来解决位姿求导的问题

![dc6b1250-f2ac-4894-af0b-d565fc13b831](./assets/so3.png)

三维球面为二维的流形，因为可由一群二维的平面图形来叠加（广义加法）表示

![earth_so3](./assets/earth_so3.png)

![so3](./assets/so3-1746013722077-4.png)

如图所示的地球球面就是一个2维流形。因此，对于球面上的一个曲面三角形，可以摊开展成（即流动变形成）一个2维欧几里得空间上的平面三角形。此外，因为地球实在太大，我们往往把地球上的一块足够小的（曲面）局部区域当作平面来丈量，而不用担心会引起大的误差。比如，你要丈量学校操场的面积，根本不用把它认为是地球上的一块曲面，而直接看作一块平面即可。所以，光滑流形其足够小的结构是“硬”的（如可以固定丈量），而整体结构则是“柔软”的（可流动变形）。也就是我们可以使用一个平面来表示局部的曲面，并且带来计算上的方便，因为很多计算在曲面上是难以实现或者完全无法实现的。

也就是这种方法可以在局部欧式空间中的使用常规方法，因此不需要考虑复杂的全局拓扑结构，也能够精确估计出高维空间中物体的运动状态。

流形（Manifold）可看作是很多曲面片的叠加（这些曲面在大的尺度上即为平面，叠加即为广义加法）。这些平面可以看成位姿增量，将平面指数映射成曲面，而曲面的叠加即构成李群，也就是地球的球面一个三维空间中的二维的流形。

![FASTLIO_eq](./assets/FASTLIO_eq.png)

我们所能观察到的数据（**r**）实际上是由一个低维流形映射到高维空间上的，即这些数据所在的空间是“嵌入在高维空间的低维流形。这个 **r** 是迭代卡尔曼每次迭代出的位姿增量，即李代数，也是欧几里得空间中的平面，只有李代数才满足广义加法。**从整体观察：流形即为李群，从局部观察：流形近似为欧式空间。**

## 李代数概念

李代数是与李群对应的一种结构，位于向量空间，对应李群的正切空间，描述了李群局部的导数，记作 ![](https://www.zhihu.com/equation?tex=so%283%29) 和 ![](https://www.zhihu.com/equation?tex=se%283%29)

从旋转矩阵可以引出李代数，我们考虑任意旋转矩阵![](https://www.zhihu.com/equation?tex=R)，满足

![](https://www.zhihu.com/equation?tex=RR%5ET%3DI)

在连续运动过程中，显然![](https://www.zhihu.com/equation?tex=R)是连续时间的函数，我们记为

![](https://www.zhihu.com/equation?tex=R%28t%29R%28t%29%5ET%3DI)

两侧对时间求导

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ET%2BR%28t%29%5Cdot%7BR%7D%28t%29%5ET%3D0%5C%5C%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ET%3D-%28%5Cdot%7BR%7D%28t%29R%28t%29%5ET%29%5ET%0A)

如果我们将![](https://www.zhihu.com/equation?tex=%5Cdot%7BR%7D%28t%29R%28t%29%5ET)看做一个整体，我们就发现其为一个反对称矩阵

我们记为

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ET%3D%5Cphi%28t%29%5E%5Cland%0A)

两边右乘![](https://www.zhihu.com/equation?tex=R%28t%29)

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ETR%28t%29%3D%5Cphi%28t%29%5E%5Cland%20R%28t%29%0A)

其中![](https://www.zhihu.com/equation?tex=R%28t%29%5ETR%28t%29%3DI)，消去后得到

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29%3D%5Cphi%28t%29%5E%5Cland%20R%28t%29%0A)

可以看成求导之后，左侧多出一个![](https://www.zhihu.com/equation?tex=%5Cphi%28t%29%5E%5Cland)，或者说，每对旋转矩阵求导一次，只需要左乘一个此矩阵（当然此矩阵不是一个常数)

这类似于指数函数的操作，变量的导数等于其本身乘以一个系数

![](https://www.zhihu.com/equation?tex=%0Ay%3De%5E%7Bkx%7D%5Crightarrow%20y%5E%5Cprime%3Dke%5E%7Bkx%7D%5Crightarrow%20y%5E%5Cprime%3Dky%0A)

从简单情况考虑，当 ![](https://www.zhihu.com/equation?tex=t_0%3D0%2CR%280%29%3DI) 的时候

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Baligned%7D%0AR%28t%29%26%5Capprox%20R%28t_0%29%2B%5Cdot%7BR%7D%28t_0%29%28t-t_0%29%5C%5C%0A%26%3DI%2B%5Cphi%28t_0%29%5E%5Cwedge%28t%29%5C%5C%0A%5Cend%7Baligned%7D%0A)

在这里，![](https://www.zhihu.com/equation?tex=%5Cphi%5E%5Cwedge) 为 ![](https://www.zhihu.com/equation?tex=R%28t%29) 的李代数，是李群在单位元 ![](https://www.zhihu.com/equation?tex=t_0)​ 处的正切空间

在 ![](https://www.zhihu.com/equation?tex=t_0) 附近，设 ![](https://www.zhihu.com/equation?tex=%5Cphi) 保持为常数向量 ![](https://www.zhihu.com/equation?tex=%5Cphi%28t_0%29%3D%5Cphi_0)​ ，则有微分方程

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29%3D%5Cphi%28t_0%29%5E%5Cwedge%20R%28t%29%3D%5Cphi_0%5E%5Cwedge%20R%28t%29%0A)

已知初始情况，解得

![](https://www.zhihu.com/equation?tex=%0AR%28t%29%3D%5Cexp%28%5Cphi_0%5E%5Cwedge%20t%29%0A)

![](https://www.zhihu.com/equation?tex=R%28t%29) 与 ![](https://www.zhihu.com/equation?tex=%5Cphi) 之间的关系称为指数映射，这里的 ![](https://www.zhihu.com/equation?tex=%5Cphi) 称为 ![](https://www.zhihu.com/equation?tex=SO%283%29) 对应的李代数： ![](https://www.zhihu.com/equation?tex=so%283%29)

但是新的问题来了，![](https://www.zhihu.com/equation?tex=so%283%29)​ 的定义和性质是什么呢？这个指数映射应该怎么求呢

实际上每个李群都有与之对应的李代数，李代数描述了李群单位元数的正切空间性质。

李代数由一个集合 ![](https://www.zhihu.com/equation?tex=%5Cmathbb%7BV%7D) ，一个数域 ![](https://www.zhihu.com/equation?tex=%5Cmathbb%7BF%7D) 和一个二元运算 ![](https://www.zhihu.com/equation?tex=%5B%2C%5D) 组成。如果它们满足以下几条性质，称 ![](https://www.zhihu.com/equation?tex=%28%5Cmathbb%7BV%7D%2C%5Cmathbb%7BF%7D%2C%5B%2C%5D%29)为一个李代数，记作 ![](https://www.zhihu.com/equation?tex=%5Cmathfrak%7Bg%7D)

1. 封闭性：![](https://www.zhihu.com/equation?tex=%5Cforall%20X%2CY%5Cin%20%5Cmathbb%7BV%7D%2C%5BX%2CY%5D%5Cin%20%5Cmathbb%7BV%7D)

2. 双线性： ![](https://www.zhihu.com/equation?tex=%5Cforall%20X%2CY%2CZ%5Cin%20%5Cmathbb%7BV%7D%2Ca%2Cb%5Cin%20%5Cmathbb%7BF%7D)，有

   ![](https://www.zhihu.com/equation?tex=%5BaX%2BbY%2CZ%5D%3Da%5BX%2CZ%5D%2Bb%5BY%2CZ%5D%2C%5BZ%2CaX%2BbY%5D%3Da%5BZ%2CX%5D%2Bb%5BZ%2CY%5D)

3. 自反性： ![](https://www.zhihu.com/equation?tex=%5Cforall%20X%5Cin%20%5Cmathbb%20V%2C%5BX%2CX%5D%3D0)

4. 雅可比等价： ![](https://www.zhihu.com/equation?tex=%5Cforall%20X%2CY%2CZ%5Cin%20%5Cmathbb%20V%2C%5BX%2C%5BY%2CZ%5D%5D%2B%5BZ%2C%5BY%2CX%5D%5D%2B%5BY%2C%5BZ%2CX%5D%5D%3D0)

二元运算被称为李括号，例子:三维空间向量+ R3 +叉积运算构成李代数，当然，实际上我们不需要去记忆这些性质

对于李群 SO(3)，有李代数 so(3)

![](https://www.zhihu.com/equation?tex=%0Aso%283%29%3D%5C%7B%5Cphi%20%5Cin%20%5Cmathbb%7BR%7D%5E3%2C%5CPhi%3D%5Cphi%5E%5Cwedge%5Cin%20%5Cmathbb%20R%5E%7B3%5Ctimes%203%7D%5C%7D%5C%5C%0A%5CPhi%3D%5Cphi%5E%5Cwedge%3D%0A%5Cbegin%7Bbmatrix%7D%0A0%20%26%20-%5Cphi_3%20%26%5Cphi_2%5C%5C%0A%5Cphi_3%20%260%20%26%20-%5Cphi_1%5C%5C%0A-%5Cphi_2%20%26%20%5Cphi_1%20%26%200%5C%5C%0A%5Cend%7Bbmatrix%7D%0A%5Cin%20%5Cmathbb%20R%20%5E%7B3%5Ctimes%203%7D%0A)

定义李括号

![](https://www.zhihu.com/equation?tex=%0A%5B%5Cphi_1%2C%5Cphi_2%5D%3D%28%5CPhi_1%20%5CPhi_2-%5CPhi_2%5CPhi_1%29%5E%5Cwedge%0A)

可以说李代数的元素是三维向量或者三维反对称矩阵，只不过向量形式更加自然一些

当然，对于变换矩阵的李群，也有相应的李代数

![](https://www.zhihu.com/equation?tex=%0ASE%283%29%3D%5C%7B%0AT%3D%0A%5Cbegin%7Bbmatrix%7D%0AR%20%26t%20%5C%5C%0A0%5ET%20%261%0A%5Cend%7Bbmatrix%7D%0A%5Cin%20%5Cmathbb%7BR%7D%5E%7B4%5Ctimes%204%7D%7CR%5Cin%20SO%283%29%2Ct%20%5Cin%20%5Cmathbb%7BR%7D%5E3%5C%7D%0A)

对应的李代数为 se(3)

![](https://www.zhihu.com/equation?tex=%0A%0A)


## 指数映射

指数映射反映了从李代数到李群的对应关系，也就是有

![](https://www.zhihu.com/equation?tex=%0AR%3D%5Cexp%28%5Cphi%5E%5Cwedge%29%0A)

![](https://www.zhihu.com/equation?tex=%5Cphi%5E%5Cwedge) 是一个矩阵，任意矩阵的指数映射可以写成一个泰勒展开

![](https://www.zhihu.com/equation?tex=%0A%5Cexp%28%5Cphi%5E%5Cwedge%29%3D%5Csum%5E%5Cinfty_%7Bn%3D0%7D%5Cfrac%7B1%7D%7Bn%21%7D%28%5Cphi%5E%5Cwedge%29%5En%0A)

其中 ![](https://www.zhihu.com/equation?tex=%5Cphi) 是一个向量，可以定义模长 ![](https://www.zhihu.com/equation?tex=%5Ctheta) 和单位方向向量 ![](https://www.zhihu.com/equation?tex=a)，也即 ![](https://www.zhihu.com/equation?tex=%5Cphi%3D%5Ctheta%20a)

其中，因为 ![](https://www.zhihu.com/equation?tex=a) 是单位向量，所以可以推导出以下性质

![](https://www.zhihu.com/equation?tex=%0Aa%5E%5Cwedge%20a%5E%5Cwedge%3Daa%5ET-I%5C%5C%0Aa%5E%5Cwedge%20a%5E%5Cwedge%20a%5E%5Cwedge%3D-a%5E%5Cwedge%0A)

这为化解展开式中的高阶项提供了有效方法

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Baligned%7D%0A%5Cexp%28%5Cphi%5E%5Cwedge%29%26%3D%5Cexp%28%5Ctheta%20a%5E%5Cwedge%29%3D%5Csum%5E%5Cinfty_%7Bn%3D0%7D%5Cfrac%7B1%7D%7Bn%21%7D%28%5Ctheta%20a%5E%5Cwedge%29%5En%5C%5C%0A%26%3DI%2B%5Ctheta%20a%5E%5Cwedge%2B%5Cfrac%7B1%7D%7B2%21%7D%5Ctheta%5E2%20a%5E%5Cwedge%20a%5E%5Cwedge%2B%5Cfrac%7B1%7D%7B3%21%7D%5Ctheta%5E3%20a%5E%5Cwedge%20a%5E%5Cwedge%20a%5E%5Cwedge%2B%5Ccdots%5C%5C%0A%26%3Daa%5ET-a%5E%5Cwedge%20a%5E%5Cwedge%2B%5Ctheta%20a%5E%5Cwedge%2B%5Cfrac%7B1%7D%7B2%21%7D%5Ctheta%5E2%20a%5E%5Cwedge%20a%5E%5Cwedge-%5Cfrac%7B1%7D%7B3%21%7D%5Ctheta%5E3%20a%5E%5Cwedge%20%2B%5Ccdots%5C%5C%0A%26%3Daa%5ET%2B%28%5Ctheta-%5Cfrac%7B1%7D%7B3%21%7D%5Ctheta%5E3%2B%5Cfrac%7B1%7D%7B5%21%7D%5Ctheta%5E5%2B%5Ccdots%29a%5E%5Cwedge-%281-%5Cfrac%7B1%7D%7B2%21%7D%5Ctheta%5E2%2B%5Cfrac%7B1%7D%7B4%21%7D%5Ctheta%5E4-%5Ccdots%29a%5E%5Cwedge%20a%5E%5Cwedge%5C%5C%0A%26%3Da%5E%5Cwedge%20a%5E%5Cwedge%2BI%2Ba%5E%5Cwedge%5Csin%20%5Ctheta%20-a%5E%5Cwedge%20a%5E%5Cwedge%20%5Ccos%20%5Ctheta%5C%5C%0A%26%3D%281-%5Ccos%20%5Ctheta%29%20a%5E%5Cwedge%20a%5E%5Cwedge%20%2BI%20%2Ba%5E%5Cwedge%20%5Csin%20%5Ctheta%5C%5C%0A%26%3D%20%5Ccos%20%5Ctheta%20%5Cmathbf%7BI%7D%20%2B%20%281%20-%20%5Ccos%5Ctheta%29%20aa%5ET%20%2B%5Csin%5Ctheta%20a%5E%5Cwedge%0A%5Cend%7Baligned%7D%0A)

实际上这是一个似曾相识的结果——罗德里格斯公式

而从旋转矩阵到旋转向量的公式如下

![](https://www.zhihu.com/equation?tex=%0AR%20%3D%20%5Ccos%5Ctheta%20I%20%2B%20%281%20-%20%5Ccos%5Ctheta%29%20a%20a%5ET%20%2B%20%5Csin%5Ctheta%20a%5E%5Cwedge%5C%5C%0A%5Cbegin%7Balign%7D%0Atr%28R%29%20%26%3D%20%5Ccos%5Ctheta%20tr%28I%29%20%2B%20%281%20-%20%5Ccos%5Ctheta%29%20tr%28a%20a%5ET%29%20%2B%20%5Csin%5Ctheta%20tr%28a%5E%5Cwedge%29%5C%5C%0A%0A%26%3D%203%5Ccos%5Ctheta%20%2B%20%281%20-%20%5Ccos%5Ctheta%29%20%3D%201%20%2B%202%5Ccos%5Ctheta%5C%5C%0A%5Cend%7Balign%7D%5C%5C%0A%5Ctheta%20%3D%20%5Carccos%5Cleft%28%5Cfrac%7Btr%28R%29%20-%201%7D%7B2%7D%5Cright%29%5C%5C%0A%0A%0AR%20a%20%3D%20a%5C%5C%0A)

so(3) 的物理意义就是旋转向量，即so(3)的李代数空间就是由旋转向量组成的线性空间。如果李群(旋转矩阵，R(t)，类似一个函数)代表一个球面，那么球上所有点的切线(单位元处李群的切空间李代数，旋转向量)，也会组成一个球面，而且这个球面和原来的球面一样。

![1717332083843](./assets/1717332083843.png)

我们可以使用下图来可视化的理解李群和李代数的关系

![CvLife_FAST-LIO_2023_14](./assets/CvLife_FAST-LIO_2023_14.png)

![CvLife_FAST-LIO_2023_15](./assets/CvLife_FAST-LIO_2023_15.png)

## BCH公式

在估计和优化过程中，求导数是非常必要的。但是李群元素只有乘法，无法定义导数(![](https://www.zhihu.com/equation?tex=R_1%2BR_2%5Cnotin%20SO%283%29%20))

已知SO(3)和SE(3)上的李群和李代数关系，但是当SO(3)中完成两个矩阵乘法时，李代数so(3)上发生什么变化？换言之，当so(3)上做两个李代数相加时，SO(3)上是否对应着两个矩阵的乘积？

两个李代数指数映射乘积的完整形式，由BCH公式给出，由于完整形式复杂，只给出展开的前几项：

![](https://www.zhihu.com/equation?tex=%0A%5Cln%20%28%5Cexp%20%28A%29%5Cexp%20%28B%29%29%3D%20A%2B%20B%20%2B%20%5Cfrac12%5BA%2C%20B%5D%20%2B%20%5Cfrac%201%20%7B12%7D%5BA%2C%20%5BA%2C%20B%5D%7C%20-%20%20%5Cfrac%201%20%7B12%7D%5BB%2C%20%5BA%2C%20B%5D%5D%2B%5Ccdots%0A)

当处理两个矩阵指数之积时，会产生一些由李括号组成的余项。为了便于计算，当其中一个量很小的时候，可以忽略高阶项，做成线性近似形式

![](https://www.zhihu.com/equation?tex=%0A%5Cln%20%28%5Cexp%20%28%5Cphi_1%5E%5Cwedge%29%5Cexp%20%28%5Cphi_2%5E%5Cwedge%29%29%5E%5Cvee%3D%0A%5Cbegin%7Bcases%7D%0AJ_l%28%5Cphi_2%29%5E%7B-1%7D%5Cphi_1%2B%5Cphi_2%20%5Cquad%20if%20%5Cquad%5Cphi_1%5Cquad%20small%20%5C%5C%0AJ_r%28%5Cphi_1%29%5E%7B-1%7D%5Cphi_2%2B%5Cphi_1%20%5Cquad%20if%20%5Cquad%5Cphi_2%5Cquad%20small%0A%5Cend%7Bcases%7D%0A)

理解一下这个式子：当对一个旋转矩阵 ![](https://www.zhihu.com/equation?tex=R_2)(李代数为 ![](https://www.zhihu.com/equation?tex=%5Cphi_2))左乘一个微小旋转矩阵 ![](https://www.zhihu.com/equation?tex=R_1)(李代数为 ![](https://www.zhihu.com/equation?tex=%5Cphi_1))时，可以近似的看作，在原有的李代数 ![](https://www.zhihu.com/equation?tex=%5Cphi_2) 上，加上了一项  ![](https://www.zhihu.com/equation?tex=J_l%28%5Cphi_2%29%5E%7B-1%7D%5Cphi_1)，其中

![](https://www.zhihu.com/equation?tex=%0AJ_l%3DJ%3D%5Cfrac%7B%5Csin%20%5Ctheta%7D%7B%5Ctheta%7DI%2B%281-%5Cfrac%7B%5Csin%20%5Ctheta%7D%7B%5Ctheta%7D%29aa%5ET%2B%5Cfrac%7B1-%5Ccos%20%5Ctheta%7D%7B%5Ctheta%7Da%5E%5Cland%5C%5C%0AJ_l%5E%7B-1%7D%3D%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Ccot%20%5Cfrac%7B%5Ctheta%7D%7B2%7D%20I%2B%281-%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Ccot%20%5Cfrac%7B%5Ctheta%7D%7B2%7D%29aa%5ET%2B%5Cfrac%7B%5Ctheta%7D%7B2%7D%5Ctheta%20a%5E%5Cland%5C%5C%0AJ_r%28%5Cphi%29%3DJ_l%28-%5Cphi%29%0A)


某个旋转 ![](https://www.zhihu.com/equation?tex=R)，对应的李代数为 ![](https://www.zhihu.com/equation?tex=%5Cphi)，给它左乘一个微小旋转，记作 ![](https://www.zhihu.com/equation?tex=%5CDelta%20R)，则对应的李代数为 ![](https://www.zhihu.com/equation?tex=%5CDelta%20%5Cphi)
在李群上，得到的为 ![](https://www.zhihu.com/equation?tex=%5CDelta%20R%5Ccdot%20R)，而在李代数上，根据BCH近似，为 ![](https://www.zhihu.com/equation?tex=J_l%28%5Cphi%29%5E%7B-1%7D%5CDelta%5Cphi%2B%5Cphi)

如果在李群上定义了左乘小量，那么李代数上的加法相差左雅可比的逆

![](https://www.zhihu.com/equation?tex=%0A%5Cexp%20%28%5CDelta%5Cphi%5E%5Cwedge%29%5Cexp%20%28%5Cphi%5E%5Cwedge%29%3D%5Cexp%28%28J_l%28%5Cphi%29%5E%7B-1%7D%5CDelta%5Cphi%2B%5Cphi%29%5E%5Cwedge%29%0A)

反之，在李代数上进行小量加法的时候，李群上相差一个一个左右的雅可比

![](https://www.zhihu.com/equation?tex=%0A%5Cexp%28%28%5Cphi%2B%5CDelta%5Cphi%29%5E%5Cwedge%29%0A%3D%5Cexp%20%28%28J_l%5CDelta%5Cphi%29%5E%5Cwedge%29%5Cexp%20%28%5Cphi%5E%5Cwedge%29%0A%3D%5Cexp%20%28%5Cphi%5E%5Cwedge%29%5Cexp%20%28%28J_r%5CDelta%5Cphi%29%5E%5Cwedge%29%0A)

需要注意的是，我们经常会构建与位姿有关的函数然后讨论该函数对于位姿的导数，从而调整当前的估计值，但是基于旋转矩阵的方法是无法计算导数的，所以使用李群和李代数的方法进行位姿导数的计算

使用李代数解决求导问题的思路分为两种

1. 用李代数表示姿态，然后根据李代数加法来对李代数求导
2. 对李群左乘或右乘微小扰动，然后对该扰动求导

## 李代数上的求导与扰动模型

假设对一个空间点 ![](https://www.zhihu.com/equation?tex=P) 进行了旋转，得到了 ![](https://www.zhihu.com/equation?tex=RP) ，现在要计算旋转之后点的坐标相对于旋转的导数，对此可以不严谨的记为

![](https://www.zhihu.com/equation?tex=%0A%5Cfrac%7B%5Cpartial%20Rp%7D%7B%5Cpartial%20%20R%7D%0A)

由于矩阵没有加法，导数无法定义，则设 ![](https://www.zhihu.com/equation?tex=R) 对应的李代数为 ![](https://www.zhihu.com/equation?tex=%5Cphi)，则进行计算如下（为导数模型，增量定义在李代数上）

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Baligned%7D%0A%5Cfrac%7B%5Cpartial%20%28%5Cexp%28%5Cphi%5E%5Cland%29p%29%7D%7B%5Cpartial%20%20%5Cphi%7D%26%3D%0A%5Clim%20_%7B%5Cdelta%20%20%5Cphi%20%5Cto%200%7D%5Cfrac%7B%5Cexp%28%28%5Cphi%2B%5Cdelta%20%20%5Cphi%29%5E%5Cland%29p-%5Cexp%28%5Cphi%5E%5Cland%29p%7D%7B%5Cdelta%20%20%5Cphi%7D%5C%5C%0A%26%3D%5Clim%20_%7B%5Cdelta%20%20%5Cphi%20%5Cto%200%7D%5Cfrac%7B%5Cexp%28%28J_l%20%5Cdelta%20%20%5Cphi%29%5E%5Cland%29%20%5Cexp%28%20%20%5Cphi%5E%5Cland%29p-%5Cexp%28%5Cphi%5E%5Cland%29p%7D%7B%5Cdelta%20%20%5Cphi%7D%5Cquad%20%28%E5%9F%BA%E4%BA%8EBCH%E7%BA%BF%E6%80%A7%E8%BF%91%E4%BC%BC%29%5C%5C%0A%26%3D%5Clim%20_%7B%5Cdelta%20%20%5Cphi%20%5Cto%200%7D%5Cfrac%7B%28I%2B%28J_l%20%5Cdelta%20%20%5Cphi%29%5E%5Cland%29%20%5Cexp%28%20%20%5Cphi%5E%5Cland%29p-%5Cexp%28%5Cphi%5E%5Cland%29p%7D%7B%5Cdelta%20%20%5Cphi%7D%5Cquad%20%28%E6%B3%B0%E5%8B%92%E5%B1%95%E5%BC%80%E8%88%8D%E5%8E%BB%E9%AB%98%E9%98%B6%E9%A1%B9%29%5C%5C%0A%26%3D%5Clim%20_%7B%5Cdelta%20%20%5Cphi%20%5Cto%200%7D%5Cfrac%7B%28J_l%20%5Cdelta%20%20%5Cphi%29%5E%5Cland%20%5Cexp%28%5Cphi%5E%5Cland%29p%7D%7B%5Cdelta%20%20%5Cphi%7D%5Cquad%20%28%E5%88%86%E5%AD%90%E7%AD%89%E4%BA%8EJ_l%20%5Cdelta%20%20%5Cphi%5Ctimes%20%5Cexp%28%5Cphi%5E%5Cland%29p%EF%BC%8C%E5%8F%89%E7%A7%AF%29%5C%5C%0A%26%3D%5Clim%20_%7B%5Cdelta%20%20%5Cphi%20%5Cto%200%7D%5Cfrac%7B-%28%5Cexp%28%5Cphi%5E%5Cland%29p%29%5E%5Cland%20J_l%20%5Cdelta%20%20%5Cphi%20%7D%7B%5Cdelta%20%5Cphi%7D%0A%5Cquad%20%28%E5%8F%89%E4%B9%98%E8%AE%A1%E7%AE%97%EF%BC%8C%E4%BA%A4%E6%8D%A2%E4%B9%8B%E5%90%8E%E5%8F%98%E7%AC%A6%E5%8F%B7%29%5C%5C%26%3D-%28Rp%29%5E%5Cland%20J_l%0A%5Cend%7Baligned%7D%0A)

根据这种方法可以得到左扰动模型（增量定义在李群上），设左扰动 ![](https://www.zhihu.com/equation?tex=%5CDelta%20R) 对应的李代数为 ![](https://www.zhihu.com/equation?tex=%5Cvarphi)，然后对 ![](https://www.zhihu.com/equation?tex=%5Cvarphi) 求导

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Baligned%7D%0A%5Cfrac%7B%5Cpartial%20%28Rp%29%29%7D%7B%5Cpartial%20%20%5Cvarphi%7D%26%3D%0A%5Clim%20_%7B%20%5Cvarphi%20%5Cto%200%7D%5Cfrac%7B%5Cexp%28%5Cvarphi%5E%5Cland%29%5Cexp%28%20%5Cphi%5E%5Cland%29p-%5Cexp%28%5Cphi%5E%5Cland%29p%7D%7B%5Cvarphi%7D%5C%5C%0A%26%3D%5Clim%20_%7B%20%5Cvarphi%20%5Cto%200%7D%5Cfrac%7B%281%2B%5Cvarphi%5E%5Cland%29%5Cexp%28%20%5Cphi%5E%5Cland%29p-%5Cexp%28%5Cphi%5E%5Cland%29p%7D%7B%5Cvarphi%7D%5C%5C%0A%26%3D%5Clim%20_%7B%20%5Cvarphi%20%5Cto%200%7D%5Cfrac%7B%5Cvarphi%5E%5Cland%5Cexp%28%20%5Cphi%5E%5Cland%29p%7D%7B%5Cvarphi%7D%0A%3D%5Clim%20_%7B%20%5Cvarphi%20%5Cto%200%7D%5Cfrac%7B-%28Rp%29%5E%5Cland%5Cvarphi%7D%7B%5Cvarphi%7D%3D-%28Rp%29%5E%5Cland%5C%5C%0A%5Cend%7Baligned%7D%0A)

这种左扰动模型相对导数模型更好，当然类似的还有右扰动模型：![](https://www.zhihu.com/equation?tex=-Rp%5E%5Cland)

1. 对李群左乘或者右乘微小扰动，然后对这个扰动求导，即把增量的扰动直接添加在李群上，然后利用李代数表示此扰动。
2. 把增量直接定义在李群上需要注意：传统上我们通常用加法表示增量，而李群对加法不封闭。所以这里的增量不再用加法表示，而是乘法。
3. 乘法：增量指的是，在原来的基础上改变一点点。当对旋转矩阵做乘法，乘以的是一个趋近于单位矩阵，也就是差不多没旋转，那这样就是对其“加了一个小量
4. 单位矩阵的李代数为0。
5. 与导数模型相比，省去了一个雅可比的计算，更为实用。

## 应用

因为李代数是线性的，而李群则是非线性的，而二者之间还有一一对应的关系，所以我们可以使用李代数来进行插值等操作，比如说两个时刻我们获取了两个位姿或者说变换矩阵，那么如何求得两个时刻之间任意时刻的位姿呢？这里就可以基于前一时刻，以前一时刻的位置为原点，计算后一时刻相对的位姿变换矩阵，然后将此矩阵变为李代数，在得到线性的李代数之后，就可以计算出中间任意时刻的对应的李代数，进而求得中间任意时刻的李群也即位姿

## Sophus库

Sophus库是一个专门处理李群和李代数的C++库

编译安装完成之后，在CMakeLists中，可以使用如下命令导入Sophus库

```cmake
find_package(Sophus REQUIRED)
```

然后就可以在CPP中使用此库了，下面是一段代码

```c++
#include <Eigen/Core>
#include"sophus/so3.h"
#include"sophus/se3.h"
//导入库，Eigen和Sophus
Eigen::Matrix3d R=Eigen::AngleAxisd(M_PI/4,Eigen::Vector3d(0.1.0)).toRotationMatrix();//沿Y轴转45度的旋转矩阵

//Sophus的构造方式
Sophus::SO3 rot_r(R);//从旋转矩阵构造
Sophus::SO3 SO3_v(0,M_PI/4,0);// 从旋转向量构造
Eigen::Quaterniond q(R);//从四元数构造
Sophus::SO3 SO3_q(q);
//当使用cout输出的时候，就会以李代数形式输出，或者可以使用.matrix()方法输出矩阵
```



# 相机模型

观测，也就是机器人如何观测外部世界，如果使用激光雷达观测或者使用相机观测，也就构成了激光slam或者视觉slam

## 缺失距离维度的照片

照片记录了真实世界在成像平面上的投影，这个过程丢弃了“距离”维度上的信息，就比如说下面这个照片，实际上两个人是一样大小，但是照片中仿佛棕色衣服的是巨人一样

![在这里插入图片描述](https://img-blog.csdnimg.cn/c38ce619f95f46fab35153cdc8d3d53b.jpeg#pic_center)

# IMU模型

## 概述

IMU也就是惯性测量单元，是测量物体三轴姿态角（或角速率）以及加速度的装置

- 6轴：三轴加速度计+三轴陀螺仪（角速度传感器）
- 9轴：6轴+三轴磁力计（角度）

那么为什么用IMU呢？主要有以下几个原因

- 直接输出加速度角速度
- 频率高（100-400HZ），相机和雷达的频率基本上只有几十赫兹
- 不受外界干扰（磁力计可能受干扰），相机容易受到光照干扰，雷达在雨雾天可能受干扰
- 价格不贵，基本上几十几百就可以买到（几千几万的也有，但是一般精度的几十几百即可）

但是受自身温度、零偏、振动等因素干扰，积分得到的平移和旋转容易漂移，并且高精度的IMU价格昂贵，所以IMU只适合计算短时间且快速的运动

## 测量原理

实际上分为三种测量，加速度计、陀螺仪和磁力计 

![Shenlan_VIO_L2_11](./assets/Shenlan_VIO_L2_11.png)

![CvLife_FAST-LIO_2023_22](./assets/CvLife_FAST-LIO_2023_22.png)

加速度计实际上就是运用了胡克定律，也就是在弹簧固定下的质量块在有加速度的情况下会有位移，根据牛顿第二定律就可以知道加速度大小，当然在IMU中是使用电容差计算位移进而计算加速度

磁力计的工作原理跟指南针类似，通过霍尔效应计算磁场强度

## 误差模型

IMU的误差可以分为确定误差（可以通过标定获取，是一个确定值）和随机误差（也就是随机噪声，无法提取获取，但是可以计算出协方差矩阵）

![CvLife_FAST-LIO_2023_23](./assets/CvLife_FAST-LIO_2023_23.png)

其中，偏置是会随着时间而变化的，所以还要在 SLAM 过程中不断标定或者说估计，很多紧耦合的框架都会在过程中不断估计

## 测量模型

世界坐标系为 G，IMU坐标系为 I，通常忽略 Scale

![CvLife_FAST-LIO_2023_24](./assets/CvLife_FAST-LIO_2023_24.png)

一般认为重力加速度不变，导数为零，偏置服从随机游走模型，其导数是高斯的

所以，在世界坐标系下的速度等状态量如下

![](https://www.zhihu.com/equation?tex=%0A%5EG%5Cdot%7Bp%7D_I%3D%5EGv_I%2C%5EG%5Cdot%7Bv%7D_I%3D%5E%7BG%7DR_Ia_m%0A)



## 运动模型（连续+离散）

基于连续的数学运动模型考虑

那么在 G 也就是全局坐标系下，连续运动的导数为

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%3DR%20%5Comega%20%5E%5Cland%20%5C%5C%0A%5Cdot%7Bv%7D%3Da%20%5C%5C%0A%5Cdot%7Bp%7D%3Dv%0A)

后面两个容易理解，速度求导为加速度，位置求导为速度

对于第一个公式，我们假设一个从原点出发的向量 ![](https://www.zhihu.com/equation?tex=r) 绕某一个方向上的单位轴 ![](https://www.zhihu.com/equation?tex=u) 旋转，那么角速度为 ![](https://www.zhihu.com/equation?tex=%5Comega%20%3D%5Cdot%7B%5Ctheta%20%7Du)，角速度大小为 ![](https://www.zhihu.com/equation?tex=%5Cdot%7B%5Ctheta%7D)，那么根据刚体旋转动力学，任意点绕固定轴旋转的线速度由角速度向量和该点位置向量的叉乘确定

![](https://www.zhihu.com/equation?tex=%0A%5Cfrac%7B%5Crm%20d%20r%7D%7B%5Crm%20d%20t%7D%3D%5Comega%20%5Ctimes%20r%0A)

那么，坐标系沿着单位轴旋转，其三个轴的导数  

![CvLife_FAST-LIO_2023_25](./assets/CvLife_FAST-LIO_2023_25.png)

但是，机器无法处理连续数值，所以只能进行离散化才可以进一步处理，也就是进行欧拉积分，当然实际上也可以使用两帧之间的平均值计算（也就是中值积分）

![CvLife_FAST-LIO_2023_26](./assets/CvLife_FAST-LIO_2023_26.png)

这里因为时间间隔很小，所以在实际代码中可以忽略二次项

然后把测量模型的数值带入离散模型，就可以得到如下的计算模型，然后循环执行计算即可，进而不断更新状态量

![CvLife_FAST-LIO_2023_27](./assets/CvLife_FAST-LIO_2023_27.png)

## 使用方法

IMU 的用途主要是作为先验估计来优化激光点，在实际中，我们希望是所有点云在同一时刻的采样，但是激光雷达的采样频率相对较慢而且存在运动畸变，而 IMU 的采样频率则非常高，所以可以使用 IMU 来去除畸变，并且把所有的点云统一到同一个时刻进行处理，文中是在 **tk** 时刻。因此，我们根据 IMU 积分估计的位姿，把 ![](https://www.zhihu.com/equation?tex=t_%7Bk-1%7D) 每个点转到 **tk** 时刻。即同一时刻与同一位姿，发射与接受激光束。

![lidar_proj_by_imu](./assets/lidar_proj_by_imu.png)

1. 从尾部开始遍历找 head 对应的imu数据
2. head对应的IMU状态之前已经计算出来了，计算当前点到head的时间间隔
3. 假设tail对应的IMU数据为 当前点对应的IMU数据，因为IMU频率比较高，短时间内变化不大
4. 计算head到当前点的位姿变换：∆T，并广义加到head对应的位姿，得到当前点 it 对应的位姿：P_i、R_i
5. 计算当前点到 end_lidar的位姿变换：T_ei、R_ei

在激光SLAM中，大多数的算法都是以启动系统的时候第一帧 IMU 为参考，但是在安装 IMU 的时候可能存在安装偏差，导致 IMU 坐标系相对地面系有一个微小旋转

## ROS1代码

在 ROS1 中，使用 `sensor_msgs/Imu` 来作为消息类型传递 IMU 数据的，其中包括了四部分

1. 头消息：`std_msgs/Header header`，其中包含时间戳 `stamp`，时间戳可以使用 toSec() 方法转换以便求时间差
2. 四元数及协方差：`geometry_msgs/Quaternion orientation`，表示朝向，并且有协方差数组`float64[9] orientation_covariance`
3. 角速度及协方差：`geometry_msgs/Vector3 angular_velocity` 和 `float64[9] angular_velocity_covariance`
4. 加速度及协方差：`geometry_msgs/Vector3 Linear_velocity` 和 `float64[9] Linear_velocity_covariance`

# 点云处理

## 激光雷达模型

目前的激光雷达基本上是光学雷达，通过发射脉冲激光，基于飞行时间法测量距离（也有部分初级雷达使用三角测距法），并且多线雷达还会同时发射多组激光来感知环境

激光雷达主要是以下三种，其中机械式激光雷达较为常见，受限于机械式结构，寿命相对较短，难以实现车规级运用

目前相对好的是混合固态的，相对机械式寿命相对更长，其中有大疆的Livox系列

![CvLife_FAST-LIO_2023_29](./assets/CvLife_FAST-LIO_2023_29.png)

## 点云运动畸变矫正

因为雷达与机器人是刚性连接，并且是不断进行采样而非一次性完成采样的，所以会发生一个情况，点云中的点不是同一时间完成采集的，并且在这个过程中机器人是在运动的，所以同一个物体会造成多个位置不同的采样点，如下图所示，五角星表示物体采样的点，在一段时间的起止时刻，因为机器人的位置或者说坐标系发生了变化，所以测量出来的位置就会不一样，或者说产生了错位，因为在起始时刻和结束时刻的点云是在两个不同的坐标系下采样的，但是统一在结束时刻的坐标系下进行处理，也就是产生了运动畸变

![3DCVer_LOAM_L2_10](./assets/3DCVer_LOAM_L2_10.png)

那么如何进行矫正呢，实际上就是把所有的点归到同一个坐标系下，然后把误差补偿掉

具体步骤如下图所示，根据从里程计（比如说IMU或者轮速里程计）处获得的位姿信息，计算出来结束时刻相对起始时刻的位姿变化，然后插值计算出来每一次点云扫描时刻的位姿，基于这个插值出来的位姿变化矫正点云坐标，使其统一转化到起始时刻的坐标下或者结束时刻的坐标下

![3DCVer_LOAM_L2_13](./assets/3DCVer_LOAM_L2_13.png)

## 解决畸变工程技巧

工程上的一些技巧

- 激光雷达安装方式，通常y轴朝向载体左右两侧
- 相对位姿变换可以通过GPS或者IMU或者轮式里程计获取

## 点云下采样

因为直接从激光雷达获取的点云中含有大量的点，所以需要进行下采样减少点的数量，其中体素滤波方法较为常见，但是要注意体素大小，太大的话会造成点云细节缺失

![CvLife_FAST-LIO_2023_30](./assets/CvLife_FAST-LIO_2023_30.png)

## 特征点提取

特征点提取方法中的角点和面点提取的方法来源于LOAM论文，这是激光SLAM的开篇之作，实际上这种特征点是根据曲率来判断并且提取的

![CvLife_FAST-LIO_2023_31](./assets/CvLife_FAST-LIO_2023_31.png)

具体的提取方法是这样的，对于同一个scan（也就是同一个射线上的点），判断其曲率，曲率公式如上所示，选取一个近邻点集进行曲率计算，然后根据阈值判断是否为角点或者面点，上图右下角的图片中，绿色圆形就是面点，橙色三角就是角点

区分角点和面点的原因是，在计算残差的时候，两种特征点的计算方式不一样

## 残差

将去除畸变后的特征点转换到全局坐标系下

两种点的残差计算方式如下，而我们要做的事情就是通过优化位姿，使得残差接近零

![CvLife_FAST-LIO_2023_32](./assets/CvLife_FAST-LIO_2023_32.png)

其中角点的残差计算方式如图，其中 ![](https://www.zhihu.com/equation?tex=%5Cwidetilde%7BX%7D%5EL_%7B%28k%2Ci%29%7D) 表示第 ![](https://www.zhihu.com/equation?tex=k) 帧的第 ![](https://www.zhihu.com/equation?tex=i) 个点的坐标（波浪线表示已经乘以待优化位姿的坐标，也就是转到世界坐标系下的坐标），然后我们计算此点到近邻直线的距离

## ROS1 代码

点云是激光雷达产生的，而不同的激光雷达因为工作原理等不同，所以给出的点云格式也不同，这需要不同的厂家给出相应的代码从而得到 `sensor_msgs::PointCloud2` 格式的 ROS点云消息，然后我们需要使用 PCL 进行下采样和滤波操作

```cpp
pcl::PointCloud<rslidar_ros::Point> pl_orig;// shengc 
pcl::fromROSMsg(msg,pl_orig);
std::cout<<"lidar point size: "<<pl_orig.points.size()<<std::endl;

pcl::VoxelGrid<pcl::PointXYZI>
```



# 激光SLAM前端

激光里程计实际上就是一种点云配准的方法，通过两帧点云之间的相对位姿变换来计算里程计，而想实现两帧点云的相对位姿变换求解，就要进行点云配准，其中经典方法有ICP和NDT，两种方法都可以在一些点云库中调用现成接口实现

## ICP方法

这是一种经典方法，思想是两帧点云中距离最近的点是近邻点，下图中 ![](https://www.zhihu.com/equation?tex=p_i) 和 ![](https://www.zhihu.com/equation?tex=q_i) 是两帧点云，![](https://www.zhihu.com/equation?tex=u_p) 和 ![](https://www.zhihu.com/equation?tex=u_q) 是两帧点云的求和

![3DCVer_LOAM_L2_16](./assets/3DCVer_LOAM_L2_16.png)

然后先后分布求解 ![](https://www.zhihu.com/equation?tex=R) 和 ![](https://www.zhihu.com/equation?tex=T)

![3DCVer_LOAM_L2_17](./assets/3DCVer_LOAM_L2_17.png)

## NDT方法

NDT方法也就是正态分布变换方法，是另一种经典的方法，思想是，两帧点云的分布要尽可能接近，具体操作是对上一帧点云计算分布，然后下一帧点云要尽可能接近这个分布，对应的姿态就是要寻找的姿态或者说最准确的姿态

![3DCVer_LOAM_L2_18](./assets/3DCVer_LOAM_L2_18.png)

如上图所示，对上一帧点云 ![](https://www.zhihu.com/equation?tex=y) 进行均值和协方差矩阵计算，得到了高斯形式的概率密度函数，然后使得概率乘积尽可能更大，然后使用极大似然估计方法进行求解

![](./assets/3DCVer_LOAM_L2_19.png)

作者提出的一种改进如上图左侧所示，原来的概率分布（红线）可能会出现概率为0或者无穷大的情况，所以要做一点改动，如上图左下角公式所示，可以避免异常情况，变成绿线所示

![3DCVer_LOAM_L2_21](./assets/3DCVer_LOAM_L2_21.png)

这里，NDT更不容易受到噪声干扰，因为点云中的点是存在噪声的，在ICP中噪声也会参与计算，造成干扰，导致优化陷入局部最优，此外NDT是点云的概率分布函数进行计算，所以计算上更快

## 特征点配准方法

这是一种优化的配准方法，选取点云中一些特殊的点来进行配准，在减少了点云数量的同时，基于特征点配准也会降低噪声的干扰，在LOAM中使用的特征点是角点和面点，也就是曲率满足一定条件的点

![3DCVer_LOAM_L2_22](./assets/3DCVer_LOAM_L2_22.png)

LOAM的代码是这样操作的，代码如下

```c++
for(size_t i = startIdx，regionIdx = 0; i <= endIdx;i++，regionIdx++){
    //遍历点云，求取第 i 个点的曲率，pointWeight是权重，这里是-10，要与采样点数量对应或者说为相反数，便于后面的计算
    float diffX = pointWeight * _laserCloud[i].x;
    float diffY = pointWeight * _laserCloud[i].y;
    float diffZ = pointWeight * _laserCloud[i].z;
    
    //_config.curvatureRegion是求曲率范围，这里是5，也就是选取半径为5，选择临近的10个点求曲率
	for(int j = 1;j <= _config.curvatureRegion; j++){
        //实际上就是计算差值，因为diffX等是负值，并且已经乘以权重了，遍历求和等于计算所有邻近点与此点的差值并且求和，相当于计算所有近邻点相对此点的偏移量之和
        diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
        diffY += _lasercloud[i + j].y + _laserCloud[i - j].y;
        diffZ += _lasercloud[i + j].z + _laserCloud[i - j].z;
    }
    _regionCurvature[regionIdx] = diffx* diffx + diffy * diffy + diffz * diffz;
    //对三维上的偏移量求平方和，以此来判断曲率是否过大或者过小
    _regionSortIndices[regionIdx]= i;
}
```



## PL-ICP方法

除去点到点的配准方法，还有一种是点到线的配准方法，也就是PL-ICP方法，也即Point-Line ICP，其精度相对点到点方法更高，这也是LOAM采用的配准方法

![3DCVer_LOAM_L2_23](./assets/3DCVer_LOAM_L2_23.png)

# VSLAM前端

## 2D-2D

假设在相邻时刻，相机分别观测到两个图像，并且在其中匹配到了对应点，如何求两个图像对应位姿之间的位姿变换

假设对应点的齐次像素坐标为 ![](https://www.zhihu.com/equation?tex=p_1) 和 ![](https://www.zhihu.com/equation?tex=p_2)，那么在两个时刻的相机坐标系下有下面的公式，其中相机内参矩阵已知

![](https://www.zhihu.com/equation?tex=%0Ap_1%3DKP_%7BC1%7D%5C%5C%0Ap_2%3DKP_%7BC2%7D%0A)


那么以第一个相机坐标系为基准（或者为世界坐标系的话），那么有

![](https://www.zhihu.com/equation?tex=%0AP_%7BC2%7D%3DRP_%7BC1%7D%2Bt%5C%5C%0AK%5E%7B-1%7Dp_2%3DP_%7BC2%7D%3DRP_%7BC1%7D%2Bt%3DRK%5E%7B-1%7Dp_%7B1%7D%2Bt%5C%5C%0At%5E%5Cwedge%20K%5E%7B-1%7Dp_2%3Dt%5E%5Cwedge%20R%20K%5E%7B-1%7Dp_%7B1%7D%2Bt%5E%5Cwedge%20t%5C%5C%0A%E5%85%B6%E4%B8%AD%EF%BC%8C%E5%90%91%E9%87%8F%E4%B8%8E%E8%87%AA%E8%BA%AB%E7%9A%84%E5%8F%89%E4%B9%98%E4%B8%BA%E9%9B%B6%0A)

然后左边乘以 ![](https://www.zhihu.com/equation?tex=%28K%5E%7B-1%7Dp_2%29%5ET)，其中左侧为 ![](https://www.zhihu.com/equation?tex=t%5E%5Cwedge%20K%5E%7B-1%7Dp_2)，也就是 t 与向量的叉乘，方向上垂直于此向量，再进行与向量的点乘结果为0，故有

![](https://www.zhihu.com/equation?tex=%0A%28K%5E%7B-1%7Dp_2%29%5ETt%5E%5Cwedge%20R%20K%5E%7B-1%7Dp_%7B1%7D%3Dp_2%5ETK%5E%7B-T%7Dt%5E%5Cwedge%20R%20K%5E%7B-1%7Dp_%7B1%7D%3D0%0A)

其中本质矩阵 E 为 ![](https://www.zhihu.com/equation?tex=t%5E%5Cwedge%20R)，也就是我们所要求解的核心内容，如果有若干组匹配点，即可使用线性方程组解法进行求解，而且在大量约束的情况下，得到的旋转矩阵会相对准确，但是因为缺失了尺度的原因，无法得到准确的位移

# VIO

## 融合方案介绍



![Shenlan_VIO_L1_17](./assets/Shenlan_VIO_L1_17-1748167765991-2.png)

![Shenlan_VIO_L1_17](./assets/Shenlan_VIO_L1_17.png)

# 激光SLAM框架

## LOAM框架

LOAM框架是14年提出的三维激光框架，非常经典的激光里程记和建图方案，也是其他 LOAM 方案的鼻祖，LOAM 只基于激光雷达（可选 IMU），通过把 SLAM 拆分成一个高频低精的前端以及一个低频高精的后端来实现 lidar 里程记的实时性。

其系统架构如下，前端就是激光配准和点云里程计，里程计输出会有一个漂移，所以后面跟着一个Mapping的环节来优化这种情况，并且LOAM没有回环检测功能

在里程计部分，通过提取特征点，加上优化点到线的距离和点到平面的距离并且采用L-M方法求最优解，得到两帧点云之间的位姿关系

![3DCVer_LOAM_L3_3](./assets/3DCVer_LOAM_L3_3.png)

在后面的地图部分，还会进行一次地图匹配，重新计算一次位姿变换，并且对两次的数据进行融合

![ ](./assets/3DCVer_LOAM_L3_5.png)

### 特征点提取

在LOAM的代码中，特征点提取部分考虑了特征点的均匀性，按照角度把点云分为了四个部分或者说四个扇形，每个扇形里面提取出固定数量的角点和面点

同时，对于一些特殊情况（如存在遮挡等情况的点云，也就是坏点）进行了剔除操作

### 点云配准

为了找到特征点的对应关系，要进行一次运动畸变矫正，然后才可以更为准确的进行运动畸变矫正

![3DCVer_LOAM_L3_8](./assets/3DCVer_LOAM_L3_8.png)

## LEGO-LOAM

这是IROS2018的文章，基于LOAM进行了改进，并且一个很大的改进就是加入了回环检测功能

![3DCVer_LOAM_L3_6](./assets/3DCVer_LOAM_L3_6.png)

## Cartographer

这是一个非常精简的框架，基于图优化算法，其在算法上并没有非常先进，但是其优点在于代码工程化非常好，而且不需要依赖于PCL、g2o等庞大的代码库

其分前端和后端，并且分别维护局部坐标系和全局坐标系（也就是 local 和 global），局部坐标系是不会动的，建图之后就固定了，而全局坐标系是动态维护更新的

在前端部分，其会将一次 scan 来与子地图（也就是submap）进行匹配，并且使用非线性优化来进行估计

地图是一种概率栅格地图，其中表达的是此处有障碍或者被占用的概率是多少，而且将雷达点分为了两部分，如果点落在某一个格子上，则此格子称为 hit（意思是打中了障碍物），而击中点与雷达中心之间连线所经过的格子则称为miss（意思是这个路径是空的或者说通畅的）

### Branch-and-Bound scan matching

分支定界算法，最初来源于数学中的混合整数线性规划问题，在Cartographer中的核心思想是，将解空间划分为树状结构，每一个节点代表一个解，叶子节点代表真正的解，这个算法分为三部分：选择，分支，定界

选择算法是使用深度优先搜索也就是DFS算法

## FAST-LIO框架

创新点有二：一是提出了一种紧耦合的迭代卡尔曼滤波方法，以此融合雷达特征点和 IMU，二是提出了新的卡尔曼滤波增益公式

FAST-LIO的主体框架如下图所示，输入部分是雷达和IMU，雷达的输入是100k-500kHz，实际上因为这是固态雷达，一个点就相当于一个采样，所以是一秒有100k-500k的点云输入，当然一个点是无法进行SLAM的，所以会积累几十或者上百k个单位的点之后才会作为一次输入送入SLAM（一般取决于里程计频率，比如说里程计频率为50Hz时，则在20ms内积累点云，成为一帧点云并处理一次，这20ms内的点云为一次scan），然后进行特征提取（方法与LOAM一致，就是面点和角点提取，或者说是平面点和边缘点）；而IMU的输入会送入前向传播模块（这里传播的是状态量，也就是待估计的位移、偏置等），前向传播的目的有

- 在两帧雷达数据之间有很多帧IMU数据，将这些IMU数据进行粗略积分，得到位姿变化的估计，用于卡尔曼滤波的预测值、反向传播的运动补偿
- 传递误差量，尤其是传递误差量的变化和协方差矩阵（在后面迭代更新的时候会用到）

然后就是反向传播和运动补偿，因为进行了前向传播，所以每一帧的IMU都有一个预估的位姿，通过这个位姿对激光雷达点云数据进行运动补偿，降低运动产生的失真

然后将补偿后的点云和预估的状态量进行一个残差的计算，然后通过误差状态卡尔曼滤波器进行迭代更新状态量，如果收敛的话就进行一个里程计输出，否则继续进行迭代

收敛并且输出里程计之后，就会根据里程计信息或者说位姿信息，把新的点云插入到全局地图中去，当然也会在全局地图中根据位姿信息进行一个下采样，得到一个子地图，根据这个子地图去寻找近邻点进行匹配

![FAST-LIO_2](./assets/FAST-LIO_2.png)

在一次scan的过程中，记录起止时间为 ![](https://www.zhihu.com/equation?tex=t_%7Bk-1%7D) 和 ![](https://www.zhihu.com/equation?tex=t_k)，后者为当前时刻，可以认为点云特征是在时间轴上均匀分布并且首尾对齐的，但是IMU的数据并非首尾对齐的，所以需要统一投影到当前时刻下或者说最后一帧点云下，这里使用 ![](https://www.zhihu.com/equation?tex=%5Ctau) 和 ![](https://www.zhihu.com/equation?tex=%5Crho) 来表示 IMU 和 LiDAR 的时刻

### 数学公式

这里定义了一种广义的加减法或者说操作，首先我们定义 ![](https://www.zhihu.com/equation?tex=%5Cmathcal%7BM%7D) 是流形，例如 ![](https://www.zhihu.com/equation?tex=SO%283%29)

![](https://www.zhihu.com/equation?tex=%0A%5Cboxplus%20%3A%20%5Cmathcal%7BM%7D%20%5Ctimes%20%5Cmathbb%7BR%7D%5En%20%5Crightarrow%20%5Cmathcal%7BM%7D%20%5Cquad%20%5Cboxminus%20%3A%20%5Cmathcal%7BM%7D%20%5Ctimes%20%5Cmathcal%7BM%7D%20%5Crightarrow%20%5Cmathbb%7BR%7D%5En%0A)

如果 ![](https://www.zhihu.com/equation?tex=%5Cmathcal%7BM%7D) 是李群，那么则有：

![](https://www.zhihu.com/equation?tex=%0AR%20%5Cboxplus%20r%3DR%20%5Cexp%28r%29%5Cquad%20R_1%5Cboxminus%20R_2%3D%5Clog%28R%5ET_2R_1%29%0A)

如果是向量的话，那么广义加减就定义为向量的加减

实际上就相当于使用了一个 C++ 中的运算重载，以此来方便运算

### IMU离散模型

在模型中，当时间间隔为 ![](https://www.zhihu.com/equation?tex=%5CDelta%20t) 的时候，设状态量为 ![](https://www.zhihu.com/equation?tex=x)，则离散模型中的数据可以表示为

![](https://www.zhihu.com/equation?tex=%0Ax%20%3D%20%5B%5EGR%5ET_I%2C%5EGp%5ET_I%2C%5EGv%5ET_I%2Cb%5ET_w%2Cb%5ET_a%2C%5EGg%5ET%5D%5ET%20%5Cin%20%5Cmathcal%7BM%7D%5C%5C%0Au%20%3D%20%5Bw%5ET_m%2Ca%5ET_m%5D%5ET%5C%5C%0Aw%20%3D%20%5Bn%5ET_w%2Cn%5ET_a%2Cn%5ET_%7Bbw%7D%2Cn%5ET_%7Bba%7D%5D%0A)

其中![](https://www.zhihu.com/equation?tex=x) 是状态量，即为所有的待估计变量，共18维，我们要实时估计的是一个18维的量，它包含旋转矩阵，位置，速度、角速度零偏、加速度零偏以及重力向量。因此它是一个紧耦合的框架。

输入是 ![](https://www.zhihu.com/equation?tex=u) （也即加速度测量值）,![](https://www.zhihu.com/equation?tex=w) 为所有的噪声量，包含测量噪声（前两项）和bias随机游走噪声（后两项），每一项都是三维的

而论文中的离散模型如下所示

![](https://www.zhihu.com/equation?tex=%0Ax_%7Bi%2B1%7D%3Dx_i%20%5Cboxplus%20%28%5CDelta%20tf%28x_i%2Cu_i%2Cw_i%29%29%5C%5C%0A)

 ![](https://www.zhihu.com/equation?tex=i) 是 IMU 测量的序号或者说索引，这个索引并不是整个过程中的，而是两帧 LiDAR 之间的索引，也就是每测量一帧的 LiDAR 都会执行一次上面的公式，然后重新计算 IMU 测量的索引

其中

![](https://www.zhihu.com/equation?tex=%0Af%28x_i%2Cu_i%2Cw_i%29%3D%0A%5Cleft%5B%5Cbegin%7Barray%7D%7Bc%7D%0Aw_%7Bmi%7D-b_%7Bwi%7D-n_%7Bwi%7D%5C%5C%0A%5EGv_%7BIi%7D%5C%5C%0AR_%7BIi%7D%28w_%7Bmi%7D-b_%7Bai%7D-n_%7Bai%7D%29%2B%5EGg_%7Bi%7D%5C%5C%0An_%7Bbwi%7D%5C%5C%0An_%7Bba%7D%5C%5C%0A0_%7B3%5Ctimes%201%7D%0A%5Cend%7Barray%7D%5Cright%5D%20%0A)

实际上广义加可以拆开来看，不同的位置单独执行广义计算

### 状态估计

这里使用纯字母表示真值，使用波浪线上标表示误差，使用横线表示最优估计，使用上三角表示估计值

### 前向传播

前向传播有两个内容，第一个就是基于IMU积分计算一个粗略的状态量，这个状态量用于后续的反向传播来补偿运动失真

![](https://www.zhihu.com/equation?tex=%0A%5Chat%7Bx%7D_%7Bi%2B1%7D%3D%5Chat%7Bx%7D_i%5Cboxplus%20%28%5CDelta%20tf%28%5Chat%7Bx%7D_i%2Cu_i%2C0%29%29%2C%5Chat%7Bx%7D_0%3D%5Cbar%7Bx%7D_%7Bk-1%7D%0A)

其中的时间差表示的是相邻两帧 IMU 的时间差，噪声量为0是因为不知道噪声的实际大小，因此在传播过程中设为0，但是会在后续的误差状态方程中考虑噪声

这个公式与离散模型公式一致，我们每接收一个 IMU 都会进行一次上述计算，直到计算到最后一个 IMU 帧为止。

另一个内容是传播误差量，并计算对应的协方差矩阵。这里的问题是我们不知道真值，怎么计算误差呢？实际上，我们计算的误差量，也是一个近似值，因此它才会有对应的协方差矩阵来评判置信度。和传统的卡尔曼滤波器不同的是，传统的卡尔曼滤波器直接估计状态量，它的运动方程和观测方程通常长这样：

![](https://www.zhihu.com/equation?tex=%0Ax_k%3Df%28x_%7Bk-1%7D%2Cu_k%29%2Bw_k%5C%5C%0Az_k%3Dh%28x_k%29%2Bv_k%0A)

而文中使用的误差状态卡尔曼滤波器(Error state Kalman flter，ESKF)，以误差量作为待估计量，也就是把上式的 x 用 ![](https://www.zhihu.com/equation?tex=%5Cwidetilde%7Bx%7D) 代替

我们现在要估计的是误差量，而不是直接估计状态量。而有了误差量的估计，再直接加上状态量的估计就是我们求得的最优估计，其中误差量

![](https://www.zhihu.com/equation?tex=%0A%5Cwidetilde%7Bx%7D_%7Bk-1%7D%3Dx_%7Bk-1%7D%5Cboxminus%20%5Cbar%7Bx%7D_%7Bk-1%7D%0A)



这将带来以下好处:

- 在旋转的处理上，ESKF的状态变量可以采用最小化的参数表达，也就是使用三维变量来表达旋转的增量。而传统KF需要用到四元数或者更高维的表达（如九维旋转矩阵），或采用带有奇异性的表达方式(欧拉角)
- ESKF 总是在原点附近，离奇异点较远，并且也不会由于离工作点太远而导致线性化近似不够的问题
- ESKF的状态量为小量，其二阶变量相对来说可以忽略。同时大多数雅可比矩阵在小量情况下变得非常简单，甚至可以用单位阵代替
- 误差状态的运动学也相比原状态变量要来得更小，因为我们可以把大量更新部分放到原状态变量中

# 非线性优化

在SLAM中，经常性会碰到各种优化问题，比如说给一个目标函数，求出使其最小化的解，并且这个目标函数往往是非线性的

当然，目前很多库，比如说Ceres、g2o还有gtsan等库都可以实现非线性优化具体求解操作，我们只需要把待求解函数输入即可

## 概述

在三维世界中，可以通过旋转向量、旋转矩阵、欧拉角和四元数等等方法来描述刚体的运动，并且可以通过李群李代数来进行优化，此外通过相机进行观测世界，但是回归最初的问题，也就是位姿方程和观测方程，其中位姿可以使用变换矩阵描述，然后使用李代数进行优化，观测方程由相机成像模型给出

![](https://www.zhihu.com/equation?tex=%0Ax_k%3Df%28x_%7Bk-1%7D%2Cu_k%29%2Bw_k%5C%5C%0Az_%7Bk%2Cj%7D%3Dh%28y_j%2Cx_k%29%2Bv_%7Bk%2Cj%7D%0A)

但是由于噪声的存在，上述等式无法准确成立，则在给定模型和具体观测的时候，需要进行优化，并且是非线性优化

首先可以考虑噪声模型，认为噪声服从正态分布，也即

![](https://www.zhihu.com/equation?tex=%0Aw_k%20%5Csim%20N%280%2CR_k%29%5C%5C%0Av_%7Bk%2Cj%7D%20%5Csim%20N%280%2CQ_%7Bk%2Cj%7D%29%0A)

并且，其他的变量可以做以下定义：

位姿变量：![](https://www.zhihu.com/equation?tex=x_k%3DT_k%3Dexp%28%5Cxi_k%5E%5Cland%29)​

路标点：![](https://www.zhihu.com/equation?tex=y_i)

在 ![](https://www.zhihu.com/equation?tex=x_k) 处对路标 ![](https://www.zhihu.com/equation?tex=y_i) 进行了一次观测，对应到图像上像素位置 ![](https://www.zhihu.com/equation?tex=z_%7Bk%2Cj%7D)

![](https://www.zhihu.com/equation?tex=%0Az_%7Bk%2Cj%7D%3DK%5Cexp%28%5Cxi%5E%5Cland%29y_j%0A)

然后我们得到了带有噪声的观测数据和传感器数据，就可以用来估计位姿 ![](https://www.zhihu.com/equation?tex=x) 和地图路标点 ![](https://www.zhihu.com/equation?tex=y)，具体方法可以使用滤波器（卡尔曼滤波）和非线性优化

从概率角度，所有待求解的量称为状态变量

![](https://www.zhihu.com/equation?tex=%0Ax%3D%5C%7Bx_1%2C%5Ccdots%2Cx_N%2Cy_1%2C%5Ccdots%2Cy_M%5C%7D%0A)

状态估计等同于求已知输入数据 ![](https://www.zhihu.com/equation?tex=u) 和观测数据 ![](https://www.zhihu.com/equation?tex=z) 的条件下，状态 ![](https://www.zhihu.com/equation?tex=x) 的条件概率分布

![](https://www.zhihu.com/equation?tex=%0AP%28x%7Cz%2Cu%29%5Cto%20%E8%BF%99%E9%87%8C%E7%9A%84z%E5%92%8Cu%E6%8C%87%E6%89%80%E6%9C%89%E6%97%B6%E5%88%BB%E4%B8%8B%E7%9A%84%EF%BC%8C%E4%B9%9F%E6%98%AF%E7%BB%9F%E7%A7%B0%0A)

先考虑特殊情况，也就是没有运动测量的传感器，只有观测数据

![](https://www.zhihu.com/equation?tex=%0AP%28x%7Cz%29%3D%5Cfrac%7BP%28z%7Cx%29P%28x%29%7D%7BP%28z%29%7D%5Cpropto%20P%28z%7Cx%29P%28x%29%5Cquad%20%E8%B4%9D%E5%8F%B6%E6%96%AF%E6%A8%A1%E5%9E%8B%0A)


## 先验、后验、似然

### 后验（知果求因）

假设，隔壁小哥要去15公里外的一个公园，他可以选择步行走路，骑自行车或者开车，然后通过其中一种方式花了一段时间到达公园。这件事中采用哪种交通方式是因，花了多长时间是果。

假设我们已经知道小哥花了1个小时到了公园，那么你猜他是怎么去的(走路or开车or自行车)，事实上我们不能百分百确定他的交通方式，我们正常人的思路是他很大可能是骑车过去的，当然也不排除开车过去却由于堵车严重花了很长时间。

这种预先已知结果(路上花的时间，在机器学习中就是观测到的X)，然后根据结果估计原因(交通方式)的概率分布即后验概率。

### 先验（由历史求因）

假设隔壁小哥还没去,我们根据他的个人历史习惯来推测他会以哪种方式出行。

假设我们比较了解小哥的个人习惯，小哥是个健身爱好者就喜欢跑步运动，这个时候我们可以猜测他更可能倾向于走路过去。当然我的隔壁小哥是个大死肥宅，这个时候我们猜测他更可能倾向于坐车，连骑自行车的可能性都不大。

这个情景中隔壁小哥的交通工具选择与花费时间不再相关。因为我们是在结果发生前就开始猜的，根据历史规律确定原因(交通方式)的概率分布，即先验概率。

### 似然估计（知因求果）

换个情景，先考虑小哥去公园的交通方式。

假设隔壁小哥步行走路去，一般情况下小哥大概要用2个小时;假设小哥决定开车，到公园半个小时是非常可能的。

这种先定下来原因，根据原因(出行方式)来估计结果的概率分布即似然估计， 根据原因来统计各种可能结果的概率即似然函数。

### 状态估计

![](https://www.zhihu.com/equation?tex=P%28x%7Cz%29) 直接求后验分布是困难的，但是求一个状态最优估计，使得在该状态下后验概率最大化，是可行的。

最大后验估计（MAP）：求一个状态最优估计，使得在该状态下后验概率最大化

![](https://www.zhihu.com/equation?tex=%0Ax%5E%2A_%7BMAP%7D%3D%5Carg%5Cmax%20P%28x%7Cz%29%3D%5Carg%5Cmax%20P%28z%7Cx%29P%28x%29%0A)

最大似然估计（MLE）：在哪种状态下，最容易产生当前的观测

![](https://www.zhihu.com/equation?tex=%0Ax%5E%2A_%7BMLE%7D%3D%5Carg%5Cmax%20P%28z%7Cx%29%0A)


## 最小二乘的引出

由观测方程和噪声服从高斯分布可知

![](https://www.zhihu.com/equation?tex=%0Az_%7Bk%2Cj%7D%7Cx_k%5Csim%20N%28h%28y_j%2Cx_k%29%2CQ_%7Bk%2Cj%7D%29%0A)

高维的高斯分布：![](https://www.zhihu.com/equation?tex=x%5Csim%20N%28%5Cmu%2C%5CSigma%29)

则概率密度为

![](https://www.zhihu.com/equation?tex=%0AP%28x%29%3D%5Cfrac%7B1%7D%7B%282%5Cpi%29%5E%7BN/2%7D%20%7C%5Cboldsymbol%7B%5CSigma%7D%7C%5E%7B1/2%7D%7D%20%5Cexp%20%5Cleft%28%20-%5Cfrac%7B1%7D%7B2%7D%20%28%5Cmathbf%7Bx%7D%20-%20%5Cboldsymbol%7B%5Cmu%7D%29%5ET%5Cboldsymbol%7B%5CSigma%7D%5E%7B-1%7D%20%28%5Cmathbf%7Bx%7D%20-%20%5Cboldsymbol%7B%5Cmu%7D%29%20%5Cright%29%0A)

对其取负对数，则有

![](https://www.zhihu.com/equation?tex=%0A-%5Cln%20P%28x%29%20%3D%20%5Cfrac%7BN%7D%7B2%7D%20%5Clog%282%5Cpi%29%20%2B%20%5Cfrac%7B1%7D%7B2%7D%20%5Clog%20%7C%5Cboldsymbol%7B%5CSigma%7D%7C%20%2B%20%5Cfrac%7B1%7D%7B2%7D%20%28%5Cmathbf%7Bx%7D%20-%20%5Cboldsymbol%7B%5Cmu%7D%29%5ET%20%5Cboldsymbol%7B%5CSigma%7D%5E%7B-1%7D%20%28%5Cmathbf%7Bx%7D%20-%20%5Cboldsymbol%7B%5Cmu%7D%29%0A)

其中，左边两项是固定的值，也就是说当最小化 ![](https://www.zhihu.com/equation?tex=x) 时，只与最右侧项有关

这样原问题的最大化，相当于负对数的最小化，进一步，最小化右侧二次型项，就得到了对状态的最大似然估计，也就是最小二乘的问题

![](https://www.zhihu.com/equation?tex=%0Ax%5E%2A%3D%5Carg%5Cmin%5Cleft%28%20%28z_%7Bk%2Cj%7D%20-%20h%28x_k%2Cy_j%29%29%5ET%20Q_%7Bk%2Cj%7D%5E%7B-1%7D%20%28z_%7Bk%2Cj%7D%20-%20h%28x_k%2Cy_j%29%29%5Cright%29%0A)

考虑实际中批量处理数据时，由于各个时刻的输入和观测相互独立，数学语言就是

![](https://www.zhihu.com/equation?tex=%0AP%28z%2Cu%7Cx%2Cy%29%3D%5Cprod%5Climits_kP%28u_k%7Cx_%7Bk-1%7D%2Cx_k%29%20%0A%5Cprod%5Climits_%7Bk%2Cj%7DP%28z_%7Bk%2Cj%7D%7Cx_k%2Cy_j%29%0A)

对数据的最小化估计误差也就是最大似然估计

![](https://www.zhihu.com/equation?tex=%0AJ%28x%29%3D%5Csum_ke%5ET_%7Bv%2Ck%7DR%5E%7B-1%7D_ke_%7Bv%2Ck%7D%2B%5Csum_k%20%5Csum_j%20e%5ET_%7By%2Ck%2Cj%7DQ%5E%7B-1%7D_%7Bk%2Cj%7De_%7By%2Ck%2Cj%7D%0A)

由SLAM的状态估计，得到了最小二乘问题：最优解等价于状态的最小二乘问题 ![](https://www.zhihu.com/equation?tex=%5Cmin%20J%28x%29)，其由许多个误差的平方和组成。虽然整体维度较高，但是每个项很简单，仅与一两个状态变量有关(比如运动误差只与 ![](https://www.zhihu.com/equation?tex=x_%7Bk-1%7D%2Cx_k) 有关,观测误差只与 ![](https://www.zhihu.com/equation?tex=x_k%2Cy_j)​ 有关)

如果用李代数表达位姿，则是无约束优化问题，那么如何求解此类的非线性最小二乘问题？

## 非线性最小二乘法

先考虑最简单的问题：

![](https://www.zhihu.com/equation?tex=%0A%5Cmin_x%20F%28x%29%3D%5Cfrac12%5CVert%20f%28x%29%5CVert_2%5E2%0A%5Cquad%20x%5Cin%5Cmathbb%20R%5En%5C%5C%0A)

其中 ![](https://www.zhihu.com/equation?tex=f) 是任意的标量非线性函数，![](https://www.zhihu.com/equation?tex=x) 可以是向量

当 ![](https://www.zhihu.com/equation?tex=f) 很简单的时候，令目标函数的导数为零，就可以求解最小值，从一系列极值中筛选

当 ![](https://www.zhihu.com/equation?tex=f)​ 很复杂的时候，就难以使用这种方法求解，所以需要一种迭代的方法求解，从一个初始值出发，不断优化当前的优化变量，使得目标函数下降，直到直到某个时刻增量非常小，无法使函数下降，则算法收敛，目标达到了个极小，也就完成寻找极小值的过程。但是迭代过程中的增量如何寻找呢？这就需要使用梯度法了。

![1717424409893](./assets/1717424409893.png)

![1717424704403](./assets/1717424704403.png)

![1717426754668](./assets/1717426754668.png)

![1717426899641](./assets/1717426899641.png)

![1717427097473](./assets/1717427097473.png)

![1717427212653](./assets/1717427212653.png)

![1717427317391](./assets/1717427317391.png)

![1717427333970](./assets/1717427333970.png)

# 视觉里程计

## 特征点提取与匹配概述

现在开始就是SLAM系统的重要算法，其中视觉里程计方法就是对图像进行特征提取与匹配，计算出来两帧之间相机的位姿变换等，这里会涉及特征点法的使用，主要是有如下几点需要注意

- 理解图像特征点的意义，掌握在单幅图像中提取特征点及多幅图像中匹配特征点的方法
- 理解对极几何的原理，利用对极几何的约束，恢复出图像之间的摄像机的三维运动
- 理解如何通过三角化，获得二维图像上对应的三维结构
- 理解PNP问题，及利用已知三维结构与图像的对应关系，求解摄像机的三维运动
- 理解ICP问题，及利用点云的匹配关系，求解摄像机的三维运动

经典SLAM模型中以相机位姿-路标来描述SLAM过程，路标是三维空间中固定不变的点，可以在特定位姿下观测到，在视觉SLAM中，可利用图像特征点作为SLAM中的路标

特征点是图像当中具有代表性的部分，如轮点，较暗区域中的亮点较亮区域中的暗点等。特征点应该有如下特点，以便于快速准确的进行匹配

- 可重复性：相同的区域可以在不同的图像中找到
- 可区别性：不同的区域有不同的表达
- 高效率：同一图像中，特征点的数量应该小于像素的位置
- 本地性：特征仅与一小片图像区域有关

特征点的信息有关键点和描述子两部分，关键点指的是位置、大小、方向、评分等，描述子是特征点周围的图像信息，如：当谈论在一张图像中计算SIFT特征时，是指“提取SIFT关键点并计算SIFT描述子”。

## ORB特征

ORB特征是一种更为高效的方法，其来源于著名的VSLAM框架ORB-SLAM，关键点是Oriented FAST(一种改进的FAST角点)，描述子是改进 BRIEF

### 关键点

FAST：主要检测局部像素灰度变化明显的地方(如果一个像素与领域的像素差别较大，则更可能是角点)

1. 在图像中选取像素 ![](https://www.zhihu.com/equation?tex=p)，假设它的亮度为 ![](https://www.zhihu.com/equation?tex=I_p)
2. 设置一个阈值 ![](https://www.zhihu.com/equation?tex=T)（比如 ![](https://www.zhihu.com/equation?tex=I_p) 的20%）。
3. 以像素 ![](https://www.zhihu.com/equation?tex=p) 为中心,选取半径为3的圆上的16个像素点。
4. 假如选取的圆上，有连续的N个点的亮度大于 ![](https://www.zhihu.com/equation?tex=I_p%2BT) 或小于 ![](https://www.zhihu.com/equation?tex=I_p-T) ，那么像素 ![](https://www.zhihu.com/equation?tex=p)​ 可以被认为是特征点(N通常取12，即为FAST-12。其它常用的N取值为9和11，他们分别被称为 FAST-9，FAST-11)。
5. 循环以上四步，对每一个像素执行相同的操作。

在FAST12中，提出一个高效的测试，来快速排除一大部分非特征点的点。该测试仅仅检查在位置1、5、9、13四个位置的像素，如果不满足至少三个角点亮度大于 ![](https://www.zhihu.com/equation?tex=I_p%2BT) 或小于 ![](https://www.zhihu.com/equation?tex=I_p-T) ，那么 ![](https://www.zhihu.com/equation?tex=p) 不可能是一个角点。

![](https://miro.medium.com/v2/resize:fit:940/format:webp/1*DWN1lJ-tgRjifQ9R7dwPAg.png)

当然FAST也有缺点，原始FAST角点经常出现扎堆的现象(分布不均匀)。所以在第一遍检测之后，还需要用非极大值抑制，在一定区域内仅保留响应极大值的角点，避免角点集中问题。

由干FAST角点不具有方向信息且存在尺度问题，ORB添加了尺度和旋转的描述：尺度不变性通过构建图像金字塔来实现，旋转是由灰度质心法来实现。

![1717428698381](./assets/1717428698381.png)

![1717428746582](./assets/1717428746582.png)

### 描述子

然后描述子部分是BRIEF，一种二进制描述子，其描述向量由许多个0和1组成，这些描述了周围128个点的亮度值，或者说对比了周围两个像素点的亮度值

![](https://miro.medium.com/v2/resize:fit:1250/format:webp/0*bTfQfO4qOxk3qL78)

实际上操作方法如下多种，只不过工程上经常性使用第二种方法：

1. 在图像块内平均采样
2. ![](https://www.zhihu.com/equation?tex=p) 和 ![](https://www.zhihu.com/equation?tex=q) 都符合 ![](https://www.zhihu.com/equation?tex=%280%2C%5Cfrac%7B1%7D%7B25%7DS%5E2%29) 的高斯分布，或者也有这种情况：
3. ![](https://www.zhihu.com/equation?tex=p) 符合 ![](https://www.zhihu.com/equation?tex=%280%2C%5Cfrac%7B1%7D%7B25%7DS%5E2%29) 的高斯分布，而 ![](https://www.zhihu.com/equation?tex=q) 符合 ![](https://www.zhihu.com/equation?tex=%280%2C%5Cfrac%7B1%7D%7B100%7DS%5E2%29) 的高斯分布
4. 在空间量化极坐标下的离散位置随机采样
5. 把 ![](https://www.zhihu.com/equation?tex=p) 固定为 ![](https://www.zhihu.com/equation?tex=%280%2C0%29)，![](https://www.zhihu.com/equation?tex=q)​ 在周围平均采样（实际上通过预先定义好的位置采用）

总结：描述子:BRIEF总结

1. 优点：BRIEF使用了随机选点的比较，速度比较快，而且由于使用了二进制表达，存储起来也十分方便。
2. 缺点：原始的BRIEF描述子不具有旋转不变性，在图像发生旋转时容易走失。而ORB在FAST特征点提取阶段计算了关键点的方向，计算了旋转之后的“BRIEF”特征使ORB的描述子具有较好的旋转不变性。
3. 注意: BRIEF是一种二进制描述，需要用汉明距离度量

### ORB流程

ORB的流程如下图所示，

1. 根据改进特征点法计算出来特征关键点（这里是篮球的中心点）为圆心（几何质心），然后以 ![](https://www.zhihu.com/equation?tex=d) 为半径作圆
2. 计算质心，得到特征点方向（绿色箭头）
3. 选取某一个模式下的pq点对，书上代码是128对，这里为了便于表示只选取2对
4. 根据特征点方向，将其旋转到与 ![](https://www.zhihu.com/equation?tex=x) 轴平行，并且对所有的pq点对进行旋转（这里利用了特征点的方向性和旋转不变性），这样，这一帧的特征点就可以跟上一帧的特征点匹配上
5. 按照规则对比所有的pq点对的光照强度或者灰度值
6. 按照顺序排列描述子

![1717467233001](./assets/1717467233001.png)

## 特征匹配

特征匹配解决了SLAM中的数据关联问题，即确定当前看到的路标与之前看到的路标之间的对应关系。

通过对图像与图像或者图像与地图之间的描述子进行准确匹配，可以为后续的姿态估计、优化等操作减轻大量负担；然而，由于图像特征的局部特性，误匹配的情况存在。

考虑两个时刻的图像：

1. 在图像 ![](https://www.zhihu.com/equation?tex=I_t) 中提取到特征点 ![](https://www.zhihu.com/equation?tex=x_t%5Em%2Cm%3D1%2C2%2C%5Cdots%2CM)；在图像 ![](https://www.zhihu.com/equation?tex=I_%7Bt%2B1%7D) 中提取到特征点 ![](https://www.zhihu.com/equation?tex=x_%7Bt%2B1%7D%5En%2Cn%3D1%2C2%2C%5Cdots%2CN)
2. 暴力匹配：对每一个特征点 ![](https://www.zhihu.com/equation?tex=x_t%5Em) 与所有的 ![](https://www.zhihu.com/equation?tex=x_%7Bt%2B1%7D%5En) 测量描述子的距离，然后排序，取最近的一个作为匹配点
3. 当特征点数量很大时，暴力匹配的运算量会变得很大，选用一些改进算法，如：快速近似最近邻(FLANN)算法

在进行了特征点匹配方法之后，就可以进一步计算相机的运动了

## 计算相机运动

计算有三种情况，也就是在输入数据是二维图像还是三维点云的情况下，如何进行计算

- 如果只有两个单目图像，得到2D-2D间的关系(对极几何)
- 得到一些3D点和2维图像投影，即得到3D-2D间的关系(PNP)
- 当相机为双目、RGB-D，或者通过某种方法得到距离信息得到3D-3D间的关系(ICP)

### 单目图像——对极几何

已知的信息如下：

- 三维空间点 ![](https://www.zhihu.com/equation?tex=P) 在像素坐标平面的投影点为 ![](https://www.zhihu.com/equation?tex=p_1) 和 ![](https://www.zhihu.com/equation?tex=p_2)（像素坐标）；![](https://www.zhihu.com/equation?tex=I_1) 的一个特征点 ![](https://www.zhihu.com/equation?tex=p_1) 在 ![](https://www.zhihu.com/equation?tex=I_2) 中对应着特征点 ![](https://www.zhihu.com/equation?tex=p_2)(匹配正确的情况下)；相机内参

想求：第一帧到第二帧的相机运动为 ![](https://www.zhihu.com/equation?tex=R%2Ct)

![](https://bucket-1259555870.cos.ap-chengdu.myqcloud.com/20191218130654.png)

上图中，基线是 ![](https://www.zhihu.com/equation?tex=O_1O_2)，极点是基线和像平面的交点 ![](https://www.zhihu.com/equation?tex=%28e_1%2Ce_2%29)，极平面是包含基线的平面（或者说，是 ![](https://www.zhihu.com/equation?tex=O_1O_2P) 的平面，三个点确定一个平面），极线就是极平面与图像平面的交线

单张图像只能确定 ![](https://www.zhihu.com/equation?tex=P) 在某个直线上，无法确定深度信息，但是两张图像就可以在有约束的情况下恢复

