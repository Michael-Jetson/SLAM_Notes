# 概述

## slam介绍

slam是什么呢？slam就是同时建图与定位，我们先不用学术的说法去介绍它，而是先用实际生活中的例子来介绍。

大家知道，如果你在一个地方生活久了，就会很熟悉这个地方，可能随便把你扔在其中某一个地点，你就可以知道你在什么位置，你应该怎么走回家，但是如果你出现在一个陌生的没去过的地方，你可能就一时半会不知道自己的位置了。

这个过程其实就是一个slam的过程，你在熟悉的环境下走路，相当于已经有一个地图在你大脑里面了，你需要根据眼睛看到的信息（相当于传感器感知环境）来判断自己在这个地图的什么地方，然后就可以进行自主导航了。

## 传感器对比

激光雷达与相机对比

- 激光雷达有效距离远，可以达到百米级别（可达300m），相机有效距离相对近
- 雷达贵，动辄上万，相机较为便宜，高速相机不过上千
- 激光雷达受天气影响较大，雨雪都会成为巨大干扰，因为雨雪也会反射激光
- 雷达重，激光较轻
- 在纹理不清楚或者光线过强过弱的地方，相机几乎无法运作

# 三维空间刚体运动

想描述三维空间中的物体运动，就必须先确定坐标系的概念，因为运动都是相对的，我们无法描述一个物体的绝对运动情况，只能描述一个物体的相对运动情况

## 点与坐标系

对于我们所关心的对象，都会有一个与之相关的坐标系，比如说一个激光雷达就会有一个雷达坐标系，有一个相机就会有一个相机坐标系，一般我们关心多少物体就会有多少坐标系

### 坐标系

在机器人领域，一般使用右手系，机器人的运动也都是在右手系里面进行讨论

![image-20230328135941278](/home/robot/文档/Notes/img/image-20230328135941278.png)

如我们在激光坐标系下观测到某些情况，在相机坐标系下拍摄到某些情况

### 向量运算

对于向量![](https://www.zhihu.com/equation?tex=a%2Cb)

定义内积（也就是点乘）：

![](https://www.zhihu.com/equation?tex=%0Aa%20%5Ccdot%20b%3Da%5ETb%3D%5Csum%5Climits%5E3_%7Bi%3D1%7Da_ib_i%3D%5Cvert%20a%20%5Cvert%20%5Cvert%20b%5Cvert%20%5Ccos%20%3Ca%2Cb%3E%0A)

定义外积（结果是一个与两个向量都垂直的向量）：

![](https://www.zhihu.com/equation?tex=%0A%5Cbegin%7Balign%7D%0Aa%5Ctimes%20b%26%3D%0A%5Cleft%7C%0A%5Cbegin%7Bmatrix%7D%0A%5Cbold%7Bi%7D%26%5Cbold%7Bj%7D%26%5Cbold%7Bk%7D%5C%5C%0Aa_1%26a_2%26a_3%5C%5C%0Ab_1%26b_2%26b_3%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%7C%5C%5C%0A%26%3D%0A%28a_2b_3-a_3b_2%29%5Cbold%7Bi%7D%2B%28a_3b_1-a_1b_3%29%5Cbold%7Bj%7D%2B%28a_1b_2-a_2b_1%29%5Cbold%7Bk%7D%5C%5C%0A%0A%26%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0Aa_2b_3-a_3b_2%5C%5C%0Aa_3b_1-a_1b_3%5C%5C%0Aa_1b_2-a_2b_1%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%5C%5C%0A%26%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0A0%26-a_3%26a_2%5C%5C%0Aa_3%260%26-a_1%5C%5C%0A-a_2%26a_1%260%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5Db%5C%5C%0A%26%5Ctriangleq%0Aa%5E%5Cland%20b%0A%5Cend%7Balign%7D%0A)


### 坐标变换

**问题**：如果我们在相机或者雷达坐标系下观测到一个对象，那么这个对象在世界坐标系下或者机器人坐标系下的位置是如何表示的呢？

这里就需要用数学公式来表述了，也就是坐标变换的问题：如何计算同一个向量在不同坐标系里的坐标？

实际上，两个坐标系之间的关系，只有旋转和平移两种，或者说，两个坐标系之间的欧式变换由旋转和平移组成

## 旋转矩阵

### 齐次坐标

![](https://www.zhihu.com/equation?tex=%5Ctilde%7Ba%7D%3D%5Cleft%5B%5Cbegin%7Bmatrix%7Da%5C%5C1%5Cend%7Bmatrix%7D%5Cright%5D%EF%BC%8C%E5%9C%A8%E6%91%84%E5%BD%B1%E5%87%A0%E4%BD%95%E9%87%8C%E9%9D%A2%E9%9D%9E%E5%B8%B8%E5%B8%B8%E8%A7%81%EF%BC%8C%E4%B9%98%E4%BB%A5%E9%9D%9E%E9%9B%B6%E5%B8%B8%E6%95%B0%E7%9A%84%E6%97%B6%E5%80%99%E4%BB%8D%E7%84%B6%E8%A1%A8%E8%BE%BE%E5%90%8C%E4%B8%80%E4%B8%AA%E5%9D%90%E6%A0%87)

即：

![](https://www.zhihu.com/equation?tex=%0A%5Ctilde%7Ba%7D%3D%5Cleft%5B%5Cbegin%7Bmatrix%7Da%5C%5C1%5Cend%7Bmatrix%7D%5Cright%5D%3Dk%5Cleft%5B%5Cbegin%7Bmatrix%7Da%5C%5C1%5Cend%7Bmatrix%7D%5Cright%5D%0A)


### 欧式群

变换矩阵的集合称为特殊欧式群![](https://www.zhihu.com/equation?tex=SE%283%29)（Special Euclidean Group）

![](https://www.zhihu.com/equation?tex=%0ASE%283%29%3D%0A%5Cleft%5C%7B%0AT%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0AR%26t%5C%5C%0A0%5ET%261%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%0A%5Cin%0A%5Cmathbb%7BR%7D%5E%7B4%5Ctimes%204%7D%0A%5Cvert%0AR%5Cin%20SO%283%29%2Ct%5Cin%20%5Cmathbb%7BR%7D%5E3%0A%5Cright.%0A)

逆形式：

![](https://www.zhihu.com/equation?tex=%0AT%5E%7B-1%7D%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0AR%5ET%26-R%5ETt%5C%5C%0A0%5ET%261%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%0A)


### 欧拉旋转定理（Euler's rotation theorem）

刚体在三维空间里的一般运动，可以分解为刚体上某一点的平移，以及绕经过此点的旋转轴的转动

即两个坐标系直接的运动可以完全用![](https://www.zhihu.com/equation?tex=R%EF%BC%8Ct)描述

![](https://www.zhihu.com/equation?tex=%0Aa%5E%7B%5Cprime%7D%3DRa%2Bt%0A)

但是，如果使用旋转矩阵加平移向量，在复合情况下会有不便之处

如果有![](https://www.zhihu.com/equation?tex=b%3DR_1a%2Bt_1)，![](https://www.zhihu.com/equation?tex=c%3DR_2b%2Bt_2)

那么复合形式就是

![](https://www.zhihu.com/equation?tex=%0Ac%3DR_2%28R_1a%2Bt_1%29%2Bt_2%0A)

如果使用齐次形式，那么就比较方便

![](https://www.zhihu.com/equation?tex=%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0A%5Ctilde%7Ba%7D%5C%5C1%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%0A%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0AR%26t%5C%5C%0A0%5ET%261%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%0A%5Cleft%5B%5Cbegin%7Bmatrix%7Da%5C%5C1%5Cend%7Bmatrix%7D%5Cright%5D%0A%5Ctriangleq%0AT%5Cleft%5B%5Cbegin%7Bmatrix%7Da%5C%5C1%5Cend%7Bmatrix%7D%5Cright%5D%0A%5C%5C%0A%5Ctilde%7Bb%7D%3DT_1%5Ctilde%7Ba%7D%2C%5Ctilde%7Bc%7D%3DT_2%5Ctilde%7Bb%7D%5CRightarrow%20%5Ctilde%7Bc%7D%3DT_2T_1%5Ctilde%7Bb%7D%0A)


## 旋转向量与欧拉角

#### 旋转向量

#### 欧拉角

实际上，欧拉角的定义方式比较多（XYZ三轴不同的先后顺序），而且会存在奇异性问题（万向锁这种），所以一般不会直接使用，在SLAM中也很少使用欧拉角表示姿态

#### 万向锁

这是欧拉角奇异性问题的一种体现，即旋转角在特定值的时候，会有两个旋转轴重合，这种情况下旋转自由度减一

正常情况下

![image-20230329201433675](https://github.com/Michael-Jetson/SLAM_Notes/blob/main/img/image-20230329201433675.png?raw=true)

奇异情况下

![image-20230329201447449](https://github.com/Michael-Jetson/SLAM_Notes/blob/main/img/image-20230329201447449.png?raw=true)

在这种时候，两轴重合，会存在这种情况

可以证明，无法在仅用三个实数表达旋转的时候避免奇异性问题

但是，如果使用四个数来表达旋转，则不会出现这种情况，这也就是四元数的用处

### 四元数

这是一种节省空间而且没有奇异性的表达形式，可以用来描述旋转

2D 情况下，可用单位复数表达旋转

![](https://www.zhihu.com/equation?tex=%0Az%3Dx%2Biy%3D%5Crho%20e%5E%7Bi%5Ctheta%7D%0A)

三维情况下，四元数就是复数的扩充

四元数（Quaternion）

**如何使用四元数描述旋转**

设有点![](https://www.zhihu.com/equation?tex=p)，在经历了一次四元数![](https://www.zhihu.com/equation?tex=q)表示的旋转之后，得到了![](https://www.zhihu.com/equation?tex=p%5E%5Cprime)，他们的关系如何表述

![](https://www.zhihu.com/equation?tex=%0Ap%5E%5Cprime%3Dqpq%5E%7B-1%7D%0A)


## 程序设计

使用动态矩阵的时候，运算会比较慢

# 李群和李代数

## 背景

当我们去估计相机位姿的时候，当我们的估计不准确的时候，如何对旋转和平移进行微调呢？

但是我们可以发现，旋转矩阵是无法相加的，相加之后就不是一个旋转矩阵了，所以我们需要用一种新理论去完成这个操作。

### 问题

首先我们知道，运动![](https://www.zhihu.com/equation?tex=x)是可以被观测的，也就是说，我们可以使用一个观测模型![](https://www.zhihu.com/equation?tex=R%2Ct)来描述，或者用一个变换矩阵![](https://www.zhihu.com/equation?tex=T)来描述

但是，在SLAM中有一个问题，值是估计出来的，如果我们发现不准确，那么就应该进行调整，得到新的估计，那么我们设调整后的旋转平移为![](https://www.zhihu.com/equation?tex=R%5E%5Cprime%2Ct%5E%5Cprime)，设他们直接相差一个微小量![](https://www.zhihu.com/equation?tex=%5CDelta%20R%2C%5CDelta%20t)，即有

![](https://www.zhihu.com/equation?tex=%0At%5E%5Cprime%3Dt%2B%5CDelta%20t%5C%5C%0AR%5E%5Cprime%20%3DR%2B%5CDelta%20R%0A)

但是，我们知道![](https://www.zhihu.com/equation?tex=R)对乘法封闭，但是对加减是不封闭的，自然也没办法进行求导，因为优化是必须基于导数的，但是如果有一个函数![](https://www.zhihu.com/equation?tex=u%28R%29)，那么有

![](https://www.zhihu.com/equation?tex=%0A%5Cfrac%7Bdu%7D%7BdR%7D%3D%5Clim%5Climits_%7B%5CDelta%20R%5Cto%200%7D%5Cfrac%7Bu%28R%2B%5CDelta%20R%29-u%28R%29%7D%7B%5CDelta%20R%7D%0A)

但是由于其性质，我们无法完成这个求导操作，自然无法完成优化

## 群

三维旋转矩阵构成了特殊正交群![](https://www.zhihu.com/equation?tex=SO%283%29)（Special Orthogonal Group）

![](https://www.zhihu.com/equation?tex=%0ASO%283%29%3D%0A%5Cleft%5C%7B%0AR%0A%5Cin%0A%5Cmathbb%7BR%7D%5E%7B3%5Ctimes%203%7D%0A%5Cvert%0ARR%5ET%3DI%2Cdet%28R%29%3D1%0A%5Cright%5C%7D%0A)



三维变换矩阵的集合称为特殊欧式群![](https://www.zhihu.com/equation?tex=SE%283%29)（Special Euclidean Group）

![](https://www.zhihu.com/equation?tex=%0ASE%283%29%3D%0A%5Cleft%5C%7B%0AT%3D%0A%5Cleft%5B%0A%5Cbegin%7Bmatrix%7D%0AR%26t%5C%5C%0A0%5ET%261%5C%5C%0A%5Cend%7Bmatrix%7D%0A%5Cright%5D%0A%5Cin%0A%5Cmathbb%7BR%7D%5E%7B4%5Ctimes%204%7D%0A%5Cvert%0AR%5Cin%20SO%283%29%2Ct%5Cin%20%5Cmathbb%7BR%7D%5E3%0A%5Cright%5C%7D%0A)

那么什么是群？

群是一种集合加上一种运算的代数结构（容易验证，旋转或者变换矩阵集合与矩阵乘法构成群，因此称为旋转矩阵群和变换矩阵群）

记集合为![](https://www.zhihu.com/equation?tex=A)，运算为![](https://www.zhihu.com/equation?tex=%5Ccdot)，那么当运算满足以下性质时，称二元组![](https://www.zhihu.com/equation?tex=%28A%2C%5Ccdot%29)构成群

1. 封闭性：![](https://www.zhihu.com/equation?tex=1)

## 李群

李群是具有连续性质的群，或者说这个群是光滑的

既是群也是流形

直观上看，一个刚体能够连续地在空间中运动，故![](https://www.zhihu.com/equation?tex=SO%283%29)和![](https://www.zhihu.com/equation?tex=SE%283%29)都是李群。

但是，![](https://www.zhihu.com/equation?tex=SO%283%29)和![](https://www.zhihu.com/equation?tex=SE%283%29)只有定义良好的乘法，没有加法，所以难以进行取极限、求导等操作。

## 李代数

李代数是与李群对应的一种结构，位于向量空间

从旋转矩阵引出李代数

我们考虑任意旋转矩阵![](https://www.zhihu.com/equation?tex=R)，满足![](https://www.zhihu.com/equation?tex=RR%5ET%3DI)

在连续运动过程中，显然![](https://www.zhihu.com/equation?tex=R)是连续时间的函数，我们记为![](https://www.zhihu.com/equation?tex=R%28t%29R%28t%29%5ET%3DI)

两侧对时间求导

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ET%2BR%28t%29%5Cdot%7BR%7D%28t%29%5ET%3D0%5C%5C%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ET%3D-%28%5Cdot%7BR%7D%28t%29R%28t%29%5ET%29%5ET%0A)

如果我们将![](https://www.zhihu.com/equation?tex=%5Cdot%7BR%7D%28t%29R%28t%29%5ET)看做一个整体，我们就发现其为一个反对称矩阵

我们记为

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ET%3D%5Cphi%28t%29%5E%5Cland%0A)

两边右乘![](https://www.zhihu.com/equation?tex=R%28t%29)

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29R%28t%29%5ETR%28t%29%3D%5Cphi%28t%29%5E%5Cland%20R%28t%29%0A)

其中![](https://www.zhihu.com/equation?tex=R%28t%29%5ETR%28t%29%3DI)，消去后得到

![](https://www.zhihu.com/equation?tex=%0A%5Cdot%7BR%7D%28t%29%3D%5Cphi%28t%29%5E%5Cland%20R%28t%29%0A)

可以看成求导之后，左侧多出一个![](https://www.zhihu.com/equation?tex=%5Cphi%28t%29%5E%5Cland)

# 相机模型

观测，也就是机器人如何观测外部世界，如果使用激光雷达观测或者使用相机观测，也就构成了激光slam或者视觉slam

## 缺失距离维度的照片

照片记录了真实世界在成像平面上的投影，这个过程丢弃了“距离”维度上的信息，就比如说下面这个照片，实际上两个人是一样大小，但是照片中仿佛棕色衣服的是巨人一样

![在这里插入图片描述](https://img-blog.csdnimg.cn/c38ce619f95f46fab35153cdc8d3d53b.jpeg#pic_center)