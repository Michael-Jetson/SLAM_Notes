# Ceres

Ceres 问题求解主要分成以下三部分：

1. 构建 cost function，即代价函数，也就是寻优的目标式(通过预测结果和测量值求误差 的函数)。这个部分需要使用函子（functor）这一技巧来实现；
2. 通过上一步的代价函数构建待求解的优化问题
3. 配置求解器参数并求解问题，这个步骤就是设置方程怎么求解、求解过程是否输出等， 然后调用一下 Solve 方法。

总体上 Ceres 的流程如下

![](https://i-blog.csdnimg.cn/blog_migrate/30b59c3bd423ca91ab1a37286f03faf4.png#pic_center)

```c++
// STEP1：构建优化问题对象
ceres::Problem problem;
// STEP2：构建代价函数
ceres::CostFunction* cost_function = ...; 
// STEP3：添加代价函数、核函数
problem.AddResidualBlock(cost_function, NULL, &x); 

// STEP4：配置求解器
ceres::Solver::Options options;
options.linear_solver_type = ceres::DENSE_QR;
options.minimizer_progress_to_stdout = true;
ceres::Solver::Summary summary;
// STEP5：运行求解器
ceres::Solve(options, &problem, &summary);
```

## 代价函数

代价函数最重要的功能就是计算残差向量和雅可比矩阵。**Ceres提供了三种求导方法，分别是：解析求导、自动求导、数值求导**。

**在SLAM中，使用的一般都是解析求导，这种方法需要自己填入雅克比函数**。

# GTSAM：因子图优化

## 概率图

概率图模型是⽤图来表⽰变量概率依赖关系的理论，结合概率论与图论的知识，利⽤图来表⽰ 与模型有关的变量的联合概率分布。在概率图模型中，注意变量有条件独⽴性。

实际上对于SLAM中的内容，可以建模为下图所示的图模型，黄色代表的就是机器人在不同时刻的位姿信息

![](https://i-blog.csdnimg.cn/blog_migrate/3772ecd3618b37c43b3487976ec0e1f6.png)

## 贝叶斯网络

⻉叶斯⽹络是概率图模型中的有向图，其⽹络拓朴结构是⼀个有向⽆环图(DAG)，在⻉叶斯⽹络中，节点表⽰随机变量，边表⽰依赖关系,在⻉叶斯⽹络中为单向依赖。在SLAM中，这⾥随机变量既可以是观测量，也可以是状态量。

![](https://i-blog.csdnimg.cn/blog_migrate/07d2b1f7335008eaec4a25c5e024facf.png)

一个贝叶斯网络的概率公式如下：
$$
p(\Theta)=\prod_{i=0}p(\theta_j|\pi_j)
$$
其中 $\pi_j$ 为 $\theta_j$ 的父节点（后者是在前者的状态上获取的，或者说后者依赖于前者），$p(\theta_j|\pi_j)$ 表示了条件概率密度

## 因子图

⻉叶斯⽹络是⽣成模型，即已知状态，推测观测，⽽SLAM问题是已知观测，估计状态，所以贝叶斯网络不是很适合SLAM问题。⻉叶斯⽹络更适合建模，⽽因⼦图更适合推断。而因⼦就是观测，因⼦图假设所有观测服从⾼斯分布。因⼦图求解就是从全局的⻆度让状态量接近观测量的过程。最终将这些概率密度函数乘积最⼤化问题转化为最⼩⼆乘问题。

GTSAM中的因子是由变量和观测组成，比如说先验因子就是变量减去观测