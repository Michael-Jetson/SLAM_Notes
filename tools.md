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