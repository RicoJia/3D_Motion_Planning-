# Motion Planning For Quadrotor Simulation 

## Introduction
This is the final project of the [Deep Blue Academy Class - Motion Planning for Mobile Robots](www.shenlanxueyuan.com). The project includes the following objectives: 
1. Path planning 
2. Trajectory generation
3. Trajectory replanning 
4. Trajectory Replanning for sensors' limited field of view

This package includes: 
- random complex: generate 3D random obstacles in point cloud 
- waypoint_generator: 
- odom visualization: visualize the quadrotor
- pcl_render_node: Simple model for camera/Lidar with a limited field of view. This module outputs point clouds of obstacles locally
- trajectory_generator_node: generate a **smooth, executable** trajectory
- traj_server: converts trajectory to control commands 
- so3_control: converts control commands to actual control variables. 
- quadrotor_simulator_so3: model of a quadrotor. 

## Installations & Dependencies
1. Dependencies
    ```bash
    sudo apt-get install cmake libopenblas-dev liblapack-dev libarpack-dev libarpack2-dev libsuperlu-dev
    ```
2. Install Armadillo
    ```
    xz -d armadillo-9.870.2.tar.xz
    tar -xvf armadillo-9.870.2.tar
    cd armadillo-9.870.2
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    ```

## Notes
1. Flowchart 
2. Demo 
3. Performance Comparison
    - ```A*``` vs ```simplified path```
    - ```trajectory reoptimization``` 
    - Other

## TODO
1. 阅读代码：画出trajectory_generator_node运行流程图，重点是厘清
   1. 几个状态之间的切换过程；
   2. 各个主要功能之间的调用关系，不需要深入到各个功能的内部例如A*的流程。
2. path planning：推荐实现方案为A*，也可采用其他方案；
3. simplify the path：将整条path简化为少数几个关键waypoints，推荐方案为RDP算法；
4. trajectory optimization：推荐实现方案为minimum snap trajectory generation，也可采用其他方案；
5. safe checking: 验证生成的轨迹是否安全；
6. trajectory reoptimization：此环节只针对使用minimum snap trajectory generation的时候。由于该方法只对连续性进行优化，并不能保证优化后的轨迹不会撞上障碍物，所以需要对撞上障碍物的部分重优化。推荐方法详见文献：["Polynomial Trajectory Planning for Aggressive Quadrotor Flight in Dense Indoor Environments" part 3.5](https://dspace.mit.edu/bitstream/handle/1721.1/106840/Roy_Polynomial%20trajectory.pdf?sequence=1&isAllowed=y)。

伪代码（来源：[维基百科](https://en.wikipedia.org/wiki/Ramer–Douglas–Peucker_algorithm)）：

```
function DouglasPeucker(PointList[], epsilon)
    // Find the point with the maximum distance
    dmax = 0
    index = 0
    end = length(PointList)
    for i = 2 to (end - 1) {
        d = perpendicularDistance(PointList[i], Line(PointList[1], PointList[end]))
        if (d > dmax) {
            index = i
            dmax = d
        }
    }

    ResultList[] = empty;

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilon) {
        // Recursive call
        recResults1[] = DouglasPeucker(PointList[1...index], epsilon)
        recResults2[] = DouglasPeucker(PointList[index...end], epsilon)

        // Build the result list
        ResultList[] = {recResults1[1...length(recResults1) - 1], recResults2[1...length(recResults2)]}
    } else {
        ResultList[] = {PointList[1], PointList[end]}
    }
    // Return the result
    return ResultList[]
end
```

