# BIRLClimbingRobot
 Matlab code of BIRL ClimbingRobot

## 代码使用

### 1、本代码所适用的机器人
    ![avatar](biped5d_picture.png)
### 2、输入壁面数据
    - 进入/Global_path/surfaces.m
    - `border_vertexs`为壁面的n个顶点，这里注意一下坐标系的统一。由于我们实验采集的数据坐标系为[x, z, y]，所以这里为满足可视化，对坐标系进行了x轴旋转90度
### 3、输入障碍物
    - 进入/Singlemotion_path/input_map2.m
    - `obstruct_border` 为障碍物在壁面上的相对坐标[如代码中的`obstruct_border{2}=[-0 0.7; 0.0 0.7; 0.0 1.2; -0 1.2];`表示障碍物在第二个壁面上的4个点]
    - `obstruct_height`为障碍物在壁面的法向高度
### 4、按以下前后排序设置matlab代码路径
    matlab_planning_code_for_BIRL_ClimbingRobot/
    matlab_planning_code_for_BIRL_ClimbingRobot/Wall_path
    matlab_planning_code_for_BIRL_ClimbingRobot/PSO_path
    matlab_planning_code_for_BIRL_ClimbingRobot/APSO_global
    matlab_planning_code_for_BIRL_ClimbingRobot/Astar
    matlab_planning_code_for_BIRL_ClimbingRobot/drawing
    matlab_planning_code_for_BIRL_ClimbingRobot/Global_path
    matlab_planning_code_for_BIRL_ClimbingRobot/Singlemotion_path

### 5、进入main.m ,运行