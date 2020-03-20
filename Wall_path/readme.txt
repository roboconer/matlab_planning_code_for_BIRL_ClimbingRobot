%本代码是用A*算法和落足点离散策略实现壁面环境中攀爬壁面序列的每个壁面上的无碰落足点路径规划（适用于双足爬壁机器人壁面路径规划），求出每一步运动的具体落足点位置

input_map 是输入环境信息，包括壁面边界和障碍物边界，需提前设置好，需要作为输入信息首先使用，用元胞数组表示每个壁面上的障碍物边界和高度信息

map 是障碍物分类和无碰区域生成（将吸附模块的吸附位置简化成一个点），不用直接使用

[ se_footstep3d,numst ] = [ se_footstep3d,step_op ] = surfaces_astar(x) 总程序，包括每一部分的程序运行，运行之后可得到求解结果和图形仿真结果，必须先有起点终点和全局路径
                                                                              优化求解得到的最优过渡落足点

foot_stepdemo 单一壁面无碰落足点规划示例，必须先输入地图和障碍物信息，随机生成起点和终点然后运行规划算法并显示结果

footstepsolution   引导路径求解和落足点搜索求解的整合

astar_path A*算法求解全局引导路径

ASTARPATH  A*算法实现

path_straight 对初步求解的全局引导路径进行拉直化处理，已包含在可视化的过程中，不用直接使用

path_des 将求得的结果显示在地图上并将其拉直化处理，使用示例：[ pathstr,path_dis] = path_des( pathxy1,ps,pe,map_border,obs_border,obs_height)

footstep_sequence( pathstr,map_border,obs_border,obs_height ) 落足点离散策略搜索，根据全局引导路径求解避障跨障或者在障碍物壁面上吸附的离散点，离散策略是优先贴近全局引导路径，一步一步寻找最优落足点，当前步无法找到可行解时，会返回
                  上一步寻找另一个次优可行解，再继续往下搜索，不直接使用，需要输入地图信息，障碍物信息和引导路径

robot_configurationw 根据壁面落足点求解机器人构型，一般不直接使用

DrawRobotfoot 画机器人每一步运动的构型,一般不直接使用

plot_obs 多壁面上画障碍物，一般不直接使用

linepolyintersection   求直线或线段与多边形交点

 ob_rotate  将障碍物旋转给定角度用


plot_environment(se_footstep,map_border,obs_border,obs_height,Vn) 单一壁面落足点规划用，显示立体化的障碍物和机器人，不直接使用
