%本代码是用粒子群算法实现无碰全局路径（适用于轮式移动机器人，对于双足爬壁机器人只能作为全局的引导路径），并且求出每一步运动的具体附着点位置

input_map 是输入环境信息，包括壁面边界和障碍物边界，需提前设置好，不用直接使用

map 是障碍物分类和无碰区域生成（将吸附模块的吸附位置简化成一个点），不用直接使用

ASPO_path 是主程序运行，运行之后可得到初步结果

path_des 将求得的结果显示在地图上并将其拉直化处理，使用示例：[ pathstr,path_dis] = path_des( interval,particle_size,gbest,TR,ps,pe)

path_straight 粒子群求解的路径进行拉直化处理，已包含在可视化的过程中，不用直接使用

footstep_sequence 在全局引导路径上运行步态搜索策略，得到最终每步的具体落足点，并且显示机器人执行每个步态的构型，使用示例：[ se_footstep,state_judge ] = footstep_sequence( pathstr )

plot_environment() 显示立体化的障碍物，最后使用
