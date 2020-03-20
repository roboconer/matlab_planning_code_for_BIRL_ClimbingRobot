%本代码可求解双足爬壁机器人单步运动情况，代码将单步运动规划分为壁面单步运动和过渡单步运动规划两种情况分别处理，输入单步运动的起始状态和目标状态的相关参数，输出机器人每个关节的关节角变化情况

Bspline(P)，Bspline2(P)，Bspline3(P)  B样条曲线生成函数，输入为样条曲线的控制点或者节点（当前单步规划算法没有用上）

basemotion( stp,enp,bap ) 根据基点，起始点和目标点生成机器人单步运动的基本路径（基于B样条曲线的方法且无障碍物），当前规划方法没采用

input_map2 输入障碍物数据，障碍物形状为矩形，形状大小数据和高度数据分开储存，障碍物的分布式相对于壁面收缩边界后的第一个点为坐标原点的，坐标系的建立可见代码，需要根据障碍物数据将障碍物转换为空间障碍数据
           如果能直接获取空间障碍物数据，就可直接在规划中使用，不需要建立相对坐标系   

map_obs( map_border,obs_border,obs_height ) 将障碍物数据转换成空间八个顶点构成的空间障碍数据

obp_tran(obp)  将障碍物的八个空间点转换成5个空间平面并绘图表示，还有一个平面与壁面重合不作考虑

enlarge_border(obs)   将障碍物的边界扩大

cir_seem_poly( O,vec,r,num_bor ) 把圆转换成等效多边形，输出多边形的顶点，可改变边的数目

colli_avoidance( M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option） 双足爬壁机器人壁面单步运动规划求解，输入壁面数据，障碍物数据，基点及姿态，起始点，目标点和起始关节角，逆解求解参数
                                                                         输出机器人从起始状态到目标状态的关节角变化情况

colli_avoidance2( M_sur,stp,enp,bap,obs_p,bas_R,stp_R,fir_joi,so_option） 双足爬壁机器人壁面间单步过渡运动规划求解，输入壁面数据，障碍物数据，基点及姿态，起始点，目标点和起始关节角，逆解求解参数
                                                                         输出机器人从起始状态到目标状态的关节角变化情况

dis_compute(testobj,obsp,option) 计算两个几何体之间的最小距离，相交则距离为0，改变参数option可计算不同类型几何体间的最小距离

Seg2SegDist(Seg1,Seg2) 求两线段间的最小距离，返回距离及相应的点对

insidepoly3(point,poly)   判断点是否在空间多边形内部

point_pro_sur(point,poly) 求点在平面上的投影

point_to_line(poi,line)  求点到线段的最小距离，返回距离及对应的点

draw_ro 画机器人例程

DrawRobotmo(robot,transparency,mode) 机器人模型建立和绘制（一般不用直接使用）

joint_angle(joint_ang)        根据规划的关节角画机器人（一般不用）

IKine5D22(robot)，IKine5DNew(robot)  机器人逆运动学求解，两个文件求得的逆解不同，应对机器人单步运动的关节角变化的不同情况（不同步态）

Kine5D( joint_ang ,option) 机器人正运动学求解，可求机器人各个关节点的位姿

input_step 输入落足点数据，该输入通过壁面落足点求解获得

motionplan2(motionp)   多个连续单步运动规划求解，单步规划算法的整合统一，输入连续单步运动的每一个落足点，输出每一步运动的关节角变化，其中需要用参数确定每一步运动是壁面单步运动还是过渡单步运动，
                        再用相应的规划方法求解

jointangle_transform( JJ ) 将关节角数据转换成攀爬机器人离线运动代码,输入求解得到的关节角数据

motion_simulation(Ba,Ba_R,Ang,filename)  将路径规划结果做仿真，显示运动规划结果，并且生成动画，需要的每一个输入都可以通过motionplan2运行得到，filename表示录制视频的文件名，如'123'

