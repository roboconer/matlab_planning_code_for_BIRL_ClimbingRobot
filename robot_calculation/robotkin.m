clear;
clc;
 L1 = Link('d', 0.675, 'a', 0.26, 'alpha', pi/2);%定义连杆
 L2 = Link('d', 0, 'a', 0.645, 'alpha', 0);
 L3 = Link('d', 0, 'a', 0.035, 'alpha', pi/2);
 L4 = Link('d', 0.67, 'a', 0, 'alpha', pi/2);
 L5 = Link('d', 0, 'a', 0, 'alpha', pi/2);
 L6 = Link('d', 0.115, 'a', 0, 'alpha', 0);
 bot = SerialLink([L1 L2 L3 L4 L5 L6]);%连接连杆
 bot.display();%显示D-H参数表
 %forward_kinematics=bot.fkine([pi/4 pi/2 pi/8 pi/2 7*pi/6 pi/2])%前向运动学
 
  forward_kinematics=bot.fkine([ 0 pi/2 0 0 pi 0])
 %bot.plot([pi/4 pi/2 pi/8 pi/2 7*pi/6 pi/2])
 bot.plot([0 pi/2 0 0 pi 0])