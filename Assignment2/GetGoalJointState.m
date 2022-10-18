goalJointState = blasterRobot.getpos() + (rand(1,6)-0.5) * 20*pi/180;
endEffectorTr = blasterRobot.fkine(goalJointState);