function T0_6=kinematics(theta1_6)
    a2=0.4318;a3=0.02032;d2=0.14909;d4=0.43307;
	theta1=theta1_6(1);
	theta2=theta1_6(2);
	theta3=theta1_6(3);
	theta4=theta1_6(4);
	theta5=theta1_6(5);
	theta6=theta1_6(6);
	c1=cos(theta1);s1=sin(theta1);
	c2=cos(theta2);s2=sin(theta2);
	c3=cos(theta3);s3=sin(theta3);
	c4=cos(theta4);s4=sin(theta4);
	c5=cos(theta5);s5=sin(theta5);
	c6=cos(theta6);s6=sin(theta6);
	%6个连杆变换矩阵
    T1=[c1 -s1 0 0;s1 c1 0 0;0 0 1 0;0 0 0 1];
    T2=[c2 -s2 0 0;0 0 1 d2;-s2 -c2 0 0;0 0 0 1];
    T3=[c3 -s3 0 a2;s3 c3 0 0 ;0 0 1 0;0 0 0 1];
    T4=[c4 -s4 0 a3;0 0 1 d4;-s4 -c4 0 0;0 0 0 1];
    T5=[c5 -s5 0 0 ;0 0 -1 0;s5 c5 0 0;0 0 0 1];
    T6=[c6 -s6 0 0 ;0 0 1 0;-s6 -c6 0 0;0 0 0 1];
    %正运动学方程
    T0_6 = T1*T2*T3*T4*T5*T6;
end
