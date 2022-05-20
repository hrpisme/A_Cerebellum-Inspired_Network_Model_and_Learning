clear;clc;
load('net1.mat.mat');
load('net1.mat.mat');
Td=20;tao=0.01;rou=0.5;h=2.2;l=0.6;

%   6R机器人模型，名称modified puma560。
%   定义机器人 a--连杆长度，d--连杆偏移量
    a2=0.4318;a3=0.02032;d2=0.14909;d4=0.43307;
%		     thetai    di      ai-1        alphai-1               关节变量取值范围，角度制    
    L1 = Link([pi/2    0       0               0], 'modified');%-160，160
    L2 = Link([0       d2      0           -pi/2], 'modified');%-225，45
    L3 = Link([-pi/2   0       a2              0], 'modified');%-45，225
    L4 = Link([0       d4      a3          -pi/2], 'modified');%-110，170
    L5 = Link([0       0  	   0            pi/2], 'modified');%-100，100
    L6 = Link([0       0       0           -pi/2], 'modified');%-200，200
    robot = SerialLink([L1,L2,L3,L4,L5,L6]);
    robot.name = 'modified puma560';
%   robot.display();
%	robot.teach();
%   robot.plot([0 0 0 0 0 0]);
%   图像生成
    for n = 1:Td/tao
        xd(1,n)=0.4*cos(2*pi*n*tao/Td)^3;% 行矩阵
        yd(1,n)=0.4*sin(2*pi*n*tao/Td)^3;
        zd(1,n)=0.4*sin(2*pi*n*tao/Td)^3;
    end
%     for n = 1:Td/tao
%         xd(1,n)=0.2*rou*cos(6*pi*n*tao/Td)+0.5*rou*cos(4*pi*n*tao/Td)-0.7*rou+0.5;% 行矩阵
%         yd(1,n)=-0.50;
%         zd(1,n)=0.2*rou*sin(6*pi*n*tao/Td)-0.5*rou*sin(4*pi*n*tao/Td);
%     end
    pd=[xd;yd;zd];
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
    %初始化参数
    Win=rand(400,15)-0.5;% 范围【-0.5，0.5】 400x15
    W=rand(400,400);
    W=W*0.3/vrho(W);% 谱半径0.8  vrho()计算谱半径
    Wout=zeros(3,400);% Wout(t0)=0
    x=zeros(400,1);% x(t0)=0;
    %pa=[1 0 0 0.5;0 1 0 -0.5;0 0 1 0;0 0 0 1];
    pa=[1 0 0 0.4;0 1 0 0;0 0 1 0;0 0 0 1];
    thetai=Inverse_kinematics(pa);
    thetai=thetai.';% 初始状态thetai
    delta_thetai=[0 0 0 0 0 0].';% 初始状态delta_thetai
    pa=kinematics(thetai.');% 正运动学解
    pa=SE3(pa);
    pa=transl(pa);% 提取出位置信息
    pa=pa.';% 初始状态下的pa
for i=1:20
    for n=1:Td/tao
        if mod(n,10)==0
            robot.plot(thetai.');
        end
        if mod(n,10)==0
            plot3(pa(1,1),pa(2,1),pa(3,1),'r.');
        end
        u=[pd(:,n);thetai;delta_thetai];% new u,new pd,old tehtai,old delta_thetai
        %x=sigmod(Win*u+W*x);% new x,new u, old x
        x=(1-h*l)*x+h*sigmod(Win*u+W*x);% LIN
        p=x;% new p,new x
        %max(max(abs(Wout*p)))
        y=tanh(Wout*p);% new y,old Wout,new p
        pd_adjust=pd(:,n)+y;% all new
        old_thetai=thetai;% 记录old thetai
        incre_thetai=net([pd_adjust;old_thetai;pa]);
        thetai=thetai+incre_thetai;% new thetai,new pd_adjust,old thetai,old pa
        delta_thetai=thetai-old_thetai;% new delta_thetai,new thetai,ole thetai
        pa=kinematics(thetai.');% new pa,new thetai
        pa=SE3(pa);
        pa=transl(pa);% 提取出位置信息
        pa=pa.';% new pa
        e=pd(:,n)-pa;% new e,new pd,new pa
        delta_Wout=e*p.';% new deltai_Wout, new e,new p
        Wout=Wout+0.001*delta_Wout;% new Wout,old Wout,new delta_Wout
    end
    clf;
    pa_adjust=[1 0 0 pa(1,1);0 1 0 pa(2,1);0 0 1 pa(3,1);0 0 0 1];%%%% 就离谱
    thetai=Inverse_kinematics(pa_adjust);
    thetai=thetai.';
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
end