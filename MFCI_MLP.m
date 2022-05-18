clear;clc;
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
    for n = 1:2000
        xd(n)=0.4*cos(2*pi*n/2000)^3;% 行矩阵
        yd(n)=0.4*sin(2*pi*n/2000)^3;
        zd(n)=0.4*sin(2*pi*n/2000)^3;
    end
    pd=[xd;yd;zd];
    %plot3(pd(1,:),pd(2,:),pd(3,:));
    %hold on;
%初始化
    pa=[1 0 0 0.5;0 1 0 0;0 0 1 0;0 0 0 1];
    thetai=Inverse_kinematics(pa);
    thetai=thetai.';% 初始状态thetai
    delta_thetai=[0 0 0 0 0 0].';% 初始状态delta_thetai
%   获取MLP训练需要的数据集
    Nt=5000;
    train_input=zeros(12,Nt);train_out=zeros(6,Nt);
    for i=1:Nt
        pa=kinematics(thetai.');
        pa=SE3(pa);
        pa=transl(pa);% 获得旧时刻EEF位置信息
        pa=pa.';
        old_thetai=thetai;% 记录旧时刻thetai
        incre_thetai=0.01*pi*2*(rand(6,1)-0.5);% 随机产生关节角度增量并调整
        thetai=thetai+incre_thetai;% 新thetai
        new_pa=kinematics(thetai.');
        new_pa=SE3(new_pa);
        new_pa=transl(new_pa);% 获得新时刻EEF位置信息
        new_pa=new_pa.';
        train_input(:,i)=[new_pa;old_thetai;pa];
        train_out(:,i)=incre_thetai;
        
    end
% 训练MLP网络
    net = fitnet(40);
    net = train(net,train_input,train_out); 
    view(net);
    save('net.mat.mat','net');
    
    x=net(train_input);
    y=train_out;
    x=x(1,:);
    y=y(1,:);
    plot(x,'g')
    hold on;
    plot(y,'r');

    
    
    
    
    
