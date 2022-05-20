clear;clc;

mdl_ur5;

Td=20;tao=0.01;rou=0.6;
%   目标轨迹图像生成
    xd=zeros(1,Td/tao);zd=zeros(1,Td/tao);yd=zeros(1,Td/tao);
%   星型
    for n = 1:Td/tao
        xd(1,n)=0.7*cos(2*pi*n*tao/Td)^3;% 行矩阵
        yd(1,n)=0.7*sin(2*pi*n*tao/Td)^3;
        zd(1,n)=0.2*sin(2*pi*n*tao/Td)^3;
    end
%   五角星
%     for n = 1:Td/tao
%         xd(1,n)=0.2*rou*cos(6*pi*n*tao/Td)+0.5*rou*cos(4*pi*n*tao/Td)-0.7*rou+0.5;% 行矩阵
%         yd(1,n)=-0.50;
%         zd(1,n)=0.2*rou*sin(6*pi*n*tao/Td)-0.5*rou*sin(4*pi*n*tao/Td);
%     end
    pd=[xd;yd;zd];%一列一个坐标
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
 
%  初始化Win,W,Wout(t0),x(t0),thetai(t0),delta_thetai(t0)
    Win=rand(400,15)-0.5;% 范围【-0.5，0.5】 400x15
    W=rand(400,400);
    W=W*0.3/vrho(W);% 谱半径0.8  vrho()计算谱半径
    Wout=zeros(6,400);% Wout(t0)=0
    x=zeros(400,1);% x(t0)=0;
    tran=[1 0 0 0.75;0 1 0 0;0 0 1 0;0 0 0 1];% 生成初始deltai的变换矩阵
    thetai=ur5.ikine(tran);% 初始状态thetai
    thetai=thetai.';% 变为列矩阵
    delta_thetai=[0 0 0 0 0 0].';% 初始状态delta_thetai
    pa=ur5.fkine(thetai.');% 正运动学解
    pa=transl(pa);% 提取出位置信息
    pa=pa.';% 初始状态下的pa
%  LIN的参数
    h=2.2;l=0.6;
%  开始
for i=1:20  % 转20圈
    for n=1:Td/tao
        delta_p=pd(:,n)-pa;% new deltai_p,new pd,old pa
        u=[thetai;delta_p;delta_thetai];% new u,old thetai,new delta_p,ole delta_thetai   输入层
        % 没有LIN的存储层
        %x=sigmod(Win*u+W*x);% new x,new u,old x   
        % LIN的存储层
        x=(1-h*l)*x+h*sigmod(Win*u+W*x);
        p=x;% new p,new x    论文里给的不知道什么层，个人感觉没必要
        % max(max(abs(Wout*p)))
        delta_thetai=tanh(Wout*p);% new delta_thetai,old Wout,new p      得到 ESN 输出
        thetai=thetai+delta_thetai;% new thetai,old thetai,new delta_thatai   得到新的thetai
        if mod(n,10)==0 % 图像30步一标
            ur5.plot(thetai.');% new thetai     控制机器人
        end
        pa=ur5.fkine(thetai.');% 正运动学解    
        pa=transl(pa);% 提取出位置信息    
        pa=pa.';% new pa     得到实际位置
        if mod(n,10)==0
            plot3(pa(1,1),pa(2,1),pa(3,1),'r.');  % 标出实际位置
        end
        J=ur5.jacob0(thetai.','trans');    % 雅可比矩阵
        % 其中前3行代表机器人末端坐标系线速度的传递比；后3行代表对手爪的角速度的传递比
        % J=[J(1,:);J(2,:);J(3,:)];
        pinv_J=pinv(J);  % 伪逆 
        % 尝试解决奇异点问题

        % 找不到方法
        e=pinv_J*(pd(:,n)-pa);% new e,new pd,new pa   得到e
        delta_Wout=e*p.';   %  更新Wout
        Wout=Wout+0.001*delta_Wout;   %  注意系数，影响很大，而且和tao关系很大
    end
    clf;  %清空图像，画第二轮图像
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
end
% 有个非常严重的问题，一旦碰到奇异点或者偏差过大，基本难以完成任务