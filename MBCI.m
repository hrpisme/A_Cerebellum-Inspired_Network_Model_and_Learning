    clear;clc;
%    6R������ģ�ͣ�modified puma560��
%    ��������� a--���˳��ȣ�d--����ƫ����
    a2=0.4318;a3=0.02032;d2=0.14909;d4=0.43307;
%		     thetai    di      ai-1        alphai-1
    L1 = Link([pi/2    0       0               0], 'modified');
    L2 = Link([0       d2      0           -pi/2], 'modified');
    L3 = Link([-pi/2   0       a2              0], 'modified');
    L4 = Link([0       d4      a3          -pi/2], 'modified');
    L5 = Link([0       0  	   0            pi/2], 'modified');
    L6 = Link([0       0       0           -pi/2], 'modified');
    robot = SerialLink([L1,L2,L3,L4,L5,L6]);
    robot.name = 'modified puma560';
%   robot.display();
%	robot.teach(); %  ����
   
    Td=20;tao=0.1;
%   Ŀ��켣ͼ������
    xd=zeros(1,Td/tao);zd=zeros(1,Td/tao);yd=zeros(1,Td/tao);
    for n = 1:Td/tao
        xd(1,n)=0.4*cos(2*pi*n*tao/Td)^3;% �о���
        yd(1,n)=0.4*sin(2*pi*n*tao/Td)^3;
        zd(1,n)=0.4*sin(2*pi*n*tao/Td)^3;
    end
    pd=[xd;yd;zd];%һ��һ������
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
 
%  ��ʼ��Win,W,Wout(t0),x(t0),thetai(t0),delta_thetai(t0)
    Win=rand(400,15)-0.5;% ��Χ��-0.5��0.5�� 400x15
    W=rand(400,400);
    W=W*0.8/vrho(W);% �װ뾶0.8  vrho()�����װ뾶
    Wout=zeros(6,400);% Wout(t0)=0
    x=zeros(400,1);% x(t0)=0;
    tran=[1 0 0 0.5;0 1 0 0;0 0 1 0;0 0 0 1];% ���ɳ�ʼdeltai�ı任����
    thetai=Inverse_kinematics(tran);% ��ʼ״̬thetai
    thetai=thetai.';% ��Ϊ�о���
    delta_thetai=[0 0 0 0 0 0].';% ��ʼ״̬delta_thetai
    pa=kinematics(thetai.');% ���˶�ѧ��
    pa=SE3(pa);
    pa=transl(pa);% ��ȡ��λ����Ϣ
    pa=pa.';% ��ʼ״̬�µ�pa
%  һЩ��������
for i=1:20  % ת20Ȧ
    for n=1:Td/tao
        delta_p=pd(:,n)-pa;% new deltai_p,new pd,old pa
        u=[thetai;delta_p;delta_thetai];% new u,old thetai,new delta_p,ole delta_thetai   �����
        x=sigmod(Win*u+W*x);% new x,new u,old x    �洢��
        p=x;% new p,new x    ��������Ĳ�֪��ʲô�㣬���˸о�û��Ҫ
        delta_thetai=tanh(Wout*p);% new delta_thetai,old Wout,new p      �õ� ESN ���
        thetai=thetai+delta_thetai;% new thetai,old thetai,new delta_thatai   �õ��µ�thetai
        robot.plot(thetai.');% new thetai     ���ƻ�����
        pa=kinematics(thetai.');% ���˶�ѧ��    
        pa=SE3(pa);
        pa=transl(pa);% ��ȡ��λ����Ϣ    
        pa=pa.';% new pa     �õ�ʵ��λ��
        plot3(pa(1,1),pa(2,1),pa(3,1),'r.');  % ���ʵ��λ��
        J=robot.jacob0(thetai.','trans');    % �ſɱȾ���
        % ����ǰ3�д��������ĩ������ϵ���ٶȵĴ��ݱȣ���3�д������צ�Ľ��ٶȵĴ��ݱ�
        % J=[J(1,:);J(2,:);J(3,:)];
        pinv_J=pinv(J);  % α��
        e=pinv_J*(pd(:,n)-pa);% new e,new pd,new pa   �õ�e
        delta_Wout=e*p.';   %  ����Wout
        Wout=Wout+0.01*delta_Wout;   %  ע��ϵ����Ӱ��ܴ�
    end
    clf;  %���ͼ�񣬻��ڶ���ͼ��
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
end