clear;clc;
%   6R������ģ�ͣ�����modified puma560��
%   ��������� a--���˳��ȣ�d--����ƫ����
    a2=0.4318;a3=0.02032;d2=0.14909;d4=0.43307;
%		     thetai    di      ai-1        alphai-1               �ؽڱ���ȡֵ��Χ���Ƕ���    
    L1 = Link([pi/2    0       0               0], 'modified');%-160��160
    L2 = Link([0       d2      0           -pi/2], 'modified');%-225��45
    L3 = Link([-pi/2   0       a2              0], 'modified');%-45��225
    L4 = Link([0       d4      a3          -pi/2], 'modified');%-110��170
    L5 = Link([0       0  	   0            pi/2], 'modified');%-100��100
    L6 = Link([0       0       0           -pi/2], 'modified');%-200��200
    robot = SerialLink([L1,L2,L3,L4,L5,L6]);
    robot.name = 'modified puma560';
%   robot.display();
%	robot.teach();
%   robot.plot([0 0 0 0 0 0]);
%   ͼ������
    xd=zeros(1,2000);yd=zeros(1,2000);zd=zeros(1,2000);
    for n = 1:2000
        xd(1,n)=0.1*cos(2*pi*n/2000)^3+0.6;% �о���
        yd(1,n)=0.1*sin(2*pi*n/2000)^3;
        zd(1,n)=0.1*sin(2*pi*n/2000)^3+0.2;
    end
    pd=[xd;yd;zd];
    %plot3(pd(1,:),pd(2,:),pd(3,:));
    %hold on;
%��ʼ��
    pa=[1 0 0 0.65 ;0 1 0 0.0;0 0 1 0.15;0 0 0 1];
    thetai=Inverse_kinematics(pa);
    thetai=thetai.';% ��ʼ״̬thetai
    thetai=[1;1;1;0;0;0].*thetai;
    delta_thetai=[0 0 0 0 0 0].';% ��ʼ״̬delta_thetai
%   ��ȡMLPѵ����Ҫ�����ݼ�
    Nt=40000;
    train_input=zeros(12,Nt);train_out=zeros(6,Nt);
    for i=1:Nt
%         if mod(i,1000)==0
%             pa=[1 0 0 pd(1,2000/16*i/1000);0 1 0 pd(2,2000/16*i/1000);0 0 1 pd(3,2000/16*i/1000);0 0 0 1];
%             thetai=Inverse_kinematics(pa);
%             thetai=thetai.';% ��ʼ״̬thetai
%             delta_thetai=[0 0 0 0 0 0].';
%         end
        pa=kinematics(thetai.');
        pa=SE3(pa);
        pa=transl(pa);% ��þ�ʱ��EEFλ����Ϣ
        pa=pa.';
        old_thetai=thetai;% ��¼��ʱ��thetai
        old_thetai=[1;1;1;0;0;0].*old_thetai;
        incre_thetai=0.01*pi*2*(rand(6,1)-0.5);% ��������ؽڽǶ�����������
        incre_thetai=[1;1;1;0;0;0].*incre_thetai;
        thetai=thetai+incre_thetai;% ��thetai
        thetai=[1;1;1;0;0;0].*thetai;
%         while max(max(abs(pinv(robot.jacob0(thetai.')))))>30
%             incre_thetai=0.01*pi*2*(rand(6,1)-0.5);% ������������ؽڽǶ�����������
%             thetai=thetai+incre_thetai;
%         end
        new_pa=kinematics(thetai.');
        new_pa=SE3(new_pa);
        new_pa=transl(new_pa);% �����ʱ��EEFλ����Ϣ
        new_pa=new_pa.';
        train_input(:,i)=[new_pa;old_thetai;pa];
        train_out(:,i)=incre_thetai;
        
    end
% ѵ��MLP����
    net = fitnet(40);
    net = train(net,train_input,train_out); 
    view(net);
    save('net.mat.mat','net');
    save('train.mat.mat','train_input','train_out');

    
    
    
    
    