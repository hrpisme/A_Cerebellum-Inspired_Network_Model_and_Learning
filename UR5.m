clear;clc;

mdl_ur5;

Td=20;tao=0.01;rou=0.6;
%   Ŀ��켣ͼ������
    xd=zeros(1,Td/tao);zd=zeros(1,Td/tao);yd=zeros(1,Td/tao);
%   ����
    for n = 1:Td/tao
        xd(1,n)=0.7*cos(2*pi*n*tao/Td)^3;% �о���
        yd(1,n)=0.7*sin(2*pi*n*tao/Td)^3;
        zd(1,n)=0.2*sin(2*pi*n*tao/Td)^3;
    end
%   �����
%     for n = 1:Td/tao
%         xd(1,n)=0.2*rou*cos(6*pi*n*tao/Td)+0.5*rou*cos(4*pi*n*tao/Td)-0.7*rou+0.5;% �о���
%         yd(1,n)=-0.50;
%         zd(1,n)=0.2*rou*sin(6*pi*n*tao/Td)-0.5*rou*sin(4*pi*n*tao/Td);
%     end
    pd=[xd;yd;zd];%һ��һ������
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
 
%  ��ʼ��Win,W,Wout(t0),x(t0),thetai(t0),delta_thetai(t0)
    Win=rand(400,15)-0.5;% ��Χ��-0.5��0.5�� 400x15
    W=rand(400,400);
    W=W*0.3/vrho(W);% �װ뾶0.8  vrho()�����װ뾶
    Wout=zeros(6,400);% Wout(t0)=0
    x=zeros(400,1);% x(t0)=0;
    tran=[1 0 0 0.75;0 1 0 0;0 0 1 0;0 0 0 1];% ���ɳ�ʼdeltai�ı任����
    thetai=ur5.ikine(tran);% ��ʼ״̬thetai
    thetai=thetai.';% ��Ϊ�о���
    delta_thetai=[0 0 0 0 0 0].';% ��ʼ״̬delta_thetai
    pa=ur5.fkine(thetai.');% ���˶�ѧ��
    pa=transl(pa);% ��ȡ��λ����Ϣ
    pa=pa.';% ��ʼ״̬�µ�pa
%  LIN�Ĳ���
    h=2.2;l=0.6;
%  ��ʼ
for i=1:20  % ת20Ȧ
    for n=1:Td/tao
        delta_p=pd(:,n)-pa;% new deltai_p,new pd,old pa
        u=[thetai;delta_p;delta_thetai];% new u,old thetai,new delta_p,ole delta_thetai   �����
        % û��LIN�Ĵ洢��
        %x=sigmod(Win*u+W*x);% new x,new u,old x   
        % LIN�Ĵ洢��
        x=(1-h*l)*x+h*sigmod(Win*u+W*x);
        p=x;% new p,new x    ��������Ĳ�֪��ʲô�㣬���˸о�û��Ҫ
        % max(max(abs(Wout*p)))
        delta_thetai=tanh(Wout*p);% new delta_thetai,old Wout,new p      �õ� ESN ���
        thetai=thetai+delta_thetai;% new thetai,old thetai,new delta_thatai   �õ��µ�thetai
        if mod(n,10)==0 % ͼ��30��һ��
            ur5.plot(thetai.');% new thetai     ���ƻ�����
        end
        pa=ur5.fkine(thetai.');% ���˶�ѧ��    
        pa=transl(pa);% ��ȡ��λ����Ϣ    
        pa=pa.';% new pa     �õ�ʵ��λ��
        if mod(n,10)==0
            plot3(pa(1,1),pa(2,1),pa(3,1),'r.');  % ���ʵ��λ��
        end
        J=ur5.jacob0(thetai.','trans');    % �ſɱȾ���
        % ����ǰ3�д��������ĩ������ϵ���ٶȵĴ��ݱȣ���3�д������צ�Ľ��ٶȵĴ��ݱ�
        % J=[J(1,:);J(2,:);J(3,:)];
        pinv_J=pinv(J);  % α�� 
        % ���Խ�����������

        % �Ҳ�������
        e=pinv_J*(pd(:,n)-pa);% new e,new pd,new pa   �õ�e
        delta_Wout=e*p.';   %  ����Wout
        Wout=Wout+0.001*delta_Wout;   %  ע��ϵ����Ӱ��ܴ󣬶��Һ�tao��ϵ�ܴ�
    end
    clf;  %���ͼ�񣬻��ڶ���ͼ��
    plot3(pd(1,:),pd(2,:),pd(3,:));
    hold on;
end
% �и��ǳ����ص����⣬һ��������������ƫ����󣬻��������������