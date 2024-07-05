clc
clear

% 设置仿真时间和步长
T=0.001; deltaT=0.001; stoptime=50; k=1;

I_x=2.0*10^(-3); I_y=5.5*10^(-3); I_z=5.4*10^(-3);
J=[I_x,0,0;0,I_y,0;0,0,I_z];
m=0.35;g=9.8; e3=[0;0;1]; 
mu=0.05; l=0.16; l3=0.25; 

% 数据初始化 分配预留空间
% 只关注姿态的变化情况，因此不需要对外环变量进行设置，但是期望的初始姿态需要设置
R_eta=zeros(3,3); dR_eta=zeros(3,3);

S_omega=zeros(3,3); A=zeros(3,3); 
M=zeros(3,3); B=zeros(3,3); rho_eta=zeros(3,1);

f=zeros(3,1); tau=zeros(3,1); % tau=A*f  f为控制输入，A为控制分配矩阵

% 偏航角及导数
phi=zeros; theta=zeros; psi=zeros;
dphi=zeros; dtheta=zeros; dpsi=zeros;
ddphi=zeros; ddtheta=zeros; ddpsi=zeros;

phi_d=zeros; theta_d=zeros; psi_d=zeros;
dphi_d=zeros; dtheta_d=zeros; dpsi_d=zeros;
ddphi_d=zeros; ddtheta_d=zeros; ddpsi_d=zeros;

% 误差及滑模面
e_eta=zeros(3,1); de_eta=zeros(3,1); s_eta=zeros(3,1);
cos_delta=zeros; sin_delta=zeros; eta_d=zeros(3,1);

e_phi=zeros; de_phi=zeros; dde_phi=zeros; 
s_phi=zeros; ds_phi=zeros; 
V_phi=zeros; dV_phi=zeros; f_phi=zeros;

e_theta=zeros; de_theta=zeros; dde_theta=zeros; 
s_theta=zeros; ds_theta=zeros; r_theta=zeros;
V_theta=zeros; dV_theta=zeros; f_theta=zeros;

e_psi=zeros; de_psi=zeros; dde_psi=zeros;
s_psi=zeros; ds_psi=zeros;
V_psi=zeros; dV_psi=zeros; f_psi=zeros;

tau_phi=zeros; tau_theta=zeros; tau_psi=zeros;

omega=zeros(3,1); domega=zeros(3,1);
omega_phi=zeros; omega_theta=zeros; omega_psi=zeros;

eta=zeros(3,1); deta=zeros(3,1); ddeta=zeros(3,1); ddeta_d=zeros(3,1);

V=zeros(3,1); dV=zeros(3,1); s=zeros(3,1); ds=zeros(3,1); delta=zeros;
A_psi=zeros(1,3);

% 控制器参数(待定）,这里都是固定的量，可以直接定义数值
% 期望均为0 lambda 0.4 0.3 0.2
c1=1.07; c2=1.07; c3=1.45; 
beta1=0.0001; beta2=0.0001; beta3=0.0001;
K11=0.2; K12=0.3; K13=0.3; 
K21=5; K22=6; K23=5;

% lambda为小数值 0.2 0.2 0.2
% c1=3; c2=3; c3=4; 
% beta1=0.3; beta2=0.9; beta3=0.5;
% K11=0.5; K12=3; K13=8; 
% K21=8; K22=8; K23=8;

% 期望数值不为0 兼容小数值
% c1=20; c2=20; c3=28; 
% beta1=1; beta2=1; beta3=15;
% K11=1; K12=1; K13=20; 
% K21=1; K22=1; K23=22;

c=[c1,0,0;0,c2,0;0,0,c3];

% 外部扰动 
rho=zeros(3,1);

% 旋翼部分失效 部分数值过小可能无法跑出正常结果 猜测是因为Lambda当有一个值小于所设置的值时，系统的参数无法向下兼容
% Lambda1=rand; Lambda2=rand; Lambda3=rand;
Lambda1=0.1; Lambda2=0.1; Lambda3=0.1;
% Lambda1=0.2; Lambda2=0.2; Lambda3=0.2;
Lambda=[Lambda1,0,0;0,Lambda2,0;0,0,Lambda3];

% 程序正文
for t=0:T:stoptime
    
    % 初始值定义
    if k==1
        % 姿态部分
        eta(:,k)=[pi/6;-pi/4;pi/2]; deta(:,k)=[1.2;-1.4;1.6]; ddeta(:,k)=[0;0;0];
        % tau_phi(k)=1; tau_theta(k)=1; tau_psi(k)=1;
        % f(:,k)=[0.1;0.1;0.1]; tau(:,k)=[0.1;0.1;0.1];
        % ddphi_d(k)=0.1; ddpsi_d(k)=0.1; ddtheta_d(k)=0.1;
        omega(:,k)=[4.19;1.13;2.8];
        delta(k)=0; 
        de_phi(k)=1.2; de_theta(k)=-1.4; de_psi(k)=1.6;
        s(:,k)=eta(:,k)+c*deta(:,k);
    end

    % 在姿态η中提取俯仰、滚转、偏航的值，并求导
    phi(k)=eta(1,k); theta(k)=eta(2,k); psi(k)=eta(3,k);
    dphi(k)=deta(1,k); dtheta(k)=deta(2,k); dpsi(k)=deta(3,k);
    omega_phi(k)=omega(1,k); omega_theta(k)=omega(2,k); omega_psi(k)=omega(3,k);
    s_phi(k)=s(1,k); s_theta(k)=s(2,k); s_psi(k)=s(3,k);
    % ddphi(k)=ddeta(1,k); ddtheta(k)=ddeta(2,k); ddpsi(k)=ddeta(3,k);
    %期望的数值
    phi_d(k)=0; theta_d(k)=0; psi_d(k)=0;
    dphi_d(k)=0; dtheta_d(k)=0; dpsi_d(k)=0;
    % phi_d(k)=0.5*sin(0.3*t); theta_d(k)=-0.3*cos(0.2*t); psi_d(k)=0.3*sin(0.2*t);
    % dphi_d(k)=-0.3*sin(0.3*t); dtheta_d(k)=0.4*sin(0.2*t); dpsi_d(k)=-0.2*sin(0.2*t);
    eta_d(:,k)=[phi_d(k);theta_d(k);psi_d(k)];
    % deta_d(:,k)=[dphi_d(k);dtheta_d(k);dpsi_d(k)];
    ddeta_d(:,k)=[0;0;0];
    
    % 变换矩阵
    R_eta(:,:,k)=[1,        0,       -sin(theta(k)) ;
                  0, cos(phi(k)),  cos(theta(k))*sin(phi(k)) ;   
                  0, -sin(phi(k)), cos(theta(k))*cos(phi(k))];
    
     % 反对称矩阵
    S_omega(:,:,k)=[       0,       -omega_psi(k),  omega_theta(k);
                      omega_psi(k),        0,        -omega_phi(k);
                    -omega_theta(k), omega_phi(k),        0       ];
       
    % 定义扰动和舵机失效时间
    if (k<=20000)
        rho(:,k+1)=rho(:,k);
    else
        rho(:,k)=rho(:,20000)+[0.5*sin(0.3*t);0.3*cos(0.2*t);0.2*sin(0.3*t)]; % 添加设置的扰动量
    end
   
    
    % 定义各个通道的误差及滑模面相关变量
    if (k<=stoptime/deltaT)&&(k>1)

        e_phi(k)=phi(k)-phi_d(k);
        de_phi(k)=dphi(k)-dphi_d(k);

        e_theta(k)=theta(k)-theta_d(k);
        de_theta(k)=dtheta(k)-dtheta_d(k);

        e_psi(k)=psi(k)-psi_d(k);
        de_psi(k)=dpsi(k)-dpsi_d(k);
    end

    % 姿态动力学模型计算 姿态在出现扰动时变化太大
    if k<=stoptime/deltaT
        % 控制输入 单通道计算没有问题
        dV_phi(k)=-K21*(0.5*sign(s_phi(k))+1.5*beta1*(abs(s_phi(k))^0.5)*sign(s_phi(k))+beta1^2*s_phi(k));
        V_phi(k+1)=V_phi(k)+deltaT*dV_phi(k);
        tau_phi(k+1)=(-(K11*((abs(s_phi(k)))^0.5*sign(s_phi(k))+beta1*s_phi(k)))+V_phi(k));
        
        dV_theta(k)=-K22*(0.5*sign(s_theta(k))+1.5*beta2*(abs(s_theta(k)))^0.5*sign(s_theta(k))+beta2^2*s_theta(k));
        V_theta(k+1)=V_theta(k)+deltaT*dV_theta(k);
        tau_theta(k+1)=(-(K12*((abs(s_theta(k)))^0.5*sign(s_theta(k))+beta2*s_theta(k)))+V_theta(k));

        dV_psi(k)=-K23*(0.5*sign(s_psi(k))+1.5*beta3*(abs(s_psi(k))^0.5)*sign(s_psi(k))+beta3^2*s_psi(k));
        V_psi(k+1)=V_psi(k)+deltaT*dV_psi(k);
        tau_psi(k+1)=(-(K13*((abs(s_psi(k))^0.5)*sign(s_psi(k))+beta3*s_psi(k)))+V_psi(k));
        
        tau(:,k)=[tau_phi(k);tau_theta(k);tau_psi(k)];
        % tau(:,k)=A(:,:,k)*f(:,k) 
        
        % 一阶系统方程
        e(:,k)=[e_phi(k);e_theta(k);e_phi(k)];
        de(:,k)=[de_phi(k);de_theta(k);de_psi(k)];
        dR_eta(:,:,k)=R_eta(:,:,k)*S_omega(:,:,k);
        M(:,:,k)=J*pinv(R_eta(:,:,k));
        B(:,:,k)=M(:,:,k)*dR_eta(:,:,k)*pinv(R_eta(:,:,k))-S_omega(:,:,k)*M(:,:,k);
        rho_eta(:,k)=R_eta(:,:,k)*rho(:,k);

        ds(:,k+1)=c*de(:,k)-ddeta_d(:,k)-pinv(M(:,:,k))*B(:,:,k)*deta(:,k)+pinv(M(:,:,k))*Lambda*tau(:,k)+rho_eta(:,k);
        s(:,k+1)=s(:,k)+deltaT*ds(:,k);
        ddeta(:,k)=ds(:,k)-c*de(:,k)+ddeta_d(:,k);
        deta(:,k+1)=deta(:,k)+deltaT*ddeta(:,k); 
        % eta(:,k+1)=e(:,k)+eta_d(:,k);
        eta(:,k+1)=eta(:,k)+deltaT*deta(:,k); % eta(:,k+1)=pinv(c)*(s(:,k)-de(:,k)); w
        omega(:,k+1)=pinv(R_eta(:,:,k))*deta(:,k);
    end
    k=k+1;
end
t=0:T:stoptime;

% 图形输出 
% 期望的姿态和真实姿态的差值 三个通道的力矩值
% 姿态跟踪曲线
figure(1)
subplot(3,1,1)
plot(t,phi(:),'Color',[0,0,1],'LineWidth',1.6);
axis([0 50 -1 1])
hold on
plot(t,phi_d(:),'Color',[1,0,0],'LineWidth',1.6);
xlabel('时间(s)','FontSize',12');
ylabel('滚转角','interpreter','tex','Fontsize',12);
grid on
legend('当前姿态','目标姿态');
hold

subplot(3,1,2)
plot(t,theta(:),'Color',[0,0,1],'LineWidth',1.6);
axis([0 50 -1 1])
hold on
plot(t,theta_d(:),'Color',[1,0,0],'LineWidth',1.6)
xlabel('时间(s)','FontSize',12');
ylabel('俯仰角','interpreter','tex','Fontsize',12);
grid on
hold

subplot(3,1,3)
plot(t,psi(:),'Color',[0,0,1],'LineWidth',1.6);
axis([0 50 -1 1])
hold on
plot(t,psi_d(:),'Color',[1,0,0],'LineWidth',1.6);
xlabel('时间(s)','FontSize',12');
ylabel('偏航角','interpreter','tex','Fontsize',12);
grid on
hold

% 力矩
figure(2)
subplot(3,1,1)
plot(t,tau_phi(:),'Color',[0,0,1],'LineWidth',1.6);
%axis([0 50 -1 1])
xlabel('时间(s)','FontSize',12');
ylabel('滚转力矩','Interpreter','tex','FontSize',12');
grid on

subplot(3,1,2)
plot(t,tau_theta(:),'Color',[0,0,1],'LineWidth',1.6);
%axis([0 50 -1 1])
xlabel('时间(s)','FontSize',12');
ylabel('俯仰力矩','Interpreter','tex','FontSize',12');
grid on

subplot(3,1,3)
plot(t,tau_psi(:),'Color',[0,0,1],'LineWidth',1.6);
%axis([0 50 -1 1])
xlabel('时间(s)','FontSize',12');
ylabel('偏航力矩','Interpreter','tex','FontSize',12');
grid on

figure(3)
subplot(3,1,1)
e_phi = [e_phi, e_phi(end)];
plot(t,e_phi(:),'Color',[0,0,1],'LineWidth',1.6);
%axis([0 50 -1 1])
xlabel('时间(s)','FontSize',12');
ylabel('滚转通道','Interpreter','tex','FontSize',12');
grid on

subplot(3,1,2)
e_theta = [e_theta, e_theta(end)];
plot(t,e_theta(:),'Color',[0,0,1],'LineWidth',1.6);
%axis([0 50 -1 1])
xlabel('时间(s)','FontSize',12');
ylabel('俯仰通道','Interpreter','tex','FontSize',12');
grid on

subplot(3,1,3)
e_psi = [e_psi, e_psi(end)];
plot(t,e_psi(:),'Color',[0,0,1],'LineWidth',1.6);
%axis([0 50 -1 1])
xlabel('时间(s)','FontSize',12');
ylabel('偏航通道','Interpreter','tex','FontSize',12');
grid on

% 扰动曲线
figure(4)
subplot(3,1,1)
plot(t,rho(1,:),'Color',[0,0,1],'LineWidth',1.6);
% axis([0 20 -4 4])
xlabel('时间(s)','FontSize',12');
ylabel('$\lambda_x(t)$','interpreter','tex','Fontsize',12);
grid on


subplot(3,1,2)
plot(t,rho(2,:),'Color',[0,0,1],'LineWidth',1.6);
xlabel('时间(s)','FontSize',12');
ylabel('$\lambda_y(t)$','interpreter','tex','Fontsize',12);
grid on

subplot(3,1,3)
plot(t,rho(3,:),'Color',[0,0,1],'LineWidth',1.6);
xlabel('时间(s)','FontSize',12');
ylabel('$\lambda_z(t)$','interpreter','latex','Fontsize',12);
grid on

