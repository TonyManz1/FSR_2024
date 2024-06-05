clear all;
close all;
clc;

%% Data
load('ws_homework_3_2024.mat'); %first thing to do is load the workspace
m=1.5; %mass(kg) of the drone
g=9.81;
I=diag([1.2416 1.2416 2*1.2416]); %inertia matrix
Ts=0.001; % sampling time of the estimator 
e3=[0 0 1]';

% Attitude Values
phi=attitude.signals.values(:,1);
theta=attitude.signals.values(:,2);
psi=attitude.signals.values(:,3);

% Velocities Values
linear_vel=linear_vel.signals.values;
attitude_dot=attitude_vel.signals.values;

% Attitude Velocities Values
phi_dot=attitude_vel.signals.values(:,1);
theta_dot=attitude_vel.signals.values(:,2);
psi_dot=attitude_vel.signals.values(:,3);

%% Definition of Trasfer Function
% I have created a transfer function through the chebishev filter 
% Given the order r, the ripple and the bandwidth the filter automatically computes the coefficients c0
% and cj

r = 7; % order
ripple = 3; % expressed in db 
omega_n= 10;  % passband edge frequency 
[c0,cj] = cheby1(r,ripple,omega_n,'low','s'); 
G=tf(c0,cj);

%% K coefficients
cj=flip(cj);
K=zeros(1,r);
product=1;
for i=r:-1:1
    if i==r
        K(i)=cj(i);
        product=K(i);
    elseif i==1
        K(i)=cj(1)/product;
    else 
        K(i)=cj(i)/product;
        product=product*K(i);
    end
end
fprintf("Gains Vector:");
K

%% Estimator

integral=zeros(6,r);
gamma_prev=[0 0 0 0 0 0]';
wrench_prev=[0 0 0 0 0 0]';
wrench=[0 0 0 0 0 0]';

for t=1:size(attitude.time,1)

    R=comp_R(phi(t),theta(t),psi(t));

    Q=comp_Q(phi(t),theta(t));
   
    Q_dot = comp_Q_dot (phi(t),theta(t),phi_dot(t),theta_dot(t));
    
    C=comp_C(Q,I,Q_dot,attitude_dot(t));

    M=comp_M(Q,I);

    q=comp_q(m,M,linear_vel(t,:),attitude_dot(t,:));
 
    estimation=comp_est(m,g,e3,thrust.signals.values(t,:),R,C,attitude_dot(t,:),Q,tau.signals.values(t,:));
    
    [gamma(:,:,t),wrench(:,1),integral]= comp_gamma(wrench_prev,integral,estimation, K,r,q,Ts);
    wrench_prev=wrench;

    fe_tilde(:,t)= gamma(1:3,end,t);
    taue_tilde(:,t)=gamma(4:6,end,t);
end

%% Results and Plots
fprintf('\n');
f_ex=gamma(1,end);
fprintf('f_ex = %f N\n',f_ex);
f_ey=gamma(2,end);
fprintf('f_ey = %f N\n',f_ey);
f_ez=gamma(3,end);
fprintf('f_ez = %f N\n',f_ez);
tau_ex=gamma(4,end);
fprintf('tau_ex = %f Nm\n',tau_ex);
tau_ey=gamma(5,end);
fprintf('tau_ey = %f Nm\n',tau_ey);
tau_ez=gamma(6,end);
fprintf('tau_ez = %f Nm\n',tau_ez);

subplot(1,2,1)
plot(attitude.time,fe_tilde(:,:),'lineWidth',3);
title('Evolution of $f_e$','fontsize',14,'interpreter','latex')
xlabel('t (s)','fontsize',14,'interpreter','latex') 
ylabel('$f_e$ (N)','fontsize',14,'interpreter','latex')
legend('$f_ex$','$f_ey$','$f_ez$','interpreter','latex')
grid on;
axis square;

subplot(1,2,2)
plot(attitude.time,taue_tilde(:,:),'lineWidth',3);
title('Evolution of $\tau_e$','fontsize',14,'interpreter','latex')
xlabel('t (s)','fontsize',14,'interpreter','latex') 
ylabel('$\tau_e$ (Nm)','fontsize',14,'interpreter','latex')
legend('$\tau_ex$','$\tau_ey$','$\tau_ez$','interpreter','latex')
grid on;
axis square;
sgtitle (r+"-order estimator");


%% Calculate Real Mass of UAV
real_mass = (f_ez/g + m);
fprintf("\nThe real mass of UAV is %f kg\n", real_mass);

%% Functions to calculate all quantities necessary for define the estimator
function R=comp_R (phi,theta,psi)
 
R =[cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
       cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
       -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)];
end

function Q=comp_Q (phi,theta)
    
Q=[ 1 0 -sin(theta);
    0 cos(phi) cos(theta)*sin(phi);
    0 -sin(phi) cos(theta)*cos(phi)];
end

function Q_dot = comp_Q_dot (phi,theta,phi_dot,theta_dot)

Q_dot=[0 0 -theta_dot*cos(theta);
       0 -phi_dot*sin(phi) -theta_dot*sin(theta)*sin(phi)+phi_dot*cos(phi)*cos(theta);
       0 -phi_dot*cos(phi) -theta_dot*sin(theta)*cos(phi)-phi_dot*sin(phi)*cos(theta)];
end

function C=comp_C(Q,I,Q_dot,attitude_dot)

C=(Q'*skew(Q*attitude_dot)*I*Q)+(Q'*I*Q_dot);

end

function M=comp_M(Q,I)
    
M=Q'*I*Q;

end

function q=comp_q(m,M,linear_vel,attitude_dot)

q=[m*eye(3) zeros(3, 3); zeros(3, 3) M]*[linear_vel'; attitude_dot'];

end

function estimation=comp_est(m,g,e3,thrust,R,C,attitude_dot,Q,tau)

estimation=[m*g*e3-thrust*R*e3; C'*attitude_dot'+Q'*tau'];
                
end

function S=skew(v)
    if(numel(v)~= 1)
    S= [0 -v(3) v(2); 
        v(3) 0 -v(1);
        -v(2) v(1) 0];
    else
    S= zeros(3);
    end
end

%% Function to calculate the estimator
function [gamma,wrench,integral]= comp_gamma(wrench_prev,integral,estimation, K,r,q,Ts) 

    if r==1
        integral=integral+Ts*(wrench_prev+estimation);
        gamma(:,1)=K(1)*(q-integral);
        wrench(:,1)=gamma(:,1);
    else
        for i=1:r
             if i==1
                integral(:,i)=integral(:,i)+Ts*(wrench_prev+estimation);
                gamma(:,i)=K(i)*(q-integral(:,i));
                gamma_prev(:,1)=gamma(:,i);
             elseif i==r
                integral(:,i)=integral(:,i)+Ts*(-wrench_prev + gamma_prev);
                gamma(:,i) = K(i)*integral(:,i);
                wrench(:,1)=gamma(:,i);
             else
               integral(:,i)=integral(:,i)+Ts*(-wrench_prev + gamma_prev);
               gamma(:,i) = K(i)*integral(:,i);
               gamma_prev(:,1)=gamma(:,i);
             end
         end
    end
end
