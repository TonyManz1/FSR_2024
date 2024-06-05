clear all;
close all;
clc;

%% Radius of the wheels and distance between the wheels and the center of the robot
r=0.5;
l=1.2;

%% Vector of the generalized coordinates
syms x;
syms y;
syms theta;
syms alfa;
syms beta;
syms gamma;
q = [x ; y ; theta ; alfa ; beta ; gamma];

[n,z] = size(q);

%% Inputs of the system
syms u1;
syms u2;
syms u3;

%% Matrix A^T
A_trasp = [(sqrt(3)*cos(theta))/2-sin(theta)/2  cos(theta)/2+(sqrt(3)*sin(theta))/2  l r 0 0;
            sin(theta)                              -cos(theta)                      l 0 r 0;
            (-sqrt(3)*cos(theta))/2-sin(theta)/2 cos(theta)/2-(sqrt(3)*sin(theta))/2 l 0 0 r];

%% Matrix G
G = null(A_trasp);
[v,m] = size(G);
g_1 = G(:,1);
g_2 = G(:,2);
g_3 = G(:,3);

%% Kinematic model
disp(" The Kinematic Model is:")
q_dot = G*[u1;u2;u3]


%% Verify if the model is holonomic or not

% Computation of the jacobian
del_g1 = jacobian(g_1,q);
del_g2 = jacobian(g_2,q);
del_g3 = jacobian(g_3,q);

% lie bracket
Lie_g12 = del_g2*g_1-del_g1*g_2;
Lie_g23 = del_g3*g_2-del_g2*g_3;
Lie_g31 = del_g1*g_3-del_g3*g_1;

% Accessibility Distribution
delta_a = [g_1 g_2 g_3 Lie_g12 Lie_g23 Lie_g31];
dim_a = size(delta_a);
% Evaluate the rank to verify if there are some vectors that are linearly
% dependent
R=rank(delta_a);
if R==n 
    disp("The system is completely nonholonomic")
elseif (R>m)&&(R<n)
    disp("The system is nonholonimic but partially integrable")
elseif R==m
    disp("The system is completely holonomic")
end
