clear all
close all
clc


%% Initial configuration
x_i = 0;
y_i = 0;
theta_i = 0;

%% Creation of the final configuration
while true
    
    final_conf = rand(1,3);

    x_f = final_conf(1);
    y_f = final_conf(2);
    theta_f = final_conf(3); 
 
    qi = [x_i;y_i;theta_i];
    qf = [x_f;y_f;theta_f];

    % save the result in a vector and divide it by its norm
    qf = qf/norm(qf);

    
    norm_vec = norm(qf-qi);
    
   
    if abs(norm_vec) == 1 
        break;  
    end
end

% Display the final configuration
disp(['Final configuration: (', num2str(x_f), ', ', num2str(y_f), ', ', num2str(theta_f), ')'])

%% Planning via Cartesian polynomial (cubic polynomial)

% Definition of the coefficients
k=6;
alfa_x = k*cos(theta_f)-3*x_f;
alfa_y = k*sin(theta_f)-3*y_f;
beta_x = k*cos(theta_i)+3*x_i;
beta_y = k*sin(theta_i)+3*y_i;

% Path 
s = 0 : 0.001 : 1;
    
x_s=s.^3.*x_f-(s-1).^3.*x_i+alfa_x.*s.^2.*(s-1)+beta_x.*s.*(s-1).^2;
y_s=s.^3.*y_f-(s-1).^3.*y_i+alfa_y.*s.^2.*(s-1)+beta_y.*s.*(s-1).^2;
x_s_dot = 3.*s.^2.*x_f-3.*(s-1).^2.*x_i+2.*alfa_x.*s.*(s-1)+alfa_x.*s.^2+beta_x.*(s-1).^2+2.*beta_x.*s.*(s-1);
y_s_dot = 3.*s.^2.*y_f-3.*(s-1).^2.*y_i+2.*alfa_y.*s.*(s-1)+alfa_y.*s.^2+beta_y.*(s-1).^2+2.*beta_y.*s.*(s-1);
theta = atan2(y_s_dot, x_s_dot);


%% Adding the time law to our path

while true

value = input('insert tf value: ');
ti = 0;
tf = value;

t = 0 : 0.001 : tf;

a0=0;
a1=0;
a2=3/tf^2;
a3=-2/tf^3;
s=a0+a1*t+a2*t.^2+a3*t.^3;
ds=2*a2*t+3*a3*t.^2;


for i=1 : length(t)
    si=s(i);
    
    x(i)=si.^3.*x_f-(si-1).^3.*x_i+alfa_x.*si.^2.*(si-1)+beta_x.*si.*(si-1).^2;
    y(i)=si.^3.*y_f-(si-1).^3.*y_i+alfa_y.*si.^2.*(si-1)+beta_y.*si.*(si-1).^2;
    xp(i)=3.*si.^2.*x_f-3.*(si-1).^2.*x_i+2.*alfa_x.*si.*(si-1)+alfa_x.*si.^2+beta_x.*(si-1).^2+2.*beta_x.*si.*(si-1);
    yp(i)=3.*si.^2.*y_f-3.*(si-1).^2.*y_i+2.*alfa_y.*si.*(si-1)+alfa_y.*si.^2+beta_y.*(si-1).^2+2.*beta_y.*si.*(si-1);
    xpp(i)=6.*si.*x_f-6.*(si-1).*x_i+2.*alfa_x.*(si-1)+2.*alfa_x.*si+2.*alfa_x.*si+2.*beta_x.*(si-1)+2.*beta_x.*(si-1)+2.*beta_x.*si;
    ypp(i)=6.*si.*y_f-6.*(si-1).*y_i+2.*alfa_y.*(si-1)+2.*alfa_y.*si+2.*alfa_y.*si+2.*beta_y.*(si-1)+2.*beta_y.*(si-1)+2.*beta_y.*si;
    theta(i)=atan2(yp(i),xp(i));

    v_tilde(i)=sqrt(xp(i)^2+yp(i)^2);
    w_tilde(i)=(ypp(i)*xp(i)-xpp(i)*yp(i))/(xp(i)^2+yp(i)^2);
    
    v(i)=(v_tilde(i)*ds(i)); 
    w(i)=(w_tilde(i)*ds(i));
end

v_max = 2;  % Max linear velocity
omega_max = 1;  % Max angular velocity
if (max(abs(v)) <= v_max && max(abs(w)) <= omega_max)
    
   break;
end
disp('It is recommended to increase the T period')
figure
subplot(1,2,1)
plot(t, v, 'LineWidth', 3)
title('Heading Velocity','FontSize',14)
xlabel('t [seconds]','FontSize',14)
ylabel('v [m/s]','FontSize',14)
axis square
grid on
subplot(1,2,2)
plot(t, w, 'LineWidth', 3)
title('Angular Velocity','FontSize',14)
xlabel('t [seconds]','FontSize',14)
ylabel('$$\omega$$ [rad/s]','Interpreter','latex','FontSize',14)
axis square
grid on
end

%% Plot
figure(1)
subplot(1,2,1)
plot(x,y,'lineWidth',3)
hold on;
plot(x_i, y_i, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r')
plot(x_f, y_f, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b')
hold off
title('Evolution of x,y','fontsize',14,'interpreter','latex')
xlabel('x [meters]','fontsize',14,'interpreter','latex')
ylabel('y [meters]','fontsize',14,'interpreter','latex')
axis square
grid on
legend('Trajectory', 'Initial Configuration', 'Final Configuration');
subplot(1,2,2)
plot(s,theta,'lineWidth',3)
title('Evolution of $\theta$','fontsize',14,'interpreter','latex')
xlabel('s','fontsize',14,'interpreter','latex')
ylabel('$\theta$ [rad]','fontsize',14,'interpreter','latex')
axis square
grid on
figure(2)
subplot(1,2,1)
plot(s,v_tilde,'lineWidth',3)
title('Geometric Heading Velocity','fontsize',14,'interpreter','latex')
xlabel('s','fontsize',14,'interpreter','latex')
ylabel('$\tilde{v}$ [m/s]','fontsize',14,'interpreter','latex')
axis square
grid on
subplot(1,2,2)
plot(s,w_tilde,'lineWidth',3)
title('Geometric Angular Velocity','fontsize',14,'interpreter','latex')
xlabel('s','fontsize',14,'interpreter','latex')
ylabel('$\tilde{\omega}$ [rad/s]','fontsize',14,'interpreter','latex')
axis square
grid on
figure(3)
subplot(1,2,1)
plot(t,v,'lineWidth',3)
title('Heading Velocity','fontsize',14,'interpreter','latex')
xlabel('t [seconds]','fontsize',14,'interpreter','latex')
ylabel('v [m/s]','fontsize',14,'interpreter','latex')
axis square
grid on
subplot(1,2,2)
plot(t,w,'lineWidth',3)
title('Angular Velocity','fontsize',14,'interpreter','latex')
xlabel('t [seconds]','fontsize',14,'interpreter','latex')
ylabel('$\omega$ [rad/s]','fontsize',14,'interpreter','latex')
axis square
grid on

disp('All constraints have been satisfied');
xd=timeseries(x,t);
yd=timeseries(y,t);
thetad=timeseries(theta,t);