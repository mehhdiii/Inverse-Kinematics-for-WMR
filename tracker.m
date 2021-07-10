% Differential drive WMR

T = 0.05; %Sampling time
W = 1/2; %m. Width of Robot
r = 1/4; %m. Radius of tires

% Create Trajectory for WMR:
N=100;t=linspace(-2*pi,2*pi,N);
f1=1;x=5*sin(f1*t);f2=2;y=5*sin(f2*t);
phi = tan(y./x);
%initialize resulting velocities
omega = zeros(1, N); v = zeros(1, N); 
plot(x,y,'linewidth',2);

for n = 1:N-1
    
    %calculating inverse kinematics variables: 
    mu = 1/2*(sin(phi(n))*(y(n+1)-y(n))+cos(phi(n)*(x(n+1)-x(n))))/(cos(phi(n))*(y(n+1)-y(n))-sin(phi(n))*(x(n+1)-x(n))); 
    x_m = (x(n)+x(n+1))/2; y_m = (y(n)+y(n+1))/2; 
    x_star = x_m - mu/2 * (y(n+1) - y(n)); y_star = y_m + mu/2 * (x(n+1)-x(n)); 
    R_n = sqrt((x(n) - x_star)^2 + (y(n)-y_star)^2); 
    theta_1 = atan2((y(n)-y_star), (x(n)-x_star)); theta_2 = atan2((y(n+1)-y_star), (x(n+1)-x_star)); 
    del_phi = theta_1 - theta_2; 
    %resulting velocities: 
    omega(n) = del_phi/T; 
    v(n) = R_n*omega(n);  
    
    
end
figure()
hold on 
plot(t,omega,'linewidth',2);
plot(t, v,'linewidth',2)
legend('Angular Velocity', 'Linear velocity')
hold off
