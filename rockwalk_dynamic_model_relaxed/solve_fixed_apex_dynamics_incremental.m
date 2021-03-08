clear all
close all

% setup simulation

time_tol = 5;

init_psi = 0;
init_theta = deg2rad(30);
init_phi = deg2rad(40);
init_phi_dot = 0;

state = init_state(init_psi, init_theta, init_phi, init_phi_dot);

ctime = 0.05;
T = 0;

% setup figures

figure(1)
h1 = subplot(3,2,1);            
p1 = plot(T, state(1),'r-','LineWidth',1);         
xlabel('Time (s)');
ylabel('x');
axis ([0, time_tol, -1, 0]);

subplot(3,2,2)
p2 = plot(T, state(2),'r-','LineWidth',1);
title('y')
axis ([0, time_tol, -1, 1]);

subplot(3,2,3)
p3 = plot(T, state(3),'r-','LineWidth',1);
title('\psi')
axis ([0, time_tol, -1, 1]);

subplot(3,2,4)
p4 = plot(T, state(4),'r-','LineWidth',1);
title('\theta')
axis ([0, time_tol, -1, 1]);

subplot(3,2,5)
p5 = plot(T, state(5),'r-','LineWidth',1);
title('\phi')
axis ([0, time_tol, -1, 1]);

subplot(3,2,6)
[X,Y,Z] = cone_wire_frame(state);
p6 = plot3(X,Y,Z);
grid on; 
axis equal;
axis ([-1, 1, -1, 1, -0.5, 1.5]);

% run simulation

while T < time_tol
   
    %[vt,vy] = ode45(@(t,x)eom_rnw_symbolic(x), [T T+ctime], state);
    [vt,vy] = ode45(@(t,x)eom_rnw_numerical(x), [T T+ctime], state);
    
    T = T+ctime;
    state = vy(end,:)';
    
          
    set(p1, 'XData', [get(p1, 'XData') T]);
    set(p1, 'YData', [get(p1, 'YData') state(1)]);

    set(p2, 'XData', [get(p2, 'XData') T]);
    set(p2, 'YData', [get(p2, 'YData') state(2)]);
    
    set(p3, 'XData', [get(p3, 'XData') T]);    
    set(p3, 'YData', [get(p3, 'YData') state(3)]);
    
    set(p4, 'XData', [get(p4, 'XData') T]);
    set(p4, 'YData', [get(p4, 'YData') state(4)]);
    
    set(p5, 'XData', [get(p5, 'XData') T]);    
    set(p5, 'YData', [get(p5, 'YData') state(5)]);

    [X,Y,Z] = cone_wire_frame(state);
    set(p6, 'XData', X);    
    set(p6, 'YData', Y);
    set(p6, 'ZData', Z);
    
    drawnow;
    
end