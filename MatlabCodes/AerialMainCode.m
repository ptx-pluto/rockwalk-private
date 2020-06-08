

clc
clear all
close all


%Generating two data files "robot" and "sim_par" to save geometric/dynamic parameters
% of the object and the parameters for simulation

workdir1=pwd;
conresdir=exist('sim_par.mat');
if conresdir~=0
    delete 'sim_par.mat'
end
conresdir=exist('robot.mat');
if conresdir~=0
    delete 'robot.mat'
end


% The parameters of the robot is defined here:

% the length from apex point A to the point C on the rim of the disk
AC=sqrt(1.5^2+(0.35+0.35)^2);

% the length from apex point A to the point B on the rim of the disk
AB=sqrt(1.5^2+(0.35-0.35)^2);

% radius of the base circle
r=0.35;

%depending variable to compute the length from apex point A to its
%projection on the base
AH=sqrt(-(1/16)*(AC^4-2*AC^2*AB^2-8*AC^2*r^2+AB^4-8*AB^2*r^2+16*r^4)/r^2);

%the length from the center of the circular base to projection of apex
%point A
OH=(1/4)*(AC^2-AB^2)/r;

% matA is the matrix of coordinates of cable attachement points in space
MatA=[0,2,3;0,-2,3;3,3,0;3,-3,0]';

%MatB_local is the coordinates of the cable attachment points on the object
%with respect to the frame attached to the object when the rolling
%orientation is zero as explaine in the manuscript
MatB_local=[-AH,0,-OH;-AH,0,-OH;0,r*cos(45*pi/180),r*sin(45*pi/180);0,r*cos(135*pi/180),r*sin(135*pi/180)]';

% coordinate of the center of mass in the frame attached to the object
r_CM_O=[0.15;0;0.29];

%matrix of moment of inertia
% Mat_I=[0,-1,0;-1,0,0;0,0,-1]*([0.21,0,0;0,0.2,0.05;0,0.05,0.09]*[0,-1,0;-1,0,0;0,0,-1]');
Mat_I=zeros(3,3);

% mass of the object
m=1.04;

% Gravitational acceleration
g=9.81;


% All the parameters of the object is saved in robot.mat which is called
% whnever required
save('robot','AC','AB','r','AH','OH','MatA','MatB_local','r_CM_O','Mat_I','m','g');

%This code solve the differential equations in state space. in general the
%pose of the object is defined by 6 general coordinate namely the
%coordinate of the center of the circular disk and the euler angles. As
%such, 12 state space first order variables q_1,q_2,...,q_12 may be defined as:

%q_1: x_O
%q_2: y_O
%q_3: z_O
%q_4: psi
%q_5: theta
%q_6: phi
%q_7: d_x_O
%q_8: d_y_O
%q_9: d_z_O
%q_10: d_psi
%q_11: d_theta
%q_12: d_phi


% Different scenarios for the simulation are considered and each are chosen
% by changing the value of the variable "sim_num" as:
% sim_num=1---->> kinematic simulation with fixed apex point
% sim_num=2---->> kinematic simulation with non-fixed apex point
% sim_num=3---->> dynamic simulation with fixed apex point
% sim_num=4---->> dynamic simulation with non-fixed apex point

sim_num=2;    
    
    %----------------------------------------------------------
    %kinematic non-fixe apex point
    % in this part it is assumed that the apex point is not fixed anymore
    % each of the euler angles are supposed to follow a priodic
    % trjectory
%     period_psi=2.3;
%     period_theta=2.3;
%     period_phi=2.3;    
%     max_amplitude_phi=60*pi/180;
%     amplitut_psi=30*pi/180;
%     amplitut_theta=0*pi/180;
%     diff_phase_psi=pi/2;
%     diff_phase_theta=0;
%     diff_phase_phi=0;
%     save('sim_par','sim_num','period_psi','period_theta','period_phi','amplitut_psi','amplitut_theta','max_amplitude_phi','diff_phase_psi','diff_phase_theta','diff_phase_phi');
%        
    % setting the time span and calling the ode45
%     t_end=12;
%     t_span=0:t_end/500:t_end;
%     initial_cond= calc_init_state();
%     
%     isterminal = 0;  % Halt integration
%     direction = 0;   % The zero can be approached from either direction
%     save('eventopt','isterminal','direction');
%     opts = odeset('Events',@EventsFcn);
%     [vect_t,Var,~]=ode45(@coneEOM,t_span,initial_cond,opts);
% 
% 
% [glob_coor_xyz,glob_coor_A]=fun_coorG(Var);
% 
% figure(1)
% h1 = subplot(2,2,1);
% h2 = subplot(2,2,2);
% h3 = subplot(2,2,3);
% h4 = subplot(2,2,4);
% 
% subplot(h1);
% plot(vect_t(:,1),Var(:,6),'b');
% 
% subplot(h2)
% plot(vect_t(:,1),Var(:,4),'b')
% hold on
% plot(vect_t(:,1),Var(:,5),'r')
% plot(vect_t(:,1),Var(:,6),'g')
% axis([0 20 -1.5 3])
% xlabel('t')
% ylabel('rad')
% legend('\psi','\theta','\phi')
% 
% 
% subplot(h3);
% plot(glob_coor_A(:,1),glob_coor_A(:,2));
% 
% subplot(h4);
% draw_cone(Var,glob_coor_xyz);

simulate_aerial_rnw();

function simulate_aerial_rnw()

load('robot')

tstep = 0.002;
vstep = 0.05;

period_psi=2.3;
period_theta=2.3;
period_phi=2.3;    
max_amplitude_phi=60*pi/180;
amplitut_psi=30*pi/180;
amplitut_theta=0*pi/180;
diff_phase_psi=pi/2;
diff_phase_theta=0;
diff_phase_phi=0;
save('sim_par','period_psi','period_theta','period_phi','amplitut_psi','amplitut_theta','max_amplitude_phi','diff_phase_psi','diff_phase_theta','diff_phase_phi');

x = calc_init_state();
t = 0;

figure(1)
h1 = subplot(2,2,1);
h2 = subplot(2,2,2);
h3 = subplot(2,2,3);
h4 = subplot(2,2,4);

subplot(h2);
l2 = plot(t,x(4),'b');
hold on;
l3 = plot(t,x(5),'r');
l4 = plot(t,x(6),'g');
axis([0 20 -1.5 3])
xlabel('t');
ylabel('rad');
legend('\psi','\theta','\phi');

subplot(h3);
[glob_coor_xyz,glob_coor_A]=coorG(x);
l5 = plot(glob_coor_A(1),glob_coor_A(2));

subplot(h4);
axis(2*[-AH 1*AH -1*AH 1*AH -0.1 1*AH]);
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

traj_g = [glob_coor_xyz,];
traj_a = [calc_apex(x),];
draw_cone_once(h4,x);

while (1)

    vt = t:tstep:t+vstep;
    [~,xsave] = ode45(@coneEOM,vt,x);
    x = xsave(end,:);
    t = t+vstep;

    set(l2, 'XData', [get(l2, 'XData') t]);
    set(l2, 'YData', [get(l2, 'YData') x(4)]);

    set(l3, 'XData', [get(l3, 'XData') t]);
    set(l3, 'YData', [get(l3, 'YData') x(5)]);

    set(l4, 'XData', [get(l4, 'XData') t]);
    set(l4, 'YData', [get(l4, 'YData') x(6)]);
   
    [glob_coor_xyz,glob_coor_A]=coorG(x);
    
    set(l5, 'XData', [get(l5, 'XData') glob_coor_A(1)]);
    set(l5, 'YData', [get(l5, 'YData') glob_coor_A(2)]);    

    subplot(h4);
    traj_g = [traj_g;glob_coor_xyz];
    traj_a = [traj_a;calc_apex(x)];
    plot3(traj_g(:,1),traj_g(:,2),traj_g(:,3),'k');
    hold on;
    plot3(traj_a(:,1),traj_a(:,2),traj_a(:,3),'k');
    draw_cone_once(h4,x);
    
    pause(vstep);
    
end

end

function x0 = calc_init_state()

    load('sim_par');
    load('robot');
    
    omega_psi=2*pi/period_psi;
    omega_theta=2*pi/period_theta;
    omega_phi=2*pi/period_phi;
    
    coeff_t_phi=max_amplitude_phi/(5*period_phi);
    
    % setting the initial conditions
    Init_psi=amplitut_psi;
    Init_theta=45*pi/180;
    Init_phi=0*pi/180;
    
    MatR1z=[cos(Init_psi) -sin(Init_psi) 0; sin(Init_psi) cos(Init_psi) 0; 0 0 1;];
    MatR2y = [cos(Init_theta) 0 sin(Init_theta); 0 1 0; -sin(Init_theta) 0 cos(Init_theta);];
    Init_r_O=[0;0;0]+MatR1z*MatR2y*[-r;0;0];
    
    Init_x_O=Init_r_O(1,1);
    Init_y_O=Init_r_O(2,1);
    Init_z_O=Init_r_O(3,1);
    
    % Setting the initial velocity of the euler angles
    Init_d_psi=amplitut_psi*omega_psi*cos(omega_psi*0+diff_phase_psi);
    Init_d_theta=amplitut_theta*omega_theta*cos(omega_theta*0+diff_phase_theta);
    Init_d_phi=(coeff_t_phi*0)*omega_phi*cos(omega_phi*0+diff_phase_phi);
    
    %The initial velocity shoud satisfy the constraint equations
    %here the euler angles are free variables and the other variables are
    %computed with respect to the constraints of rolling without slippage
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const([Init_x_O,Init_y_O,Init_z_O,Init_psi,Init_theta,Init_phi],zeros(6,1));
    depend_vel_var=-inv(Mat_a_const(1:3,1:3))*Mat_a_const(1:3,4:6)*[Init_d_psi;Init_d_theta;Init_d_phi];
    Init_d_x_O=depend_vel_var(1,1);Init_d_y_O=depend_vel_var(2,1);Init_d_z_O=depend_vel_var(3,1);  

    x0 = [Init_x_O;Init_y_O;Init_z_O;Init_psi;Init_theta;Init_phi;Init_d_x_O;Init_d_y_O;Init_d_z_O;Init_d_psi;Init_d_theta;Init_d_phi];

end

function vect_d_q=coneEOM(vect_t,initial_cond)

% We have two sets of differential equaiotns.

% the first set is the differential equations relating to the system
% dynamic as: [M]*dd_q=[CG];
%[M] is the matrix of inertia
%[CG] is the vector of sum of the coriolis and gravity vector

% the second set is related to the velocity constriants as:
%[a]*d_q=-[b]
%[a] is the coefficient of the first order derivative of the generelized
%coordinat

    load('robot');
    load('sim_par')

    %-----------------------------------------------------------------
    %kinematic with non-fixed apex point
    
    %defining the trajectories of the euler angles depnting to time
    omega_psi=2*pi/period_psi;
    omega_theta=2*pi/period_theta;
    omega_phi=2*pi/period_phi;
    coeff_t_phi=max_amplitude_phi/(5*period_phi);
    

    if vect_t<0*period_phi
        Dphi=(coeff_t_phi*vect_t)*omega_phi*cos(omega_phi*vect_t+diff_phase_phi);
    else
        Dphi=max_amplitude_phi*omega_phi*cos(omega_phi*vect_t+diff_phase_phi);
    end
    
    Dpsi=amplitut_psi*omega_psi*cos(omega_psi*vect_t+diff_phase_psi);
    Dtheta= amplitut_theta*omega_theta*cos(omega_theta*vect_t+diff_phase_theta);
    
    vect_q=initial_cond;
    vect_d_q=zeros(length(initial_cond),1);
    
    % for the current general coordinates the [a] and its time derivative
    % are obtained by callig the Matlab function fun_Mat_a_const 
    % here we only consider the first three constraints and those relating
    % to the fixed apex point is not considered here.
    [Mat_a_const,Mat_d_a_const]=fun_Mat_a_const(vect_q(1:6,1),vect_q(7:12,1));
    depend_vel_var=-inv(Mat_a_const(1:3,1:3))*Mat_a_const(1:3,4:6)*[Dpsi;Dtheta;Dphi];
    vect_d_q(1:3)=depend_vel_var(1:3);
    vect_d_q(4:6)=[Dpsi;Dtheta;Dphi];

end

function draw_cone(Var,glob_coor_xyz)
load('robot');

for it_sim=1:size(Var,1)
    

    plot3(glob_coor_xyz(:,1),glob_coor_xyz(:,2),glob_coor_xyz(:,3),'k');
    axis(2*[-AH 1*AH -1*AH 1*AH -0.1 1*AH]);
    xlabel('x(m)')
    ylabel('y(m)')
    zlabel('z(m)')
    hold on
    box on
    
    coor_O=[Var(it_sim,1);Var(it_sim,2);Var(it_sim,3)];
    MatR1z=[-sin(Var(it_sim,4)) -cos(Var(it_sim,4)) 0; cos(Var(it_sim,4)) -sin(Var(it_sim,4)) 0; 0 0 1;];
    MatR2y=[cos(Var(it_sim,5)) 0 sin(Var(it_sim,5)); 0 1 0; -sin(Var(it_sim,5)) 0 cos(Var(it_sim,5));];
    MatR3z=[cos(Var(it_sim,6)),-sin(Var(it_sim,6)),0;sin(Var(it_sim,6)),cos(Var(it_sim,6)),0;0,0,1];
    MatRp=MatR1z*MatR2y;
    MatR=MatRp*MatR3z;
    coor_A=coor_O+MatR*[OH;0;AH];
    plotcircle3d(r,coor_O,MatR(:,3));
    
    Coor_B=coor_O+MatR*[-r;0;0];
    Coor_C=coor_O+MatR*[r;0;0];
    Coor_D=coor_O+MatR*[0;-r;0];
    Coor_E=coor_O+MatR*[0;r;0];
    Coor_F=coor_O+MatR*r*[cos(pi/3);sin(pi/3);0];
    Coor_G=coor_O+MatR*r*[cos(pi+pi/3);sin(pi+pi/3);0];
    Coor_H=coor_O+MatR*r*[cos(pi/6);sin(pi/6);0];
    Coor_I=coor_O+MatR*r*[cos(pi+pi/6);sin(pi+pi/6);0];
    MatB=coor_O*ones(1,4)+MatR*MatB_local;
    
    plot3([coor_A(1),Coor_B(1)],[coor_A(2),Coor_B(2)],[coor_A(3),Coor_B(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_C(1)],[coor_A(2),Coor_C(2)],[coor_A(3),Coor_C(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_D(1)],[coor_A(2),Coor_D(2)],[coor_A(3),Coor_D(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_E(1)],[coor_A(2),Coor_E(2)],[coor_A(3),Coor_E(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_F(1)],[coor_A(2),Coor_F(2)],[coor_A(3),Coor_F(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_G(1)],[coor_A(2),Coor_G(2)],[coor_A(3),Coor_G(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_H(1)],[coor_A(2),Coor_H(2)],[coor_A(3),Coor_H(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_I(1)],[coor_A(2),Coor_I(2)],[coor_A(3),Coor_I(3)],'g','LineWidth',2);


    hold off 
    pause(1/30);
end

end

function coor_A = calc_apex(Var)
    load('robot');
    coor_O=[Var(1);Var(2);Var(3)];
    MatR1z=[-sin(Var(4)) -cos(Var(4)) 0; cos(Var(4)) -sin(Var(4)) 0; 0 0 1;];
    MatR2y=[cos(Var(5)) 0 sin(Var(5)); 0 1 0; -sin(Var(5)) 0 cos(Var(5));];
    MatR3z=[cos(Var(6)),-sin(Var(6)),0;sin(Var(6)),cos(Var(6)),0;0,0,1];
    MatRp=MatR1z*MatR2y;
    MatR=MatRp*MatR3z;
    coor_A=(coor_O+MatR*[OH;0;AH])';    
end

function draw_cone_once(h,Var)
    
    load('robot');

    subplot(h);
    
    hold on
    box on
    
    coor_O=[Var(1);Var(2);Var(3)];
    MatR1z=[-sin(Var(4)) -cos(Var(4)) 0; cos(Var(4)) -sin(Var(4)) 0; 0 0 1;];
    MatR2y=[cos(Var(5)) 0 sin(Var(5)); 0 1 0; -sin(Var(5)) 0 cos(Var(5));];
    MatR3z=[cos(Var(6)),-sin(Var(6)),0;sin(Var(6)),cos(Var(6)),0;0,0,1];
    MatRp=MatR1z*MatR2y;
    MatR=MatRp*MatR3z;
    coor_A=coor_O+MatR*[OH;0;AH];
    plotcircle3d(r,coor_O,MatR(:,3));
    
    Coor_B=coor_O+MatR*[-r;0;0];
    Coor_C=coor_O+MatR*[r;0;0];
    Coor_D=coor_O+MatR*[0;-r;0];
    Coor_E=coor_O+MatR*[0;r;0];
    Coor_F=coor_O+MatR*r*[cos(pi/3);sin(pi/3);0];
    Coor_G=coor_O+MatR*r*[cos(pi+pi/3);sin(pi+pi/3);0];
    Coor_H=coor_O+MatR*r*[cos(pi/6);sin(pi/6);0];
    Coor_I=coor_O+MatR*r*[cos(pi+pi/6);sin(pi+pi/6);0];
    MatB=coor_O*ones(1,4)+MatR*MatB_local;
    
    plot3([coor_A(1),Coor_B(1)],[coor_A(2),Coor_B(2)],[coor_A(3),Coor_B(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_C(1)],[coor_A(2),Coor_C(2)],[coor_A(3),Coor_C(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_D(1)],[coor_A(2),Coor_D(2)],[coor_A(3),Coor_D(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_E(1)],[coor_A(2),Coor_E(2)],[coor_A(3),Coor_E(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_F(1)],[coor_A(2),Coor_F(2)],[coor_A(3),Coor_F(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_G(1)],[coor_A(2),Coor_G(2)],[coor_A(3),Coor_G(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_H(1)],[coor_A(2),Coor_H(2)],[coor_A(3),Coor_H(3)],'g','LineWidth',2);
    plot3([coor_A(1),Coor_I(1)],[coor_A(2),Coor_I(2)],[coor_A(3),Coor_I(3)],'g','LineWidth',2);


    hold off 

end

function [coor_G,coor_A]=coorG(Var)

load('robot');

MatR1z=[-sin(Var(4)) -cos(Var(4)) 0; cos(Var(4)) -sin(Var(4)) 0; 0 0 1;];
MatR2y = [cos(Var(5)) 0 sin(Var(5)); 0 1 0; -sin(Var(5)) 0 cos(Var(5));];
MatR3z=[cos(Var(6)) -sin(Var(6)) 0; sin(Var(6)) cos(Var(6)) 0; 0 0 1;];
coor_G=[Var(1);Var(2);Var(3)]+MatR1z*MatR2y*[r;0;0];
coor_A=[Var(1);Var(2);Var(3)]+MatR1z*MatR2y*MatR3z*[OH;0;AH];

coor_G=coor_G';
coor_A=coor_A';

end