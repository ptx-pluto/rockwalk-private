function [t,y] = solve_fixed_apex_dynamics_param(cone_params)

    load('DynamicEquations.mat', 'q', 'dqdt')

    FPS = 30;
    Duration = 3;
    N = round(Duration*FPS);

    init_psi = 0;
    init_theta = deg2rad(10);
    init_phi = deg2rad(40);
    init_phi_dot = 0;

    xinit = compute_initial_states(q, dqdt, init_psi, init_theta, init_phi, init_phi_dot,cone_params);

    cone_ode = @(t,x)state_dynamics(x,cone_params);

    [t,y] = ode45(cone_ode, linspace(0,Duration,N), xinit);

end


function dxdt = state_dynamics(x,cone_params)

    %---------------------------------------------------------------------%
    
    m = double(cone_params.mass);
    g = double(cone_params.gravity);
    R = double(cone_params.radius);
    xCM = double(cone_params.lateral_CM_offset);
    zCM = double(cone_params.vertical_CM_offset);
    xA = double(cone_params.lateral_apex_offset);
    zA = double(cone_params.vertical_apex_offset);
    
    %---------------------------------------------------------------------%

    
    q1 = x(1); 
    q2 = x(2); 
    q4 = x(3);
    q5 = x(4);
    q6 = x(5);

    dqdt1 = x(6);
    dqdt2 = x(7);
    dqdt4 = x(8);
    dqdt5 = x(9);
    dqdt6 = x(10);

    vddqdt1 = ddqdt1(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt2 = ddqdt2(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt4 = ddqdt4(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt5 = ddqdt5(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    vddqdt6 = ddqdt6(R,dqdt4,dqdt5,dqdt6,g,m,q4,q5,q6,xA,xCM,zA,zCM);
    

    dxdt = [dqdt1;dqdt2;dqdt4;dqdt5;dqdt6;...
            vddqdt1;vddqdt2;vddqdt4;vddqdt5;vddqdt6];
end


function xinit = compute_initial_states(q, dqdt, init_psi, init_theta, init_phi, init_phi_dot, cone_params)


    %---------------------------------------------------------------------%
    m = cone_params.mass;
    g = cone_params.gravity;
    R = cone_params.radius;
    xCM = cone_params.lateral_CM_offset;
    zCM = cone_params.vertical_CM_offset;
    xA = cone_params.lateral_apex_offset;
    zA = cone_params.vertical_apex_offset;
    %---------------------------------------------------------------------%

    A = constraint_marix(R,init_psi, init_theta, init_phi,xA,zA);
                        
                        
    velocity_vector = [dqdt(1);dqdt(2);dqdt(4);dqdt(5);init_phi_dot];
    
    constraint_eqns = A*velocity_vector;
    
    sol_constraint_eqns = solve(constraint_eqns,[dqdt(1),dqdt(2),dqdt(4),dqdt(5)]);
    
    % now, setup initial states
    
    xinit = zeros(10,1);
    xinit(1) = 0; % x
    xinit(2) = 0; % y
    xinit(3) = init_psi; % psi
    xinit(4) = init_theta; %theta
    xinit(5) = init_phi; %phi
    
    xinit(6) = sol_constraint_eqns.dqdt1; % x_dot
    xinit(7) = sol_constraint_eqns.dqdt2; % y_dot
    xinit(8) = sol_constraint_eqns.dqdt4; % psi_dot
    xinit(9) = sol_constraint_eqns.dqdt5; % theta_dot
    
    xinit(10) = init_phi_dot; % phi_dot

end