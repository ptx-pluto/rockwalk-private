function xinit = init_state(init_psi, init_theta, init_phi, init_phi_dot)


    load('DynamicEquations.mat', 'q', 'dqdt');

    %load FixedApexFunctions.mat


  %---------------------------------------------------------------------%
    isSymbolic = false;
    cone_params = cone_parameters(isSymbolic);
    
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