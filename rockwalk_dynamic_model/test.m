load DynamicEquations.mat

addpath('./helper_functions')

[R1, R2] = coordinate_frames(q);


syms R L

pt = [R; 0; L];

C = R2*pt;

C = simplify(C);

syms q4(t) q5(t) q6(t)

x1 = subs(C(1),[q(4) q(5) q(6)],[q4(t) q5(t) q6(t)]);

dx = diff(x1,t);

ddx = subs(dx, [q4(t), q5(t), q6(t), diff(q4(t), t), diff(q5(t), t), diff(q6(t), t)], [q(4), q(5), q(6), dqdt(4), dqdt(5), dqdt(6)]);

ddx = simplify(ddx);

pc = coeffs(ddx,dqdt(4));
coe = pc(2)


%colors = gen_color_profile(6);

%plot_multi_color(results,colors);


%plot_full_state_multi(results);

% pds = zeros(1,N);
% 
% for i=1:N
%     t = results(i).t;
%     y = results(i).y;
%     [ts,ps] = find_peaks(t,y);
%     pds(i) = ts(3);
% end
% 
% ls = zeros(1,N);
% 
% for i=1:N
%     c = cone_move_com_x(cone_params,offsets(i));
%     ls(i) = cone2l(c);
% end
% 
% plot(ls,pds,'-o');
% 

% dqdt1 = 0;
% dqdt2 = 0;
% dqdt3 = 0;
% dqdt4 = 0;
% dqdt5 = 0;
% dqdt6 = 0;
% 
% matrix_eom.gravity_vector
% 
% function rst = cone2l(cone)
%     
%     G_x = cone.radius;
%     G_z = 0;
%     
%     G = [G_x G_z];
%     
%     com_x = cone.lateral_CM_offset;
%     com_z = cone.vertical_CM_offset;
% 
%     CoM = [com_x com_z];
% 
%     rst = norm(G-CoM);
%     
% end
% 
% function rst = cone_move_com_x(cone, ratio)
% 
%     G_x = cone.radius;
%     G_z = 0;
%     
%     G = [G_x G_z];
%     
%     com_x = cone.lateral_CM_offset;
%     com_z = cone.vertical_CM_offset;
% 
%     CoM = [com_x com_z];
%     
%     dir = G - CoM;
%     
%     new_CoM = CoM + ratio*dir;
%     
%     rst = cone;
%     rst.lateral_CM_offset = new_CoM(1);
%     rst.vertical_CM_offset = new_CoM(2);
%     
% end

function rst = gen_color_profile(N)

    rst = zeros(2*N+1,3);
    
    for i=1:N
        opa = 1-i/N;
        rst(i,:) = [1 1 1] - opa*[0 1 1];
    end
    
    rst(N+1,:) = [0 0 0];
    
    for j=1:N
        opa = j/N;
        rst(N+1+j,:) = [1 1 1] - opa*[1 1 0];  
    end
    
end
