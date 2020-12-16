function cone_params = cone_parameters(isSymbolic)
    %{
    Define disk parameters

    %}

    if isSymbolic

        syms m g R xCM zCM xA zA

        cone_params.mass = m; 
        cone_params.gravity = g;
        cone_params.radius = R;
        cone_params.lateral_CM_offset = xCM; 
        cone_params.vertical_CM_offset = zCM;
        cone_params.lateral_apex_offset = xA;
        cone_params.vertical_apex_offset = zA;
      

    else

        %statue
%         cone_params.mass = sym(1);
%         cone_params.gravity = sym(9.8);
%         cone_params.radius = sym(1.26);
%         cone_params.lateral_CM_offset = sym(1.5);
%         cone_params.vertical_CM_offset = sym(3.175);
%         cone_params.lateral_apex_offset = sym(1.26);
%         cone_params.vertical_apex_offset = sym(7.35);
        
        % scaled statue
        cone_params.mass = sym(1);
        cone_params.gravity = sym(9.8);
        cone_params.radius = sym(1.26*0.4082);
        cone_params.lateral_CM_offset = sym(1.5*0.4082);
        cone_params.vertical_CM_offset = sym(3.175*0.4082);
        cone_params.lateral_apex_offset = sym(1.26*0.4082);
        cone_params.vertical_apex_offset = sym(7.35*0.4082);
        
                % large-steel object
%         cone_params.mass = sym(1.80);
%         cone_params.gravity = sym(9.8);
%         cone_params.radius = sym(0.35);
%         cone_params.lateral_CM_offset = sym(0.15);
%         cone_params.vertical_CM_offset = sym(0.29);
%         cone_params.lateral_apex_offset = sym(0.35);
%         cone_params.vertical_apex_offset = sym(1.5); 
        

        % medium-steel object
%         cone_params.mass = sym(1.40);
%         cone_params.gravity = sym(9.8);
%         cone_params.radius = sym(0.225);
%         cone_params.lateral_CM_offset = sym(0.11);
%         cone_params.vertical_CM_offset = sym(0.36);
%         cone_params.lateral_apex_offset = sym(0.225);
%         cone_params.vertical_apex_offset = sym(1.5); 
        

        % small-steel object
%         cone_params.mass = sym(1.0);
%         cone_params.gravity = sym(9.8);
%         cone_params.radius = sym(0.15);
%         cone_params.lateral_CM_offset = sym(0.09);
%         cone_params.vertical_CM_offset = sym(0.40);
%         cone_params.lateral_apex_offset = sym(0.15);
%         cone_params.vertical_apex_offset = sym(1.5); 
        
        % default
%         cone_params.mass = sym(1);
%         cone_params.gravity = sym(9.8);
%         cone_params.radius = sym(0.35);
%         cone_params.lateral_CM_offset = sym(0.15);
%         cone_params.vertical_CM_offset = sym(0.30);
%         cone_params.lateral_apex_offset = sym(0.35);
%         cone_params.vertical_apex_offset = sym(1.50);
    end
    
end
