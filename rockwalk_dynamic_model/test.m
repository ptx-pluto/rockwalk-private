colors = gen_color_profile(6);

plot_multi_color(results,colors);


%plot_full_state_multi(results);


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
