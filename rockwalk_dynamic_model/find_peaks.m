function [tp,sp] = find_peaks(t,y)
    
    dat = y(:,5);
    
    n = size(dat,1);
    
    tp = [t(1)];
    sp = [dat(1)];
    
    for i=3:n-3
        v1 = dat(i-2);
        v2 = dat(i-1);
        v3 = dat(i);
        v4 = dat(i+1);
        v5 = dat(i+2);
    
        if v1 <= v2 & v2 <= v3 & v3 >= v4 & v4 >= v5
            tp = [tp;t(i)];
            sp = [sp;dat(i)];
        end
        
        if v1 >= v2 & v2 >= v3 & v3 <= v4 & v4 <= v5
            tp = [tp;t(i)];
            sp = [sp;dat(i)];
        end
        
    end
    
end