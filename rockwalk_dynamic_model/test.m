load scaled_statue.mat

[ts,ps] = find_peaks(t,y);

plot(t,y(:,5));
hold on;
plot(ts,ps,'o');

disp(ts(3));