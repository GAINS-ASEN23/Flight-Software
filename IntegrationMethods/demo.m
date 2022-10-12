clc; close all; clear all;
n = 100;
k = zeros(1,n);
for i = 1:n
    j = 1;
    while j <= i
        k(i) = k(i)+1;
        j = j*3;
    
    end
end

plot(1:n, k)

