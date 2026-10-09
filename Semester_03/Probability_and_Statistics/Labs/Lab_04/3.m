clc
clear all
pause(1);

S = input('Number of simulations = ');
p = -1;
while p < 0 || p > 1
    p = input('Probability of success = ');
end
r = input('Number of successes needed = '); 

X = zeros(1,S);

for i = 1:S
    suc = 0;
    fail = 0;
    while suc < r
        if rand > p
            fail = fail + 1;
        else
            suc = suc + 1;
        end
    end
    X(i) = fail;
end

max_x = max(X);
xR = 0:max_x;
prob = nbinpdf(xR, r, p);

uX = unique(X);
nX = hist(X, length(uX));
rel_freq = nX/S;

figure;
plot(uX, rel_freq, 'x');
hold on;
plot(xR, prob, 'o');
hold off;
