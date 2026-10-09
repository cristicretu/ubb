clc;
clear all;

a = [3.26 1.89 2.42 2.03 3.07 2.95 1.39 3.06 2.46 3.35 1.56 1.79 1.76 3.82 2.42 2.96];

alpha = 0.05;

[h, p, ci, stats] = ttest(a, alpha);

% test for manual calculations

aMean = mean(a)
t = tinv(1-alpha/2, length(a) - 1);

lb = aMean - std(a)/sqrt(length(a)) * t;
rb = aMean + std(a)/sqrt(length(a)) * t;

fprintf("\nPart A.\n");
fprintf('95%% Confidence Interval: [%f, %f]\n', ci(1), ci(2));

fprintf("\nPart B.\n");


alpha = 0.01;
tail = -1; % left tailed test

[h, p] = ttest(a, 3, 'Alpha', alpha, 'Tail', 'left');

if h == 0
    fprintf("H0 is not rejected. On average the nickel particles aare not smaller than 3.")
else
    fprintf("H0 is rejected. On average, nickled particles are smaller than 3.")
end

fprintf('\np-value: %f\n', p);

aStd = std(a);

hypothesized_mean = 3;
std_error = aStd / sqrt(length(a));
t_stat = (aMean - hypothesized_mean) / std_error;

df = length(a) - 1;
critical_value = tinv(alpha, df);

p_value = tcdf(t_stat, df)

if t_stat < critical_value
    fprintf("H0 is rejected.")
else
    fprintf("H0 is not rejected")
end


