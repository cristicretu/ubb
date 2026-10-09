clc;
clear all;
pause(1);

premium = [22.4 24.5 21.6 22.4 24.8 21.7 23.4 23.3 21.6 20.0];
regular = [17.7 19.6 12.1 15.4 14.0 14.8 19.6 14.8 12.6 12.2];

mean_premium = mean(premium);
mean_regular = mean(regular);

n1 = length(premium)
n2 = length(regular)

diff_means = mean_premium - mean_regular


df = n1 + n2 - 2; % degrees of freedom

alpha = 0.05; % 95 confidence interval
t = tinv(1-alpha/2, df);

ss1 = (std(premium)^2) * (n1 - 1);
ss2 = (std(regular)^2) * (n2 - 1);

% pooled variance, with equal variances)
sp = sqrt((ss1+ss2)/(n1 + n2 - 2));

% standard error of the difference between means
se_diff = sp * sqrt(1/n1 + 1/n2);

margin_error = t * se_diff;
ci_lower = diff_means - margin_error;
ci_upper = diff_means + margin_error;

fprintf('95%% Confidence Interval: (%.4f, %.4f)\n', ci_lower, ci_upper);

% -------------- for diff variances

var1 = var(premium);
var2 = var(regular);

df2 = ((var1/n1 + var2/n2)^2) / ((var1^2/(n1^2*(n1-1))) + (var2^2/(n2^2*(n2-1))));
se_diff2 = sqrt(var1/n1 + var2/n2);


t = tinv(1-alpha/2, df2);
margin_error = t * se_diff2;

ci_lower = diff_means - margin_error;
ci_upper = diff_means + margin_error;

fprintf('95%% Confidence Interval: (%.4f, %.4f)\n', ci_lower, ci_upper);

% ------------- for F values


df1 = n1 - 1;
df2 = n2 - 1;

F_ratio = var1/var2;

F_lower = finv(alpha/2, df1, df2);
F_upper = finv(1-alpha/2, df1, df2);

ci_lower = F_ratio/F_upper;
ci_upper = F_ratio/F_lower;

fprintf('95%% Confidence Interval: (%.4f, %.4f)\n', ci_lower, ci_upper);

 
