clc;
clear all;

a = [3.1 2.9 3.8 3.3 2.7 3.0 2.8 2.5 2.6 2.0 3.2 2.4 2.3 3.1 2.1 3.4];
n1 = length(a);
b = [6.9 6.4 4.7 5.1 6.3 5.9 5.4 5.3 5.2 5.1 5.9 5.8 4.9];
n2 = length(b);

alpha = 0.05;

[h, p, ci, stats] = vartest2(a, b, alpha, 0);

fprintf("\n Part. a\n");

if h == 0
    fprintf("H0 is not rejected. The variances are the same");
else
    fprintf("H0 is rejected. The variances differ");
end

fprintf("\n\n");
fprintf("Observed value %f\n", stats.fstat); 
fprintf("Pvalue: %f\n", p);

q1 = finv(alpha, stats.df1, stats.df2);
q2 = finv(alpha, stats.df2, stats.df1);

fprintf("Rejection interval = (-inf, %f) U (%f, +inf)", q1, q2);

fprintf("\npart b\n");

tail = 1; % right tailed test

[h, p, ci, stats] = ttest2(a, b, alpha, tail, 'equal');

if h == 0
    fprintf("H0 is not rejected. Other employees do not dispose more than bank ppl");
else
    fprintf("H0 is rejected. Other employees dispose more than bank people");
end

q1 = tinv(alpha, stats.df);

fprintf("Pvalue: %f\n", p);
fprintf("\nRejection region is (%f, +inf)", q1);

