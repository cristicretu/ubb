clc;
clear all;

standard = [46 37 39 48 47 44 35 31 44 37];
new = [35 33 31 35 34 30 27 32 31 31];

n1 = length(standard);
n2 = length(new);

standardMean = mean(standard);
newMean = mean(new);

standardStd = std(standard);
newStd = std(new);

standardVar = standardStd^2;
newVar = newStd^2;

significance = 0.05;

[h, p, ci, stats] = vartest2(standard, new, significance, 'both');
% h1: they differ
% h0: they are the same

fprintf("\nPart a.\n");

if h == 0
    fprintf("H0 is not rejected. The variances are the same.\n")
else
    fprintf("H0 is rtejected. The variances differ");
end

alpha = 0.05;

lb = finv(alpha/2, stats.df1, stats.df2);
rb = finv(1-alpha/2, stats.df2, stats.df1);

fprintf('\nP-value is %1.4f\n', p);
fprintf('\nRejection region R is (-inf, %3.4f) U (%3.4f, inf)\n', lb, rb);




diff_means = standardMean-newMean;
fprintf("\npoint b\n");
if h == 0 % same variances
    sp = sqrt((stats.df1 * standardVar + stats.df2 * newVar)/(stats.df1 + stats.df2));
    err = sp * sqrt(1/n1 + 1/n2);
    tval = tinv(1-alpha/2, stats.df1 + stats.df2);
    lb = diff_means - tval * err;
    rb = diff_means + tval * err;
    fprintf("Confidence interval is %f %f", lb, rb);
else
    c = (standardVar/n1)/(standardVar/n1+newVar/n2);
    n = 1/(c^2/stats.df1+(1-c)^2/stats.df2);
    err = sqrt(standardVar/n1+newVar/n2);
    tval = tinv(1-alpha/2, stats.df1 + stats.df2);
    lb = diff_means - tval * err;
    rb = diff_means + tval * err;
    fprintf("Confidence interval is %f %f", lb, rb);
end

[h, p, ci, stats] = ttest2(standard, new, significance, 'both');

ci

