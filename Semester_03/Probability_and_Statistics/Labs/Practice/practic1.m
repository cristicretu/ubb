clc;
clear all;

significance = 0.05

steel = [4.6 0.7 4.2 1.9 4.8 6.1 4.7 5.5 5.4];
glass = [2.5 1.3 2.0 1.8 2.7 3.2 3.0 3.5 3.4];

[h, p, ci, stats] = vartest2(steel, glass, significance, 0);

fprintf("part. A\n")

if h == 0
    fprintf("H0 is not rejected. The variances do not differ.\n")
else
    fprintf("H0 is rejected. The variances differ\n")
end



q2 = finv(1-significance/2, stats.df1, stats.df2);
q1 = finv(significance/2, stats.df2, stats.df1);

fprintf('P-value is %1.4f\n', p);
fprintf("the rejection region is (-inf, %f) U (%f, +inf)", q1, q2)


fprintf("\npart. B\n")

[h, p, ci, stats] = ttest2(steel, glass, significance, 1, 'equal');

if h == 0
      fprintf('H0 is not rejected. Steel pipes do NOT lose more heat than glass.\n')
else
      fprintf('H0 is rejected. Steel pipes DO lose more heat than glass pipes.\n')
end

fprintf('P-value of the test statistic is %e.\n', p)

q1 = tinv(1 - significance, stats.df);

fprintf('Rejection region R is (%3.4f, +inf)\n', q1)
fprintf('\n\n\n\n');
