clc;
clear all;

steel = [4.6 0.7 4.2 1.9 4.8 6.1 4.7 5.5 5.4];
glass = [2.5 1.3 2.0 1.8 2.7 3.2 3.0 3.5 3.4];

significance = 0.05;

fprintf("\nPart a.\n");

fprintf("Do the populations variances seem to differ?\n");
fprintf("(Q1) => H1 (alternative hypothesis) is that Vsteel != VGlass\n");
fprintf("(Q2) => H0 (null hypothesis) is that Vsteel = VGlass\n");

tail = 0; % because we're checking both sides

[h, p, ~, stats] = vartest2(steel, glass, significance, tail);

if h == 0
    fprintf("(Q3) H0 is not rejected.\n");
    fprintf("(Q4) The variances are the same. (for the 5 percent significance level)\n");
else
    fprintf("(Q3) H0 is rejected.\n");
    fprintf("(Q4) The variances are different. (for the 5 percent significance level)\n")
end

fprintf("(Q5) Using a F test\n")

% two tailed test so we need two quintiles
q2 = finv(1-significance/2, stats.df1, stats.df2);
q1 = finv(significance/2, stats.df2, stats.df1);

fprintf("(Q6) the rejection region is (-inf, %f) U (%f, +inf)\n", q1, q2)
fprintf('(Q7) P-value is %1.4f\n', p);
fprintf("(Q8) Observed value is %f\n", stats.fstat)
fprintf("(Q9) The rejection region tells us to reject the null hypothesis if the P value lies within the insides of the interval. \n The P-value also tells to reject the hypothesis or not, based also on the significance level. \nIf P-value was bigger then significance (0.05) then we would have not rejected H0. But since P-value is %f, which is smaller than our significance, then we reject H0\n", p)

fprintf("\n\nPart b.\n");

% significance is the same so 0.05
% steel lose more so steel > glass, so we use a right tailed test
tail = 1;

fprintf("Does it seem that, on average, steel pipes lose more heat than glass pipes?\n");
fprintf("(Q1) => H1 (alternative hypothesis) is that Avg(Steel) > Avg(Glass)\n");
fprintf("(Q2) => H0 (null hypothesis) is that Avg(steel) < Avg(glass)\n");

[h, p, ci, stats] = ttest2(steel, glass, significance, tail, 'unequal');


if h == 0
    fprintf("(Q3) H0 is not rejected.\n");
    fprintf("(Q4) Steel pipes DO NOT lose on average more heat than glass pipes. (for the 5 percent significance level)\n");
else
    fprintf("(Q3) H0 is rejected.\n");
    fprintf("(Q4) Steel pipes DO lose on average more heat than glass pipes. (for the 5 percent significance level)\n")
end

fprintf("(Q5) Using a T (student) test\n");

% since we has a right tailed test, the interval of rejection will be
% quintile, + innf
q1 = tinv(1 - significance, stats.df);

fprintf("(Q6) the rejection region is (%f, +inf)\n", q1)
fprintf('(Q7) P-value is %1.4f\n', p);
fprintf("(Q8) Observed value is %f\n", stats.tstat)
fprintf("(Q9) The rejection region tells us to reject the null hypothesis if the P value lies within the insides of the interval. \n If the P-value was very small and close to 0, then we reject the null hypothesis.  \nIf P-value was bigger then significance (0.05) then we would have not rejected H0. But since P-value is %f, which is smaller than our significance, then we reject H0\n", p)
