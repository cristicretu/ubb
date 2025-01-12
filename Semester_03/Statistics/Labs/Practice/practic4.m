% a food store owner receives 1-liter bottles from 2 suppliers. after some
% complains from the customers, he wants to check the accuracy of the
% bottles
% he finds the following weights
% X1 = [1021, 980, 1017, 988, 1005, 998, 1014, 985, 995, 1004, 1030, 1015, 995, 1023];
% X2 = [1070, 970, 993, 1013, 1006, 1002, 1014, 997, 1002, 1010,  975];
% a) at 5% significance level, do the populations seem to differ?
% b) at the same significance level, on average, does supplier A seem to be
% more reliable?
clear All
clc
%h0: sigma1 = sigma2
%h1: sigma1 != sigma 2 - two tailed test

a = [1021 980 1017 988 1005 998 1014 985 995 1004 1030 1015 995 1023];
b = [1070 970 993 1013 1006 1002 1014 997 1002 1010 975];

alpha = 0.05;

[h, p, ci, stats] = vartest2(a, b, alpha, 0);

fprintf('POINT a.\n');

if h == 0
        fprintf('H0 is not rejected, sigmas are equal!\n');
else
        fprintf('H0 is rejected, population variances differ \n');
end

q1 = finv(alpha/2, stats.df1, stats.df2);
q2 = finv(alpha/2, stats.df2, stats.df1);

fprintf("Observed value %f\n", stats.fstat);
fprintf('P-value is %f\n', p);
fprintf('Rejection region R is (-inf, %f) U (%f, inf)\n', q1, q2);


[h1, p1, ci1, stats1] = ttest2(a, b, alpha, 1, 'equal');

fprintf('\nPOINT b.\n');


if h1 == 0
    fprintf("H0 is not rejected. A is not more reliable than b.\n");
else
    fprintf("H0 is rejected. A is more reliable than B");
end

q1 = tinv(1-alpha, stats1.df)

fprintf('Rejection region R is (%f, +inf)\n', q1)



