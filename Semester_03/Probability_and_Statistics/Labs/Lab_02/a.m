k=0:3;

px=binopdf(k, 3, 0.5);

A=[k; px]

kk=0:.01:3;

fx=binocdf(kk, 3, 0.5);

% plot(kk, fx)

title("Binomial model")
legend('CDF')

fprintf('P(X=0) = %.3f\n', binopdf(0, 3, 0.5))
fprintf('P(X!=1) = %.3f\n', 1 - binopdf(1, 3, 0.5))
fprintf('P(X<=2) = %.3f\n', binocdf(2, 3, 0.5))
fprintf('P(X<2) = %.3f\n', binocdf(2, 3, 0.5) - binopdf(2,3,0.5))
fprintf('P(X>=1) = %.3f\n', 1 - binocdf(1, 3, 0.5) + binopdf(1,3, 0.5))
fprintf('P(X>1) = %.3f\n', 1 - binocdf(1, 3, 0.5))
