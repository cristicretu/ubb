clear all;
clc;

p = 0;
while p < 0.05 | p > 0.95
    p=input("prob. of success (0.05 <= p <= 0.95 = ")
end

for n=1:30
    k=0:n;
    px=binopdf(k, n, p);
    plot(k, px, '+');
    hold on

    mu = n * p;
    sigma=sqrt(n*p*(1-p));
    kk=0:0.01:n;
    normpx=normpdf(kk, mu, sigma);

    plot(kk, normpx);
    hold off
    title("Apro of bino model with normal dist model")
    legend("bino", "norm")
    pause(0.5);
end


