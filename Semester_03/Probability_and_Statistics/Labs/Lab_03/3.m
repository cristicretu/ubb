clear all;
clc;

p = 1;
while p > 0.05
    p=input("prob. of success (0.05 <= p = ")
end

for n=30:35
    k=0:n;

    px=poisspdf(k, n * p);
    plot(k, px, 'o');    
    hold on

    pxx=binopdf(k, n, p)
        plot(k, px, '*');    
        hold off

    title("Apro of bino model with normal dist model")
    legend("poiss")
    pause(0.5);
end


