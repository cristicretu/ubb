clc
clear all
pause(1);

S=input("number of simulations=")

p = -1
while p < 0 | p > 1
    p=input("prob. of succ=")
end

n=input("number of trials");

X=zeros(1,S);

for i=1:S
    while rand>=p
        X(i)=X(i) + 1
    end
end


U=rand(n, S);
X=sum(U<p);
uX=unique(X);
nX=hist(X, length(uX));
rel_freq=nX/S;

plot(uX, rel_freq, 'x')
hold on;
plot(0:n, binopdf(0:n, n, p), 'o')
hold off;


