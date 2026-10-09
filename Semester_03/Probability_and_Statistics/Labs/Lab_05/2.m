clc
clear all
pause(1);

x=[7,7,4,5,9,9,4,12,8,1,8,7,3,13,2,1,17,7,12,5,6,2,1,13,14,10,2,4,9,11,3,5,12,6,10,7];

meanX=mean(x)
length(x)
confidence=input("Enter confidence (0, 1)")


sigma=5
alpha=1-confidence

lb=meanX-sigma/sqrt(length(x))*norminv(1-alpha/2, 0, 1)
rb=meanX-sigma/sqrt(length(x))*norminv(alpha/2, 0, 1)

fprintf("the %.2f CI for the average buttary duration is (%.2f, %.2f)\n", confidence*100, lb, rb);

lb1=meanX-std(x)/sqrt(length(x))*tinv(1-alpha/2,length(x)-1)
rb1=meanX-std(x)/sqrt(length(x))*tinv(alpha/2,length(x)-1)

fprintf("the %.2f CI for the average buttary duration is (%.2f, %.2f)\n", confidence*100, lb1, rb1);

n=length(x)
lb2=(n-1)*var(x)/chi2inv(1-alpha/2,n-1)
rb2=(n-1)*var(x)/chi2inv(alpha/2,n-1)

fprintf("sigma^2 is from (%.2f,%.2f)", lb2, rb2)
fprintf("sigma is from (%.2f, %.2f)", sqrt(lb2), sqrt(rb2))


