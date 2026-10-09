clc
clear all
pause(1);


x=[20,20,21,22,22,22,23*ones(1,6),24*ones(1,5),25*ones(1,9),26,26,27, 27];
y=[75,75,75,76,76,77,77,78*ones(1,5),79*ones(1,8),80*ones(1,8), 81,82];
length(x)
xMean=mean(x)
yMean=mean(y)

varX=var(x,1)
varY=var(y,1)
stdVarX=sqrt(varX)
stdVarY=sqrt(varY)
