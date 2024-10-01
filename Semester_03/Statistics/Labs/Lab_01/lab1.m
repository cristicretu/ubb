% Problem 1
A=[1 0 -2; 2 1 3; 0 1 0];
B=[2 1 1; 1 0 -1; 1 1 0];
C=A-B
D=A*B
E=A.*B

% Problem 2
clf
x=0:.01:3;
y1=x.^5 / 10;
y2=x.*sin(x)
plot(x, y1, '--r', x, y2, ':m')

