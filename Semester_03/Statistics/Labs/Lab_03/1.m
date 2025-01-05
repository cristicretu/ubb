clear all;
clc;

opt=input("What model do you want to use brother? (Normal, Student, Chi2, Fisher):", 's')
alfa=input("alfa=")
beta=input("beta=")

switch opt
    case 'Normal'
        fprintf("Normal dist\n")
        miu=input("μ=");
        sigma=input("σ=");
        cdf=normcdf(0, miu, sigma);
        cdf2=1-cdf;

        cdf3 = normcdf(1, miu, sigma);
        cdf4 = normcdf(-1, miu, sigma);
        fprintf("a)\nP(x<=0)=%f\nP(x>=0)=%f\n----------\n", cdf, cdf2)
        fprintf("b)\nP(-1<=X<=1)=%f\nP(X<=-1 || X>= 1)=%f\n-----\n", cdf3 - cdf4, 1 - cdf3 + cdf4)
        quintile1=norminv(alfa, miu, sigma)
        quintile2=norminv(1-beta, miu, sigma)
        fprintf("c)\nP(X < X_alfa) = Alfa => X_alfa=%f\n-----\n", quintile1)
        fprintf("d)\nP(X > X_beta) = Beta => X_Beta=%f\n-----\n", quintile2)

    case 'Student'
         fprintf("Student dist\n")
         n=input("n=");
         cdf=tcdf(0, n);
         cdf2=1-cdf;
        fprintf("a)\nP(x<=0)=%f\nP(x>=0)=%f", cdf, cdf2)
    case 'Chi2'
        fprintf("Chi2 dist\n")
        n=input("n=");
        cdf=chi2cdf(0, n);
        cdf2 = 1 - cdf
        fprintf("a)\nP(x<=0)=%f\nP(x>=0)=%f", cdf, cdf2)
    case 'Fisher'
        fprintf("Fisher dist\n")
        m=input("m=");
        n=input("n=");
        cdf=fcdf(0, m, n);
        cdf2 = 1 -cdf
        fprintf("a)\nP(x<=0)=%f\nP(x>=0)=%f", cdf, cdf2)
    otherwise
        fprintf("Wrong option brother!\n")
end

