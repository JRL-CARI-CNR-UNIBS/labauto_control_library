function regressor = regressorFcn(q1,Dq1,DDq1)
%regressorFcn
%    REGRESSOR = regressorFcn(Q1,Dq1,DDq1)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    12-Feb-2024 17:10:09

regressor = [DDq1,DDq1./4.0+cos(q1).*(9.81e+2./2.0e+2)];