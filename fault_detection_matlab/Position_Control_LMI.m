function controller = Position_Control_LMI(SYSTEM)

    eps  = 10e-8;

    Ap   =  SYSTEM.Ap;
    Bp   =  SYSTEM.Bp;

    n    =  size(Ap{1},1);
    m    =  size(Bp{1},2);

    X    =  sdpvar(n,n,'symmetric');
    N    =  sdpvar(m,n,'full');
    
    LMI  =  [];
    LMI  =  [LMI, X - eps*eye(size(X,1)) >= 0];

    for i = 1 : 2
        LMI = [LMI, X*Ap{i}' + N'*Bp{i}' + Ap{i}*X + Bp{i}*N + eps*eye(n,n) <= 0];  
    end

    Obj          =   [];
    options      =   sdpsettings('solver','sdpt3');
    LMI_SOLVE    =   optimize(LMI,Obj,options)

    X            =   value(X);
    N            =   value(N);
    controller   =   N*inv(X);
end
