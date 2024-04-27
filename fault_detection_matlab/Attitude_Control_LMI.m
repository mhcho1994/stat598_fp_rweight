function controller = Attitude_Control_LMI(SYSTEM)

    eps  = 10e-8;

    Aa   =  SYSTEM.Aa;
    Ba   =  SYSTEM.Ba;

    n    =  size(Aa{1},1);
    m    =  size(Ba,2);

    X    =  sdpvar(n,n,'symmetric');
    N    =  sdpvar(m,n,'full');
    
    LMI  =  [];
    LMI  =  [LMI, X - eps*eye(size(X,1)) >= 0];

    for i = 1 : 4
        LMI = [LMI, X*Aa{i}' + N'*Ba' + Aa{i}*X + Ba*N + eps*eye(n,n) <= 0];  
    end

    Obj          =   [];
    options      =   sdpsettings('solver','sdpt3');
    LMI_SOLVE    =   optimize(LMI,Obj,options)

    X            =   value(X);
    N            =   value(N);
    controller   =   N*inv(X);
end
