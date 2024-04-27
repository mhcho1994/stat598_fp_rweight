function GAIN1 = Attitude_Estimator_LMI(SYSTEM)

    eps  = 10e-8;

    for i = 1 : 4
        block{i}       =  blkvar;
        N{i}           =  sdpvar(15,6,'full');
    end

    P                  =  sdpvar(15,15,'symmetric');
    gamma              =  15;

    bar_A              =  SYSTEM.bar_A;
    bar_F              =  SYSTEM.bar_F;
    bar_C              =  SYSTEM.bar_C;
    bar_H              =  SYSTEM.bar_H;

    for i = 1 : 4
        block{i}(1,1)  = bar_A{i}'*P - bar_C'*N{i}' + P*bar_A{i} - N{i}*bar_C + eye(15,15);
        block{i}(2,1)  = bar_F'*P - bar_H'*N{i}';
        block{i}(2,2)  = -gamma*eye(12,12); 
    end
    
    LMI1  =   [];                                              
    LMI1  =   [LMI1, P - eps*eye(size(P,1)) >= 0];
    
    for i = 1 : 4
        LMI1  =  [LMI1, block{i} + eps*eye(27,27) <= 0];  
    end
    
    %% Solve & Return
    Obj          =   [];
    options      =   sdpsettings('solver','sdpt3');
    LMI_SOLVE1   =   optimize(LMI1,Obj,options)

    P            =   value(P);
    gamma        =   value(gamma);

    for i = 1 : 4
        N{i} = value(N{i});
        K{i} = inv(P)*N{i};
    end
    
    GAIN1{1}     =  K{1};
    GAIN1{2}     =  K{2};
    GAIN1{3}     =  K{3};
    GAIN1{4}     =  K{4};

end
