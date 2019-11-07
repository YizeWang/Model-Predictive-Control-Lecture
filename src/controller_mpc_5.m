function p = controller_mpc_5(T)

    % controller variables
    persistent param yalmip_optimizer

    % initialize controller, if not done already
    if isempty(param)
        [param,yalmip_optimizer] = init();
        param.T_aug = [T;param.d];        
    end
    
    % compute steady state
    [T_sp,P_sp] = computeSP(param);
    
    %% evaluate control action by solving MPC problem
    [p_mpc,errorcode] = yalmip_optimizer( [param.T_aug ; T_sp ; P_sp] );
    if (errorcode ~= 0)
        warning('MPC infeasible');
    end
    p = p_mpc;

    %% store estimated state
    param.T_aug = estimate(param,p,T);
end

function [param, yalmip_optimizer] = init()

    % initialize
    param = compute_controller_base_parameters;
    
    % initialize
    [A_x, b_x] = compute_X_LQR;

    %% implement your MPC using Yalmip
    N = 31;
    nx = size(param.A,1);
    nu = size(param.B,2);

    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    
    xs = sdpvar(nx,1);
    us = sdpvar(nu,1);
    
    T_aug = sdpvar(nx+nx,1);

    objective = 0;
    constraints = [];

    for i = 1:N-1
        constraints = [constraints, X{i+1} == param.A * X{i} + param.B * U{i} + param.B_d * T_aug(4:6)];
        constraints = [constraints, param.Tcons(:,1) <= X{i+1} <= param.Tcons(:,2)];
        constraints = [constraints, param.Pcons(:,1) <= U{ i } <= param.Pcons(:,2)];
        objective = objective + (X{i} - xs)' * param.Q *(X{i} - xs) + (U{i}-us)' * param.R * (U{i}-us);
    end
    constraints = [constraints, A_x * (X{end} - xs) <= b_x];
    objective = objective + (X{end}-xs)' * param.P * (X{end}-xs);
    constraints = [constraints, X{1} == T_aug(1:3)];

    % generate yalmip optimizer object
    ops = sdpsettings('verbose',0,'solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops,[T_aug;xs;us],U{1});
    
end

function T_aug = estimate(param,p,T)

    T_aug = param.A_aug * param.T_aug + param.B_aug * p + ...
            param.L * ( T - param.C_aug * param.T_aug );

end

function [T_sp,P_sp] = computeSP(param)

    TR = [-param.B_d * param.T_aug(4:6); param.r];
    TL = [param.A - eye(3) param.B; param.H * param.C zeros(2)];
    SP = inv(TL) * TR;
    T_sp = SP(1:3);
    P_sp = SP(4:5);
    
end