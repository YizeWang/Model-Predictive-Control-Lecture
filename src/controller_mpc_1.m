function p = controller_mpc_1(T)

    % controller variables
    persistent param yalmip_optimizer

    % initialize controller, if not done already
    if isempty(param)
        [param, yalmip_optimizer] = init();
    end

    %% evaluate control action by solving MPC problem
    [u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp);
    if (errorcode ~= 0)
        warning('MPC infeasible');
    end
    p = u_mpc + param.p_sp;
    
end

function [param, yalmip_optimizer] = init()

    % initialize
    param = compute_controller_base_parameters;

    %% implementation of MPC
    N = 31;
    nx = size(param.A,1);
    nu = size(param.B,2);

    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');

    objective = 0;
    constraints = [];

    for i = 1:N-1
        constraints = [constraints, X{i+1} == param.A * X{i} + param.B * U{i}];
        constraints = [constraints, param.Ax * X{i+1} <= param.bx];
        constraints = [constraints, param.Au * U{ i } <= param.bu];
        objective = objective + X{i}' * param.Q * X{i} + U{i}' * param.R * U{i};
    end
    objective = objective + X{end}' * param.P * X{end};
    x0 = sdpvar(3,1);
    constraints = [constraints, X{1} == x0];

    % generate yalmip optimizer object
    ops = sdpsettings('verbose',0,'solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1});

end