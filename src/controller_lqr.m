function p = controller_lqr(T)
    % controller variables
    persistent param;

    % initialize controller, if not done already
    if isempty(param)
        param = init();
    end

    % compute control action
    x_0 = T - param.T_sp;
    p = param.F*x_0 + param.p_sp;
end

function param = init()
    param = compute_controller_base_parameters;
end