function param = compute_controller_base_parameters

    % load truck parameters
    load('system/parameters_truck');

    %% (2) discretization
    Ts = 60;  % sampling time

    % continuous-time dynamics
    B_d_c = diag([1/truck.m1,1/truck.m2,1/truck.m3]);
    A_c = B_d_c * [-truck.a12-truck.a1o truck.a12 0;
                   truck.a12 -truck.a12-truck.a23-truck.a2o truck.a23;
                   0 truck.a23 -truck.a23-truck.a3o];
    B_c = B_d_c * truck.C_ref';
    d_c = [truck.w(1) + truck.a1o*truck.To;
           truck.w(2) + truck.a2o*truck.To;
           truck.w(3) + truck.a3o*truck.To];

    % euler discrete-time dynamics
    sys = ss(A_c,[B_c B_d_c],zeros(3),zeros(3,5));
    sysd = c2d(sys,Ts);
    A = sysd.A;
    B = sysd.B(:,1:2);
    B_d = sysd.B(:,3:5);
    d = d_c;

    %% (3) set point computation
    T_sp = [-20.0;0.25;6.1];
    temp = (eye(3) - A) * T_sp - B_d * d;
    p_sp = B(1:2,1:2) \ temp(1:2);

    %% (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;

    %% (4) constraints for delta formulation
    Ucons = Pcons - p_sp;
    Xcons = Tcons - T_sp;

    %% (5) LQR cost function
    Q = 50000 * diag([1,1,0]);
    R = eye(2);

    %% P_inf & F_inf
    [K,P] = dlqr(A,B,Q,R);
    F = -K;

    %% for further use to compute constraints
    Ax = [eye(3);-eye(3)];
    bx = [Xcons(:,2);-Xcons(:,1)];
    Au = [eye(2);-eye(2)];
    bu = [Ucons(:,2);-Ucons(:,1)];

    %% for off-set free MPC
    C = eye(3);
    C_d = zeros(3);
    H = [1 0 0;0 1 0];
    A_aug = [A B_d;zeros(3) eye(3)];
    B_aug = [B;zeros(3,2)];
    C_aug = [C zeros(3)];
    D_aug = zeros(3,2);

    %% for soft constraints
    S_soft = Q;
    v_soft = 1000000;
    P_soft = S_soft;

    %% for observer
    L = (place(A_aug',C_aug',[0.06,0.07,0.015,0.03,0.04,0.045]))';
    lambda = eig(A_aug - L * C_aug);
    paramT_aug = [];
    param.r = [-20;0.25];

    %% put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.B_d = B_d;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
    param.P = P;
    param.F = F;
    param.Ax = Ax;
    param.bx = bx;
    param.Au = Au;
    param.bu = bu;
    param.A_aug = A_aug;
    param.B_aug = B_aug;
    param.C_aug = C_aug;
    param.D_aug = D_aug;
    param.S_soft = S_soft;
    param.v_soft = v_soft;
    param.P_soft = P_soft;
    param.H = H;
    param.C = C;
    param.C_d = C_d;
    param.L = L;
    param.lambda = lambda;
    param.d = d;

end