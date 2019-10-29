function [A_x, b_x] = compute_X_LQR

    % initialize
    param = compute_controller_base_parameters;
    A = param.A;
    B = param.B;
    Q = param.Q;
    R = param.R;
    Xcons = param.Xcons;
    Ucons = param.Ucons;
    
    %% approximate control invariance by specifying control law
    K = -dlqr(A,B,Q,R);
    Xp = Polyhedron('A',[eye(3);-eye(3);K;-K],'b',[Xcons(:,2);-Xcons(:,1);Ucons(:,2);-Ucons(:,1)]);

    disp('Computing Invariant Set');
    systemLQR = LTISystem('A',A+B*K);
    systemLQR.x.with('setConstraint');
    systemLQR.x.setConstraint = Xp;
    InvSetLQR = systemLQR.invariantSet();
    disp('Found Invariant Set');
    
    %% plot invariant set
%     InvSetLQR.plot()
%     xlabel('x_1');
%     ylabel('x_2');
%     zlabel('x_3');
    
    %% return vectors
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
    
end

