function [Vb,Verr_accu,Verr] = FeedbackControl(T_se, T_se_d, T_se_d_next, Kp, Ki, dt,Verr_accu)

Vd_M = TransInv(T_se_d)*T_se_d_next;
% Vd = zeros(6,1);
% Vd = [Vd_M(3,2); Vd_M(1,3); Vd_M(2,1); Vd_M(1,4); Vd_M(2,4); Vd_M(3,4)];
Vd = se3ToVec(MatrixLog6(Vd_M));
Vd = Vd/dt;

Verr_M = TransInv(T_se)*T_se_d;
% Verr = zeros(6,1);
% Verr = [Verr_M(3,2); Verr_M(1,3); Verr_M(2,1); Verr_M(1,4); Verr_M(2,4); Verr_M(3,4)];
Verr = se3ToVec(MatrixLog6(Verr_M));

AdT = Adjoint(TransInv(T_se)*T_se_d);

Verr_accu = Verr_accu + Verr;
Vb = AdT*Vd + Kp*Verr + Ki*Verr_accu*dt;

end
