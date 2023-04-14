function trajOutput = TrajectoryGenerator...
    (T_se_init,  T_sc_init, T_sc_final, T_ce_grasp, T_ce_standoff )

trajOutput = [];
dt = 0.01;
method = 5;
% 1. Move the gripper from its initial configuration to a “standoff” 
% configuration a few cm above the block.
N = 200;
Tf = N*dt;
Xstart = T_se_init;
Xend = T_sc_init*T_ce_standoff;
traj = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
for i = 1:N
    tt = traj{i};
    trajVec = [tt(1,1),tt(1,2),tt(1,3),tt(2,1),tt(2,2),tt(2,3),...
               tt(3,1),tt(3,2),tt(3,3),tt(1,4),tt(2,4),tt(3,4),...
               0];
    trajOutput = [trajOutput; trajVec];
end

% 2. Move the gripper down to the grasp position.
N = 100;
Tf = N*dt;
Xstart = traj{end};
Xend = T_sc_init*T_ce_grasp;
traj = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
for i = 1:N
    tt = traj{i};
    trajVec = [tt(1,1),tt(1,2),tt(1,3),tt(2,1),tt(2,2),tt(2,3),...
               tt(3,1),tt(3,2),tt(3,3),tt(1,4),tt(2,4),tt(3,4),...
               0];
    trajOutput = [trajOutput; trajVec];
end

% 3. Close the gripper.
trajVec = trajOutput(end,:);
trajVec(end) = 1;
trajOutput = [trajOutput; trajVec];

% 4. Move the gripper back to the "standoff" configuration
N = 100;
Tf = N*dt;
Xstart = traj{end};
Xend = T_sc_init*T_ce_standoff;
traj = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
for i = 1:N
    tt = traj{i};
    trajVec = [tt(1,1),tt(1,2),tt(1,3),tt(2,1),tt(2,2),tt(2,3),...
               tt(3,1),tt(3,2),tt(3,3),tt(1,4),tt(2,4),tt(3,4),...
               1];
    trajOutput = [trajOutput; trajVec];
end

% 5. Move the gripper to a “standoff” configuration above the final configuration.
N = 200;
Tf = N*dt;
Xstart = traj{end};
Xend = T_sc_final*T_ce_standoff;
traj = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
for i = 1:N
    tt = traj{i};
    trajVec = [tt(1,1),tt(1,2),tt(1,3),tt(2,1),tt(2,2),tt(2,3),...
               tt(3,1),tt(3,2),tt(3,3),tt(1,4),tt(2,4),tt(3,4),...
               1];
    trajOutput = [trajOutput; trajVec];
end

% 6. Move the gripper to the final configuration of the object.
N = 100;
Tf = N*dt;
Xstart = traj{end};
Xend = T_sc_final*T_ce_grasp;
traj = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
for i = 1:N
    tt = traj{i};
    trajVec = [tt(1,1),tt(1,2),tt(1,3),tt(2,1),tt(2,2),tt(2,3),...
               tt(3,1),tt(3,2),tt(3,3),tt(1,4),tt(2,4),tt(3,4),...
               1];
    trajOutput = [trajOutput; trajVec];
end

% 7. Open the gripper.
trajVec = trajOutput(end,:);
trajVec(end) = 0;
trajOutput = [trajOutput; trajVec];

% 8. Move the gripper back to the “standoff” configuration
N = 100;
Tf = N*dt;
Xstart = traj{end};
Xend = T_sc_final*T_ce_standoff;
traj = ScrewTrajectory(Xstart, Xend, Tf, N, method);
% traj = CartesianTrajectory(Xstart, Xend, Tf, N, method);
for i = 1:N
    tt = traj{i};
    trajVec = [tt(1,1),tt(1,2),tt(1,3),tt(2,1),tt(2,2),tt(2,3),...
               tt(3,1),tt(3,2),tt(3,3),tt(1,4),tt(2,4),tt(3,4),...
               0];
    trajOutput = [trajOutput; trajVec];
end

trajOutput(301:702,end) = 1; % set gripper state

end
