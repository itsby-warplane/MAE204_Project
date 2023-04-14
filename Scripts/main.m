%% Note:
% these scripts and functions should be runned with the Modern Robotics library
%% clear
clear;
clc;

%% inputs definition
d_cube = 30.5; % cube size
h_standoff = 80; % standoff position
h_grasp = 20; % grasp position

Ki = 3*eye(6);
Kp = 8*eye(6);

dt = 0.01;

% The initial resting configuration of the cube frame
T_sc_init = [0, 0, -1, 450;...
             1, 0, 0, -300;...
             0, -1, 0, 0;...
             0, 0, 0, 1];
% The initial resting configuration of the cube frame (new task)
% T_sc_init = [0, 0, -1, -300;...
%              1, 0, 0,  100;...
%              0, -1, 0, 0;...
%              0, 0, 0, 1];

% The desired final resting configuration of the cube frame
T_sc_final = [-1, 0, 0, 0;...
              0, 0, -1, 100;...
              0, -1, 0, 0;...
              0, 0, 0, 1];

% The actual initial configuration of the UR3e
T_se_init = [0, 1, 0, 247;...
             1, 0, 0, -169;...
             0, 0, -1, 782;...
             0, 0, 0, 1];

T_ce_grasp = [1, 0, 0, 0;...
              0, 1, 0, -h_grasp;...
              0, 0, 1, 0;
              0, 0, 0, 1;];

T_ce_standoff = [1, 0, 0, 0;...
                 0, 1, 0, -h_standoff;...
                 0, 0, 1, 0;
                 0, 0, 0, 1];

%% generate trajectory
trajOutput = TrajectoryGenerator(T_se_init,  T_sc_init, T_sc_final, T_ce_grasp, T_ce_standoff);
csvwrite('trajectoryOutput.csv',trajOutput);

%% loop
[n,~] = size(trajOutput); % get number of steps

thetalist = [0; -pi/2;0;-pi/2;-pi/2;0]; % initial joint angles(reference)
M = [[1, 0, 0, 457]; [0, 1, 0, 78]; [0, 0, 1, 155]; [0, 0, 0, 1]]; % initial M
Slist = [[0; 0; 1; -300; 0; 0], ...
         [0; 1; 0; -240; 0; 0], ...
         [0; 1; 0; -240; 0; 244], ...
         [0; 1; 0; -240; 0; 457], ...
         [0; 0; -1; 169; 457; 0], ...
         [0; 1; 0; -155; 0; 457]];
T_se = FKinSpace(M, Slist, thetalist); % initial gripper configuration

Blist = [[0; 0;  1; -378; 457; 0   ], ...
         [0; 1;  0;  -85;   0; -457], ...
         [0; 1;  0;  -85;   0; -213], ...
         [0; 1;  0;  -85;   0; 0   ], ...
         [0; 0; -1;  247;   0; 0   ], ...
         [0; 1;  0;    0;   0; 0   ]];

% currentState = [0; -pi/2;0;-pi/2;-pi/2;0];
currentState = [pi/6; -pi/2; pi/2; -pi/2; -pi/2; pi/6]; % actual initial state 

allStates = [currentState',0];
allErrors = [0,0,0,0,0,0];
Verr_accu = 0;
for i = 1:n-1

    T_se = FKinSpace(M, Slist, currentState);
    T_se_d = row2T(trajOutput(i,:));
    T_se_d_next = row2T(trajOutput(i+1,:));

    [Vb,Verr_accu,Verr] = FeedbackControl(T_se, T_se_d, T_se_d_next, Kp, Ki, dt,Verr_accu);

    Jb = JacobianBody(Blist, currentState);
    Jb_T = pinv(Jb,0.01);
    jointVel = Jb_T*Vb;

    maxJointVel = 50*ones(6,1);

    NextState1 = NextState(currentState, jointVel, dt, maxJointVel);

    N = [NextState1',0];
    allStates = [allStates; N];
    
    E = Verr;
    allErrors = [allErrors;E'];

    currentState = NextState1;

end

allStates(301:702,7) = 1;

csvwrite('allstatesOutput.csv',allStates);

%% plot error
figure()

subplot(1,2,1)
for i = 1:3
    plot(allErrors(:,i))
    hold on   
end
xlabel('Time','FontSize',18)
ylabel('Twist (Angular) Error','FontSize',18)
ax = gca;
ax.FontSize = 15;
axis padded

subplot(1,2,2)
for i = 4:6
    plot(allErrors(:,i))
    hold on   
end
xlabel('Time','FontSize',18)
ylabel('Twist (Linear) Error','FontSize',18)
ax = gca;
ax.FontSize = 15;
axis padded

%% from rows in trajOutput to T
function T = row2T(T_V)
T = zeros(4);
T(4,:) = [0,0,0,1];
T(1,1) = T_V(1);
T(1,2) = T_V(2);
T(1,3) = T_V(3);
T(2,1) = T_V(4);
T(2,2) = T_V(5);
T(2,3) = T_V(6);
T(3,1) = T_V(7);
T(3,2) = T_V(8);
T(3,3) = T_V(9);
T(1,4) = T_V(10);
T(2,4) = T_V(11);
T(3,4) = T_V(12);
end
