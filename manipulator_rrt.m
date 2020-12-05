kin = loadrobot('kinovaJacoJ2S7S300');
floor = collisionBox(1, 1, 0.01);
tabletop1 = collisionBox(0.4,1,0.02);...0.4,1,0.02
tabletop1.Pose = trvec2tform([0.3,0,0.6]);...0.3,0,0.6
tabletop2 = collisionBox(0.6,0.2,0.02);...0.6,0.2,0.02
tabletop2.Pose = trvec2tform([-0.2,0.4,0.5]);...-0.2,0.4,0.5
can = collisionCylinder(0.03,0.16);
can1 = collisionCylinder(0.03,0.16);
can2 = collisionCylinder(0.03,0.16);
can3 = collisionCylinder(0.03,0.16);
can4 = collisionCylinder(0.03,0.16);
can5 = collisionCylinder(0.03,0.16);
...can6 = collisionCylinder(0.03,0.16);
can.Pose = trvec2tform([0.3,0.0,0.7]);
can1.Pose = trvec2tform([0.3,-0.1,0.7]);
can2.Pose = trvec2tform([0.3,-0.2,0.7]);
can3.Pose = trvec2tform([0.4,-0.2,0.7]);
can4.Pose = trvec2tform([0.4,-0.1,0.7]);
can5.Pose = trvec2tform([0.4,0,0.7]);
...can6.Pose = trvec2tform([0.4,-0.0,0.7]);

% Create state space and set workspace goal regions (WGRs)
ss = ExampleHelperRigidBodyTreeStateSpace(kin);
ss.EndEffector = 'j2s7s300_end_effector';

% Define the workspace goal region (WGR)
% This WGR tells the planner that the can shall be grasped from
% the side and the actual grasp height may wiggle at most 1 cm.

% This is the orientation offset between the end-effector in grasping pose and the can frame
R = [0 0 1; 1 0 0; 0 1 0]; 

Tw_0 = can.Pose;
Te_w = rotm2tform(R);
bounds = [0 0;       % x
          0 0;       % y
          0 0.01;    % z
          0 0;       % R
          0 0;       % P
         -pi pi];    % Y
setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);

sv = ExampleHelperValidatorRigidBodyTree(ss);

% Add obstacles in the environment
addFixedObstacle(sv,tabletop1, 'tabletop1', [1,0.5,0]);
addFixedObstacle(sv,tabletop2, 'tabletop2', [1,0.5,0]);
addFixedObstacle(sv,can, 'can', 'r');
addFixedObstacle(sv,can1, 'can1', 'r');
addFixedObstacle(sv,can2, 'can2', 'r');
addFixedObstacle(sv,can3, 'can3', 'r');
addFixedObstacle(sv,can4, 'can4', 'r');
addFixedObstacle(sv,can5, 'can5', 'r');
...addFixedObstacle(sv,can6, 'can6', 'r');
addFixedObstacle(sv,floor, 'floor', [245 245 220]/256);

% Skip collision checking for certain bodies for performance
skipCollisionCheck(sv,'root'); % root will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_link_base'); % base will never touch any obstacles
skipCollisionCheck(sv,'j2s7s300_end_effector'); % this is a virtual frame

% Set the validation distance
sv.ValidationDistance = 0.01;

% Set random seeds for repeatable results
rng(0,'twister') % 0

% Compute the reference goal configuration. Note this is applicable only when goal bias is larger than 0. 
Te_0ref = Tw_0*Te_w; % Reference end-effector pose in world coordinates, derived from WGR
ik = inverseKinematics('RigidBodyTree',kin);
refGoalConfig = ik(ss.EndEffector,Te_0ref,ones(1,6),homeConfiguration(ss.RigidBodyTree));

% Compute the initial configuration (end-effector is initially under the table)
T = Te_0ref;
T(1,4) = 0.3;%T(1,4) = 0.3;
T(2,4) = 0.2;%T(2,4) = 0.2;
T(3,4) = 0.5;%T(3,4) = 0.5;
initConfig = ik(ss.EndEffector,T,ones(1,6),homeConfiguration(ss.RigidBodyTree));

tic
% Create the planner from previously created state space and state validator
planner = plannerRRT(ss,sv);

% If a node in the tree falls in the WGR, a path is considered found.
planner.GoalReachedFcn = @exampleHelperIsStateInWorkspaceGoalRegion;

% Set the max connection distance.
planner.MaxConnectionDistance = 0.5;

% With WGR, there is no need to specify a particular goal configuration (use
% initConfig to hold the place).
% As a result, one can set GoalBias to zero.
planner.GoalBias = 0;
[pthObj,solnInfo] = plan(planner,initConfig, initConfig);


% Smooth the path.
interpolate(pthObj,100);
newPathObj = exampleHelperPathSmoothing(pthObj,sv);
interpolate(newPathObj,200);

figure
states = newPathObj.States;

% Draw the robot.
ax = show(kin,states(1,:));
zlim(ax, [-0.03, 1.4])
xlim(ax, [-1, 1])
ylim(ax, [-1, 1])


% Render the environment.
hold on
showObstacles(sv, ax);
view(146, 33)
camzoom(1.5)
...여기두
% Show the motion.
for i = 2:length(states)
    show(kin,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end


q = states(i,:);

% Gab the can.
q = exampleHelperEndEffectorGrab(sv,'can5',q, ax);

toc
targetPos = [-0.15,0.35,0.51];
exampleHelperDrawHorizontalCircle(targetPos,0.02,'y',ax);

...여기 주석 풀어야함
%Plan the Move motion
Tw_0 = trvec2tform(targetPos+[0,0,0.08]); 
Te_w = rotm2tform(R);
bounds =  [0 0;       % x
           0 0;       % y
           0 0;       % z
           0 0;       % R
           0 0;       % P
          -pi pi];    % Y
setWorkspaceGoalRegion(ss,Tw_0,Te_w,bounds);
ss.UseConstrainedSampling = true;
planner.MaxConnectionDistance = 0.05;
[pthObj2,~] = plan(planner,q,q);

states = pthObj2.States;

view(ax, 152,45)
for i = 2:length(states)
    show(kin,states(i,:),'PreservePlot',false,'Frames','off','Parent',ax);
    drawnow
end
q = states(i,:);

...************추가한 obstacle***********
can6 = collisionCylinder(0.03,0.16);
can6.Pose = trvec2tform([-0.2,0.4,0.5]);
addFixedObstacle(sv,can6, 'can6', 'r');
...**************************************

...*******************추가부분****************
%여기에 collision개수를 세는 함수를 만든다
%ExampleHelperValidatorRigidBodyTree.m의 isvalid function을 참고
%저기서 obj=sv

m = countCollision(sv,Tw_0,Te_w);
% for j=1:length(sv.Parts)
%     m=0; %충돌하는 obstacle수를 위한 변수
%     for k=1:length(sv.Obstacles)%이건 내가 더 추가해줘야하는 obstacles
%         if sv.PartMask(j) && sv.ObstacleMask{k}
%             %quat1=Te_w, pos1=Tw_0이라고 생각합니다....
%             inCollision = robotics.core.internal.intersect(sv.Parts{j}.Geomtryinternal,Tw_0,Te_w,...
%                                                            sv.Obstacles{k}.Geomtryinternal,sv.Obstacles{k}.Position, sv.Obstacles{k},false);
%             if inCollision
%                 isValid = false;
%                 disp('Number of Collision:',m)
%                 break;
%             end
%         end
%     end
% end
...1)이 논리가 맞다면 이 전체(manipulator.rrt)에서 처음에 매니풀레이터가 target object까지 가는 구간(131까지)이후에
...target object와 target position사이에 obstacle추가해주는 선언문추가해주고 위의 '추가부분'까지 다 실행시키면
...추가해준 obstacle들과 path사이에 collision갯수(m)가 나옴
...->안됨. 168번 obstacleMask가 없다는 오류
...2) 1번 오류가 해결된다면 ExampleHelperRigidBodyTreeStateSpace.m의 rstate라는 함수(제 생각으로는
...랜덤으로 configuration을 구하고 valid한지 안 한지를 확인하는 함수)에서 Endeffector만을 고려하는 것에 link들도 추가해주기 위해
...ExampleHelperRigidBodyTreeStateSpace.m의 properties에 LINK1,LINK2를 추가하고
...constrainConfig_EE_YUp 함수에서 obj.Endeffector부분만 obj.LINK1, obj.LINK2로 바꾼 constrainConfig_L1_YUp,constrainConfig_L2_YUp함수를
...만들고 rstate라는 함수의 116번째 줄에서 obj.Endeffector를 obj.LINK1으로 바꾸고 일단 실행->안됨
...*******************************************
% let go of the can
disp(m)
q = exampleHelperEndEffectorRelease(sv,q,ax);


%q = states(i,:);

% Gab the can.
q = exampleHelperEndEffectorGrab(sv,'can5',q, ax);
