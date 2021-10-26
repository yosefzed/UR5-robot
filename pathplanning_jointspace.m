%%           Path Planning, Trajectory Generation and Inverse Kinematics Solving 
clear
clc

%%              Loading and Showing the Robot
load 'robot.mat'
endeffector = 'Link_6';
robot.DataFormat = 'row';
conf=robot.homeConfiguration;
axes=show(robot,conf,'Frames','off','PreservePlot',false);
%xlim([-1 1]), ylim([-1 1]), zlim([0 1.2])
axis auto;
view([30 30]);

%%          creating and plotting way points
waypointType = 'circle';
switch waypointType
    case 'p2p'
        waypoints = [0.45 -0.3 0.44; 0.45 -0.6 0.44]';
        waypointTimes = [0 6];
    case 'circle'
        radius = 0.6;
        t= (0:2:12)';
        theta= t*(2*pi/t(end))-(pi/2);
        points = [0 0 0.5] + radius * [cos(theta) sin(theta) 0*ones(size(theta))];
        waypoints = points';
        waypointTimes=0:2:12;
    case 'final'
        points=[0.425 0.0161 -0.0867;
        0.425 0.0161 -0.3031;
        0.425 0.0161 -0.0867;
        -0.2895    0.4691   -0.0867
        -0.425 -0.0161 -0.0867;
        -0.425 -0.0161 -0.3031;
        -0.425 -0.0161 -0.0867;
        -0.2895    0.4691   -0.0867;
        0.425 0.0161 -0.0867];
        waypoints=points' ;
        waypointTimes =0:2:16;
end

ts=0.2;
trajtimes=0:ts:waypointTimes(end);        
hold on 
plot3(waypoints(1,:),waypoints(2,:),waypoints(3,:),'go','LineWidth',2);
axis auto;
view([30 30]);

%%         Inverse Kinematics Solving

ik = inverseKinematics('RigidBodyTree',robot);
ikWeights = [1 1 1 1 1 1];
ikInitGuess = conf;
ikInitGuess(ikInitGuess > pi) = ikInitGuess(ikInitGuess > pi) - 2*pi;
ikInitGuess(ikInitGuess < -pi) = ikInitGuess(ikInitGuess < -pi) + 2*pi;

numWaypoints = size(waypoints,2);
numJoints = numel(conf);
jointWaypoints = zeros(numJoints,numWaypoints);

for i = 1:numWaypoints
    
    tgtPose =  trvec2tform(waypoints(:,i)');
    [config,info] = ik(endeffector,tgtPose,ikWeights,ikInitGuess);
    ikInitGuess = config;
    jointWaypoints(:,i) = config;
end

%%              Trajectory Generation

[q,qd,qdd,pp]=cubicpolytraj(jointWaypoints,waypointTimes,trajtimes);
%[q,qd,qdd] = trapveltraj(jointWaypoints,numel(trajtimes));

hTraj = plot3(waypoints(1,1),waypoints(2,1),waypoints(3,1),'r');

        % Trajectory following loop
for i = 1:numel(trajtimes)  

    config = q(:,i)';
    
    % Find Cartesian points for visualization
    eeTform = getTransform(robot,config,endeffector);
    
    eePos = tform2trvec(eeTform);
    set(hTraj,'xdata',[hTraj.XData eePos(1)], ...
        'ydata',[hTraj.YData eePos(2)], ...
        'zdata',[hTraj.ZData eePos(3)]);

    % Show the robot
    show(robot,config,'Frames','off','PreservePlot',false);
    title(['Trajectory at t = ' num2str(trajtimes(i))])
    drawnow   
    
end