%{
Supplementary code for the paper:
"Sensor Observability Analysis for Maximizing Task-Space Observability of Articulated Robots"

By: Christopher Yee WONG and Wael SULEIMAN
Affiliation: Université de Sherbrooke (Sherbrooke, Canada)
Code Author: Christopher Yee WONG
Contact: christopher.wong2 [at] usherbrooke.ca
Last updated: April 28, 2023
Link: https://github.com/chrisywong/SensorObservabilityAnalysisDataset

Files:
1. main.m (this file) -- main script for running code
2. calc3DOFdwdq.m -- function file for calculating partial derivatives for kinematic manipulability
3. calc3DOFdodq.m -- function file for calculating partial derivatives for sensor observability 
4. calc3DOFdodq1axis.m -- function file for calculating partial derivatives for sensor observability in a single axis

How to use this code: 
1. Modify the flags below to:
    * Select the redundancy resolution strategy that you want to use
    * Choose whether to animate the solution or not (the animation runs fairly slowly)
2. Run code in full
%}

% Choose which redundancy resolution strategy to use
% 1 = Minimize joint motion -- this strategy passes through the sensor observability singularity
% 2 = Increase overall kinematic manipulability
% 3 = Increase overall sensor observability
% 4 = Increase sensor observability in the x-axis only
% 5 = Increase sensor observability in the y-axis only
flag_nullspace_option = 1;

% Choose whether to animate the solution or not
% 0 = do not animate
% 1 = animate
flag_animate_solution = 1;

% close all
clc
%% Define Robot -- https://www.mathworks.com/help/robotics/ref/rigidbodytree.html
robot = rigidBodyTree('DataFormat','column','MaxNumBodies',3);

%%%% Define link lengths
L1 = 1;
L2 = 1;
L3 = 0.5;
ndof = 3;

%%%% Define visual length of sensors
Lsens = 0.3;
num_sens = 3;

collisionObj = collisionCylinder(0.02, Lsens); %for illustration  purposes
tf_col_along = [cos(pi/2) 0 -sin(pi/2) Lsens/2; 0 1 0 0; sin(pi/2) 0 cos(pi/2) 0; 0 0 0 1]; % adds cylinder along link
tf_col_perp = [1 0 0 0; 0  cos(pi/2) -sin(pi/2) 0.15; 0 sin(pi/2) cos(pi/2) 0; 0 0 0 1]; % adds cylinder perpendicular to link

%%%% Adding robot links
body = rigidBody('link1');
joint = rigidBodyJoint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');

body = rigidBody('link2');
joint = rigidBodyJoint('joint2','revolute');
setFixedTransform(joint, trvec2tform([L1,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link1');

body = rigidBody('link3');
joint = rigidBodyJoint('joint3','revolute');
setFixedTransform(joint, trvec2tform([L2,0,0]));
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'link2');

body = rigidBody('tool');
joint = rigidBodyJoint('fix1','fixed');
setFixedTransform(joint, trvec2tform([L3, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'link3');


%%%% Defining Linear Sensors
body = rigidBody('L1sensorbase');
joint = rigidBodyJoint('L1sensbase','fixed');
setFixedTransform(joint, trvec2tform([L1/2, 0, 0]));
body.Joint = joint;
addCollision(body,collisionObj, tf_col_perp)
addBody(robot, body, 'link1');
body = rigidBody('L1sensortip');
joint = rigidBodyJoint('L1senstip','fixed');
setFixedTransform(joint, trvec2tform([0, Lsens, 0]));
body.Joint = joint;
addBody(robot, body, 'L1sensorbase');

body = rigidBody('L2sensorbase');
joint = rigidBodyJoint('L2sensbase','fixed');
setFixedTransform(joint, trvec2tform([L2/2, 0, 0]));
body.Joint = joint;
addCollision(body,collisionObj, tf_col_along)
addBody(robot, body, 'link2');
body = rigidBody('L2sensortip');
joint = rigidBodyJoint('L2senstip','fixed');
setFixedTransform(joint, trvec2tform([Lsens, 0, 0]));
body.Joint = joint;
addBody(robot, body, 'L2sensorbase');

body = rigidBody('L3sensorbase');
joint = rigidBodyJoint('L3sensbase','fixed');
setFixedTransform(joint, trvec2tform([L3/2, 0, 0]));
body.Joint = joint;
addCollision(body,collisionObj, tf_col_perp)
addBody(robot, body, 'link3');
body = rigidBody('L3sensortip');
joint = rigidBodyJoint('L3senstip','fixed');
setFixedTransform(joint, trvec2tform([0, Lsens, 0]));
body.Joint = joint;
addBody(robot, body, 'L3sensorbase');

endEffector = 'tool';

%%%% Print robot details
showdetails(robot) 

%% Define Sinusoidal Trajectory
dt = 0.001;
t = (0:dt:1)'; % Time
t = linspace(0,pi,numel(t));
count = length(t);
y = cos(t-pi);
t = t'/4;
y = -y'/2;
center = [0.5 1 - (pi/2)/4 0];
points = zeros(count, 3);
points = [y + center(1), t + center(2), zeros(size(t))];

% figure
% plot(points(:,1), points(:,2))

%% Generate Joint Trajectories using Redundancy Resolution 

qs = zeros(count, ndof);
qs(1,:) = [-0.4432    1.4275    1.7387];
I = eye(3,3);
flag_1axisonly = 0;

if flag_nullspace_option == 1
  display('===== USING NULL SPACE TO MINIMIZE JOINT MOTION =====')
  display('===== USING NULL SPACE TO MINIMIZE JOINT MOTION =====')
  display('===== USING NULL SPACE TO MINIMIZE JOINT MOTION =====') %USE THIS FOR SIGULARITY 
elseif flag_nullspace_option == 2
  display('===== USING NULL SPACE TO INCREASE KINEMATIC MANIPULABILITY =====')
  display('===== USING NULL SPACE TO INCREASE KINEMATIC MANIPULABILITY =====')
  display('===== USING NULL SPACE TO INCREASE KINEMATIC MANIPULABILITY =====')
elseif flag_nullspace_option == 3
  display('===== USING NULL SPACE TO INCREASE OVERALL SENSOR OBSERVABILITY =====')
  display('===== USING NULL SPACE TO INCREASE OVERALL SENSOR OBSERVABILITY =====')
  display('===== USING NULL SPACE TO INCREASE OVERALL SENSOR OBSERVABILITY =====')
elseif flag_nullspace_option == 4
  display('===== USING NULL SPACE TO INCREASE SENSOR OBSERVABILITY IN X AXIS ONLY =====')
  display('===== USING NULL SPACE TO INCREASE SENSOR OBSERVABILITY IN X AXIS ONLY =====')
  display('===== USING NULL SPACE TO INCREASE SENSOR OBSERVABILITY IN X AXIS ONLY =====')
elseif flag_nullspace_option == 5
  display('===== USING NULL SPACE TO INCREASE SENSOR OBSERVABILITY IN Y AXIS ONLY =====')
  display('===== USING NULL SPACE TO INCREASE SENSOR OBSERVABILITY IN Y AXIS ONLY =====')
  display('===== USING NULL SPACE TO INCREASE SENSOR OBSERVABILITY IN Y AXIS ONLY =====')
else
    error('[ERROR] Unknown redundancy resolution choice')
end

tic
for i = 2:count
    % Solve for the configuration satisfying the desired end effector position
    point = points(i,:);
    
    % Using redudancy resolution (see p. 124 of Robots Modelling, Planning and Control by Bruno Siciliano
    dv = ((points(i,:) - points(i-1,:)))';
    dv = dv(1:2);
    J = geometricJacobian(robot, qs(i-1,:)', endEffector); %[rot pos]
    J = J(4:5,:);
    
    % Homogeneous solution
    dq_homo = pinv(J)*dv; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  CHOOSING REDUNDANCY RESOLUTION STRATEGY  %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if flag_nullspace_option == 1
%%% using Jacobian null space to MINIMIZE JOINT MOTION
    dq_redun = [0; 0; 0];
    
elseif flag_nullspace_option == 2
%%% using Jacobian null space to INCREASE KINEMATIC MANIPULABILITY
    k0 = 0.1;
    wq_redun = k0*calc3DOFdwdq(qs(i-1,:)');
    dq_redun = (I - pinv(J)*J)*wq_redun';

elseif flag_nullspace_option == 3
%%% using Jacobian null space to INCREASE SENSOR OBSERVABILITY
    k0 = 0.025; %0.01 is good, maybe too fast?
    wq_redun = k0*calc3DOFdodq(qs(i-1,:)');
    dq_redun = (I - pinv(J)*J)*wq_redun';

elseif flag_nullspace_option == 4
%%% using Jacobian null space to INCREASE SENSOR OBSERVABILITY IN X-AXIS ONLY
    k0 = 0.025;
    wq_redun = k0*calc3DOFdodq1axis(qs(i-1,:)', 1);
    dq_redun = (I - pinv(J)*J)*wq_redun';
    flag_1axisonly = 1;
    
elseif flag_nullspace_option == 5
%%% using Jacobian null space to INCREASE SENSOR OBSERVABILITY IN Y-AXIS ONLY
    k0 = 0.05;
    wq_redun = k0*calc3DOFdodq1axis(qs(i-1,:)', 2);
    dq_redun = (I - pinv(J)*J)*wq_redun';
    flag_1axisonly = 1;
    
else
    error('[ERROR] Unknown redundancy resolution choice')
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%  CHOOSING REDUNDANCY RESOLUTION STRATEGY  %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Calculate full solution
    dq_tot = dq_homo + dq_redun;
    qSol = qs(i-1,:) + dq_tot';
%     [i qSol; 0 dq_homo'; 0 dq_tot']

    % Store the configuration
    qs(i,:) = qSol;

    % Start from prior solution
    qInitial = qSol;
end
toc

%% Animate Solution
t = linspace(0,1,numel(t))';

if flag_animate_solution
  if (ishandle(99))
    close(99)
  end
  fig99 = figure(99);
  xlabel('x (m)')
  ylabel('y (m)')
  view(2)
  ax = gca;
  ax.Projection = 'orthographic';
  hold on
  plot(points(:,1),points(:,2),'k')
  axis([-1 2 -0.5 2])
  s_sum_ind = zeros(count, 1);
  s_sum = zeros(count, num_sens);
  s_max = zeros(count, num_sens);
  s_max_ind = zeros(count, 1);

  framesPerSecond = 500;
  frameStep = 10; % skips certain frames to make animation faster
  r = rateControl(framesPerSecond);

  w_k_ell = zeros(count, 2);

  for i = 1:frameStep:count
      curq = qs(i,:)';

      if flag_animate_solution
        show(robot,curq,'PreservePlot',false, 'Collisions','on');
      end

    jacobian = geometricJacobian(robot, curq, endEffector); %[rot pos]
    [~, J_eigval] = eig(jacobian*transpose(jacobian));
    w_k_ell(i,:) = [sqrt(J_eigval(4,4)), sqrt(J_eigval(5,5))];
    
      tf_L1sens1 = getTransform(robot,curq, 'L1sensortip', 'base');
      tf_L1sens2 = getTransform(robot,curq, 'L1sensorbase', 'base');
      tf_L1sens = (tf_L1sens1(1:3,4) - tf_L1sens2(1:3,4))/Lsens;

      tf_L2sens1 = getTransform(robot,curq, 'L2sensortip', 'base');
      tf_L2sens2 = getTransform(robot,curq, 'L2sensorbase', 'base');
      tf_L2sens = (tf_L2sens1(1:3,4) - tf_L2sens2(1:3,4))/Lsens;

      tf_L3sens1 = getTransform(robot,curq, 'L3sensortip', 'base');
      tf_L3sens2 = getTransform(robot,curq, 'L3sensorbase', 'base');
      tf_L3sens = (tf_L3sens1(1:3,4) - tf_L3sens2(1:3,4))/Lsens;

      [tf_L1sens tf_L2sens tf_L3sens ];
      s_sum(i,:) = abs(tf_L1sens') + abs(tf_L2sens') + abs(tf_L3sens');
      s_sum_ind(i,:) = prod(s_sum(i,1:2));
      s_max(i,:) = max(abs([tf_L1sens tf_L2sens tf_L3sens]),[],2);
      s_max_ind(i,:) = prod(s_max(i,1:2));

      % 2D SOA via ellipse
      tf_base_EE = getTransform(robot,curq, 'tool', 'base');
      ell = drawellipse('Center',tf_base_EE(1:2,end)','SemiAxes',s_max(i,1:2)/6, 'InteractionsAllowed', 'none');
      ell_w_k = drawellipse('Center',tf_base_EE(1:2,end)','SemiAxes',w_k_ell(i,:)/6, 'InteractionsAllowed', 'none', 'Color','r');

      pdes = plot(points(i,1), points(i,2), 'r.', 'markersize', 20);
      drawnow
      waitfor(r);

      if i ~= count %so that it holds the last one
        delete(ell) %ellipse for 2D SOA
        delete(ell_w_k)
        delete(pdes) %ellipse for 2D SOA
      end
  end
end

% Recalculate sensor observability for all timesteps (this step is required)
w_k = zeros(count, 1);
w_k_ell = zeros(count, 2);
for i = 1:count 
    curq = qs(i,:)';
    
    jacobian = geometricJacobian(robot, curq, endEffector); %[rot pos]
    [~, J_eigval] = eig(jacobian*transpose(jacobian));
    w_k_ell(i,:) = [sqrt(J_eigval(4,4)), sqrt(J_eigval(5,5))]; %calculate ellipsoid
    
    tf_L1sens1 = getTransform(robot,curq, 'L1sensortip', 'base');
    tf_L1sens2 = getTransform(robot,curq, 'L1sensorbase', 'base');
    tf_L1sens = (tf_L1sens1(1:3,4) - tf_L1sens2(1:3,4))/Lsens;
    
    tf_L2sens1 = getTransform(robot,curq, 'L2sensortip', 'base');
    tf_L2sens2 = getTransform(robot,curq, 'L2sensorbase', 'base');
    tf_L2sens = (tf_L2sens1(1:3,4) - tf_L2sens2(1:3,4))/Lsens;
    
    tf_L3sens1 = getTransform(robot,curq, 'L3sensortip', 'base');
    tf_L3sens2 = getTransform(robot,curq, 'L3sensorbase', 'base');
    tf_L3sens = (tf_L3sens1(1:3,4) - tf_L3sens2(1:3,4))/Lsens;
    
    [tf_L1sens tf_L2sens tf_L3sens ];
    s_sum(i,:) = abs(tf_L1sens') + abs(tf_L2sens') + abs(tf_L3sens');
    s_sum_ind(i,:) = prod(s_sum(i,1:2));
    s_max(i,:) = max(abs([tf_L1sens tf_L2sens tf_L3sens]),[],2);
    s_max_ind(i,:) = prod(s_max(i,1:2));
    
    w_k(i) = ((cos(qs(i,1) + qs(i,2) + qs(i,3))^2*sin(qs(i,1))^2)/2 + (sin(qs(i,1) + qs(i,2) + qs(i,3))^2*cos(qs(i,1))^2)/2 + cos(qs(i,1) + qs(i,2))^2*sin(qs(i,1))^2 + sin(qs(i,1) + qs(i,2))^2*cos(qs(i,1))^2 + (cos(qs(i,1) + qs(i,2) + qs(i,3))^2*sin(qs(i,1) + qs(i,2))^2)/2 + (sin(qs(i,1) + qs(i,2) + qs(i,3))^2*cos(qs(i,1) + qs(i,2))^2)/2 + cos(qs(i,1) + qs(i,2) + qs(i,3))*cos(qs(i,1) + qs(i,2))*sin(qs(i,1))^2 + (sin(qs(i,1) + qs(i,2) + qs(i,3))^2*cos(qs(i,1) + qs(i,2))*cos(qs(i,1)))/2 + sin(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2))*cos(qs(i,1))^2 + (cos(qs(i,1) + qs(i,2) + qs(i,3))^2*sin(qs(i,1) + qs(i,2))*sin(qs(i,1)))/2 - (cos(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2) + qs(i,3))*cos(qs(i,1) + qs(i,2))*sin(qs(i,1)))/2 - (cos(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2))*cos(qs(i,1)))/2 - cos(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2) + qs(i,3))*cos(qs(i,1))*sin(qs(i,1)) - 2*cos(qs(i,1) + qs(i,2))*sin(qs(i,1) + qs(i,2))*cos(qs(i,1))*sin(qs(i,1)) - cos(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2))*cos(qs(i,1))*sin(qs(i,1)) - sin(qs(i,1) + qs(i,2) + qs(i,3))*cos(qs(i,1) + qs(i,2))*cos(qs(i,1))*sin(qs(i,1)) - cos(qs(i,1) + qs(i,2) + qs(i,3))*sin(qs(i,1) + qs(i,2) + qs(i,3))*cos(qs(i,1) + qs(i,2))*sin(qs(i,1) + qs(i,2)))^(1/2);
end
w_k_max = 1.65; % 1.6495 is global max for this custom 3-DOF robot
s_sum_ind_max = 4.5; % 4.5 is global max for this custom 3-DOF robot
i_plot = [1, int16(count*1/4), int16(count*2/4), int16(count*3/4), int16(count*4/4)];

%% Plotting 

% Colour-blind-friendly colour map for plotting
% https://www.mathworks.com/matlabcentral/fileexchange/68546-crameri-perceptually-uniform-scientific-colormaps
% https://www.nature.com/articles/s41467-020-19160-7
batlowcmap = [0.0052    0.0982    0.3498;
              0.0582    0.2262    0.3716;
              0.0854    0.3163    0.3843;
              0.1566    0.3913    0.3721;
              0.2825    0.4439    0.3111;
              0.4285    0.4878    0.2314;
              0.5935    0.5319    0.1714;
              0.7709    0.5680    0.2224;
              0.9205    0.6020    0.3727;
              0.9892    0.6633    0.5810;
              0.9912    0.7300    0.7771;
              0.9814    0.8004    0.9813];

figure('Position',[0 400 1500 250])
for j = 1:size(i_plot,2)
  HA(j) = subplot(1,6,j);
  view(2)
  ax = gca;
  ax.Projection = 'orthographic';
  hold on
  grid on
  
  plotpos = get(ax, 'Position');
  plotpos(1) = 0.025 + 0.005*j + 0.105*(j-1); %plotpos(1) = 0.03 + 0.012*j + 0.175*(j-1);
  plotpos(3) = 0.105;
  plotpos(2) = plotpos(2) + 0.05;
  HA(j).Position = plotpos;
%   set(ax, 'Position', plotpos);

  plot(points(:,1),points(:,2),'k--', 'LineWidth', 2)
  axis equal
  xlim([-0.25 1.55])
  ylim([-0.6 1.75])
  
  curq = qs(i_plot(j),:)';
  [double(j) double(i_plot(j)) curq'];
%   show(robot,curq,'PreservePlot',true, 'Collisions','on');
  tf_L1sens1 = getTransform(robot,curq, 'L1sensortip', 'base');
  tf_L1sens2 = getTransform(robot,curq, 'L1sensorbase', 'base');
  tf_L1sens = (tf_L1sens1(1:3,4) - tf_L1sens2(1:3,4))/Lsens;

  tf_L2sens1 = getTransform(robot,curq, 'L2sensortip', 'base');
  tf_L2sens2 = getTransform(robot,curq, 'L2sensorbase', 'base');
  tf_L2sens = (tf_L2sens1(1:3,4) - tf_L2sens2(1:3,4))/Lsens;

  tf_L3sens1 = getTransform(robot,curq, 'L3sensortip', 'base');
  tf_L3sens2 = getTransform(robot,curq, 'L3sensorbase', 'base');
  tf_L3sens = (tf_L3sens1(1:3,4) - tf_L3sens2(1:3,4))/Lsens;
  
  tf_base_EE = getTransform(robot, curq, 'tool', 'base');

  tf_base_q2 = getTransform(robot,curq, 'link2', 'base');
  tf_base_q3 = getTransform(robot,curq, 'link3', 'base');
  
  ell_SO = drawellipse('Center',tf_base_EE(1:2,end)','SemiAxes',s_max(i_plot(j),1:2)/3, 'InteractionsAllowed', 'none', 'Color',batlowcmap(2,:), 'LineWidth', 2);
  ell_w_k = drawellipse('Center',tf_base_EE(1:2,end)','SemiAxes',w_k_ell(i_plot(j),:)/4, 'InteractionsAllowed', 'none', 'Color',batlowcmap(10,:), 'LineWidth', 2);
  
%   text(1.4, 1.6, strcat("t = ", num2str(round(t(i_plot(j)),2)), ' s'),'FontSize',10,'HorizontalAlignment','right','FontWeight','Bold');
  text(1.5, 1.6, strcat("t = ", num2str(round(t(i_plot(j)),2)), ' s'),'FontSize',10,'HorizontalAlignment','right','FontWeight','Bold');
  
  probotjoints = [[0 0]; tf_base_q2(1:2,4)'; tf_base_q3(1:2,4)'; tf_base_EE(1:2,4)'];
%   plot(probotjoints(:,1), probotjoints(:,2),'b-o','MarkerSize', 5, 'MarkerFaceColor', 'b')

  batlowcmap_cur = batlowcmap(2*j,:);
  link_colour = [0 0.4470 0.7410]; %batlowcmap(2,:);
  sens_colour = [0.9290 0.6940 0.1250]; %batlowcmap(10,:);
  plot(probotjoints(:,1), probotjoints(:,2),'-','MarkerSize', 5, 'MarkerFaceColor', link_colour ,'color', link_colour)
  plot(probotjoints(1:end-1,1), probotjoints(1:end-1,2),'o','MarkerSize', 5, 'MarkerFaceColor', link_colour ,'color', link_colour)
  
  plot([tf_L1sens2(1,4), tf_L1sens1(1,4)], [tf_L1sens2(2,4), tf_L1sens1(2,4)], '-', 'LineWidth', 4, 'Marker', 'none', 'MarkerSize', 5, 'MarkerFaceColor', sens_colour,'color', sens_colour)
  plot([tf_L2sens2(1,4), tf_L2sens1(1,4)], [tf_L2sens2(2,4), tf_L2sens1(2,4)], '-', 'LineWidth', 4, 'Marker', 'none', 'MarkerSize', 5, 'MarkerFaceColor', sens_colour,'color', sens_colour)
  plot([tf_L3sens2(1,4), tf_L3sens1(1,4)], [tf_L3sens2(2,4), tf_L3sens1(2,4)], '-', 'LineWidth', 4, 'Marker', 'none', 'MarkerSize', 5, 'MarkerFaceColor', sens_colour,'color', sens_colour)
  
  plot([tf_L1sens2(1,4)], [tf_L1sens2(2,4)], 'x', 'LineWidth', 5, 'MarkerSize', 6, 'MarkerFaceColor', sens_colour,'color', sens_colour)
  plot([tf_L2sens2(1,4)], [tf_L2sens2(2,4)], 'x', 'LineWidth', 5, 'MarkerSize', 6, 'MarkerFaceColor', sens_colour,'color', sens_colour)
  plot([tf_L3sens2(1,4)], [tf_L3sens2(2,4)], 'x', 'LineWidth', 5, 'MarkerSize', 6, 'MarkerFaceColor', sens_colour,'color', sens_colour)
  
%   quiver(tf_L1sens2(1,4), tf_L1sens2(2,4), tf_L1sens(1), tf_L1sens(2), 'color', sens_colour, 'AutoScaleFactor', 0.3, 'LineWidth', 3)

  plot(0,0,'ks','MarkerSize', 11, 'MarkerFaceColor', 'k')
  
  if(j == 3)
    xlabel('x (m)')
  end
  if(j == 1)
    ylabel('y (m)')
  end
  if (j > 1)
    set(gca,'yticklabel',[])
  end
end

view(2)
ax = gca;
ax.Projection = 'orthographic';

%%% Plotting sensor observability indices
j = 6;
HA(j) = subplot(1,6,j);
ax = gca;
hold on

plotpos = get(ax, 'Position');
plotpos(1) = 0.036 + 0.01*j + 0.103*(j-1); %plotpos(1) = 0.03 + 0.012*j + 0.175*(j-1);
plotpos(3) = 0.18;
plotpos(2) = plotpos(2) + 0.05;
plotpos(4) = plotpos(4) - 0.01;
HA(j).Position = plotpos;

t = linspace(0,1,numel(t))';
if flag_1axisonly == 0
  plot(t, s_sum_ind/s_sum_ind_max, 'color',batlowcmap(2,:),'LineWidth',1.2)
  plot(t, s_max_ind, 'color',batlowcmap(7,:),'LineStyle',':','LineWidth',1.5)
  plot(t, w_k/w_k_max, 'color',batlowcmap(10,:),'LineStyle','-.','LineWidth',1.2)

  leg = legend(strcat('$o_{sum}\ (max = $'," ",num2str(s_sum_ind_max), ')'),'$o_{max}$',strcat('$w_k\ (max = $'," ",num2str(w_k_max), ')'), 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'Latex');
  leg.ItemTokenSize = [15,18];
  xlabel('t (s)', 'Interpreter', 'Latex')
  ylabel('Index Value (normalized)', 'Interpreter', 'Latex')
  ylim([0 1])
  grid on
  [min(s_max_ind) min(s_sum_ind/s_sum_ind_max)];
elseif flag_1axisonly == 1
  plot(t, s_sum(:,1), 'color','r','LineStyle',':','LineWidth',1.2)%,'DisplayName',strcat('gravity=',num2str(n)))
  plot(t, s_sum(:,2), 'color','b','LineStyle','-','LineWidth',1.2)%,'DisplayName',strcat('gravity=',num2str(n)))
  legend('$s_x$','$s_y$', 'Location', 'northoutside', 'Orientation', 'horizontal', 'Interpreter', 'Latex')
  ylim([0 max(max(s_sum))])
  ylim([0 3])
  xlabel('t (s)', 'Interpreter', 'Latex')
  ylabel('Index Value', 'Interpreter', 'Latex')
%   ylim([0 1])
  grid on
%   [min(s_max_ind) min(s_sum_ind/s_sum_ind_max)]
end