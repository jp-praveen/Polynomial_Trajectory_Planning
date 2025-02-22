% Author: Praveen
% Advance Trajectory Optimization Project
% This script will solve a minimum snap trajectory problem as a QP. Time Allocation
% and predefined obstacles are included in this script
%%
clear
clc
close all

%% Problem Setup
order = 4;
n = 4;       % number of nodes
m = n - 1;   % number of segments
PolyDeg = 2*order - 1;
Coeff = PolyDeg + 1;


% Predefined Hoop position
for i = 1:2:3
    figure(1)
    hold on
    if i==1
        % center of circle 
        C = [i,i,0];   
    elseif i==2
        C = [i,i,1];
    else
        C = [i,i,0.5];
    end
    
    % Radius of circle
    R = 1.;    
    
    % Angle theta
    teta=0:0.01:2*pi ;
    x=C(1)+R*cos(teta);
    y=C(2)+R*sin(teta) ;
    z = C(3)+zeros(size(x));
    
    patch(x,y,z,'w','LineWidth',2)
    hold on
    set(gca,'FontSize', 12, 'FontWeight', 'bold')
    hold on; grid on; box on; 
    plot3(C(1),C(2),C(3),'*r','LineWidth',2)
    xlabel('x(t)','FontSize', 14,'FontWeight', 'bold'); 
    ylabel('y(t)','FontSize', 14,'FontWeight', 'bold'); 
    zlabel('z(t)','FontSize', 14,'FontWeight', 'bold');
    title('Choose x and y nodes', 'interpreter', 'latex','FontSize', 16);
    xlim([-1 4])
    ylim([-1 4])
    zlim([-1 2])
end

% Selection of the four nodes using GUI. The selection should be made such
% that the nodes are not near the hoops as minimum snap trajectories 
% generally overshoots near node points

[xnodes,ynodes] = ginput(n); 
znodes = [-0.5;0.25;0.75;0];
ti = 0;
tf = 6;
tnodes =[optimvar('t0',1, 1, 'LowerBound', ti, 'UpperBound', ti);
         optimvar('t1',1, 1, 'LowerBound', ti+1, 'UpperBound', ti+2);
         optimvar('t2',1, 1, 'LowerBound', ti+3, 'UpperBound', ti+4);
         optimvar('t3',1, 1, 'LowerBound', ti+5, 'UpperBound', ti+6)].';
     
% Defining the time arrays     
tarrPos = zeros(Coeff,n);
tarrVel = zeros(Coeff,n);
tarrAcc = zeros(Coeff,n);
tarrJerk = zeros(Coeff,n);
tarrSnap = zeros(Coeff,n);
tarrFive = zeros(Coeff,n);
tarrSix = zeros(Coeff,n);

for i = 1:Coeff
    if i == 1
        tarrPos(i,:) = ones(1,n);
        tarrVel(i,:) = zeros(1,n);
        tarrAcc(i,:) = zeros(1,n);
    else
        if i == 2
            tarrAcc(i,:) = zeros(1,n);
        else
            tarrAcc(i,:) = factorial(i-1)/factorial(i-3);
        end
        tarrVel(i,:) = (i-1);
        tarrPos(i,:) = 1;
    end
end
for i = 1:Coeff
    if i <= 3
        tarrJerk(i,:) = zeros(1,n);
    elseif i == 4 
        tarrJerk(i,:) = 6;
    elseif i == 5
         tarrJerk(i,:) = 24; 
    elseif i == 6
         tarrJerk(i,:) = 60; 
    elseif i == 7
         tarrJerk(i,:) = 120; 
    elseif i == 8
         tarrJerk(i,:) = 210; 
    end
end
for i = 1:Coeff
    if i<=4
        tarrSnap(i,:) = zeros(1,n);
    else
        tarrSnap(i,:) = factorial(i-1)/factorial(i-5);
    
    end
end

for i = 1:Coeff
    if i <= 5
        tarrFive(i,:) = zeros(1,n);
        tarrSix(i,:) = zeros(1,n);
    else
        if i == 6
            tarrSix(i,:) = zeros(1,n);
            tarrFive(i,:) = factorial(i-1)/factorial(i-6);
        else
            tarrSix(i,:) =  factorial(i-1);
            tarrFive(i,:) = (factorial(i-1)/factorial(i-6));
        end
        
    end
    
end
% Initialize fuel-optimal Problem 
MinimumSnap = optimproblem('ObjectiveSense', 'minimize');

C = [optimvar('c0',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c1',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c2',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c3',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c4',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c5',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c6',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000);
     optimvar('c7',1, 3*m, 'LowerBound', -10000000, 'UpperBound', 10000000)];
 
% Position Constraints
for i = 1:m
    j = i+m;
    k = i+2*m;
    PosNodeOddx(i) = C(1,i)*tarrPos(1,i)*tnodes(i)^0 + tarrPos(2,i)*C(2,i)*tnodes(i)^1 + tarrPos(3,i)*C(3,i)*tnodes(i)^2 + tarrPos(4,i)*C(4,i)*tnodes(i)^3 + tarrPos(5,i)*C(5,i)*tnodes(i)^4 +tarrPos(6,i)*C(6,i)*tnodes(i)^5 + tarrPos(7,i)*C(7,i)*tnodes(i)^6 +tarrPos(8,i)*C(8,i)*tnodes(i)^7 == xnodes(i);
    PosNodeOddy(i) = C(1,j)*tarrPos(1,i)*tnodes(i)^0 + tarrPos(2,i)*C(2,j)*tnodes(i)^1 + tarrPos(3,i)*C(3,j)*tnodes(i)^2 + tarrPos(4,i)*C(4,j)*tnodes(i)^3 + tarrPos(5,i)*C(5,j)*tnodes(i)^4 +tarrPos(6,i)*C(6,j)*tnodes(i)^5 + tarrPos(7,i)*C(7,j)*tnodes(i)^6 +tarrPos(8,i)*C(8,j)*tnodes(i)^7 == ynodes(i);
    PosNodeOddz(i) = C(1,k)*tarrPos(1,i)*tnodes(i)^0 + tarrPos(2,i)*C(2,k)*tnodes(i)^1 + tarrPos(3,i)*C(3,k)*tnodes(i)^2 + tarrPos(4,i)*C(4,k)*tnodes(i)^3 + tarrPos(5,i)*C(5,k)*tnodes(i)^4 +tarrPos(6,i)*C(6,k)*tnodes(i)^5 + tarrPos(7,i)*C(7,k)*tnodes(i)^6 +tarrPos(8,i)*C(8,k)*tnodes(i)^7 == znodes(i);
end

for i = 1:m
    j = i+m;
    k = i+2*m;
    PosNodeEvenx(i) = C(1,i)*tarrPos(1,i+1)*tnodes(i+1)^0 + tarrPos(2,i+1)*C(2,i)*tnodes(i+1)^1 + tarrPos(3,i+1)*C(3,i)*tnodes(i+1)^2 + tarrPos(4,i+1)*C(4,i)*tnodes(i+1)^3 + tarrPos(5,i+1)*C(5,i)*tnodes(i+1)^4 +tarrPos(6,i+1)*C(6,i)*tnodes(i+1)^5 + tarrPos(7,i+1)*C(7,i)*tnodes(i+1)^6 +tarrPos(8,i+1)*C(8,i)*tnodes(i+1)^7 == xnodes(i+1);
    PosNodeEveny(i) = C(1,j)*tarrPos(1,i+1)*tnodes(i+1)^0 + tarrPos(2,i+1)*C(2,j)*tnodes(i+1)^1 + tarrPos(3,i+1)*C(3,j)*tnodes(i+1)^2 + tarrPos(4,i+1)*C(4,j)*tnodes(i+1)^3 + tarrPos(5,i+1)*C(5,j)*tnodes(i+1)^4 +tarrPos(6,i+1)*C(6,j)*tnodes(i+1)^5 + tarrPos(7,i+1)*C(7,j)*tnodes(i+1)^6 +tarrPos(8,i+1)*C(8,j)*tnodes(i+1)^7 == ynodes(i+1);
    PosNodeEvenz(i) = C(1,k)*tarrPos(1,i+1)*tnodes(i+1)^0 + tarrPos(2,i+1)*C(2,k)*tnodes(i+1)^1 + tarrPos(3,i+1)*C(3,k)*tnodes(i+1)^2 + tarrPos(4,i+1)*C(4,k)*tnodes(i+1)^3 + tarrPos(5,i+1)*C(5,k)*tnodes(i+1)^4 +tarrPos(6,i+1)*C(6,k)*tnodes(i+1)^5 + tarrPos(7,i+1)*C(7,k)*tnodes(i+1)^6 +tarrPos(8,i+1)*C(8,k)*tnodes(i+1)^7 == znodes(i+1);
end

MinimumSnap.Constraints.PosNodeOddx = PosNodeOddx; 
MinimumSnap.Constraints.PosNodeEvenx = PosNodeEvenx; 
MinimumSnap.Constraints.PosNodeOddy = PosNodeOddy; 
MinimumSnap.Constraints.PosNodeEveny = PosNodeEveny; 
MinimumSnap.Constraints.PosNodeOddz = PosNodeOddz; 
MinimumSnap.Constraints.PosNodeEvenz = PosNodeEvenz; 

% Initial and Final velocity Constraint
VelNode1x = tarrVel(1,1)*C(1,1)*tnodes(1)^0 + tarrVel(2,1)*C(2,1)*tnodes(1)^0 + tarrVel(3,1)*C(3,1)*tnodes(1)^1 + tarrVel(4,1)*C(4,1)*tnodes(1)^2 + tarrVel(5,1)*C(5,1)*tnodes(1)^3 +tarrVel(6,1)*C(6,1)*tnodes(1)^4 + tarrVel(7,1)*C(7,1)*tnodes(1)^5 +tarrVel(8,1)*C(8,1)*tnodes(1)^6 == 0;
VelNodefx = tarrVel(1,end)*C(1,m)*tnodes(end)^0 + tarrVel(2,end)*C(2,m)*tnodes(end)^0 + tarrVel(3,end)*C(3,m)*tnodes(end)^1 + tarrVel(4,end)*C(4,m)*tnodes(end)^2 + tarrVel(5,end)*C(5,m)*tnodes(end)^3 +tarrVel(6,end)*C(6,m)*tnodes(end)^4 + tarrVel(7,end)*C(7,m)*tnodes(end)^5 +tarrVel(8,end)*C(8,m)*tnodes(end)^6 == 0;
VelNode1y = tarrVel(1,1)*C(1,1+m)*tnodes(1)^0 + tarrVel(2,1)*C(2,1+m)*tnodes(1)^0 + tarrVel(3,1)*C(3,1+m)*tnodes(1)^1 + tarrVel(4,1)*C(4,1+m)*tnodes(1)^2 + tarrVel(5,1)*C(5,1+m)*tnodes(1)^3 +tarrVel(6,1)*C(6,1+m)*tnodes(1)^4 + tarrVel(7,1)*C(7,1+m)*tnodes(1)^5 +tarrVel(8,1)*C(8,1+m)*tnodes(1)^6 == 0;
VelNodefy = tarrVel(1,end)*C(1,2*m)*tnodes(end)^0 + tarrVel(2,end)*C(2,2*m)*tnodes(end)^0 + tarrVel(3,end)*C(3,2*m)*tnodes(end)^1 + tarrVel(4,end)*C(4,2*m)*tnodes(end)^2 + tarrVel(5,end)*C(5,2*m)*tnodes(end)^3 +tarrVel(6,end)*C(6,2*m)*tnodes(end)^4 + tarrVel(7,end)*C(7,2*m)*tnodes(end)^5 +tarrVel(8,end)*C(8,2*m)*tnodes(end)^6 == 0;
VelNode1z = tarrVel(1,1)*C(1,1+2*m)*tnodes(1)^0 + tarrVel(2,1)*C(2,1+2*m)*tnodes(1)^0 + tarrVel(3,1)*C(3,1+2*m)*tnodes(1)^1 + tarrVel(4,1)*C(4,1+2*m)*tnodes(1)^2 + tarrVel(5,1)*C(5,1+2*m)*tnodes(1)^3 +tarrVel(6,1)*C(6,1+2*m)*tnodes(1)^4 + tarrVel(7,1)*C(7,1+2*m)*tnodes(1)^5 +tarrVel(8,1)*C(8,1+2*m)*tnodes(1)^6 == 0;
VelNodefz = tarrVel(1,end)*C(1,3*m)*tnodes(end)^0 + tarrVel(2,end)*C(2,3*m)*tnodes(end)^0 + tarrVel(3,end)*C(3,3*m)*tnodes(end)^1 + tarrVel(4,end)*C(4,3*m)*tnodes(end)^2 + tarrVel(5,end)*C(5,3*m)*tnodes(end)^3 +tarrVel(6,end)*C(6,3*m)*tnodes(end)^4 + tarrVel(7,end)*C(7,3*m)*tnodes(end)^5 +tarrVel(8,end)*C(8,3*m)*tnodes(end)^6 == 0;

MinimumSnap.Constraints.VelNode1x = VelNode1x;
MinimumSnap.Constraints.VelNodefx = VelNodefx;
MinimumSnap.Constraints.VelNode1y = VelNode1y;
MinimumSnap.Constraints.VelNodefy = VelNodefy;
MinimumSnap.Constraints.VelNode1z = VelNode1z;
MinimumSnap.Constraints.VelNodefz = VelNodefz;

% Initial and Final acceleration Constraint
AccNode1x = tarrAcc(1,1)*C(1,1)*tnodes(1)^0 + tarrAcc(2,1)*C(2,1)*tnodes(1)^0 + tarrAcc(3,1)*C(3,1)*tnodes(1)^1 + tarrAcc(4,1)*C(4,1)*tnodes(1)^2 + tarrAcc(5,1)*C(5,1)*tnodes(1)^3 +tarrAcc(6,1)*C(6,1)*tnodes(1)^4 + tarrAcc(7,1)*C(7,1)*tnodes(1)^5 +tarrAcc(8,1)*C(8,1)*tnodes(1)^6 == 0;
AccNodefx = tarrAcc(1,end)*C(1,m)*tnodes(end)^0 + tarrAcc(2,end)*C(2,m)*tnodes(end)^0 + tarrAcc(3,end)*C(3,m)*tnodes(end)^1 + tarrAcc(4,end)*C(4,m)*tnodes(end)^2 + tarrAcc(5,end)*C(5,m)*tnodes(end)^3 +tarrAcc(6,end)*C(6,m)*tnodes(end)^4 + tarrAcc(7,end)*C(7,m)*tnodes(end)^5 +tarrAcc(8,end)*C(8,m)*tnodes(end)^6 == 0;
AccNode1y = tarrAcc(1,1)*C(1,1+m)*tnodes(1)^0 + tarrAcc(2,1)*C(2,1+m)*tnodes(1)^0 + tarrAcc(3,1)*C(3,1+m)*tnodes(1)^1 + tarrAcc(4,1)*C(4,1+m)*tnodes(1)^2 + tarrAcc(5,1)*C(5,1+m)*tnodes(1)^3 +tarrAcc(6,1)*C(6,1+m)*tnodes(1)^4 + tarrAcc(7,1)*C(7,1+m)*tnodes(1)^5 +tarrAcc(8,1)*C(8,1+m)*tnodes(1)^6 == 0;
AccNodefy = tarrAcc(1,end)*C(1,2*m)*tnodes(end)^0 + tarrAcc(2,end)*C(2,2*m)*tnodes(end)^0 + tarrAcc(3,end)*C(3,2*m)*tnodes(end)^1 + tarrAcc(4,end)*C(4,2*m)*tnodes(end)^2 + tarrAcc(5,end)*C(5,2*m)*tnodes(end)^3 +tarrAcc(6,end)*C(6,2*m)*tnodes(end)^4 + tarrAcc(7,end)*C(7,2*m)*tnodes(end)^5 +tarrAcc(8,end)*C(8,2*m)*tnodes(end)^6 == 0;
AccNode1z = tarrAcc(1,1)*C(1,1+2*m)*tnodes(1)^0 + tarrAcc(2,1)*C(2,1+2*m)*tnodes(1)^0 + tarrAcc(3,1)*C(3,1+2*m)*tnodes(1)^1 + tarrAcc(4,1)*C(4,1+2*m)*tnodes(1)^2 + tarrAcc(5,1)*C(5,1+2*m)*tnodes(1)^3 +tarrAcc(6,1)*C(6,1+2*m)*tnodes(1)^4 + tarrAcc(7,1)*C(7,1+2*m)*tnodes(1)^5 +tarrAcc(8,1)*C(8,1+2*m)*tnodes(1)^6 == 0;
AccNodefz = tarrAcc(1,end)*C(1,3*m)*tnodes(end)^0 + tarrAcc(2,end)*C(2,3*m)*tnodes(end)^0 + tarrAcc(3,end)*C(3,3*m)*tnodes(end)^1 + tarrAcc(4,end)*C(4,3*m)*tnodes(end)^2 + tarrAcc(5,end)*C(5,3*m)*tnodes(end)^3 +tarrAcc(6,end)*C(6,3*m)*tnodes(end)^4 + tarrAcc(7,end)*C(7,3*m)*tnodes(end)^5 +tarrAcc(8,end)*C(8,3*m)*tnodes(end)^6 == 0;

MinimumSnap.Constraints.AccNode1x = AccNode1x;
MinimumSnap.Constraints.AccNodefx = AccNodefx;
MinimumSnap.Constraints.AccNode1y = AccNode1y;
MinimumSnap.Constraints.AccNodefy = AccNodefy;
MinimumSnap.Constraints.AccNode1z = AccNode1z;
MinimumSnap.Constraints.AccNodefz = AccNodefz;

% Initial and Final Jerk Constraint
JerkNode1x = tarrJerk(1,1)*C(1,1)*tnodes(1)^0 + tarrJerk(2,1)*C(2,1)*tnodes(1)^0 + tarrJerk(3,1)*C(3,1)*tnodes(1)^1 + tarrJerk(4,1)*C(4,1)*tnodes(1)^2 + tarrJerk(5,1)*C(5,1)*tnodes(1)^3 +tarrJerk(6,1)*C(6,1)*tnodes(1)^4 + tarrJerk(7,1)*C(7,1)*tnodes(1)^5 +tarrJerk(8,1)*C(8,1)*tnodes(1)^6 == 0;
JerkNodefx = tarrJerk(1,end)*C(1,m)*tnodes(end)^0 + tarrJerk(2,end)*C(2,m)*tnodes(end)^0 + tarrJerk(3,end)*C(3,m)*tnodes(end)^1 + tarrJerk(4,end)*C(4,m)*tnodes(end)^2 + tarrJerk(5,end)*C(5,m)*tnodes(end)^3 +tarrJerk(6,end)*C(6,m)*tnodes(end)^4 + tarrJerk(7,end)*C(7,m)*tnodes(end)^5 +tarrJerk(8,end)*C(8,m)*tnodes(end)^6 == 0;
JerkNode1y = tarrJerk(1,1)*C(1,1+m)*tnodes(1)^0 + tarrJerk(2,1)*C(2,1+m)*tnodes(1)^0 + tarrJerk(3,1)*C(3,1+m)*tnodes(1)^1 + tarrJerk(4,1)*C(4,1+m)*tnodes(1)^2 + tarrJerk(5,1)*C(5,1+m)*tnodes(1)^3 +tarrJerk(6,1)*C(6,1+m)*tnodes(1)^4 + tarrJerk(7,1)*C(7,1+m)*tnodes(1)^5 +tarrJerk(8,1)*C(8,1+m)*tnodes(1)^6 == 0;
JerkNodefy = tarrJerk(1,end)*C(1,2*m)*tnodes(end)^0 + tarrJerk(2,end)*C(2,2*m)*tnodes(end)^0 + tarrJerk(3,end)*C(3,2*m)*tnodes(end)^1 + tarrJerk(4,end)*C(4,2*m)*tnodes(end)^2 + tarrJerk(5,end)*C(5,2*m)*tnodes(end)^3 +tarrJerk(6,end)*C(6,2*m)*tnodes(end)^4 + tarrJerk(7,end)*C(7,2*m)*tnodes(end)^5 +tarrJerk(8,end)*C(8,2*m)*tnodes(end)^6 == 0;
JerkNode1z = tarrJerk(1,1)*C(1,1+2*m)*tnodes(1)^0 + tarrJerk(2,1)*C(2,1+2*m)*tnodes(1)^0 + tarrJerk(3,1)*C(3,1+2*m)*tnodes(1)^1 + tarrJerk(4,1)*C(4,1+2*m)*tnodes(1)^2 + tarrJerk(5,1)*C(5,1+2*m)*tnodes(1)^3 +tarrJerk(6,1)*C(6,1+2*m)*tnodes(1)^4 + tarrJerk(7,1)*C(7,1+2*m)*tnodes(1)^5 +tarrJerk(8,1)*C(8,1+2*m)*tnodes(1)^6 == 0;
JerkNodefz = tarrJerk(1,end)*C(1,3*m)*tnodes(end)^0 + tarrJerk(2,end)*C(2,3*m)*tnodes(end)^0 + tarrJerk(3,end)*C(3,3*m)*tnodes(end)^1 + tarrJerk(4,end)*C(4,3*m)*tnodes(end)^2 + tarrJerk(5,end)*C(5,3*m)*tnodes(end)^3 +tarrJerk(6,end)*C(6,3*m)*tnodes(end)^4 + tarrJerk(7,end)*C(7,3*m)*tnodes(end)^5 +tarrJerk(8,end)*C(8,3*m)*tnodes(end)^6 == 0;

MinimumSnap.Constraints.JerkNode1x = JerkNode1x;
MinimumSnap.Constraints.JerkNodefx = JerkNodefx;
MinimumSnap.Constraints.JerkNode1y = JerkNode1y;
MinimumSnap.Constraints.JerkNodefy = JerkNodefy;
MinimumSnap.Constraints.JerkNode1z = JerkNode1z;
MinimumSnap.Constraints.JerkNodefz = JerkNodefz;

% Smoothing higher order Constraints 
for i = 1:m-1
%     SmoothVelx(i) = C(:,i)'*tarrVel(:,i+1) == C(:,i+1)'*tarrVel(:,i+1);
    SmoothVelx(i) = tarrVel(1,i+1)*C(1,i)*tnodes(i+1)^0 + tarrVel(2,i+1)*C(2,i)*tnodes(i+1)^0 + tarrVel(3,i+1)*C(3,i)*tnodes(i+1)^1 + tarrVel(4,i+1)*C(4,i)*tnodes(i+1)^2 + tarrVel(5,i+1)*C(5,i)*tnodes(i+1)^3 +tarrVel(6,i+1)*C(6,i)*tnodes(i+1)^4 + tarrVel(7,i+1)*C(7,i)*tnodes(i+1)^5 +tarrVel(8,i+1)*C(8,i)*tnodes(i+1)^6 == tarrVel(1,i+1)*C(1,i+1)*tnodes(i+1)^0 + tarrVel(2,i+1)*C(2,i+1)*tnodes(i+1)^0 + tarrVel(3,i+1)*C(3,i+1)*tnodes(i+1)^1 + tarrVel(4,i+1)*C(4,i+1)*tnodes(i+1)^2 + tarrVel(5,i+1)*C(5,i+1)*tnodes(i+1)^3 +tarrVel(6,i+1)*C(6,i+1)*tnodes(i+1)^4 + tarrVel(7,i+1)*C(7,i+1)*tnodes(i+1)^5 +tarrVel(8,i+1)*C(8,i+1)*tnodes(i+1)^6;  
    SmoothAccx(i) = tarrAcc(1,i+1)*C(1,i)*tnodes(i+1)^0 + tarrAcc(2,i+1)*C(2,i)*tnodes(i+1)^0 + tarrAcc(3,i+1)*C(3,i)*tnodes(i+1)^1 + tarrAcc(4,i+1)*C(4,i)*tnodes(i+1)^2 + tarrAcc(5,i+1)*C(5,i)*tnodes(i+1)^3 +tarrAcc(6,i+1)*C(6,i)*tnodes(i+1)^4 + tarrAcc(7,i+1)*C(7,i)*tnodes(i+1)^5 +tarrAcc(8,i+1)*C(8,i)*tnodes(i+1)^6 == tarrAcc(1,i+1)*C(1,i+1)*tnodes(i+1)^0 + tarrAcc(2,i+1)*C(2,i+1)*tnodes(i+1)^0 + tarrAcc(3,i+1)*C(3,i+1)*tnodes(i+1)^1 + tarrAcc(4,i+1)*C(4,i+1)*tnodes(i+1)^2 + tarrAcc(5,i+1)*C(5,i+1)*tnodes(i+1)^3 +tarrAcc(6,i+1)*C(6,i+1)*tnodes(i+1)^4 + tarrAcc(7,i+1)*C(7,i+1)*tnodes(i+1)^5 +tarrAcc(8,i+1)*C(8,i+1)*tnodes(i+1)^6;
    SmoothJerkx(i) = tarrJerk(1,i+1)*C(1,i)*tnodes(i+1)^0 + tarrJerk(2,i+1)*C(2,i)*tnodes(i+1)^0 + tarrJerk(3,i+1)*C(3,i)*tnodes(i+1)^1 + tarrJerk(4,i+1)*C(4,i)*tnodes(i+1)^2 + tarrJerk(5,i+1)*C(5,i)*tnodes(i+1)^3 +tarrJerk(6,i+1)*C(6,i)*tnodes(i+1)^4 + tarrJerk(7,i+1)*C(7,i)*tnodes(i+1)^5 +tarrJerk(8,i+1)*C(8,i)*tnodes(i+1)^6 == tarrJerk(1,i+1)*C(1,i+1)*tnodes(i+1)^0 + tarrJerk(2,i+1)*C(2,i+1)*tnodes(i+1)^0 + tarrJerk(3,i+1)*C(3,i+1)*tnodes(i+1)^1 + tarrJerk(4,i+1)*C(4,i+1)*tnodes(i+1)^2 + tarrJerk(5,i+1)*C(5,i+1)*tnodes(i+1)^3 +tarrJerk(6,i+1)*C(6,i+1)*tnodes(i+1)^4 + tarrJerk(7,i+1)*C(7,i+1)*tnodes(i+1)^5 +tarrJerk(8,i+1)*C(8,i+1)*tnodes(i+1)^6; 
    SmoothSnapx(i) = tarrSnap(1,i+1)*C(1,i)*tnodes(i+1)^0 + tarrSnap(2,i+1)*C(2,i)*tnodes(i+1)^0 + tarrSnap(3,i+1)*C(3,i)*tnodes(i+1)^1 + tarrSnap(4,i+1)*C(4,i)*tnodes(i+1)^2 + tarrSnap(5,i+1)*C(5,i)*tnodes(i+1)^3 +tarrSnap(6,i+1)*C(6,i)*tnodes(i+1)^4 + tarrSnap(7,i+1)*C(7,i)*tnodes(i+1)^5 +tarrSnap(8,i+1)*C(8,i)*tnodes(i+1)^6 == tarrSnap(1,i+1)*C(1,i+1)*tnodes(i+1)^0 + tarrSnap(2,i+1)*C(2,i+1)*tnodes(i+1)^0 + tarrSnap(3,i+1)*C(3,i+1)*tnodes(i+1)^1 + tarrSnap(4,i+1)*C(4,i+1)*tnodes(i+1)^2 + tarrSnap(5,i+1)*C(5,i+1)*tnodes(i+1)^3 +tarrSnap(6,i+1)*C(6,i+1)*tnodes(i+1)^4 + tarrSnap(7,i+1)*C(7,i+1)*tnodes(i+1)^5 +tarrSnap(8,i+1)*C(8,i+1)*tnodes(i+1)^6; 
    SmoothFivex(i) = tarrFive(1,i+1)*C(1,i)*tnodes(i+1)^0 + tarrFive(2,i+1)*C(2,i)*tnodes(i+1)^0 + tarrFive(3,i+1)*C(3,i)*tnodes(i+1)^1 + tarrFive(4,i+1)*C(4,i)*tnodes(i+1)^2 + tarrFive(5,i+1)*C(5,i)*tnodes(i+1)^3 +tarrFive(6,i+1)*C(6,i)*tnodes(i+1)^4 + tarrFive(7,i+1)*C(7,i)*tnodes(i+1)^5 +tarrFive(8,i+1)*C(8,i)*tnodes(i+1)^6 == tarrFive(1,i+1)*C(1,i+1)*tnodes(i+1)^0 + tarrFive(2,i+1)*C(2,i+1)*tnodes(i+1)^0 + tarrFive(3,i+1)*C(3,i+1)*tnodes(i+1)^1 + tarrFive(4,i+1)*C(4,i+1)*tnodes(i+1)^2 + tarrFive(5,i+1)*C(5,i+1)*tnodes(i+1)^3 +tarrFive(6,i+1)*C(6,i+1)*tnodes(i+1)^4 + tarrFive(7,i+1)*C(7,i+1)*tnodes(i+1)^5 +tarrFive(8,i+1)*C(8,i+1)*tnodes(i+1)^6; 
    SmoothSixx(i) = tarrSix(1,i+1)*C(1,i)*tnodes(i+1)^0 + tarrSix(2,i+1)*C(2,i)*tnodes(i+1)^0 + tarrSix(3,i+1)*C(3,i)*tnodes(i+1)^1 + tarrSix(4,i+1)*C(4,i)*tnodes(i+1)^2 + tarrSix(5,i+1)*C(5,i)*tnodes(i+1)^3 +tarrSix(6,i+1)*C(6,i)*tnodes(i+1)^4 + tarrSix(7,i+1)*C(7,i)*tnodes(i+1)^5 +tarrSix(8,i+1)*C(8,i)*tnodes(i+1)^6 == tarrSix(1,i+1)*C(1,i+1)*tnodes(i+1)^0 + tarrSix(2,i+1)*C(2,i+1)*tnodes(i+1)^0 + tarrSix(3,i+1)*C(3,i+1)*tnodes(i+1)^1 + tarrSix(4,i+1)*C(4,i+1)*tnodes(i+1)^2 + tarrSix(5,i+1)*C(5,i+1)*tnodes(i+1)^3 +tarrSix(6,i+1)*C(6,i+1)*tnodes(i+1)^4 + tarrSix(7,i+1)*C(7,i+1)*tnodes(i+1)^5 +tarrSix(8,i+1)*C(8,i+1)*tnodes(i+1)^6; 
    
    SmoothVely(i) = tarrVel(1,i+1)*C(1,m+i)*tnodes(i+1)^0 + tarrVel(2,i+1)*C(2,m+i)*tnodes(i+1)^0 + tarrVel(3,i+1)*C(3,m+i)*tnodes(i+1)^1 + tarrVel(4,i+1)*C(4,m+i)*tnodes(i+1)^2 + tarrVel(5,i+1)*C(5,m+i)*tnodes(i+1)^3 +tarrVel(6,i+1)*C(6,m+i)*tnodes(i+1)^4 + tarrVel(7,i+1)*C(7,m+i)*tnodes(i+1)^5 +tarrVel(8,i+1)*C(8,m+i)*tnodes(i+1)^6 == tarrVel(1,i+1)*C(1,m+i+1)*tnodes(i+1)^0 + tarrVel(2,i+1)*C(2,m+i+1)*tnodes(i+1)^0 + tarrVel(3,i+1)*C(3,m+i+1)*tnodes(i+1)^1 + tarrVel(4,i+1)*C(4,m+i+1)*tnodes(i+1)^2 + tarrVel(5,i+1)*C(5,m+i+1)*tnodes(i+1)^3 +tarrVel(6,i+1)*C(6,m+i+1)*tnodes(i+1)^4 + tarrVel(7,i+1)*C(7,m+i+1)*tnodes(i+1)^5 +tarrVel(8,i+1)*C(8,m+i+1)*tnodes(i+1)^6;
%     SmoothVely(i) = C(:,m+i)'*tarrVel(:,i+1) == C(:,m+i+1)'*tarrVel(:,i+1);
    SmoothAccy(i) = tarrAcc(1,i+1)*C(1,m+i)*tnodes(i+1)^0 + tarrAcc(2,i+1)*C(2,m+i)*tnodes(i+1)^0 + tarrAcc(3,i+1)*C(3,m+i)*tnodes(i+1)^1 + tarrAcc(4,i+1)*C(4,m+i)*tnodes(i+1)^2 + tarrAcc(5,i+1)*C(5,m+i)*tnodes(i+1)^3 +tarrAcc(6,i+1)*C(6,m+i)*tnodes(i+1)^4 + tarrAcc(7,i+1)*C(7,m+i)*tnodes(i+1)^5 +tarrAcc(8,i+1)*C(8,m+i)*tnodes(i+1)^6 == tarrAcc(1,i+1)*C(1,m+i+1)*tnodes(i+1)^0 + tarrAcc(2,i+1)*C(2,m+i+1)*tnodes(i+1)^0 + tarrAcc(3,i+1)*C(3,m+i+1)*tnodes(i+1)^1 + tarrAcc(4,i+1)*C(4,m+i+1)*tnodes(i+1)^2 + tarrAcc(5,i+1)*C(5,m+i+1)*tnodes(i+1)^3 +tarrAcc(6,i+1)*C(6,m+i+1)*tnodes(i+1)^4 + tarrAcc(7,i+1)*C(7,m+i+1)*tnodes(i+1)^5 +tarrAcc(8,i+1)*C(8,m+i+1)*tnodes(i+1)^6;
    SmoothJerky(i) = tarrJerk(1,i+1)*C(1,m+i)*tnodes(i+1)^0 + tarrJerk(2,i+1)*C(2,m+i)*tnodes(i+1)^0 + tarrJerk(3,i+1)*C(3,m+i)*tnodes(i+1)^1 + tarrJerk(4,i+1)*C(4,m+i)*tnodes(i+1)^2 + tarrJerk(5,i+1)*C(5,m+i)*tnodes(i+1)^3 +tarrJerk(6,i+1)*C(6,m+i)*tnodes(i+1)^4 + tarrJerk(7,i+1)*C(7,m+i)*tnodes(i+1)^5 +tarrJerk(8,i+1)*C(8,m+i)*tnodes(i+1)^6 == tarrJerk(1,i+1)*C(1,m+i+1)*tnodes(i+1)^0 + tarrJerk(2,i+1)*C(2,m+i+1)*tnodes(i+1)^0 + tarrJerk(3,i+1)*C(3,m+i+1)*tnodes(i+1)^1 + tarrJerk(4,i+1)*C(4,m+i+1)*tnodes(i+1)^2 + tarrJerk(5,i+1)*C(5,m+i+1)*tnodes(i+1)^3 +tarrJerk(6,i+1)*C(6,m+i+1)*tnodes(i+1)^4 + tarrJerk(7,i+1)*C(7,m+i+1)*tnodes(i+1)^5 +tarrJerk(8,i+1)*C(8,m+i+1)*tnodes(i+1)^6;
    SmoothSnapy(i) = tarrSnap(1,i+1)*C(1,m+i)*tnodes(i+1)^0 + tarrSnap(2,i+1)*C(2,m+i)*tnodes(i+1)^0 + tarrSnap(3,i+1)*C(3,m+i)*tnodes(i+1)^1 + tarrSnap(4,i+1)*C(4,m+i)*tnodes(i+1)^2 + tarrSnap(5,i+1)*C(5,m+i)*tnodes(i+1)^3 +tarrSnap(6,i+1)*C(6,m+i)*tnodes(i+1)^4 + tarrSnap(7,i+1)*C(7,m+i)*tnodes(i+1)^5 +tarrSnap(8,i+1)*C(8,m+i)*tnodes(i+1)^6 == tarrSnap(1,i+1)*C(1,m+i+1)*tnodes(i+1)^0 + tarrSnap(2,i+1)*C(2,m+i+1)*tnodes(i+1)^0 + tarrSnap(3,i+1)*C(3,m+i+1)*tnodes(i+1)^1 + tarrSnap(4,i+1)*C(4,m+i+1)*tnodes(i+1)^2 + tarrSnap(5,i+1)*C(5,m+i+1)*tnodes(i+1)^3 +tarrSnap(6,i+1)*C(6,m+i+1)*tnodes(i+1)^4 + tarrSnap(7,i+1)*C(7,m+i+1)*tnodes(i+1)^5 +tarrSnap(8,i+1)*C(8,m+i+1)*tnodes(i+1)^6;
    SmoothFivey(i) = tarrFive(1,i+1)*C(1,m+i)*tnodes(i+1)^0 + tarrFive(2,i+1)*C(2,m+i)*tnodes(i+1)^0 + tarrFive(3,i+1)*C(3,m+i)*tnodes(i+1)^1 + tarrFive(4,i+1)*C(4,m+i)*tnodes(i+1)^2 + tarrFive(5,i+1)*C(5,m+i)*tnodes(i+1)^3 +tarrFive(6,i+1)*C(6,m+i)*tnodes(i+1)^4 + tarrFive(7,i+1)*C(7,m+i)*tnodes(i+1)^5 +tarrFive(8,i+1)*C(8,m+i)*tnodes(i+1)^6 == tarrFive(1,i+1)*C(1,m+i+1)*tnodes(i+1)^0 + tarrFive(2,i+1)*C(2,m+i+1)*tnodes(i+1)^0 + tarrFive(3,i+1)*C(3,m+i+1)*tnodes(i+1)^1 + tarrFive(4,i+1)*C(4,m+i+1)*tnodes(i+1)^2 + tarrFive(5,i+1)*C(5,m+i+1)*tnodes(i+1)^3 +tarrFive(6,i+1)*C(6,m+i+1)*tnodes(i+1)^4 + tarrFive(7,i+1)*C(7,m+i+1)*tnodes(i+1)^5 +tarrFive(8,i+1)*C(8,m+i+1)*tnodes(i+1)^6;
    SmoothSixy(i) = tarrSix(1,i+1)*C(1,m+i)*tnodes(i+1)^0 + tarrSix(2,i+1)*C(2,m+i)*tnodes(i+1)^0 + tarrSix(3,i+1)*C(3,m+i)*tnodes(i+1)^1 + tarrSix(4,i+1)*C(4,m+i)*tnodes(i+1)^2 + tarrSix(5,i+1)*C(5,m+i)*tnodes(i+1)^3 +tarrSix(6,i+1)*C(6,m+i)*tnodes(i+1)^4 + tarrSix(7,i+1)*C(7,m+i)*tnodes(i+1)^5 +tarrSix(8,i+1)*C(8,m+i)*tnodes(i+1)^6 == tarrSix(1,i+1)*C(1,m+i+1)*tnodes(i+1)^0 + tarrSix(2,i+1)*C(2,m+i+1)*tnodes(i+1)^0 + tarrSix(3,i+1)*C(3,m+i+1)*tnodes(i+1)^1 + tarrSix(4,i+1)*C(4,m+i+1)*tnodes(i+1)^2 + tarrSix(5,i+1)*C(5,m+i+1)*tnodes(i+1)^3 +tarrSix(6,i+1)*C(6,m+i+1)*tnodes(i+1)^4 + tarrSix(7,i+1)*C(7,m+i+1)*tnodes(i+1)^5 +tarrSix(8,i+1)*C(8,m+i+1)*tnodes(i+1)^6;

    SmoothVelz(i) = tarrVel(1,i+1)*C(1,2*m+i)*tnodes(i+1)^0 + tarrVel(2,i+1)*C(2,2*m+i)*tnodes(i+1)^0 + tarrVel(3,i+1)*C(3,2*m+i)*tnodes(i+1)^1 + tarrVel(4,i+1)*C(4,2*m+i)*tnodes(i+1)^2 + tarrVel(5,i+1)*C(5,2*m+i)*tnodes(i+1)^3 +tarrVel(6,i+1)*C(6,2*m+i)*tnodes(i+1)^4 + tarrVel(7,i+1)*C(7,2*m+i)*tnodes(i+1)^5 +tarrVel(8,i+1)*C(8,2*m+i)*tnodes(i+1)^6 == tarrVel(1,i+1)*C(1,2*m+i+1)*tnodes(i+1)^0 + tarrVel(2,i+1)*C(2,2*m+i+1)*tnodes(i+1)^0 + tarrVel(3,i+1)*C(3,2*m+i+1)*tnodes(i+1)^1 + tarrVel(4,i+1)*C(4,2*m+i+1)*tnodes(i+1)^2 + tarrVel(5,i+1)*C(5,2*m+i+1)*tnodes(i+1)^3 +tarrVel(6,i+1)*C(6,2*m+i+1)*tnodes(i+1)^4 + tarrVel(7,i+1)*C(7,2*m+i+1)*tnodes(i+1)^5 +tarrVel(8,i+1)*C(8,2*m+i+1)*tnodes(i+1)^6;
    SmoothAccz(i) = tarrAcc(1,i+1)*C(1,2*m+i)*tnodes(i+1)^0 + tarrAcc(2,i+1)*C(2,2*m+i)*tnodes(i+1)^0 + tarrAcc(3,i+1)*C(3,2*m+i)*tnodes(i+1)^1 + tarrAcc(4,i+1)*C(4,2*m+i)*tnodes(i+1)^2 + tarrAcc(5,i+1)*C(5,2*m+i)*tnodes(i+1)^3 +tarrAcc(6,i+1)*C(6,2*m+i)*tnodes(i+1)^4 + tarrAcc(7,i+1)*C(7,2*m+i)*tnodes(i+1)^5 +tarrAcc(8,i+1)*C(8,2*m+i)*tnodes(i+1)^6 == tarrAcc(1,i+1)*C(1,2*m+i+1)*tnodes(i+1)^0 + tarrAcc(2,i+1)*C(2,2*m+i+1)*tnodes(i+1)^0 + tarrAcc(3,i+1)*C(3,2*m+i+1)*tnodes(i+1)^1 + tarrAcc(4,i+1)*C(4,2*m+i+1)*tnodes(i+1)^2 + tarrAcc(5,i+1)*C(5,2*m+i+1)*tnodes(i+1)^3 +tarrAcc(6,i+1)*C(6,2*m+i+1)*tnodes(i+1)^4 + tarrAcc(7,i+1)*C(7,2*m+i+1)*tnodes(i+1)^5 +tarrAcc(8,i+1)*C(8,2*m+i+1)*tnodes(i+1)^6;
    SmoothJerkz(i) = tarrJerk(1,i+1)*C(1,2*m+i)*tnodes(i+1)^0 + tarrJerk(2,i+1)*C(2,2*m+i)*tnodes(i+1)^0 + tarrJerk(3,i+1)*C(3,2*m+i)*tnodes(i+1)^1 + tarrJerk(4,i+1)*C(4,2*m+i)*tnodes(i+1)^2 + tarrJerk(5,i+1)*C(5,2*m+i)*tnodes(i+1)^3 +tarrJerk(6,i+1)*C(6,2*m+i)*tnodes(i+1)^4 + tarrJerk(7,i+1)*C(7,2*m+i)*tnodes(i+1)^5 +tarrJerk(8,i+1)*C(8,2*m+i)*tnodes(i+1)^6 == tarrJerk(1,i+1)*C(1,2*m+i+1)*tnodes(i+1)^0 + tarrJerk(2,i+1)*C(2,2*m+i+1)*tnodes(i+1)^0 + tarrJerk(3,i+1)*C(3,2*m+i+1)*tnodes(i+1)^1 + tarrJerk(4,i+1)*C(4,2*m+i+1)*tnodes(i+1)^2 + tarrJerk(5,i+1)*C(5,2*m+i+1)*tnodes(i+1)^3 +tarrJerk(6,i+1)*C(6,2*m+i+1)*tnodes(i+1)^4 + tarrJerk(7,i+1)*C(7,2*m+i+1)*tnodes(i+1)^5 +tarrJerk(8,i+1)*C(8,2*m+i+1)*tnodes(i+1)^6;
    SmoothSnapz(i) = tarrSnap(1,i+1)*C(1,2*m+i)*tnodes(i+1)^0 + tarrSnap(2,i+1)*C(2,2*m+i)*tnodes(i+1)^0 + tarrSnap(3,i+1)*C(3,2*m+i)*tnodes(i+1)^1 + tarrSnap(4,i+1)*C(4,2*m+i)*tnodes(i+1)^2 + tarrSnap(5,i+1)*C(5,2*m+i)*tnodes(i+1)^3 +tarrSnap(6,i+1)*C(6,2*m+i)*tnodes(i+1)^4 + tarrSnap(7,i+1)*C(7,2*m+i)*tnodes(i+1)^5 +tarrSnap(8,i+1)*C(8,2*m+i)*tnodes(i+1)^6 == tarrSnap(1,i+1)*C(1,2*m+i+1)*tnodes(i+1)^0 + tarrSnap(2,i+1)*C(2,2*m+i+1)*tnodes(i+1)^0 + tarrSnap(3,i+1)*C(3,2*m+i+1)*tnodes(i+1)^1 + tarrSnap(4,i+1)*C(4,2*m+i+1)*tnodes(i+1)^2 + tarrSnap(5,i+1)*C(5,2*m+i+1)*tnodes(i+1)^3 +tarrSnap(6,i+1)*C(6,2*m+i+1)*tnodes(i+1)^4 + tarrSnap(7,i+1)*C(7,2*m+i+1)*tnodes(i+1)^5 +tarrSnap(8,i+1)*C(8,2*m+i+1)*tnodes(i+1)^6;
    SmoothFivez(i) = tarrFive(1,i+1)*C(1,2*m+i)*tnodes(i+1)^0 + tarrFive(2,i+1)*C(2,2*m+i)*tnodes(i+1)^0 + tarrFive(3,i+1)*C(3,2*m+i)*tnodes(i+1)^1 + tarrFive(4,i+1)*C(4,2*m+i)*tnodes(i+1)^2 + tarrFive(5,i+1)*C(5,2*m+i)*tnodes(i+1)^3 +tarrFive(6,i+1)*C(6,2*m+i)*tnodes(i+1)^4 + tarrFive(7,i+1)*C(7,2*m+i)*tnodes(i+1)^5 +tarrFive(8,i+1)*C(8,2*m+i)*tnodes(i+1)^6 == tarrFive(1,i+1)*C(1,2*m+i+1)*tnodes(i+1)^0 + tarrFive(2,i+1)*C(2,2*m+i+1)*tnodes(i+1)^0 + tarrFive(3,i+1)*C(3,2*m+i+1)*tnodes(i+1)^1 + tarrFive(4,i+1)*C(4,2*m+i+1)*tnodes(i+1)^2 + tarrFive(5,i+1)*C(5,2*m+i+1)*tnodes(i+1)^3 +tarrFive(6,i+1)*C(6,2*m+i+1)*tnodes(i+1)^4 + tarrFive(7,i+1)*C(7,2*m+i+1)*tnodes(i+1)^5 +tarrFive(8,i+1)*C(8,2*m+i+1)*tnodes(i+1)^6;
    SmoothSixz(i) = tarrSix(1,i+1)*C(1,2*m+i)*tnodes(i+1)^0 + tarrSix(2,i+1)*C(2,2*m+i)*tnodes(i+1)^0 + tarrSix(3,i+1)*C(3,2*m+i)*tnodes(i+1)^1 + tarrSix(4,i+1)*C(4,2*m+i)*tnodes(i+1)^2 + tarrSix(5,i+1)*C(5,2*m+i)*tnodes(i+1)^3 +tarrSix(6,i+1)*C(6,2*m+i)*tnodes(i+1)^4 + tarrSix(7,i+1)*C(7,2*m+i)*tnodes(i+1)^5 +tarrSix(8,i+1)*C(8,2*m+i)*tnodes(i+1)^6 == tarrSix(1,i+1)*C(1,2*m+i+1)*tnodes(i+1)^0 + tarrSix(2,i+1)*C(2,2*m+i+1)*tnodes(i+1)^0 + tarrSix(3,i+1)*C(3,2*m+i+1)*tnodes(i+1)^1 + tarrSix(4,i+1)*C(4,2*m+i+1)*tnodes(i+1)^2 + tarrSix(5,i+1)*C(5,2*m+i+1)*tnodes(i+1)^3 +tarrSix(6,i+1)*C(6,2*m+i+1)*tnodes(i+1)^4 + tarrSix(7,i+1)*C(7,2*m+i+1)*tnodes(i+1)^5 +tarrSix(8,i+1)*C(8,2*m+i+1)*tnodes(i+1)^6;
    
end

MinimumSnap.Constraints.SmoothVelx = SmoothVelx;
MinimumSnap.Constraints.SmoothAccx = SmoothAccx;
MinimumSnap.Constraints.SmoothJerkx = SmoothJerkx;
MinimumSnap.Constraints.SmoothSnapx = SmoothSnapx;
MinimumSnap.Constraints.SmoothFivex = SmoothFivex;
MinimumSnap.Constraints.SmoothSixx = SmoothSixx;

MinimumSnap.Constraints.SmoothVely = SmoothVely;
MinimumSnap.Constraints.SmoothAccy = SmoothAccy;
MinimumSnap.Constraints.SmoothJerky = SmoothJerky;
MinimumSnap.Constraints.SmoothSnapy = SmoothSnapy;
MinimumSnap.Constraints.SmoothFivey = SmoothFivey;
MinimumSnap.Constraints.SmoothSixy = SmoothSixy;

MinimumSnap.Constraints.SmoothVelz = SmoothVelz;
MinimumSnap.Constraints.SmoothAccz = SmoothAccz;
MinimumSnap.Constraints.SmoothJerkz = SmoothJerkz;
MinimumSnap.Constraints.SmoothSnapz = SmoothSnapz;
MinimumSnap.Constraints.SmoothFivez = SmoothFivez;
MinimumSnap.Constraints.SmoothSixz = SmoothSixz;

% Cost
Ccostx = reshape(C(:,1:m),[Coeff*m,1]);
Ccosty = reshape(C(:,m+1:2*m),[Coeff*m,1]);
Ccostz = reshape(C(:,2*m+1:3*m),[Coeff*m,1]);
H = eye(Coeff*m);

Cost = Ccostx'*H*Ccostx + Ccosty'*H*Ccosty + Ccostz'*H*Ccostz + sum(tnodes*tnodes.'); 
MinimumSnap.Objective = Cost;

%=================================
% Solve QP Problem
%=================================
% optionsLP = optimoptions('quadprog','Display','iter'); 
Z_Guess.c0 = 10*ones(3*m,1);
Z_Guess.c1 = 10*ones(3*m,1);
Z_Guess.c2 = 10*ones(3*m,1);
Z_Guess.c3 = 10*ones(3*m,1);
Z_Guess.c4 = 10*ones(3*m,1);
Z_Guess.c5 = 10*ones(3*m,1);
Z_Guess.c6 = 10*ones(3*m,1);
Z_Guess.c7 = 10*ones(3*m,1);
%Z_Guess.tnodes = ones(n,1);
Z_Guess.t0 = zeros(1,1);
Z_Guess.t3 = tf*ones(1,1);
Z_Guess.t1 = 1.5*ones(1,1);
Z_Guess.t2 = 3*ones(1,1);

options_fmincon = optimoptions(@fmincon,'Algorithm','interior-point',...
    'MaxIter',2000*3,'MaxFunEvals',800000*3,...
    'TolX',1e-10,'TolFun',1e-10,'TolCon',1e-10,'Display','iter-detailed');
[sol, fval] = solve(MinimumSnap,Z_Guess, 'options', options_fmincon);

C0 = sol.c0;
C1 = sol.c1;
C2 = sol.c2;
C3 = sol.c3;
C4 = sol.c4;
C5 = sol.c5;
C6 = sol.c6;
C7 = sol.c7;
T0 = sol.t0;
T1 = sol.t1;
T2 = sol.t2;
T3 = sol.t3;
tnodesS = [T0, T1, T2, T3];
nsub = 15;
timeS1 = linspace(0,tnodesS(2),nsub); 
timeS2 = linspace(tnodesS(2),tnodesS(3),nsub); 
timeS3 = linspace(tnodesS(3),tnodesS(4),nsub); 
timeS = [timeS1,timeS2,timeS3];

for i = 1:m
    for j = 1:nsub
        xS(j+nsub*(i-1))= C0(i) + C1(i)*timeS(j+nsub*(i-1)) + C2(i)*timeS(j+nsub*(i-1))^2 + C3(i)*timeS(j+nsub*(i-1))^3 + C4(i)*timeS(j+nsub*(i-1))^4 + C5(i)*timeS(j+nsub*(i-1))^5 + C6(i)*timeS(j+nsub*(i-1))^6 + C7(i)*timeS(j+nsub*(i-1))^7;  
        yS(j+nsub*(i-1))= C0(m+i) + C1(m+i)*timeS(j+nsub*(i-1)) + C2(m+i)*timeS(j+nsub*(i-1))^2 + C3(m+i)*timeS(j+nsub*(i-1))^3 + C4(m+i)*timeS(j+nsub*(i-1))^4 + C5(m+i)*timeS(j+nsub*(i-1))^5 + C6(m+i)*timeS(j+nsub*(i-1))^6 + C7(m+i)*timeS(j+nsub*(i-1))^7;  
        zS(j+nsub*(i-1))= C0(2*m+i) + C1(2*m+i)*timeS(j+nsub*(i-1)) + C2(2*m+i)*timeS(j+nsub*(i-1))^2 + C3(2*m+i)*timeS(j+nsub*(i-1))^3 + C4(2*m+i)*timeS(j+nsub*(i-1))^4 + C5(2*m+i)*timeS(j+nsub*(i-1))^5 + C6(2*m+i)*timeS(j+nsub*(i-1))^6 + C7(2*m+i)*timeS(j+nsub*(i-1))^7;  
    end
end

figure(1)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on; 
plot3(xS,yS,zS,'b','LineWidth',2)
xlabel('x(t)','FontSize', 14,'FontWeight', 'bold'); 
ylabel('y(t)','FontSize', 14,'FontWeight', 'bold'); 
zlabel('z(t)','FontSize', 14,'FontWeight', 'bold'); 
title('Minimum Snap Trajectory', 'interpreter', 'latex','FontSize', 16);
legend('Hoops','','','','Minimum Snap Trajectory','FontSize', 12,'FontWeight', 'bold'); 

