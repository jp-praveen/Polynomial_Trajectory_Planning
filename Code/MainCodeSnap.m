% Author: Praveen
% Advance Trajectory Optimization Project
% This script will solve a minimum snap trajectory problem as a QP.

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

% For a predefined obstacle use this
% figure(1)
% hold on
% rectangle('Position',[1 3 0.5 2],'FaceColor','b')
% rectangle('Position',[1 0 0.5 2],'FaceColor','b')
% rectangle('Position',[2 2.5 0.5 2.5],'FaceColor','b')
% rectangle('Position',[2 0 0.5 1.5],'FaceColor','b')
% axis([0 5 0 5])
% % 
% [xnodes,ynodes] = ginput(n); 

% For predefined node points use this
xnodes = [0; 2; 3; 1];
ynodes = [0; 2; 3; 1];
znodes = [0; 2; 3; 1];
tnodes = [0, 1, 3, 5];

% znodes = [0; 0; 0; 0];
% tnodes = [0, 1, 2, 3];

% Empty time arrays
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

Cost = Ccostx'*H*Ccostx + Ccosty'*H*Ccosty + Ccostz'*H*Ccostz; 
MinimumSnap.Objective = Cost;

%=================================
% Solve QP Problem
%=================================
optionsLP = optimoptions('quadprog','Display','iter'); 
[sol, fval] = solve(MinimumSnap, 'options', optionsLP);

C0 = sol.c0;
C1 = sol.c1;
C2 = sol.c2;
C3 = sol.c3;
C4 = sol.c4;
C5 = sol.c5;
C6 = sol.c6;
C7 = sol.c7;

nsub = 15;
timeS1 = linspace(0,tnodes(2),nsub); 
timeS2 = linspace(tnodes(2),tnodes(3),nsub); 
timeS3 = linspace(tnodes(3),tnodes(4),nsub); 

for i = 1:nsub
    xS1(i) = C0(1) + C1(1)*timeS1(i) + C2(1)*timeS1(i)^2 + C3(1)*timeS1(i)^3 + C4(1)*timeS1(i)^4 + C5(1)*timeS1(i)^5 + C6(1)*timeS1(i)^6 + C7(1)*timeS1(i)^7; 
    xS2(i) = C0(2) + C1(2)*timeS2(i) + C2(2)*timeS2(i)^2 + C3(2)*timeS2(i)^3 + C4(2)*timeS2(i)^4 + C5(2)*timeS2(i)^5 + C6(2)*timeS2(i)^6 + C7(2)*timeS2(i)^7; 
    xS3(i) = C0(3) + C1(3)*timeS3(i) + C2(3)*timeS3(i)^2 + C3(3)*timeS3(i)^3 + C4(3)*timeS3(i)^4 + C5(3)*timeS3(i)^5 + C6(3)*timeS3(i)^6 + C7(3)*timeS3(i)^7;
    
    yS1(i) = C0(4) + C1(4)*timeS1(i) + C2(4)*timeS1(i)^2 + C3(4)*timeS1(i)^3 + C4(4)*timeS1(i)^4 + C5(4)*timeS1(i)^5 + C6(4)*timeS1(i)^6 + C7(4)*timeS1(i)^7; 
    yS2(i) = C0(5) + C1(5)*timeS2(i) + C2(5)*timeS2(i)^2 + C3(5)*timeS2(i)^3 + C4(5)*timeS2(i)^4 + C5(5)*timeS2(i)^5 + C6(5)*timeS2(i)^6 + C7(5)*timeS2(i)^7; 
    yS3(i) = C0(6) + C1(6)*timeS3(i) + C2(6)*timeS3(i)^2 + C3(6)*timeS3(i)^3 + C4(6)*timeS3(i)^4 + C5(6)*timeS3(i)^5 + C6(6)*timeS3(i)^6 + C7(6)*timeS3(i)^7;
    
    zS1(i) = C0(7) + C1(7)*timeS1(i) + C2(7)*timeS1(i)^2 + C3(7)*timeS1(i)^3 + C4(7)*timeS1(i)^4 + C5(7)*timeS1(i)^5 + C6(7)*timeS1(i)^6 + C7(7)*timeS1(i)^7; 
    zS2(i) = C0(8) + C1(8)*timeS2(i) + C2(8)*timeS2(i)^2 + C3(8)*timeS2(i)^3 + C4(8)*timeS2(i)^4 + C5(8)*timeS2(i)^5 + C6(8)*timeS2(i)^6 + C7(8)*timeS2(i)^7; 
    zS3(i) = C0(9) + C1(9)*timeS3(i) + C2(9)*timeS3(i)^2 + C3(9)*timeS3(i)^3 + C4(9)*timeS3(i)^4 + C5(9)*timeS3(i)^5 + C6(9)*timeS3(i)^6 + C7(9)*timeS3(i)^7;
    
end

%PLOTTING

figure(1)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on; 
plot3(xS1,yS1,zS1, 'linewidth', 2)
plot3(xS2,yS2,zS2, 'linewidth', 2)
plot3(xS3,yS3,zS3, 'linewidth', 2)
xlabel('$x$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
ylabel('$y$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
zlabel('$z$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
title('Minimum Snap Trajectory', 'interpreter', 'latex','FontSize', 16); 
legend('Segment-1', 'Segment-2', 'Segment-3','interpreter', 'latex','FontSize', 15,'FontWeight', 'bold'); 

figure(2)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on; 
plot(timeS1,xS1, 'linewidth', 2)
plot(timeS2,xS2, 'linewidth', 2)
plot(timeS3,xS3, 'linewidth', 2)
xlabel('Time ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
ylabel('$x$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
title('Propagation along x-axis', 'interpreter', 'latex','FontSize', 16); 
legend('Segment-1', 'Segment-2', 'Segment-3','interpreter', 'latex','FontSize', 15,'FontWeight', 'bold'); 

figure(3)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on; 
plot(timeS1,yS1, 'linewidth', 2)
plot(timeS2,yS2, 'linewidth', 2)
plot(timeS3,yS3, 'linewidth', 2)
xlabel('Time ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
ylabel('$y$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
title('Propagation along y-axis', 'interpreter', 'latex','FontSize', 16);
legend('Segment-1', 'Segment-2', 'Segment-3','interpreter', 'latex','FontSize', 15,'FontWeight', 'bold'); 

figure(4)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on;  
plot(timeS1,zS1, 'linewidth', 2)
plot(timeS2,zS2, 'linewidth', 2)
plot(timeS3,zS3, 'linewidth', 2)
xlabel('Time ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
ylabel('$z$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
title('Propagation along z-axis', 'interpreter', 'latex','FontSize', 16);
legend('Segment-1', 'Segment-2', 'Segment-3','interpreter', 'latex','FontSize', 15,'FontWeight', 'bold'); 

