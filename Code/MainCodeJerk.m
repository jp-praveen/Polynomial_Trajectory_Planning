% Author: Praveen
% Advance Trajectory Optimization Project
% This script will solve a minimum jerk trajectory problem as a QP.

clear
clc
close all

%% Problem Setup
order = 3;
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
% 
% [ynodes,xnodes] = ginput(n); 
% xnodes = [0; 2; 3; 1];
% ynodes = [0; 2; 3; 1];
% znodes = [0; 2; 3; 1];
% %angnodes = [5; 10; 5; 7];
% angnodes = [0; 2; 3; 1];
% tnodes = [0, 1, 3, 5];

% For predefined node points use this
xnodes = [0; 2; 0; -2];
ynodes = [0; 1; 0; -1];
znodes = [1; 0; -1; 0];
angnodes = [0; 2; 3; 1];
tnodes = [0, 1.5, 3, 4.5];

% Empty time arrays
tarrPos = zeros(Coeff,n);
tarrVel = zeros(Coeff,n);
tarrAcc = zeros(Coeff,n);
tarrJerk = zeros(Coeff,n);
tarrSnap = zeros(Coeff,n);

for i = 1:Coeff
    if i == 1
        tarrPos(i,:) = ones(1,n);
        tarrVel(i,:) = zeros(1,n);
        tarrAcc(i,:) = zeros(1,n);
    else
        if i == 2
            tarrAcc(i,:) = zeros(1,n);
        else
            tarrAcc(i,:) = factorial(i-1)/factorial(i-3)*tnodes.^(i-3);
        end
        tarrVel(i,:) = (i-1)*tnodes.^(i-2);
        tarrPos(i,:) = tnodes.^(i-1);
    end
end
for i = 1:Coeff
    if i <= 3
        tarrJerk(i,:) = zeros(1,n);
    elseif i == 4 
        tarrJerk(i,:) = 6*tnodes.^(i-4);
    elseif i == 5
         tarrJerk(i,:) = 24*tnodes.^(i-4); 
    elseif i == 6
         tarrJerk(i,:) = 60*tnodes.^(i-4); 
    end
end
for i = 1:Coeff
    if i<=4
        tarrSnap(i,:) = zeros(1,n);
    elseif i == 5
        tarrSnap(i,:) = 24*tnodes.^(i-5);
    elseif i == 6
        tarrSnap(i,:) = 120*tnodes.^(i-5);
    end
end

% Initialize fuel-optimal Problem 
MinimumJerk = optimproblem('ObjectiveSense', 'minimize');

C = [optimvar('c0',1, 4*m, 'LowerBound', -1000000, 'UpperBound', 1000000);
     optimvar('c1',1, 4*m, 'LowerBound', -1000000, 'UpperBound', 1000000);
     optimvar('c2',1, 4*m, 'LowerBound', -1000000, 'UpperBound', 1000000);
     optimvar('c3',1, 4*m, 'LowerBound', -1000000, 'UpperBound', 1000000);
     optimvar('c4',1, 4*m, 'LowerBound', -1000000, 'UpperBound', 1000000);
     optimvar('c5',1, 4*m, 'LowerBound', -1000000, 'UpperBound', 1000000)];
 
% Position Constraints
for i = 1:m
    PosNodeOddx(i) = C(:,i)'*tarrPos(:,i) == xnodes(i);
    PosNodeOddy(i) = C(:,i+m)'*tarrPos(:,i) == ynodes(i);
    PosNodeOddz(i) = C(:,i+2*m)'*tarrPos(:,i) == znodes(i);
    PosNodeOddang(i) = C(:,i+3*m)'*tarrPos(:,i) == angnodes(i);
end

for i = 1:m
    PosNodeEvenx(i) = C(:,i)'*tarrPos(:,i+1) == xnodes(i+1);
    PosNodeEveny(i) = C(:,i+m)'*tarrPos(:,i+1) == ynodes(i+1);
    PosNodeEvenz(i) = C(:,i+2*m)'*tarrPos(:,i+1) == znodes(i+1);
    PosNodeEvenang(i) = C(:,i+3*m)'*tarrPos(:,i+1) == angnodes(i+1);
end

MinimumJerk.Constraints.PosNodeOddx = PosNodeOddx; 
MinimumJerk.Constraints.PosNodeEvenx = PosNodeEvenx; 
MinimumJerk.Constraints.PosNodeOddy = PosNodeOddy; 
MinimumJerk.Constraints.PosNodeEveny = PosNodeEveny; 
MinimumJerk.Constraints.PosNodeOddz = PosNodeOddz; 
MinimumJerk.Constraints.PosNodeEvenz = PosNodeEvenz; 
MinimumJerk.Constraints.PosNodeOddang = PosNodeOddang; 
MinimumJerk.Constraints.PosNodeEvenang = PosNodeEvenang; 

% Initial and Final velocity Constraint
VelNode1x = C(:,1)'*tarrVel(:,1) == 0;
VelNodefx = C(:,m)'*tarrVel(:,end) == 0;
VelNode1y = C(:,1+m)'*tarrVel(:,1) == 0;
VelNodefy = C(:,2*m)'*tarrVel(:,end) == 0;
VelNode1z = C(:,1+2*m)'*tarrVel(:,1) == 0;
VelNodefz = C(:,3*m)'*tarrVel(:,end) == 0;
VelNode1ang = C(:,1+3*m)'*tarrVel(:,1) == 0;
VelNodefang = C(:,4*m)'*tarrVel(:,end) == 0;

MinimumJerk.Constraints.VelNode1x = VelNode1x;
MinimumJerk.Constraints.VelNodefx = VelNodefx;
MinimumJerk.Constraints.VelNode1y = VelNode1y;
MinimumJerk.Constraints.VelNodefy = VelNodefy;
MinimumJerk.Constraints.VelNode1z = VelNode1z;
MinimumJerk.Constraints.VelNodefz = VelNodefz;
MinimumJerk.Constraints.VelNode1ang = VelNode1ang;
MinimumJerk.Constraints.VelNodefang = VelNodefang;

% Initial and Final acceleration Constraint
AccNode1x = C(:,1)'*tarrAcc(:,1) == 0;
AccNodefx = C(:,m)'*tarrAcc(:,end) == 0;
AccNode1y = C(:,1+m)'*tarrAcc(:,1) == 0;
AccNodefy = C(:,2*m)'*tarrAcc(:,end) == 0;
AccNode1z = C(:,1+2*m)'*tarrAcc(:,1) == 0;
AccNodefz = C(:,3*m)'*tarrAcc(:,end) == 0;
AccNode1ang = C(:,1+3*m)'*tarrAcc(:,1) == 0;
AccNodefang = C(:,4*m)'*tarrAcc(:,end) == 0;

MinimumJerk.Constraints.AccNode1x = AccNode1x;
MinimumJerk.Constraints.AccNodefx = AccNodefx;
MinimumJerk.Constraints.AccNode1y = AccNode1y;
MinimumJerk.Constraints.AccNodefy = AccNodefy;
MinimumJerk.Constraints.AccNode1z = AccNode1z;
MinimumJerk.Constraints.AccNodefz = AccNodefz;
MinimumJerk.Constraints.AccNode1ang = AccNode1ang;
MinimumJerk.Constraints.AccNodefang = AccNodefang;

% Smoothing higher order Constraints 
for i = 1:m-1
    SmoothVelx(i) = C(:,i)'*tarrVel(:,i+1) == C(:,i+1)'*tarrVel(:,i+1);
    SmoothAccx(i) = C(:,i)'*tarrAcc(:,i+1) == C(:,i+1)'*tarrAcc(:,i+1);
    SmoothJerkx(i) = C(:,i)'*tarrJerk(:,i+1) == C(:,i+1)'*tarrJerk(:,i+1); 
    SmoothSnapx(i) = C(:,i)'*tarrSnap(:,i+1) == C(:,i+1)'*tarrSnap(:,i+1); 
    
    SmoothVely(i) = C(:,m+i)'*tarrVel(:,i+1) == C(:,m+i+1)'*tarrVel(:,i+1);
    SmoothAccy(i) = C(:,m+i)'*tarrAcc(:,i+1) == C(:,m+i+1)'*tarrAcc(:,i+1);
    SmoothJerky(i) = C(:,m+i)'*tarrJerk(:,i+1) == C(:,m+i+1)'*tarrJerk(:,i+1); 
    SmoothSnapy(i) = C(:,m+i)'*tarrSnap(:,i+1) == C(:,m+i+1)'*tarrSnap(:,i+1); 

    SmoothVelz(i) = C(:,2*m+i)'*tarrVel(:,i+1) == C(:,2*m+i+1)'*tarrVel(:,i+1);
    SmoothAccz(i) = C(:,2*m+i)'*tarrAcc(:,i+1) == C(:,2*m+i+1)'*tarrAcc(:,i+1);
    SmoothJerkz(i) = C(:,2*m+i)'*tarrJerk(:,i+1) == C(:,2*m+i+1)'*tarrJerk(:,i+1); 
    SmoothSnapz(i) = C(:,2*m+i)'*tarrSnap(:,i+1) == C(:,2*m+i+1)'*tarrSnap(:,i+1); 
    
    SmoothVelang(i) = C(:,3*m+i)'*tarrVel(:,i+1) == C(:,3*m+i+1)'*tarrVel(:,i+1);
    SmoothAccang(i) = C(:,3*m+i)'*tarrAcc(:,i+1) == C(:,3*m+i+1)'*tarrAcc(:,i+1);
    SmoothJerkang(i) = C(:,3*m+i)'*tarrJerk(:,i+1) == C(:,3*m+i+1)'*tarrJerk(:,i+1); 
    SmoothSnapang(i) = C(:,3*m+i)'*tarrSnap(:,i+1) == C(:,3*m+i+1)'*tarrSnap(:,i+1); 
  
end

MinimumJerk.Constraints.SmoothVelx = SmoothVelx;
MinimumJerk.Constraints.SmoothAccx = SmoothAccx;
MinimumJerk.Constraints.SmoothJerkx = SmoothJerkx;
MinimumJerk.Constraints.SmoothSnapx = SmoothSnapx;

MinimumJerk.Constraints.SmoothVely = SmoothVely;
MinimumJerk.Constraints.SmoothAccy = SmoothAccy;
MinimumJerk.Constraints.SmoothJerky = SmoothJerky;
MinimumJerk.Constraints.SmoothSnapy = SmoothSnapy;

MinimumJerk.Constraints.SmoothVelz = SmoothVelz;
MinimumJerk.Constraints.SmoothAccz = SmoothAccz;
MinimumJerk.Constraints.SmoothJerkz = SmoothJerkz;
MinimumJerk.Constraints.SmoothSnapz = SmoothSnapz;

MinimumJerk.Constraints.SmoothVelang = SmoothVelang;
MinimumJerk.Constraints.SmoothAccang = SmoothAccang;
MinimumJerk.Constraints.SmoothJerkang = SmoothJerkang;
MinimumJerk.Constraints.SmoothSnapang = SmoothSnapang;

% Cost to minimize
Ccostx = reshape(C(:,1:m),[Coeff*m,1]);
Ccosty = reshape(C(:,m+1:2*m),[Coeff*m,1]);
Ccostz = reshape(C(:,2*m+1:3*m),[Coeff*m,1]);
Ccostang = reshape(C(:,3*m+1:4*m),[Coeff*m,1]);

H = eye(Coeff*m);

Cost = Ccostx'*H*Ccostx + Ccosty'*H*Ccosty + Ccostz'*H*Ccostz + Ccostang'*H*Ccostang;
MinimumJerk.Objective = Cost;

%=================================
% Solve QP Problem
%=================================
optionsLP = optimoptions('quadprog','Display','iter'); 
[sol, fval] = solve(MinimumJerk, 'options', optionsLP);

C0 = sol.c0;
C1 = sol.c1;
C2 = sol.c2;
C3 = sol.c3;
C4 = sol.c4;
C5 = sol.c5;

nsub = 15;
timeS1 = linspace(0,tnodes(2),nsub); 
timeS2 = linspace(tnodes(2),tnodes(3),nsub); 
timeS3 = linspace(tnodes(3),tnodes(4),nsub); 


for i = 1:nsub
    xS1(i) = C0(1) + C1(1)*timeS1(i) + C2(1)*timeS1(i)^2 + C3(1)*timeS1(i)^3 + C4(1)*timeS1(i)^4 + C5(1)*timeS1(i)^5; 
    xS2(i) = C0(2) + C1(2)*timeS2(i) + C2(2)*timeS2(i)^2 + C3(2)*timeS2(i)^3 + C4(2)*timeS2(i)^4 + C5(2)*timeS2(i)^5; 
    xS3(i) = C0(3) + C1(3)*timeS3(i) + C2(3)*timeS3(i)^2 + C3(3)*timeS3(i)^3 + C4(3)*timeS3(i)^4 + C5(3)*timeS3(i)^5;   
    
    yS1(i) = C0(4) + C1(4)*timeS1(i) + C2(4)*timeS1(i)^2 + C3(4)*timeS1(i)^3 + C4(4)*timeS1(i)^4 + C5(4)*timeS1(i)^5; 
    yS2(i) = C0(5) + C1(5)*timeS2(i) + C2(5)*timeS2(i)^2 + C3(5)*timeS2(i)^3 + C4(5)*timeS2(i)^4 + C5(5)*timeS2(i)^5; 
    yS3(i) = C0(6) + C1(6)*timeS3(i) + C2(6)*timeS3(i)^2 + C3(6)*timeS3(i)^3 + C4(6)*timeS3(i)^4 + C5(6)*timeS3(i)^5;
    
    zS1(i) = C0(7) + C1(7)*timeS1(i) + C2(7)*timeS1(i)^2 + C3(7)*timeS1(i)^3 + C4(7)*timeS1(i)^4 + C5(7)*timeS1(i)^5; 
    zS2(i) = C0(8) + C1(8)*timeS2(i) + C2(8)*timeS2(i)^2 + C3(8)*timeS2(i)^3 + C4(8)*timeS2(i)^4 + C5(8)*timeS2(i)^5; 
    zS3(i) = C0(9) + C1(9)*timeS3(i) + C2(9)*timeS3(i)^2 + C3(9)*timeS3(i)^3 + C4(9)*timeS3(i)^4 + C5(9)*timeS3(i)^5;
    
    angS1(i) = C0(10) + C1(10)*timeS1(i) + C2(10)*timeS1(i)^2 + C3(10)*timeS1(i)^3 + C4(10)*timeS1(i)^4 + C5(10)*timeS1(i)^5; 
    angS2(i) = C0(11) + C1(11)*timeS2(i) + C2(11)*timeS2(i)^2 + C3(11)*timeS2(i)^3 + C4(11)*timeS2(i)^4 + C5(11)*timeS2(i)^5; 
    angS3(i) = C0(12) + C1(12)*timeS3(i) + C2(12)*timeS3(i)^2 + C3(12)*timeS3(i)^3 + C4(12)*timeS3(i)^4 + C5(12)*timeS3(i)^5;
    
end

% PLOTTING

figure(1)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on; 
plot3(xS1,yS1,zS1, 'linewidth', 2)
plot3(xS2,yS2,zS2, 'linewidth', 2)
plot3(xS3,yS3,zS3, 'linewidth', 2)
xlabel('$x$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
ylabel('$y$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
zlabel('$z$ ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
title('Minimum Jerk Trajectory', 'interpreter', 'latex','FontSize', 16); 
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

figure(5)
set(gca,'FontSize', 12, 'FontWeight', 'bold')
hold on; grid on; box on; 
plot(timeS1,angS1, 'linewidth', 2)
plot(timeS2,angS2, 'linewidth', 2)
plot(timeS3,angS3, 'linewidth', 2)
xlabel('Time ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
ylabel('heading ', 'interpreter', 'latex','FontSize', 16,'FontWeight', 'bold'); 
title('Heading Propagation', 'interpreter', 'latex','FontSize', 16);
legend('Segment-1', 'Segment-2', 'Segment-3','interpreter', 'latex','FontSize', 15,'FontWeight', 'bold'); 

