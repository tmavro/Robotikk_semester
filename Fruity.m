clear all; close all; clc 

%startup_rvc 

% ----------
% Robot arm 
% ----------

syms L1 L2 L3 L4 L5 L6 q1 q2 q3 q4 q5 q6

L1 = 1; L2 = 3; L3 =3; L4 = 1; L5 = 3; L6 = 3;

L(1) = Link('a',L1,'d',0,'alpha',pi/2); 
L(2) = Link('a',L2,'d',0,'alpha',0);
L(3) = Link('a',L3,'d',0,'alpha',-pi/2);
L(4) = Link('a',L4,'d',0,'alpha',pi/2); 
L(5) = Link('a',L5,'d',0,'alpha',0);
L(6) = Link('a',L6,'d',0,'alpha',0);

Fruity = SerialLink(L, 'name', 'Fruity')

% -------------------
% Forward Kinematics 
% -------------------

% Forward Kinematics Matrix

% Symbolic matrix
%T = simplify(vpa(Fruity.fkine([q1 q2 q3 q4 q5 q6]),2))

% Numeric matrix set to '0' position
T = Fruity.fkine([0 0 0 0 0 0])

%Defining poses we'd like robot to assume
q1 = [0 pi/2 pi/2 0 pi/2 -pi/2]; %idle pos
q2 = [pi/4 pi/4 -pi/4 -pi/4 pi/4 pi/4]; %fruitgrab
q3 = [pi/4 pi/4 -pi/4 -pi/4 0 pi/2]; %fruitpick
q4 = [pi/4 pi/4 -pi/2 pi/2 0 pi/2]; %intermediate
q5 = [0 pi/2 pi/2 pi/4 pi/4 0]; %dropfruit

%Finding the joint space trajectories between poses
qj1 = jtraj(q1, q3, 30);
qj2 = jtraj(q3, q2, 10);
qj3 = jtraj(q2, q3, 20);
qj4 = jtraj(q3, q4, 20);
qj5 = jtraj(q4, q5, 20);
qj6 = jtraj(q5, q1, 20);

%Animating motion of arm using forward kinematics
 for i=1:6
   fn = eval(sprintf('qj%d', i));
   Fruity.plot(fn)
 end

% -------------------
% Inverse Kinematics
% -------------------

% We're able to take the transfer of a pose
T1 = Fruity.fkine(q2)
% And, through inverse kinematics, find a pose with the 
% end-effector in the same orientation as the previous pose
qi = Fruity.ikine(T1)


