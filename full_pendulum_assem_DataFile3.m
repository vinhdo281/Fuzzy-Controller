% Simscape(TM) Multibody(TM) version: 7.6

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(5).translation = [0.0 0.0 0.0];
smiData.RigidTransform(5).angle = 0.0;
smiData.RigidTransform(5).axis = [0.0 0.0 0.0];
smiData.RigidTransform(5).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [40.000000000000007 -27.500000000000004 19.999999999999989];  % mm
smiData.RigidTransform(1).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(1).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(1).ID = "B[cart-1:-:pole-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [-27.500000000000014 -20 -49.999999999999943];  % mm
smiData.RigidTransform(2).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(2).axis = [1 -5.5511151231257827e-17 -5.5511151231257827e-17];
smiData.RigidTransform(2).ID = "F[cart-1:-:pole-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 45.388543819998326 -19.999999999999989];  % mm
smiData.RigidTransform(3).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(3).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(3).ID = "B[cart-1:-:worldframe-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [0 -7.1054273576010019e-15 -45.388543819998326];  % mm
smiData.RigidTransform(4).angle = 1.5707963267948966;  % rad
smiData.RigidTransform(4).axis = [-7.8504622934188758e-17 -7.8504622934188758e-17 -1];
smiData.RigidTransform(4).ID = "F[cart-1:-:worldframe-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [17.751058450381148 -41.141762203706932 -188.28865932851707];  % mm
smiData.RigidTransform(5).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(5).axis = [0 -0.70710678118654746 0.70710678118654757];
smiData.RigidTransform(5).ID = "RootGround[worldframe-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(3).mass = 0.0;
smiData.Solid(3).CoM = [0.0 0.0 0.0];
smiData.Solid(3).MoI = [0.0 0.0 0.0];
smiData.Solid(3).PoI = [0.0 0.0 0.0];
smiData.Solid(3).color = [0.0 0.0 0.0];
smiData.Solid(3).opacity = 0.0;
smiData.Solid(3).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 0.019633826078982265;  % kg
smiData.Solid(1).CoM = [-0.00076311386108514985 110.6285277622751 33.478932080894545];  % mm
smiData.Solid(1).MoI = [197.33090023537565 11.898645849201205 185.58958145071156];  % kg*mm^2
smiData.Solid(1).PoI = [-25.023301917275614 4.4059549081425583e-05 0.00050313328266398836];  % kg*mm^2
smiData.Solid(1).color = [0.89411764705882346 0.89411764705882346 0.89411764705882346];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "pole*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.05911769838837861;  % kg
smiData.Solid(2).CoM = [0 0 -8.4268671803051269];  % mm
smiData.Solid(2).MoI = [21.955477069530993 58.486924496413558 65.128817780223798];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0.89411764705882346 0.89411764705882346 0.89411764705882346];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "cart*:*Default";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.11026990214100178;  % kg
smiData.Solid(3).CoM = [0 0 0];  % mm
smiData.Solid(3).MoI = [8736.4362393517513 8736.4362393517513 1.9848582385380329];  % kg*mm^2
smiData.Solid(3).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).color = [0.89411764705882346 0.89411764705882346 0.89411764705882346];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "worldframe*:*Default";

