clear all

% Arm params and model
ua_l = 0.5;
fa_l = 0.5;
ha_l = 0.2;
ISB_UL = ISB7DofUL(ua_l, fa_l, ha_l);
arm_mass = 4.777;

% Min Jerk hand traj in shoulder parasagittal plane
T=0.7; %s
prefix_filename = "UpAndStopAndDown"+T+"s_";
plot=true;

% I changed the first value in the jerk by times 2
%[ti, Xi, dXi]=Jerk(0.2*500, 0.2, [0 0.7 -0.5], [0 0.7 -0.5]); %stop
[tup, Xup, dXup]=Jerk(T*500, T, [0 0.7 -0.5], [0 0.7 -0.5]); %wrist sholder elbow%Forward up mvt
%[tup, Xup, dXup]=Jerk(T*500, T, [0 0.7 -0.5], [0 1.1 0]); %Forward up mvt
%[tst, Xst, dXst]=Jerk(0.2*500, 0.2, [0 1.1 0], [0 1.1 0]); %stop
%[tdw, Xdw, dXdw]=Jerk(T*500, T, [0 1.1 0], [0 0.7 -0.5]); %Backward down mvt
% t=[ti,tup+ti(end),tst+tup(end)+ti(end)];
% X=[Xi,Xup,Xst];
% dX=[dXi,dXup,Xst];

t=[tup];
X=[Xup];
dX=[dXup];
% Quick trick for now: constant orientation (will be ignored by IK mask
% option)
for i=1:length(X)
    TT(:,:,i)=eye(4);
    TT(1:3,4,i)=X(:,i);
end

% IK with arbitrary redundancy
q=ISB_UL.ikine(TT, 'q0', [pi/2 0 0 pi/2 0 0 0], 'mask', [1 1 1 0 0 0]);
q=round(q,3);%don't want 10-26 precision

% Plot arm posture
if(plot)
    figure();
    for i=1:50:length(q)
         ISB_UL.plot(q(i,:));
    end
end

% Compute gravity compensation

%Prepare transform from toolbox frame to opensim model frame: z -> y, y -> x, x -> z
RefFrameToolboxToOpenSIM = [0 1 0 0;0 0 1 0; 1 0 0 0; 0 0 0 1];
g=9.81;
%Constant force
F_const_Ref = zeros(4,length(q));
for i=1:length(q)
    F_const_Ref(:,i) = RefFrameToolboxToOpenSIM*[0;0;g*arm_mass;1];
end
%Relative position (to wrist) of force application: 0
F_p = zeros(length(F_const_Ref),3);
F_zero = zeros(length(F_const_Ref),3);

%% Export to OpenSIM .sto file (https://simtk-confluence.stanford.edu/display/OpenSim/Storage+%28.sto%29+Files)
% One file for joint angles and one for external forces (generate 3 different ones: 0, constant up, our deweighting)
%OpenSIM MoBL_ARMS model frame is defined with x+ forward, y+ up, z+ external right
%Forces are to be expressed in this ground frame



% Motion file
motion_filename=prefix_filename+"JointsKin.sto";
fileD = fopen(motion_filename, 'w');
%header
fprintf(fileD, "%s\n", motion_filename);
fprintf(fileD, "nRows=%d\n", size(q,1)+1);
fprintf(fileD, "nColumns=%d\n", 21); %% All these required by MoBL_ARMS model. Most are zero
fprintf(fileD, "inDegrees=no\n");
fprintf(fileD, "endheader\n");
z=zeros(length(q), 1);
o=ones(length(q), 1);
%MoBL_ARMS requires shoulder1_r2 to be -pi/2 to match...
fprintf(fileD,'time\tsternoclavicular_r2\tsternoclavicular_r3\tunrotscap_r3\tunrotscap_r2\tacromioclavicular_r2\tacromioclavicular_r3\tacromioclavicular_r1\tunrothum_r1\tunrothum_r3\tunrothum_r2\telv_angle\tshoulder_elv\tshoulder1_r2\tshoulder_rot\telbow_flexion\tpro_sup\tdeviation\tflexion\twrist_hand_r1\twrist_hand_r3\n');
writematrix([t' zeros(length(q), 10) q(:,1) q(:,2) -1.57*o q(:,3:7) z z], motion_filename, "FileType", "text", "delimiter", "\t", "WriteMode", "append");
fclose(fileD);

% Constant force file: forces and point of application
F_const=F_const_Ref(1:3,:)';
constantforce_filename=prefix_filename+"ConstForceLoad.sto";
fileD = fopen(constantforce_filename, 'w');
%header
fprintf(fileD, "%s\n", constantforce_filename);
fprintf(fileD, "nRows=%d\n", size(F_const,1)+1);
fprintf(fileD, "nColumns=7\n");%, size(F_const,2)+1);
fprintf(fileD, "inDegrees=no\n");
fprintf(fileD, "endheader\n");
fprintf(fileD,'time\tF_const_x\tF_const_y\tF_const_z\tX_x\tX_y\tX_z\n');
writematrix([t' F_const F_p], constantforce_filename, "FileType", "text", "delimiter", "\t", "WriteMode", "append");
fclose(fileD);

% No force file: forces and point of application
constantforce_filename=prefix_filename+"NoForceLoad.sto";
fileD = fopen(constantforce_filename, 'w');
%header
fprintf(fileD, "%s\n", constantforce_filename);
fprintf(fileD, "nRows=%d\n", size(F_const,1)+1);
fprintf(fileD, "nColumns=7\n");%, size(F_const,2)+1);
fprintf(fileD, "inDegrees=no\n");
fprintf(fileD, "endheader\n");
fprintf(fileD,'time\tF_const_x\tF_const_y\tF_const_z\tX_x\tX_y\tX_z\n');
writematrix([t' F_zero F_p], constantforce_filename, "FileType", "text", "delimiter", "\t", "WriteMode", "append");
fclose(fileD);

