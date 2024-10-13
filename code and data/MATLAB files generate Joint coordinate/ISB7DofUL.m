function ISBUL = ISB7DofUL(ua_l, fa_l, ha_l)
%ISB7DOFUL Create a Robot of robotic toolbox ISB compatible arm w/ shoulder
%, elbow and wrist
% ua_l: upper-arm length
% fa_l: forearm length
% ha_l: hand length

	%%DH parameters definition
        		% ?     d_i		a_i-1       alpha	0/1(prism)	Offset
	L(1) = Link([ 0		0			0       0		0           0],		'modified'); L(1).qlim=[-pi/2 130/180*pi];	%Planeele
	L(2) = Link([ 0 	0			0       pi/2	0           0],		'modified'); L(2).qlim=[0 pi];              %Elevation
	L(3) = Link([ 0 	ua_l		0       pi/2	0           pi],	'modified'); L(3).qlim=[-pi/2 20/180*pi];   %Int/ext
	L(4) = Link([ 0		0   		0       pi/2   	0           pi],	'modified'); L(4).qlim=[0 130/180*pi];      %Elbow flex
	L(5) = Link([ 0		fa_l		0       pi/2	0           0],     'modified'); L(5).qlim=[-pi/2 pi/2];        %Pronosupination 
	L(6) = Link([ 0     0           0       pi/2	0           pi/2],  'modified'); L(6).qlim=[-20/180*pi 20/180*pi];              %Wrist deviation: 0
    L(7) = Link([ 0     0           0       pi/2	0           pi/2],     'modified'); L(7).qlim=[-20/180*pi pi/2];%Wrist flexion
	ISBUL=SerialLink(L, 'name', 'ISB UL');
    %Add hand transformation (tool) to match OpenSIM model wrist offset
    %frame: z -> x, x -> -y, y -> -z
    t=[0 0 1 0;-1 0 0 0; 0 -1 0 0; 0 0 0 1];
    t(2,4)=-ha_l;
    ISBUL.tool=t;
	
	%figure()
    %ISBUL.teach([pi/2 0 0 pi/2 0 0 0]);
	%ISBUL.plot([0 0 0 1.5 0]);
end

