function [t, X, dX]=Jerk(nbre_pts, tf, P0, Pf)
  t=[0:tf/nbre_pts:tf];
  to=t/tf;
  for i=1:3
	X(i,:)=P0(i)+(P0(i)-Pf(i)).*(15*to.^4-6*to.^5-10*to.^3);
	dX(i,:)=(P0(i)-Pf(i)).*(4*15*to.^4./t-5*6*to.^5./t-3*10*to.^3./t);
    dX(i,1)=0; %dX(t=0) is undefined, should be 0
  end

  %figure();
  %plot3(X(1,:), X(2,:), X(3,:));
  %figure();
  %plot(t,dX);
end

