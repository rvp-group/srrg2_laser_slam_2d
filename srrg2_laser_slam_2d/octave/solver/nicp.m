source "geometry_helpers_3d.m"

# error and jacobian for the direct case (moving remapped in fixed)
function [e,J]=directErrorAndJacobian(moving, fixed, T)
  
  R=T(1:3,1:3);
  t=T(1:3,4);
  p_moving=moving(1:3);
  n_moving=moving(4:6);
  p_fixed=fixed(1:3);
  n_fixed=fixed(4:6);

  p_pred=R*p_moving+t;
  n_pred=R*n_moving;

  e=zeros(4,1);
  J=zeros(4,6);
  e(1)=n_fixed' * (p_pred-p_fixed);
  e(2:4)=n_pred-n_fixed;

  J(1,1:3)=n_fixed';
  J(1,4:6)=-n_fixed'*skew(p_pred);
  J(2:4,4:6)=-skew(n_pred);
endfunction;

# error and jacobian for the inverse case (fixed remapped in moving)
function [e,J]=inverseErrorAndJacobian(moving, fixed, T)
  R=T(1:3,1:3);
  t=T(1:3,4);
  p_moving=moving(1:3);
  n_moving=moving(4:6);
  p_fixed=fixed(1:3);
  n_fixed=fixed(4:6);

  p_pred=R'*(p_fixed-t);
  n_pred=R'*n_fixed;

  e=zeros(4,1);
  J=zeros(4,6);

  e(1)=n_moving' * (p_tilde-p_moving);
  e(2:4)=n_tilde-n_moving;

  nt=n_moving'*R';
  J=zero(4,6);
  J(1,1:3)=nt;
  J(1,4:6)=nt*skew(p_fixed);
  J(2:4,4:6)=R'*skew(n_fixed);
endfunction;

function Dest=remapPoints(Src, T)
  R=T(1:3,1:3);
  t=T(1:3,4);
  Dest=zeros(size(Src,1), size(Src,2));
  Dest(1:3,:)=R*Src(1:3,:)+repmat(t,1,size(Src,2));
  Dest(4:6,:)=R*Src(4:6,:);
endfunction;

function P=makeSamples(n_points)
  P=100*(rand(6,n_points)-0.5);
  for(i=1:n_points)
    n=P(4:6,i);
    P(4:6,i)/=sqrt(n'*n);
  endfor;
endfunction;

function [chi, H,b]=linearize(Moving, Fixed, T, direct, inverse)
  chi=0;
  H=zeros(6,6);
  b=zeros(6,1);
  for (idx=1:size(Moving,2))
    moving=Moving(:,idx);
    fixed=Fixed(:,idx);
    if (direct)
      [e,J]=directErrorAndJacobian(moving,fixed,T);
      chi+=e'*e;
      H+=J'*J;
      b+=J'*e;
    endif;

    if (inverse)
      [e,J]=directErrorAndJacobian(moving,fixed,T);
      chi+=e'*e;
      H+=J'*J;
      b+=J'*e;
    endif;
  endfor
endfunction;

function [chi, T]=solve(Moving, Fixed, T, direct, inverse)
  [chi, H, b]=linearize(Moving, Fixed, T, direct, inverse);
  dx=-H\b;
  dX=v2t(dx);
  T=dX*T;
endfunction;

function chi_evolution=chiEvolution(Moving, Fixed, T, direct, inverse, iterations)
  chi_evolution=zeros(1,iterations);
  for(i=1:iterations)
    [chi, T]=solve(Moving, Fixed, T, direct, inverse);
    chi_evolution(i)=chi;
  endfor;
  T
endfunction
