#rotation matrix around x axis
function R=Rx(rot_x)
 c=cos(rot_x);
 s=sin(rot_x);
 R= [1  0  0;
     0  c  -s;
     0  s  c];
endfunction

#rotation matrix around y axis
function R=Ry(rot_y)
 c=cos(rot_y);
 s=sin(rot_y);
 R= [c  0  s;
     0  1  0;
     -s  0 c];
endfunction

#rotation matrix around z axis
function R=Rz(rot_z)
 c=cos(rot_z);
 s=sin(rot_z);
 R= [ c  -s  0;
      s  c  0;
      0  0  1];
endfunction

#derivative of rotation matrix around z
function R=Rx_prime(rot_x)
 dc=-sin(rot_x); #derivative of cos(rot(x)
 ds=cos(rot_x);  #derivative of sin(rot(x)
 R= [0  0  0;
     0  dc  -ds;
     0  ds  dc];
endfunction

#derivative of rotation matrix around y
function R=Ry_prime(rot_y)
 dc=-sin(rot_y); #derivative of cos(rot(x)
 ds=cos(rot_y);  #derivative of sin(rot(x)
 R= [dc  0 ds;
     0  0  0;
     -ds  0 dc];
endfunction

#derivative of rotation matrix around z
function R=Rz_prime(rot_z)
 dc=-sin(rot_z); #derivative of cos(rot(x)
 ds=cos(rot_z);  #derivative of sin(rot(x)
 R= [ dc  -ds  0;
      ds  dc  0;
      0  0  0];
endfunction

#from 6d vector to homogeneous matrix
function T=v2t(v)
    T=eye(4);
    T(1:3,1:3)=Rx(v(4))*Ry(v(5))*Rz(v(6));
    T(1:3,4)=v(1:3);
endfunction;

function S=skew(v)
  S=[0,    -v(3), v(2);
     v(3),  0,    -v(1);
     -v(2), v(1), 0];
endfunction

function v=flattenIsometry(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3)',9,1);
v(10:12)=T(1:3,4);
endfunction

function T=unflattenIsometry(v)
  T=eye(4);
  T(1:3,1:3)=reshape(v(1:9),3,3)';
  T(1:3,4)=v(10:12);
endfunction

function v=flattenIsometryByColumns(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3),9,1);
v(10:12)=T(1:3,4);
endfunction

function T=unflattenIsometryByColumns(v)
  T=eye(4);
  T(1:3,1:3)=reshape(v(1:9),3,3);
  T(1:3,4)=v(10:12);
endfunction

function M=flatTransformationMatrix(v)
  T=unflattenIsometry(v);
  R=T(1:3,1:3);
  t=T(1:3,4);
  M=eye(12);
  M(1:3,1:3)=R';
  M(4:6,4:6)=R';
  M(7:9,7:9)=R';
  M(10,1:3)=t';
  M(11,4:6)=t';
  M(12,7:9)=t';
endfunction;

#derivative of rotation matrix w.r.t rotation around x, in 0
global  Rx0=[0 0 0;
	     0 0 -1;
	     0 1 0];

#derivative of rotation matrix w.r.t rotation around y, in 0
global  Ry0=[0 0 1;
	     0 0 0;
	     -1 0 0];

#derivative of rotation matrix w.r.t rotation around z, in 0
global  Rz0=[0 -1 0;
	     1  0 0;
	     0  0 0];
