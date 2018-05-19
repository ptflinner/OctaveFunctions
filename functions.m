# matrices rotation functions using angles in degrees
# global rotation multiple the angle by -1
# the X rotation matrix
function [Rx] = rotx(a)
Rx = [1, 0, 0; 0, cos(pi*a/180),  -sin(pi*a/180); 0, sin(pi*a/180), cos(pi*a/180)];
endfunction

# the Y rotation matrix
function [Ry] = roty(b)
Ry = [cos(pi*b/180) 0 sin(pi*b/180); 0 1 0;-sin(pi*b/180) 0 cos(pi*b/180)];
endfunction

# the Z rotation matrix
function [Rz] = rotz(c)
Rz = [cos(pi*c/180) -sin(pi*c/180) 0;sin(pi*c/180) cos(pi*c/180) 0; 0 0 1];
endfunction

function [Rx] = rotxRad(a)
Rx = [1, 0, 0; 0, cos(a),  -sin(a); 0, sin(a), cos(a)];
endfunction

# the Y rotation matrix
function [Ry] = rotyRad(b)
Ry = [cos(b) 0 sin(b); 0 1 0;-sin(b) 0 cos(b)];
endfunction

# the Z rotation matrix
function [Rz] = rotzRad(c)
Rz = [cos(c) -sin(c) 0;sin(c) cos(c) 0; 0 0 1];
endfunction

# the X-Y-Z rotation matrix
function [R] = rotxyz(c, b, a)
R = [cos(pi*a/180)*cos(pi*b/180)  cos(pi*a/180)*sin(pi*b/180)*sin(pi*c/180)-sin(pi*a/180)*cos(pi*c/180)  cos(pi*a/180)*sin(pi*b/180)*cos(pi*c/180)+sin(pi*a/180)*sin(pi*c/180);
     sin(pi*a/180)*cos(pi*b/180)  sin(pi*a/180)*sin(pi*b/180)*sin(pi*c/180)+cos(pi*a/180)*cos(pi*c/180)  sin(pi*a/180)*sin(pi*b/180)*cos(pi*c/180)-cos(pi*a/180)*sin(pi*c/180);
        -sin(pi*b/180)                cos(pi*b/180)*sin(pi*c/180)                        cos(pi*b/180)*cos(pi*c/180)];
endfunction


#recovering the angles of the X-Y-Z rotation
function [A] = ang(c, b, a)
r11 = cos(a*pi/180)*cos(b*pi/180);
r21 = sin(a*pi/180)*cos(b*pi/180);
r31 = -sin(b*pi/180);
r32 = cos(pi*b/180)*sin(pi*c/180);
r33 = cos(pi*b/180)*cos(pi*c/180);
A = [atan2(r32/cos(b*pi/180),r33/cos(b*pi/180));atan2(-r31, sqrt(r11^2+r21^2));atan2(r21/cos(b*pi/180),r11/cos(b*pi/180))]*(180/pi);
endfunction

function [d]= distance3D(a,b)
d=sqrt(power((a(1)-b(1)),2)+power((a(2)-b(2)),2)+power((a(3)-b(3)),2))
endfunction 

function [d]= distance2D(a,b)
d=sqrt(power((a(1)-b(1)),2)+power((a(2)-b(2)),2))
endfunction 

#Fixed Angles Formula
function [Angles]= fixedRecAng(A)
  beta=atan2(-A(3),sqrt((power(A(1),2)+power(A(2),2))))
  alpha=atan2(A(2)/(cos(beta)),A(1)/cos(beta))
  gamma=atan2(A(6)/cos(beta),A(9)/cos(beta))
  Angles=[beta*180/pi;alpha*180/pi;gamma*180/pi]
endfunction

#Fixed Rads Formula
function [Angles]= fixedRecRad(A)
  beta=atan2(-A(3),sqrt((power(A(1),2)+power(A(2),2))))
  alpha=atan2(A(2)/(cos(beta)),A(1)/cos(beta))
  gamma=atan2(A(6)/cos(beta),A(9)/cos(beta))
  Angles=[beta;alpha;gamma]
endfunction

#Yaw-Pitch-Roll Angles Formula
function [Angles]= yprRecAng(A)
  alpha=atan2(A(6),A(9))
  beta=-asin(A(3))
  gamma=atan2(A(2),A(1))
  Angles=[alpha*180/pi;beta*180/pi;gamma*180/pi]
endfunction

#Yaw-Pitch-Roll Rads Formula
function [Angles]= yprRecRad(A)
  beta=-asin(A(3))
  alpha=atan2(A(6),A(9))
  gamma=atan2(A(2),A(1))
  Angles=[beta*180/pi;alpha*180/pi;gamma*180/pi]
endfunction

function [Rotate]= rotateCoord2D(A,a)
  Rotate=[cos(pi*a/180), sin(pi*a/180);-sin(pi*a/180),cos(pi*a/180)]*A
endfunction
function [RotMatrix]=rotationMatrix2D(a)
  RotMatrix=[cos(pi*a/180), sin(pi*a/180);-sin(pi*a/180),cos(pi*a/180)]
endfunction
function [Rotate]=rotateVector2D(A,a)
    Rotate=[cos(pi*a/180), -sin(pi*a/180);sin(pi*a/180),cos(pi*a/180)]*A
endfunction

function [Translation]=translationMatrix2D(h,k)
  Translation=[1,0,h;0,1,k;0,0,1]
endfunction

function [Translation]=translationMatrix3D(h,k,i)
  Translation=[1,0,0,h;0,1,0,k;0,0,1,i;0,0,0,1]
endfunction

# the X rotation derivation matrix
function [Rx] = drotx(a)
Rx = [1, 0, 0; 0, -sin(pi*a/180),  -cos(pi*a/180); 0, cos(pi*a/180), -sin(pi*a/180)];
endfunction

# the Y rotation derivation matrix
function [Ry] = droty(b)
Ry = [-sin(pi*b/180) 0 cos(pi*b/180); 0 1 0;-cos(pi*b/180) 0 -sin(pi*b/180)];
endfunction

# the Z rotation derivation matrix
function [Rz] = drotz(c)
Rz = [-sin(pi*c/180) -cos(pi*c/180) 0;cos(pi*c/180) -sin(pi*c/180) 0; 0 0 1];
endfunction

# the X rotation derivation matrix
function [Rx] = drotxRad(a)
Rx = [1, 0, 0; 0, -sin(a),  -cos(a); 0, cos(a), -sin(a)];
endfunction

# the Y rotation derivation matrix
function [Ry] = drotyRad(b)
Ry = [-sin(b) 0 cos(b); 0 1 0;-cos(b) 0 -sin(b)];
endfunction

# the Z rotation derivation matrix
function [Rz] = drotzRad(c)
Rz = [-sin(c) -cos(c) 0;cos(c) -sin(c) 0; 0 0 1];
endfunction

#-----------------------------QUATERNIONS------------------

#Euler Identity
function [e]=euler(x)
x=pi*x/180
e=cos(x)-sin(x)
endfunction

#Euler z
function [e] = precession(x)
x=pi*x/180
e= [cos(x) sin(x) 0;-sin(x) cos(x) 0; 0 0 1];
endfunction

#Euler y
function [e] = spin(x)
x=pi*x/180
e= [cos(x) sin(x) 0;-sin(x) cos(x) 0; 0 0 1];
endfunction

#Euler x
function [e] = nutation(x)
x=pi*x/180
e= [1,0,0;0,cos(x),sin(x);0,-sin(x) cos(x)];
endfunction

function [a]=eulerAng(A)
  theta=acos(A(9))
  phi=-atan (A(3)/A(6))
  psi=atan(A(7)/A(8))
  angles=[phi*180/pi;theta*180/pi;psi*180/pi]
endfunction

function [r]=unitVector2D(r)
  r=r/sqrt(power(r(1),2)+power(r(2),2))
endfunction

function [r]=unitVector3D(r)
  r=r/sqrt(power(r(1),2)+power(r(2),2)+power(r(3),2))
endfunction

function [b]=orthogonal(q)
  eq(inv(q),q') 
endfunction


#-----------------------------------------------------
# slide 2C 23 Not sure how to do it
function [m]=systemConversion(g,b)
  m=[dot(g(1),b(1)),dot(g(1),b(2)),dot(g(1),b(3));
     dot(g(2),b(1)),dot(g(2),b(2)),dot(g(2),b(3));
     dot(g(3),b(1)),dot(g(3),b(2)),dot(g(3),b(3))]
endfunction

function [m]=cosineSystem(g)
  m=[cos(g(1)),cos(g(4)),cos(g(7));
     cos(g(2)),cos(g(5)),cos(g(8));
     cos(g(3)),cos(g(6)),cos(g(9));]
endfunction

#g is systemConversion Matrix
#b is the single line matrix fed into systemConversion, either global or local
#a is the angle to be given
#I GOT THIS ONE TO WORK WITH EXERCISE 16
function [m]=axisAngleRot(g,b,a)
  #a=a*pi/180
  v=1-cos(a)
  m=[g(1)*v+cos(a),g(4)*v-b(3)*sin(a),g(7)*v+b(2)*sin(a);
     g(2)*v+b(3)*sin(a),g(5)*v+cos(a),g(8)*v-b(1)*sin(a);
     g(3)*v-b(2)*sin(a),g(6)*v+b(1)*sin(a),g(9)*v+cos(a)]
  endfunction
#-----------------------------------------------------
#STUFF BETWEEN LINES IS STUPID AND IS PROBABLY WRONG 

function [m]=isSkew(v)
  eq(v',-1*v)
endfunction

function [m]=skew(v)
  m=[0 -v(3) v(2);
     v(3) 0 -v(1);
     -v(2) v(1) 0]
endfunction


#u unit vector
#a angle
function [m] =rodriguez(u,a)
  a=a*pi/180
  vers=1-cos(a)
  m=eye(3)*cos(a)+(u*u')*vers+skew(u)*sin(a)
endfunction

#u is the rotation matrix
function [phi]=axis_angle(u)
  phi=(180/pi)*acos((u(1)+u(5)+u(9)-1)/2)
endfunction

# Extract vector from Rodriguez formula rotation matrix
# v is the overall matrix
# u is the initial unit vector
function [u] = extractu(u)
  v = (u'-u)/(2*sin(axis_angle(u)))
  u = [v(3,2),v(1,3),v(2,1)]'
endfunction

# Product of Quanternions
#3c slide 7
function [A] = quantMult(p0, p1, p2, p3, q0, q1, q2, q3)
A = [p0 -p1 -p2 -p3;p1 p0 -p3 p2;p2 p3 p0 -p1;p3 -p2 p1 p0] * [q0; q1; q2; q3];
endfunction

function [q] = qConjugate(q)
    q=q
endfunction


#2D Rotation

#Rotation of coordinates
function[Rx] = rotcoord(a)
Rx = [cos(pi*a/180) sin(pi*a/180); -sin(pi*a/180) cos(pi*a/180)];
endfunction

#Rotation of vector
function[Rx] = rotvect(a)
Rx = [cos(pi*a/180) -sin(pi*a/180); sin(pi*a/180) cos(pi*a/180)];
endfunction

# Rotation Operator w = qvq* = Qv
function [A] = rotationOperator(q0, q1, q2, q3, v0, v1, v2)
A = [((2*(qo^2)-1)+(2*(q1^2))) (2*q2*q1-2*q0*q3) (2*q3*q1+2*q0*q2);(2*q1*q2+2*q0*q3) (((2*(q0^2)-1)-1)+(2*(q2^2))) (2*q3*q2-2*q0*q1);(2*q1*q3-2*q0*q2) (2*q2*q3+2*q0*q1) (((2*(q0^2)-1)-1)+(2*(q3^2)))] * [v0;v1;v2];
endfunction


# Transformation between two adjacent coordinate frames, i-1Ti (4a, slide 8)
function [A] = transform(ai, alphai, di, thetai)
A = [cos(deg2rad(thetai)) -sin(deg2rad(thetai))*cos(deg2rad(alphai)) sin(deg2rad(thetai))*sin(deg2rad(alphai)) ai*cos(deg2rad(thetai));sin(deg2rad(thetai)) cos(deg2rad(thetai))*cos(deg2rad(alphai)) -cos(deg2rad(thetai))*sin(deg2rad(alphai)) ai*sin(deg2rad(thetai));0 sin(deg2rad(alphai)) cos(deg2rad(alphai)) di;0 0 0 1];
endfunction

# same as above but inverse, iTi-1 (4a, slide 9)
function [A] = inversetransform(ai, alphai, di, thetai)
A = [cos(deg2rad(thetai)) sin(deg2rad(thetai)) 0 -ai;-sin(deg2rad(thetai))*cos(deg2rad(alphai)) cos(deg2rad(thetai))*cos(deg2rad(alphai)) sin(deg2rad(alphai)) -di*sin(deg2rad(alphai));sin(deg2rad(thetai))*sin(deg2rad(alphai)) -cos(deg2rad(thetai))*sin(deg2rad(alphai)) cos(deg2rad(alphai)) -di*cos(deg2rad(alphai));0 0 0 1];
endfunction
