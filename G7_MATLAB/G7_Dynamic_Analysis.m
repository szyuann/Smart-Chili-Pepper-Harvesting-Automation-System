%Dynamic Analysis for the Smart Chili Harvesting Automation System
                       

%Declaring the angle and link variable for the robotic arm

syms th1 th2 th3 th4 l0 l1 l2 l3 l4;

%  The Link length measured units in meter 
l0 = 0.04;
l1 = 0.036; 
l2 = 0.082; 
l3 = 0.060;
l4 = 0.052;

%Denavit-Hatenberg Parameters
% i  thi  ai   alphai    di
% 1  th1   0    pi/2     l1
% 2  th2  l2       0      0 
% 3  th3  l3       0      0  

% Transformation matrix
A0= [1 0 0 0; 
     0 1 0 0;
     0 0 1 l0;
     0 0 0 1];
 
A1= [cos(th1) -sin(th1) 0 0;
     sin(th1)  cos(th1) 0 0;
     0 0 1 l1;
     0 0 0 1];
 
A2= [cos(th2) 0 sin(th2) 0;
     sin(th2)  0 -cos(th2) 0;
     0 1 0 0;
     0 0 0 1];
 
A3= [cos(th3) -sin(th3) 0 l3*cos(th3);
     sin(th3) cos(th3) 0 l3*sin(th3);
     0 0 1 0;
     0 0 0 1];
 
 A4= [cos(th4) 0 -sin(th4) 0;
      sin(th4)  0 cos(th4) 0;
      0 -1 0 l4;
      0 0 0 1];

% Transformation from base to each link
A_1=A0*A1;
A_2=A_1*A2;
A_3=A_2*A3;
A_4=A_3*A4;

% Mass of the links in kg that is obtained from solidworks 
m1=0.00495;  m2=0.027; m3=0.02787; m4=0.01538;

%Centre of mass for each of the link
r1=[0;	0.14/(10^3); 10.31/(10^3);        0];
r2=[20.59/(10^3);	-8.5/(10^3);     48.22/(10^3);      0];
r3=[24.74/(10^3);	-8.27/(10^3);     61.72/(10^3);      0];
r4=[1.99/(10^3);     9.34/(10^3);     20.46/(10^3);      0];

%Moment of Inertia matrixes that were obtained from Solidworks (kgm^2)
  I1=[311.76/(10^9)   -0.01/(10^9) 0;
     -0.01/(10^9)   398.04/(10^9) 2.20/(10^9);
     0   2.20/(10^9) 268.68/(10^9)];
  
 I2=[9292.76/(10^9)   0.09/(10^9) -150.92/(10^9);
     0.09/(10^9)   13191.10/(10^9) -0.20/(10^9);
     -150.92/(10^9)   -0.20/(10^9) 5247.63/(10^9)];
        
 I3=[26987.87/(10^9)   -78/(10^9) 3873.29/(10^9);
     -78/(10^9)   32701.87/(10^9) 11.57/(10^9);
     3873.29/(10^9)   11.57/(10^9) 7084.58/(10^9)];
        
I4=[5175.24/(10^9)   -769.24/(10^9) -69.91/(10^9);
     -769.24/(10^9)   167.29/(10^9) 2953.22/(10^9);
     -69.91/(10^9)   167.29/(10^9) 2953.22/(10^9)];

 %Gravity column matrix  
  g=[0 0 -9.81 0];

 %Uij Matrixes, zero if the term does not contain the angle
 U11=diff(A_1,th1);       U12=diff(A_1,th2);     U13=diff(A_1,th3);       U14=diff(A_1,th4);  
 U21=diff(A_2,th1);       U22=diff(A_2,th2);     U23=diff(A_2,th3);       U24=diff(A_2,th4);
 U31=diff(A_3,th1);       U32=diff(A_3,th2);     U33=diff(A_3,th3);       U34=diff(A_3,th4);
 U41=diff(A_4,th1);       U42=diff(A_4,th2);     U43=diff(A_4,th3);       U44=diff(A_4,th4);
   
 %Uijk Matrixes
 %i = 1
 %U1jk Matrixes
 U111=diff(U11,th1);      U112=diff(U11,th2);     U113=diff(U11,th3);     U114=diff(U11,th4)
 U121=diff(U12,th1);      U122=diff(U12,th2);     U123=diff(U12,th3);     U124=diff(U12,th4)
 U131=diff(U13,th1);      U132=diff(U13,th2);     U133=diff(U13,th3);     U134=diff(U13,th4)
 U141=diff(U14,th1);      U142=diff(U14,th2);     U143=diff(U14,th3);     U144=diff(U14,th4)


 %i = 2
 %U2jk Matrixes
 U211=diff(U21,th1);      U212=diff(U21,th2);     U213=diff(U21,th3);     U214=diff(U21,th4);
 U221=diff(U22,th1);      U222=diff(U22,th2);     U223=diff(U22,th3);     U224=diff(U22,th4);
 U231=diff(U23,th1);      U232=diff(U23,th2);     U233=diff(U23,th3);     U234=diff(U23,th4);
 U241=diff(U24,th1);      U242=diff(U24,th2);     U243=diff(U24,th3);     U244=diff(U24,th4);
 
 %i = 3
 %U3jk Matrixes
 U311=diff(U31,th1);      U312=diff(U31,th2);     U313=diff(U31,th3);     U314=diff(U31,th4);
 U321=diff(U32,th1);      U322=diff(U32,th2);     U323=diff(U32,th3);     U324=diff(U32,th4);
 U331=diff(U33,th1);      U332=diff(U33,th2);     U333=diff(U33,th3);     U334=diff(U33,th4);
 U341=diff(U34,th1);      U342=diff(U34,th2);     U343=diff(U34,th3);     U344=diff(U34,th4);
 
 %i = 4
 %U4jk Matrixes
 U411=diff(U41,th1);      U412=diff(U41,th2);     U413=diff(U41,th3);    U414=diff(U41,th4);
 U421=diff(U42,th1);      U422=diff(U42,th2);     U423=diff(U42,th3);     U424=diff(U42,th4);
 U431=diff(U43,th1);      U432=diff(U43,th2);     U433=diff(U43,th3);     U434=diff(U43,th4);
 U441=diff(U44,th1);      U442=diff(U44,th2);     U443=diff(U44,th3);     U444=diff(U44,th4);


  %Construction for the Pseudo inertia matrix
  I=zeros(4);
  for i=1:4
      I=eval(['I' num2str(i)]);
      m=eval(['m' num2str(i)]);
      r=eval(['r' num2str(i)]);
      
      eval(['j11' '=((-I(1,1)+I(2,2)+I(3,3))/2)']);
      eval(['j12' '=I(1,2)']);
      eval(['j13' '=I(1,3)']);
      eval(['j14' '=m*r(1)']);
      
      eval(['j21' '=I(1,2)']);
      eval(['j22' '=((I(1,1)-I(2,2)+I(3,3))/2)']);
      eval(['j23' '=I(2,3)']);
      eval(['j24' '=m*r(2)']);
      
      eval(['j31' '=I(1,3)']);
      eval(['j32' '=I(2,3)']);
      eval(['j33' '=((I(1,1)+I(2,2)-I(3,3))/2)']);
      eval(['j34' '=m*r(3)']);
      
      eval(['j41' '=m*r(1)']);
      eval(['j42' '=m*r(2)']);
      eval(['j43' '=m*r(3)']);
      eval(['j44' '=m']);
      
      J=[j11 j12 j13 j14;
         j21 j22 j23 j24; 
         j31 j32 j33 j34;
         j41 j42 j43 j44];
      eval(['J' num2str(i) '=J']);
  end
  
 %Computation for the Dij Matrix
  Uaux=zeros(4);
for i=1:4
   for j=1:4
       m=max([i j]);
       x=0;
       for k=m:4 %p in the formula
           Uaux=eval(['U' num2str(k) num2str(i)]);
           Ud=Uaux';
           A=eval(['U' num2str(k) num2str(j)])*eval(['J' num2str(k)])*Ud;
           x=x+trace(A);
       end
   eval(['d' num2str(i) num2str(j) '=x']);
   end
end

D=[d11 d12 d13 d14;
   d21 d22 d23 d24;
   d31 d32 d33 d34;
   d41 d42 d43 d44];

%Dijk matrix 
  for i=1:4
   for k=1:4
     for m=1:4
       j=max([i m k]);
       x=0;
       for l=j:4
       Uaux=eval(['U' num2str(j) num2str(i)]);
       Uh=Uaux'; %transpose
       x=x+trace(eval(['U' num2str(j) num2str(k) num2str(m)])*eval(['J' num2str(j)])*Uh);
       end
       eval(['h' num2str(i) num2str(k) num2str(m) '=x']);
     end 
   end
  end
  
 %Coriolis column matrices
 %Angular velocity matrices 
 syms dth1 dth2 dth3 dth4  
 
  for i=1:4
      y=0;
      for k=1:4
        y=y+x;
        x=0;  
          for m=1:4
          y=x+eval(['h' num2str(i) num2str(k) num2str(m)])*eval(['dth' num2str(k)])*eval(['dth' num2str(m)]);
          end    
      end
  eval(['h' num2str(i) '=y']);
  end
  
  H=[h1;h2;h3;h4]; 
  
  for i=1:4
      x=0;
      for j=i:4
      x=x+(-eval(['m' num2str(j)])*g*eval(['U' num2str(j) num2str(i)])*eval(['r' num2str(j)]));  
      end
  eval(['c' num2str(i) '=x']);
  end
   
  C=[c1; c2; c3; c4];
     
  %Angular acceleration matrix
  
  syms ddth1 ddth2 ddth3 ddth4
  ddth=[ddth1; ddth2; ddth3; ddth4];   
  
  T=D*ddth+H+C;
  
%Final dynamic equation
diary ('EEM343 Group 7 Dynamic Analysis Result.txt')
T_1 = simplify(T(1,:));
T_2 = simplify(T(2,:));
T_3 = simplify(T(3,:));
T_4 = simplify(T(4,:));
diary off
