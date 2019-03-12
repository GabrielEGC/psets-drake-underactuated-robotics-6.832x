%%Part III
clear all;
p = CompassGaitPlant();

x0=[-0.323388548952647
   0.218668793832964
  -0.377182134794561
  -1.091826923680050];
M=zeros(4);
N=zeros(4);
itn=0;
xm=eye(4);
for it=1:4
    x01 = x0+0.0001*xm(:,it);
    xf = strideFunction(p, x01);
    itn=itn+1;
    N(:,itn)=x01-x0;
    M(:,itn)=xf-x0;
end
A=M*N^-1
%%Part IV
eig(A)