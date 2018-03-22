clc
clear all
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1=[106,68,92,145,145,107,106]-22;
x1=x1/339*1000;
xcenter(1)=mean(x1(1:end-1));
y1=204-[44,84,112,112,84,84,44];
y1=y1/204*500;
ycenter(1)=mean(y1(1:end-1));
x2=[195,165,254,254,195]-22;
x2=x2/339*1000;
xcenter(2)=mean(x2(1:end-1));
y2=204-[84,157,157,84,84];
y2=y2/204*500;
ycenter(2)=mean(y2(1:end-1));
x3=[361,312,361,361]-22;
x3=x3/339*1000;
xcenter(3)=mean(x3(1:end-1));
y3=204-[32,122,122,32];
y3=y3/204*500;
ycenter(3)=mean(y3(1:end-1));
confx=[37,348]-22;
confx=confx/339*1000;
% confx(1)=50;
confx(2)=950;
confy=204-[83,164];
confy=confy/204*500;
% confy(1)=350;
confy(2)=100;
mainPlot=figure;
hold on;
patch(x1,y1,'k');
patch(x2,y2,'k')
patch(x3,y3,'k')
box on;
xlim([-1000,1000]);
ylim([-500,500]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
xpath=0:5:1000;
ypath=0:5:500;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
z1=zeros(length(ypath),length(xpath));
for i=1:length(xpath)
    for j=1:length(ypath)
        z1(j,i)=sqrt((xpath(i)-confx(2))^2+(ypath(j)-confy(2))^2);
    end
end
attract=1;
Uatt=0.5*attract.*z1.^2;
z2=ones(length(ypath),length(xpath))*0.005;
candtemp1=zeros(1,length(x1)-1);
candtemp2=zeros(1,length(x2)-1);
candtemp3=zeros(1,length(x3)-1);
p0=25;
Urep=zeros(length(ypath),length(xpath));
repell=100;
for i=1:length(xpath)
    for j=1:length(ypath)
        if inpolygon(xpath(i),ypath(j),x1(1:length(x1)-1),y1(1:length(y1)-1))==0&&inpolygon(xpath(i),ypath(j),x2(1:length(x2)-1),y2(1:length(y2)-1))==0&&inpolygon(xpath(i),ypath(j),x3(1:length(x3)-1),y3(1:length(y3)-1))==0
            for k=1:length(x1)-1
                candtemp1(k)=point_to_line([xpath(i),ypath(j)],[x1(k) y1(k)],[x1(k+1) y1(k+1)]);
            end
            for k=1:length(x2)-1
                candtemp2(k)=point_to_line([xpath(i),ypath(j)],[x2(k) y2(k)],[x2(k+1) y2(k+1)]);
            end
            for k=1:length(x3)-1
                candtemp3(k)=point_to_line([xpath(i),ypath(j)],[x3(k) y3(k)],[x3(k+1) y3(k+1)]);
            end
                z2(j,i)=min(cat(2,candtemp1,candtemp2,candtemp3));
        end
        if z2(j,i)<=p0
            Urep(j,i)=0.5*repell*(1/z2(j,i)-1/p0)^2;
        end
    end
end
[px1,py1] = gradient(Uatt);
[px2,py2] = gradient(Urep);
px=px1+px2;
py=py1+py2;
quiver(xpath,ypath,-1*px,-1*py)

plot(confx(1),confy(1),'b.','MarkerSize',29)
plot(confx(2),confy(2),'b.','MarkerSize',29)

% saveas(gcf,'setup','jpg')

xroute=zeros(1,1000);
yroute=zeros(1,1000);
xroute(1)=confx(1);
yroute(1)=confy(1);
xx=cat(2,x1(1:end-1),x2(1:end-1),x3(1:end-1));
yy=cat(2,y1(1:end-1),y2(1:end-1),y3(1:end-1));

count=2;

step=5;

disttemp=zeros(1,2);
centertemp=zeros(1,3);


%     if inpolygon(xroute(count-1),yroute(count-1),x1(1:end-1),y1(1:end-1))==0&&inpolygon(xroute(count-1),yroute(count-1),x2(1:length(x2)-1),y2(1:length(y2)-1))==0&&inpolygon(xroute(count-1),yroute(count-1),x3(1:length(x3)-1),y3(1:length(y3)-1))==0
  while sqrt((confx(2)-xroute(count-1))^2+(confy(2)-yroute(count-1))^2)>10
        for k=1:length(x1)-1
            candtemp1(k)=point_to_line([xroute(count-1),yroute(count-1)],[x1(k) y1(k)],[x1(k+1) y1(k+1)]);
        end
        for k=1:length(x2)-1
            candtemp2(k)=point_to_line([xroute(count-1),yroute(count-1)],[x2(k) y2(k)],[x2(k+1) y2(k+1)]);
        end
        for k=1:length(x3)-1
            candtemp3(k)=point_to_line([xroute(count-1),yroute(count-1)],[x3(k) y3(k)],[x3(k+1) y3(k+1)]);
        end
        [r,idx]=min(cat(2,candtemp1,candtemp2,candtemp3));
        if idx~=6&&idx~=10&&idx~=13
            slope=(yy(idx)-yy(idx+1))/(xx(idx)-xx(idx+1));
        elseif idx==6
            slope=(yy(6)-yy(1))/(xx(6)-xx(1));
        elseif idx==10
            slope=(yy(7)-yy(10))/(xx(7)-xx(10));
        elseif idx==13
            slope=(yy(13)-yy(11))/(xx(13)-xx(11));
        end
        intercpt=yy(idx)-slope*xx(idx);
        syms x y
        if slope==inf||slope==-inf
            eqn1=x==xx(idx);
        else
            eqn1=y==slope*x+intercpt;
        end            
        eqn2=r^2==(x-xroute(count-1))^2+(y-yroute(count-1))^2;
        [xout,yout] = vpasolve(eqn1, eqn2);
        if length(xout)==2
            if xout(1)-xout(2)<0.1
                xtemp=xout(1);
                ytemp=yout(1);
            else
            if idx<=6
                for j=1:2
                    disttemp(j)=sqrt((xout(j)-xcenter(1))^2+(yout(j)-xcenter(1))^2);
                end
            elseif idx<=10&&idx>6
                for j=1:2
                    disttemp(j)=sqrt((xout(j)-xcenter(2))^2+(yout(j)-xcenter(2))^2);
                end
            elseif idx>10
                for j=1:2
                    disttemp(j)=sqrt((xout(j)-xcenter(3))^2+(yout(j)-xcenter(3))^2);
                end                    
            end
            [~,distidx]=min(disttemp);
            xtemp=xout(distidx);
            ytemp=yout(distidx);
            end
        else
            xtemp=xout;
            ytemp=yout;
        end
        Uobs=0.5*repell*(1/0.05-1/p0)^2;
        zz1=[confx(2),confy(2)]-[xroute(count-1),yroute(count-1)];
        Fatt=attract*zz1;
        zz2=[xroute(count-1),yroute(count-1)]-[xtemp,ytemp];
        if norm(zz2)<=p0
            Urep=0.5*repell*(1/norm(zz2)-1/p0)^2;
            Frep=repell*(1/norm(zz2)-1/p0)*1/norm(zz2)^2*[(Urep-Uobs)/(zz2(1)),(Urep-Uobs)/(zz2(2))];
        else
            Urep=0;
            Frep=0;
        end
        
        Ftot=Fatt+Frep;
%         Fx=-1*Uatt/abs(xroute(count-1)-confx(2))+-1*Urep/abs(xroute(count-1)-xtemp);
%         Fy=-1*Uatt/abs(yroute(count-1)-confy(2))+-1*Urep/abs(yroute(count-1)-ytemp);
%         xroute(count)=Fy/sqrt(Fx^2+Fy^2)*step+xroute(count-1);
%         yroute(count)=Fx/sqrt(Fx^2+Fy^2)*step+yroute(count-1);
        xroute(count)=Ftot(1)*step+xroute(count-1);
        yroute(count)=Ftot(2)*step+yroute(count-1);
        plot(xroute(count),yroute(count),'r.','MarkerSize',9)
        count=count+1;    
%    else
%         disp('Wrong!!!!!!!!!!!!!!!!!!!')
%    end
end

saveas(gcf,'result_10','jpg')
