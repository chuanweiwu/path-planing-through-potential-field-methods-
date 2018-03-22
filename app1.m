clc
clear all
close all
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1=[106,68,92,145,145,107,106]-22;
x1=x1/339*1000;
y1=204-[44,84,112,112,84,84,44];
y1=y1/204*500;
x2=[165,254,254,195,165]-22;
x2=x2/339*1000;
y2=204-[157,157,84,84,157];
y2=y2/204*500;
x3=[312,361,361,312]-22;
x3=x3/339*1000;
y3=204-[122,122,32,122];
y3=y3/204*500;
confx=[37,348]-22;
confx=confx/339*1000;
confx(2)=900;
confy=204-[83,164];
confy=confy/204*500;
confy(2)=100;
mainPlot=figure;
hold on;
patch(x1,y1,'k');
patch(x2,y2,'k')
patch(x3,y3,'k')
box on;
xlim([0,1000]);
ylim([0,500]);
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
z2=ones(length(ypath),length(xpath))*0.01;
candtemp1=zeros(1,length(x1)-1);
candtemp2=zeros(1,length(x2)-1);
candtemp3=zeros(1,length(x3)-1);
p0=100;
Urep=zeros(length(ypath),length(xpath));
repell=10;
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
angle=atan2(-1*px,-1*py)*180/pi;
inditable=[0 45 90 135 180 -135 -90 -45];
direction=zeros(length(ypath),length(xpath));
inditemp=zeros(1,8);
for i=1:length(xpath)
    for j=1:length(ypath)
        if angle(j,i)<-157.5
            direction(j,i)=5;
        else
            for k=1:8
                inditemp(k)= abs(angle(j,i)-inditable(k));
            end
            [~,direction(j,i)]=min(inditemp);
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
i=10;j=60;
xroute=zeros(1,1000);
yroute=zeros(1,1000);
xroute(1)=xpath(i);
yroute(1)=ypath(j);
plot(xroute(1),yroute(1),'r.','MarkerSize',29)
plot(confx(2),confy(2),'r.','MarkerSize',29)
count=2;
while xroute(count-1)~=confx(2)&&yroute(count-1)~=confy(2)
    if i==101&&j==59
        j=61;
    else
        if direction(j,i)==1
            xroute(count)=xpath(i);
            yroute(count)=ypath(j+1);
            j=j+1;
        elseif direction(j,i)==2
            xroute(count)=xpath(i+1);
            yroute(count)=ypath(j+1);
            i=i+1;
            j=j+1;
        elseif direction(j,i)==3
            xroute(count)=xpath(i+1);
            yroute(count)=ypath(j);
            i=i+1;
        elseif direction(j,i)==4
            xroute(count)=xpath(i+1);
            yroute(count)=ypath(j-1);
            i=i+1;
            j=j-1;
        elseif direction(j,i)==5
            xroute(count)=xpath(i);
            yroute(count)=ypath(j-1);
            j=j-1;
        elseif direction(j,i)==6
            xroute(count)=xpath(i-1);
            yroute(count)=ypath(j-1);
            i=i-1;
            j=j-1;
        elseif direction(j,i)==7
            xroute(count)=xpath(i-1);
            yroute(count)=ypath(j);
            i=i-1;
        elseif direction(j,i)==8
            xroute(count)=xpath(i-1);
            yroute(count)=ypath(j+1);
            i=i-1;
            j=j+1;
        end
        if count>=3
            if xroute(count)==xroute(count-2)&&yroute(count)==yroute(count-2)
                count=count-1;
                if direction(j,i)==2
                    xroute(count)=xpath(i);
                    yroute(count)=ypath(j+1);
                    j=j+1;
                elseif direction(j,i)==3
                    xroute(count)=xpath(i+1);
                    yroute(count)=ypath(j+1);
                    i=i+1;
                    j=j+1;
                elseif direction(j,i)==4
                    xroute(count)=xpath(i+1);
                    yroute(count)=ypath(j);
                    i=i+1;
                elseif direction(j,i)==5
                    xroute(count)=xpath(i+1);
                    yroute(count)=ypath(j-1);
                    i=i+1;
                    j=j-1;
                elseif direction(j,i)==6
                    xroute(count)=xpath(i);
                    yroute(count)=ypath(j-1);
                    j=j-1;
                elseif direction(j,i)==7
                    xroute(count)=xpath(i-1);
                    yroute(count)=ypath(j-1);
                    i=i-1;
                    j=j-1;
                elseif direction(j,i)==8
                    xroute(count)=xpath(i-1);
                    yroute(count)=ypath(j);
                    i=i-1;
                elseif direction(j,i)==1
                    xroute(count)=xpath(i-1);
                    yroute(count)=ypath(j+1);
                    i=i-1;
                    j=j+1;
                end
            end
        end
        count=count+1;
    end
end
xroute=xroute(xroute~=0);
yroute=yroute(yroute~=0);
for i=2:length(xroute)-1
    h=line([xroute(i),xroute(i+1)],[yroute(i),yroute(i+1)]);
    h.LineWidth=2.5;
    h.Color=[1 0 1];
end