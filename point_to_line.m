function dist = point_to_line(pt, v1, v2)
if length(pt)==2
    pt=[pt 0];
    v1=[v1 0];
    v2=[v2 0];
end
a=v1 - v2;
b=pt - v2;
D=norm(cross(a,b)) / norm(a);
A=sqrt((v1(1)-v2(1))^2+(v1(2)-v2(2))^2+(v1(3)-v2(3))^2);
B=sqrt((pt(1)-v1(1))^2+(pt(2)-v1(2))^2+(pt(3)-v1(3))^2);
C=sqrt((pt(1)-v2(1))^2+(pt(2)-v2(2))^2+(pt(3)-v2(3))^2);
if C^2>=A^2+B^2
    dist=B;
elseif B^2>=A^2+C^2
    dist=C;
else
    dist=D;
end
end