function Pt=GetCrossPtOf2Lines(k1,b1,k2,b2)

if k1==k2
    errordlg('两直线平行，没有交点！','警告！')
end

Pt(1)=-(b1-b2)/(k1-k2);
Pt(2)=k1*Pt(1)+b1;

end