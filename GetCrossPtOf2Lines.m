function Pt=GetCrossPtOf2Lines(k1,b1,k2,b2)

if k1==k2
    errordlg('��ֱ��ƽ�У�û�н��㣡','���棡')
end

Pt(1)=-(b1-b2)/(k1-k2);
Pt(2)=k1*Pt(1)+b1;

end