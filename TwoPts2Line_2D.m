%�����ǽ������������̵ķ�����
%a*x1+b*y1+c=0
%a*x2+b*y2+c=0
%a^2+b^2=1
%����ϵ��[a b c]
function line=TwoPts2Line_2D(data)
    x = data(:,1);
    y = data(:,2);

    k=(y(1)-y(2))/(x(1)-x(2));      %ֱ��б�ʣ���Щ����϶���Ҫ�����жϣ����������
    b=y(1)-k*x(1);  
    
    line=[k -1 b]./sqrt(1+k^2);
end