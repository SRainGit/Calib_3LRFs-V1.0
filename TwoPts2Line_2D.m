%这里是解如下三个方程的方程组
%a*x1+b*y1+c=0
%a*x2+b*y2+c=0
%a^2+b^2=1
%返回系数[a b c]
function line=TwoPts2Line_2D(data)
    x = data(:,1);
    y = data(:,2);

    k=(y(1)-y(2))/(x(1)-x(2));      %直线斜率，有些情况肯定需要个别判断，这里忽略了
    b=y(1)-k*x(1);  
    
    line=[k -1 b]./sqrt(1+k^2);
end