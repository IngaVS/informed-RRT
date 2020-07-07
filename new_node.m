function feasible = new_node( x,y,path_dist )
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
    feasible = 0;
    diff = 350/2;
    alpha = 1/4*pi;
    a = path_dist / 2;
    c = sqrt(2) * (350-1) / 2;
    if a>c
        b = sqrt(a*a - c*c);
    else
        fprintf("a<c\n");
    end
    u = (x-diff)*cos(alpha) + (y-diff)*sin(alpha);
    v = -(x-diff)*sin(alpha) + (y-diff)*cos(alpha);
    dist = (((x-diff)*cos(1/4*pi) + (y-diff)*sin(1/4*pi))^2) / (a^2) + ((-(x-diff)*sin(1/4*pi) + (y-diff)*cos(1/4*pi))^2 ) / (b^2);

    if dist <= 1
         feasible = 1;
    else 
         feasible = 0;
    end
end

