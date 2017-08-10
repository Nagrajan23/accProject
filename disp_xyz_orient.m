x = a(:,1);
y = a(:,2);
z = a(:,3);
dist = a(:,3);
radians = a(:,3);

dist_x = sqrt((y.*y)+(z.*z));
radians_x = atan(x ./ dist_x);
degree_x = mod(radtodeg(radians_x), 360);

dist_y = sqrt((x.*x)+(z.*z));
radians_y = atan(y ./ dist_y);
degree_y = mod(radtodeg(radians_y), 360);

