% Rotation Matrix
% Takes the components of a unit vector in 3d space (x y z) and three angle measures around those axes (tx ty tz)
% Returns the vector formed when the inputted unit vector is rotated at the inputted angles
% Author: Chris Powell

function pos_ = rotmatrix(x, y, z, tx, ty, tz)
 % Calculate the three of rotation matrices in 3d space
 rx=[1 0 0;
     0 cos(tx) -sin(tx);
     0 sin(tx) cos(tx)];
 ry=[cos(ty) 0 sin(ty);
     0 1 0;
     -sin(ty) 0 cos(ty)];
 rz=[cos(tz) -sin(tz) 0;
     sin(tz) cos(tz) 0;
     0 0 1];
 % Calculate total rotation matrix by multiplying together the x, y, and z rotation matrices
 r_=rx*ry*rz
 % Calculate final vector by multiplying total rotation matrix by inputted vector
 pos_=(r_*[x;y;z])'; % Return
end
