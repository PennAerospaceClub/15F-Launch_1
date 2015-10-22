function pos_ = rotmatrix(x, y, z, tx, ty, tz)
 rx=[1 0 0;
     0 cos(tx) -sin(tx);
     0 sin(tx) cos(tx)];
 ry=[cos(ty) 0 sin(ty);
     0 1 0;
     -sin(ty) 0 cos(ty)];
 rz=[cos(tz) -sin(tz) 0;
     sin(tz) cos(tz) 0;
     0 0 1];
 r_=rx*ry*rz
 pos_=(r_*[x;y;z])';
end