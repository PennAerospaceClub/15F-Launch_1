% Create time array
time=(0:0.1:(length(pitch)-1)/10)';

% Create position unit vector
pos=zeros((length(pitch)),3);
pos(1,:)=[1 0 0];

% Calibrate each angular velocity vector by subtracting the initial (at rest)
% velocity
ox=roll-roll(1);
oy=pitch-pitch(1);
oz=yaw-yaw(1);

% Integrate angular velocities to get theta
tx=cumtrapz(time,ox);
ty=cumtrapz(time,oy);
tz=cumtrapz(time,oz);

for n = 2:length(pitch)
    pos(n,:)=rotmatrix(pos(n-1,1), pos(n-1,2), pos(n-1,3), ...
    tx(n)-tx(n-1), ty(n)-ty(n-1), tz(n)-tz(n-1));
end

% Plot theta vs. time for all 3 DOF
subplot(3,1,1)
plot(time,pos(:,1))
title('Roll(x) vs. Time')
xlabel('Time (s)')
ylabel('Roll (deg)')

subplot(3,1,2)
plot(time,pos(:,2))
title('Pitch(y) vs. Time')
xlabel('Time (s)')
ylabel('Pitch (deg)')

subplot(3,1,3)
plot(time,pos(:,3))
title('Yaw(z) vs. Time')
xlabel('Time (s)')
ylabel('Yaw (deg)')