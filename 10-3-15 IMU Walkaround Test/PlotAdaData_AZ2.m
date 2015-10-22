%Graphs path traveled, x displacement over time, and y displacement over
%time.
%Returns an array with time, x displacement, and y displacement
%Expects array ax, ay, gz, and time

%Integrates gz(omega) ax, and ay into theta (with 0 degrees = East), x
%displacement, and y displacement.
theta = cumtrapz(time, gz);

ax_ = ax .* cos(theta) - ay .* sin(theta);
ay_ = ax .* sin(theta) + ay .* cos(theta);

%vx = cumtrapz(time, ax_);
%vy = cumtrapz(time, ay_);

%dx = cumtrapz(time, vx);
%dy = cumtrapz(time, vy);

%dxy = [time dx dy];



subplot(1,3,1);
plot(time, gz)
title('theta vs. time')
%axis([-125, 125, 0, 250])
axis square
grid on

subplot(1,3,2);
plot(time, cumtrapz(gz))
title('theta vs. time')
%axis([-125, 125, 0, 250])
axis square
grid on