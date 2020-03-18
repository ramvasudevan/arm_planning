% testing out the 3D dynamics to make sure i'm getting x-y-z correct

l1 = 0.33;
x0 = [l1; 0; 0; 2*pi; 0; 2*pi; 0; 0];

[tout, xout] = ode45(@arm_dyn_toPeak_3D, [0, 0.5], x0);

figure(1); clf; hold on; view(3);
xlim([-1, 1]);
ylim([-1, 1]);
zlim([-1, 1]);
plot3(xout(:, 1), xout(:, 2), xout(:, 3), 'b.', 'MarkerSize', 5);