clc; clear all;

drone = ryze("Tello");

dronecamera = camera(drone);

preview(dronecamera);

takeoff(drone);

moveforward(drone, 'Distance', 2.0);

turn(drone, deg2rad(120));

moveforward(drone, 'Distance', 1.2);

turn(drone, deg2rad(60));

moveforward(drone, 'Distance', 1.2);

img = snapshot(dronecamera);

imwrite(img, '22DroneBoy_image.png');

turn(drone, deg2rad(60));

moveforward(drone, 'Distance', 1.2);

land(drone);

closePreview(dronecamera);
