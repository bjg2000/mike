clc;clear;close all;
accuracy_calc = @(z, y) (2 * z / 12)^2 + (2 * y / 18)^2;
z = 6;
y = 0;

accuracy_calc(z,y)