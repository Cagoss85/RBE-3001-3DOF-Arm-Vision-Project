function [ball_data] = lab5imageTesting
cam = Camera();

%cam.DEBUG = DEBUG_CAM;

%cam.cam_pose = cam.getCameraPose();

[img, newParams] = cam.undistortSnapshot();

x1 = [710.256097560976 1198.20034843206 1370.04703832753 500.778745644599];
y1 = [364.890243902439 403.775261324042 728.65331010453 679.733449477352];

mask = roipoly(img, x1, y1);
img = im2double(img);
img = mask.*img;


lime_green = lime_green_Mask(img);
red_orange = red_orange_Mask(img);
pink = pink_Mask(img);
yellow = yellow_Mask(img);

noise_reduction_green = medfilt2(lime_green, [20 20]);
noise_reduction_red = medfilt2(red_orange, [20 20]);
noise_reduction_pink = medfilt2(pink, [20 20]);
noise_reduction_yellow = medfilt2(yellow, [20 20]);

region_green = regionprops(noise_reduction_green, 'centroid');
region_red = regionprops(noise_reduction_red, 'centroid');
region_pink = regionprops(noise_reduction_pink, 'centroid');
region_yellow = regionprops(noise_reduction_yellow, 'centroid');




 
% figure(2)
% 
% subplot(4,1,1)
% imshow(lime_green);
% 
% subplot(4,1,2)
% imshow(red_orange);
% 
% subplot(4,1,3)
% imshow(pink);
% 
% subplot(4,1,4)
% imshow(yellow);
% 
% 
% figure(3)
% 
% subplot(4,1,1)
% imshow(noise_reduction_green);
% 
% subplot(4,1,2)
% imshow(noise_reduction_red);
% 
% subplot(4,1,3)
% imshow(noise_reduction_pink);
% 
% subplot(4,1,4)
% imshow(noise_reduction_yellow);


if isempty(region_green) 
    green = [];
else
    green = [1 region_green.Centroid(1) region_green.Centroid(2)];
end    

if isempty(region_red) 
    red = [];
else
    red = [2 region_red.Centroid(1) region_red.Centroid(2)];
end  

if isempty(region_pink) 
    pink = [];
else
    pink = [3 region_pink.Centroid(1) region_pink.Centroid(2)];
end  

if isempty(region_yellow) 
    yellow = [];
else
    yellow = [4 region_yellow.Centroid(1) region_yellow.Centroid(2)];
end  

ball_data = cat(1, green, red, pink, yellow);

figure(1)
imshow(img)
hold on
if isempty(region_green) == 0
    plot(region_green.Centroid(1), region_green.Centroid(2), 'b*')
end
if isempty(region_red) == 0
    plot(region_red.Centroid(1), region_red.Centroid(2), 'b*')
end
if isempty(region_pink) == 0 
    plot(region_pink.Centroid(1), region_pink.Centroid(2), 'b*')
end
if isempty(region_yellow) == 0
    plot(region_yellow.Centroid(1), region_yellow.Centroid(2), 'b*')
end
hold off


