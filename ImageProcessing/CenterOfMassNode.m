I = imread('/home/gustavo/Pictures/red-ball1.jpg');
I = im2double(I);
range = [300 50; 0 1; 0 1];
I_m = colorfilter(I,range);
imshow(I_m,[]);

C = centerOfMass(I(:,:,3));

C
