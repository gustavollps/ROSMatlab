function I = colorfilter(image, range)

    % RGB to HSV conversion
    I = rgb2hsv(image);         
    
    % Normalization range between 0 and 1
    range(:,1) = range(:,1)./360;    

    if(range(1) > range(2))
        % Red hue case
        mask =    (I(:,:,1)>range(1,1) & (I(:,:,1)<=1)) + (I(:,:,1)<range(1,2) & (I(:,:,1)>=0))...
                & (I(:,:,2)>range(2,1) & (I(:,:,2)<=1)) + (I(:,:,2)<range(2,2) & (I(:,:,2)>=0))...
                & (I(:,:,3)>range(3,1) & (I(:,:,3)<=1)) + (I(:,:,3)<range(3,2) & (I(:,:,3)>=0));        
    else
        % Regular case
        mask =   (I(:,:,1)>range(1,1)) & (I(:,:,1)<range(1,2))...
               & (I(:,:,2)>range(2,1)) & (I(:,:,2)<range(2,2))...
               & (I(:,:,3)>range(3,1)) & (I(:,:,3)<range(3,2));
    end
    
    % Saturation is modified according to the mask
    
    I(:,:,1) = mask;
    %I(:,:,2) = mask;
    %I(:,:,3) = mask;

    
    % HSV to RGB conversion
    I = hsv2rgb(I);
    
end