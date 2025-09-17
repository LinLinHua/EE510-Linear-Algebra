%This program is a template for rotating a JPG image.
%Mike Neely, Spring 2025. 
%
%
%theta = angle of rotation
%Input: File of type .jpg or .jpeg 
%       (parasite.jpeg is used in this case)
%Output: File RotatedImage.jpg
%Matlab display: the processed image Mrotate.
%
%This is currently configured to have Mrotate = M. You 
%need to do some type of processing to get the rotation. 
%
%Eventual homework structure: Use the rotation matrix R 
%(or its inverse Rinv) somehow. Do two types of rotations: 
%One is a hard rounding that maps integer pixel locations to 
%integer pixel locations using ceil() or floor().  Another
%is something that you think up to make it look nicer. Then
%make a 2-3 page writeup that briefly describes your approaches
%and displays the image results. You can use the parasite.jpeg
%and/or other jpg images of your choice. 

clear; 
theta = .1*pi/2;
colorimage=imread('parasite.jpeg');
%colorimage=imread('tulsi.jpg');

%bwimage = rgb2gray(colorimage); 
%imwrite(bwimage, 'bwimage.jpg', 'jpg');

% This converts the jpeg to 3 rectangular matrices 
% of same size and with double precision
% entries for data processing. The M data structure
%contains all 3 matrices, and MR, MG, MB contain the RGB
%components.

M = im2double(colorimage); 
MR = M(:,:,1); % This is the Red intensity matrix.
MG = M(:,:,2); % This is the Green intensity matrix (same size as MR).
MB = M(:,:,3); % This is the Blue intensity matrix (same size as MR).
rows = size(MR,1); %This is the number of rows.  
cols = size(MR,2); %This is the number of columns. 

%This is the rotation matrix you can use. 
R = [cos(theta) -1*sin(theta); sin(theta) cos(theta)];
Rinv = R^(-1);

%% Now you can do data processing on the M, and/or separately
%% on the 3 RGB component matrices MR, MG, MB. In both my basic
%% rotation and enhanced rotation programs, I just did the same procedure
%% separately to MR, MG, MB. 
new_rows = ceil(rows * abs(cos(theta)) + cols * abs(sin(theta)));
new_cols = ceil(rows * abs(sin(theta)) + cols * abs(cos(theta)));

%Mrotate = M; 
Mrotate = zeros(new_rows, new_cols, 3);
center_old = [cols / 2; rows / 2];
center_new = [new_cols / 2; new_rows / 2];


%% Step 1: Floor/Ceil Pull Technique
% Loop through each pixel in the output image
for x_new = 1:cols
    for y_new = 1:rows
        % Transform cooridinates to the original image
        % retrive the difference of x,y, fine the Rinv and then add back the center to make sure the image is in the center position 
        original_cor = Rinv * ([x_new; y_new] - center_new) + center_old;
        x_old=original_cor(1);
        y_old=original_cor(2);

        % Apply floor/ceil rounding
        x_floor = floor(x_old);
        y_floor = floor(y_old);
        x_ceil=ceil(x_old);
        y_ceil=ceil(y_old);

        %Check bounds and assign values from the nearest valid pixel
        if x_floor>=1 && x_floor<=cols && y_floor>= 1 && y_floor<=rows
            Mrotate(y_new, x_new, 1)= MR(y_floor,x_floor);
            Mrotate(y_new, x_new, 2)= MG(y_floor,x_floor);
            Mrotate(y_new, x_new, 3)= MB(y_floor, x_floor);
        % elseif (x_ceil >=1 && x_floor <= cols && y_floor >=1 && y_floor <= rows)
        %     Mrotate(y_new, x_new, 1)= MR(y_ceil,x_ceil);
        %     Mrotate(y_new, x_new, 2)= MG(y_ceil,x_ceil);
        %     Mrotate(y_new, x_new, 3)= MB(y_ceil, x_ceil);
        end
    end
end

%newbwimage = im2uint8(B);
imwrite(Mrotate, 'RotatedImage.jpg', 'jpg');
imshow(Mrotate);

size(M) % This shows how many pixels are used in the original image. 


%% Step 2: Interpolation
Mrotate = zeros(new_rows, new_cols, 3);
center_old = [cols / 2; rows / 2];
center_new = [new_cols / 2; new_rows / 2];

for x_new = 1:cols
    for y_new = 1:rows
        % Transform cooridinates to the original image
        % retrive the difference of x,y, fine the Rinv and then add back the center to make sure the image is in the center position 
        original_cor = Rinv * ( [x_new; y_new] - center_new) + center_old; 
        x_old=original_cor(1);
        y_old=original_cor(2);

        % Interpolation
        if x_old >= 1 && x_old <= cols && y_old >= 1 && y_old <= rows
            %Surrounding Position
            x1 = floor(x_old); 
            x2 = ceil(x_old);
            y1 = floor(y_old); 
            y2 = ceil(y_old);
            
            %Check the boundary
            if x1 < 1, x1 = 1; end
            if x2 > cols, x2 = cols; end
            if y1 < 1, y1 = 1; end
            if y2 > rows, y2 = rows; end
            
            %Get the surrounding pixel
            Q11 = [MR(y1, x1), MG(y1, x1), MB(y1, x1)];
            Q12 = [MR(y2, x1), MG(y2, x1), MB(y2, x1)];
            Q21 = [MR(y1, x2), MG(y1, x2), MB(y1, x2)];
            Q22 = [MR(y2, x2), MG(y2, x2), MB(y2, x2)];
            
            %Interpolation Calculation
            fxy1 = (x2 - x_old) * Q11 + (x_old - x1) * Q21;
            fxy2 = (x2 - x_old) * Q12 + (x_old - x1) * Q22;
            fxy = (y2 - y_old) * fxy1 + (y_old - y1) * fxy2;
            
            % New Interpolation
            Mrotate(y_new, x_new, :) = fxy;
        end
    end
end


%newbwimage = im2uint8(B);
imwrite(Mrotate, 'EnhancedRotatedImage.jpg', 'jpg');
imshow(Mrotate);

size(M) % This shows how many pixels are used in the original image. 

%subplot(1,2,1), imshow(M);
%subplot(1,2,2), imshow(Mrotate);



 