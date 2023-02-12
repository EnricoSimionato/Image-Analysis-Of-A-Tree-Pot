clc;
clear;
close all;

% Loading the original image and making it black and white
% Run this section every time it is needed to reset what has been done
figure(1);
image = imread('homework_image.jpg');
image = rgb2gray(image);
imshow(image);
hold on;

%saveas(1,'homework_image_b&w.jpg')

%% Note
% For saving the main figure with the current state of plot run the
% following command:
% saveas(1,'filename.jpg')

%% Feature extraction
% Before tunning the following sections, used for extracting the models of
% the conics C1 and C2 and of the straight lines l1 and l2, it is necessary
% to run the code in the section "Function definition for ransac execution"
% at the end of this file. In the section are defined the function needed
% for performing the MSAC algorithm using the Matlab function "ransac".

% It is possible to avoid the feature extraction sections since the
% extraction has already been performed and in the section "Image analysis
% practical part" there will be a good definition for the conics C1, C2 and
% the edges l1, l2.

%% Extraction of the first conic C1

% Parameters of the crop
crop_row_start = 2160;
crop_row_end = 2380;
crop_column_start = 1520;
crop_column_end = 2705;

% Cropping the image
first_conic = image(crop_row_start:crop_row_end,crop_column_start:crop_column_end);

% Manipulation of the image in order to make more visible the edge points
% Some plots are displayed in order to evaluate the changes
figure(2);
subplot(2,2,1)
imshow(first_conic);

%modified_first_conic = imsharpen(first_conic);
%imshow(modified_first_conic);

modified_first_conic = imadjust(first_conic,[],[],0.1);
subplot(2,2,2)
imshow(modified_first_conic);

modified_first_conic = imsharpen(modified_first_conic);
subplot(2,2,3)
imshow(modified_first_conic);

% Extraction of the edge points using the Canny edge detector
edges_canny = edge(modified_first_conic,'Canny', [0.1 0.3]);
subplot(2,2,4);
imshow(edges_canny);

% Manual manipulation of the image in order to eliminate some outliers
% The manipulation is done in order to fit a best model through the points
for i = 1:height(edges_canny)
    for j = 1:width(edges_canny)
        if (j <= -2.5 * i + 260) || (j >= 1.1* i + 1040)
            edges_canny(i,j) = 0;
        end
    end
end

% Creation of a list containing the edge points 
% Extraction of the coordinates of the edges from the binary image returned
% by the Canny edge detector
total = sum(edges_canny,'all');
points = zeros(total,2);
k = 1;
for i = 1:height(edges_canny)
    for j = 1:width(edges_canny)
        if edges_canny(i,j) == 1
            points(k,2) = i;
            points(k,1) = j;
            k = k + 1;
        end
    end
end

figure(3)
imshow(edges_canny);

% Estimation of the model through MSAC algorithm given the points
sampleSize = 5;
maxDistance = 10;

fit_ellipse_fcn = @(points) fit_ellipse(points);
eval_ellipse_fcn = @(model, points) evaluate_ellipse(model, points);
validate_ellipse_fnc = @(model) validate_ellipse(model);

[C1, inlierIdx] = ransac(points, fit_ellipse_fcn, eval_ellipse_fcn, sampleSize, maxDistance, ValidateModelFcn = validate_ellipse_fnc,  MaxSamplingAttempts = 1000000, MaxNumTrials = 1000000);
a = C1(1);
b = C1(2);
c = C1(3);
d = C1(4);
e = C1(5);
f = C1(6);

% Translating the conic in the original frame
C1 = [a, b/2, (d-2*a*crop_column_start-b*crop_row_start)/2;
    b/2, c, (e-2*c*crop_row_start-b*crop_column_start)/2;
    (d-2*a*crop_column_start-b*crop_row_start)/2, (e-2*c*crop_row_start-b*crop_column_start)/2, f+a*crop_column_start^2+c*crop_row_start^2+b*crop_row_start*crop_column_start-d*crop_column_start-e*crop_row_start];
figure(1);

% Plotting the model
fC1 = @(x,y) C1(1,1)*x^2 + 2*C1(1,2)*x*y + C1(2,2)*y^2 + 2*C1(1,3)*x+ 2*C1(2,3)*y + C1(3,3);
fimplicit(fC1,Color='green',LineWidth=1.5);
hold on;

%% Extraction of the second conic C2
% This part of code tries to automatically extract the conic C2 which is 
% almost a line because of the camera position from which the photo has
% been taken. For a better execution of the image analysis part the conic
% C2 could be extracted manually or it can be run this code many time in 
% order to evaluate the best fit for the conic in the image.
% Since the conic is very squeezed zoom in for seing it.

% Parameters the crop
crop_row_start = 3250;
crop_row_end = 3330;
crop_column_start = 1740;
crop_column_end = 2541;

% Cropping of the image
second_conic = image(crop_row_start:crop_row_end,crop_column_start:crop_column_end);

% Manipulation of the image in order to make more visible the edge points
% Some plots are displayed in order to evaluate the changes
figure(4);
subplot(2,2,1)
imshow(second_conic);

%modified_second_conic = imsharpen(second_conic);
%imshow(modified_second_conic);

modified_second_conic = imadjust(second_conic,[],[],0.45);
subplot(2,2,2)
imshow(modified_second_conic);

modified_second_conic = imsharpen(modified_second_conic);
subplot(2,2,3)
imshow(modified_second_conic);

% Extraction of the edge points using the Canny edge detector
edges_canny = edge(modified_second_conic,'Canny', [0.1 0.3]);
subplot(2,2,4)
imshow(edges_canny)

% Manual manipulation of the image in order to eliminate some outliers
% The manipulation is done in order to fit a best model through the points
for i = 1:height(edges_canny)
    for j = 1:width(edges_canny)
        if (i > 40 && j < 230) || (i>34 && j>100 && j<210)
            edges_canny(i,j) = 0;
        end
    end
end

% Creation of a list containing the edge points 
% Extraction of the coordinates of the edges from the binary image returned
% by the Canny edge detector
total = sum(edges_canny,'all');
points = zeros(total,2);
k = 1;
for i = 1:height(edges_canny)
    for j = 1:width(edges_canny)
        if edges_canny(i,j) == 1
            points(k,2) = i;
            points(k,1) = j;
            k = k + 1;
        end
    end
end

figure(5)
imshow(edges_canny);

% Estimation of the model through MSAC algorithm given the points
sampleSize = 5;
maxDistance = 10;

fit_ellipse_fcn = @(points) fit_ellipse(points);
eval_ellipse_fcn = @(model, points) evaluate_ellipse(model, points);
validate_ellipse_fnc = @(model) validate_ellipse(model);

[C2, inlierIdx] = ransac(points, fit_ellipse_fcn, eval_ellipse_fcn, sampleSize, maxDistance, ValidateModelFcn = validate_ellipse_fnc,  MaxSamplingAttempts = 10000000, MaxNumTrials = 1000000);
a = C2(1);
b = C2(2);
c = C2(3);
d = C2(4);
e = C2(5);
f = C2(6);

% Translating the conic in the original frame
C2 = [a, b/2, (d-2*a*crop_column_start-b*crop_row_start)/2;
    b/2, c, (e-2*c*crop_row_start-b*crop_column_start)/2;
    (d-2*a*crop_column_start-b*crop_row_start)/2, (e-2*c*crop_row_start-b*crop_column_start)/2, f+a*crop_column_start^2+c*crop_row_start^2+b*crop_row_start*crop_column_start-d*crop_column_start-e*crop_row_start];

% Plotting the model
figure(1);
fC2 = @(x,y) C2(1,1)*x^2 + 2*C2(1,2)*x*y + C2(2,2)*y^2 + 2*C2(1,3)*x+ 2*C2(2,3)*y + C2(3,3);
fimplicit(fC2,Color='green',LineWidth=1.5);
hold on;

%% Manual extraction of the second conic C2
% Since the second conic C2 is almost a straight line due to the position
% from which the camera took the picture, it is very difficult to perform
% the automatic extraction of the conic.
% Here is provided the code for manually extract the conic C2 from the image

% Select 5 points belonging to the conic C2 and then press enter

% Fitting the second conic
[x, y] = getpts;
scatter(x, y, 'filled');

% Matrix of the system to be resolved for finding the conic
A = [x.^2 x.*y y.^2 x y ones(size(x))];

% Solving the system A * [a b c d e f]' = 0
N = null(A);
cc = N(:, 1);
[a,b,c,d,e,f] = deal(cc(1),cc(2),cc(3),cc(4),cc(5),cc(6));
C2 = [a b/2 d/2; b/2 c e/2; d/2 e/2 f];

figure(1);
fC2 = @(x,y) C2(1,1)*x^2 + 2*C2(1,2)*x*y + C2(2,2)*y^2 + 2*C2(1,3)*x + 2*C2(2,3)*y + C2(3,3);
fimplicit(fC2,Color='green',LineWidth=1.5);
hold on;

%% Extraction of the first edge of the contour l1

% Parameters of the crop
crop_row_start = 2400;
crop_row_end = 3200;
crop_column_start = 1500;
crop_column_end = 1850;

% Cropping the image
first_edge = image(crop_row_start:crop_row_end,crop_column_start:crop_column_end);

% Manipulation of the image in order to make more visible the edge points
% Some plots are displayed in order to evaluate the changes
figure(6);
subplot(2,2,1)
imshow(first_edge);

%modified_first_edge = imsharpen(first_edge);
%imshow(modified_first_edge);

modified_first_edge = imadjust(first_edge,[],[],0.45);
subplot(2,2,2)
imshow(modified_first_edge);

modified_first_edge = imsharpen(modified_first_edge);
subplot(2,2,3)
imshow(modified_first_edge);

% Extraction of the edge points using the Canny edge detector
edges_canny = edge(modified_first_edge,'Canny', [0.1 0.25]);
subplot(2,2,4)
imshow(edges_canny)

% Creation of a list containing the edge points 
% Extraction of the coordinates of the edges from the binary image returned
% by the Canny edge detector
total = sum(edges_canny,'all');
points = zeros(total,2);
k = 1;
for i = 1:height(edges_canny)
    for j = 1:width(edges_canny)
        if edges_canny(i,j) == 1
            points(k,2) = i+crop_row_start;
            points(k,1) = j+crop_column_start;
            k = k + 1;
        end
    end
end

% Estimation of the model through MSAC algorithm given the points
sampleSize = 2;
maxDistance = 10;

fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1);
evalLineFcn = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);

[model_l1, inlierIdx] = ransac(points, fitLineFcn, evalLineFcn, sampleSize, maxDistance);
l1 = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);

l1 = [l1(1)/(l1(2)); -1/(l1(2)); 1];

% Plotting the model
figure(1)
fl1 = @(x,y) l1(1)*x + l1(2)*y + l1(3);
fimplicit(fl1,Color='green',LineWidth=1.5);
hold on;

%% Extraction of the second edge of the contour l2

% Parameters of the crop
crop_row_start = 2280;
crop_row_end = 3250;
crop_column_start = 2400;
crop_column_end = 2710;

% Cropping the image
second_edge = image(crop_row_start:crop_row_end,crop_column_start:crop_column_end);

% Manipulation of the image in order to make more visible the edge points
% Some plots are displayed in order to evaluate the changes
figure(7);
subplot(2,2,1)
imshow(second_edge);

%modified_second_edge = imsharpen(second_edge);
%imshow(modified_second_edge);

modified_second_edge = imadjust(second_edge,[],[],0.3);
subplot(2,2,2)
imshow(modified_second_edge);

modified_second_edge = imsharpen(modified_second_edge);
subplot(2,2,3)
imshow(modified_second_edge);

% Extraction of the edge points using the Canny edge detector
edges_canny = edge(modified_second_edge,'Canny', [0.1 0.23]);
subplot(2,2,4)
imshow(edges_canny)

% Creation of a list containing the edge points 
% Extraction of the coordinates of the edges from the binary image returned
% by the Canny edge detector
total = sum(edges_canny,'all');
points = zeros(total,2);
k = 1;
for i = 1:height(edges_canny)
    for j = 1:width(edges_canny)
        if edges_canny(i,j) == 1
            points(k,2) = i+crop_row_start;
            points(k,1) = j+crop_column_start;
            k = k + 1;
        end
    end
end

% Estimation of the model through MSAC algorithm given the points
sampleSize = 2;
maxDistance = 10;

fitLineFcn = @(points) polyfit(points(:,1),points(:,2),1);
evalLineFcn = @(model, points) sum((points(:, 2) - polyval(model, points(:,1))).^2,2);

[model_l2, inlierIdx] = ransac(points, fitLineFcn, evalLineFcn, sampleSize, maxDistance);
l2 = polyfit(points(inlierIdx,1),points(inlierIdx,2),1);

l2 = [l2(1)/(l2(2)); -1/(l2(2)); 1];

% Plotting of the model
figure(1)
fl2 = @(x,y) l2(1)*x + l2(2)*y + l2(3);
fimplicit(fl2,Color='green',LineWidth=1.5);
hold on;

%% Image analysis practical part
% Since the extraction can be sometimes a tedious procedure, from now on I
% use some previously computed values for the conics C1, C2 and the edges
% l1, l2. For using the extracted geometrical entities, avoid running this
% section of code.

% Loading the image
figure(1);
image = imread('homework_image.jpg');
image = rgb2gray(image);
imshow(image);
hold on;

% Conics C1, C2 and edges l1, l2
C1 = [3.161957855588481e-06,3.411132373639874e-07,-0.007469209039088;
    3.411132373639874e-07,3.276147383366991e-05,-0.076985722524439;
    -0.007469209039088,-0.076985722524439,1.938874386044703e+02];

C2 = [2.917454387914770e-10,-4.022747243160237e-09,1.266548245681135e-05;
    -4.022747243160237e-09,9.670585431909434e-08,-3.108899697224607e-04;
    1.266548245681135e-05,-3.108899697224607e-04,0.999999806374001];

l1 = [-0.001025010788568;2.392688341199368e-04;1];

l2 = [-3.289875548609350e-04;-4.905786617675837e-05;1];

% Plotting the conics and the edges on the image
figure(1)
fC1 = @(x,y) C1(1,1)*x^2 + 2*C1(1,2)*x*y + C1(2,2)*y^2 + 2*C1(1,3)*x + 2*C1(2,3)*y + C1(3,3);
fimplicit(fC1,Color='green',LineWidth=1.5);
hold on;

fC2 = @(x,y) C2(1,1)*x^2 + 2*C2(1,2)*x*y + C2(2,2)*y^2 + 2*C2(1,3)*x + 2*C2(2,3)*y + C2(3,3);
fimplicit(fC2,Color='green',LineWidth=1.5);
hold on;

fl1 = @(x,y) l1(1)*x + l1(2)*y + l1(3);
fimplicit(fl1,Color='green',LineWidth=1.5);
hold on;

fl2 = @(x,y) l2(1)*x + l2(2)*y + l2(3);
fimplicit(fl2,Color='green',LineWidth=1.5);
hold on;

%saveas(1,'homework_image_contour.jpg')

%% Question 1
% From C1 and C2, find the horizon (vanishing) line h of the plane 
% orthogonal to the cone axis

% Solving the system for finding the point of intersection between the two
% conics C1, C2
syms 'x';
syms 'y';

eq1 = C1(1,1)*x^2 + 2*C1(1,2)*x*y + C1(2,2)*y^2 + 2*C1(1,3)*x + 2*C1(2,3)*y + C1(3,3);
eq2 = C2(1,1)*x^2 + 2*C2(1,2)*x*y + C2(2,2)*y^2 + 2*C2(1,3)*x + 2*C2(2,3)*y + C2(3,3);

eqns = [eq1 == 0, eq2 == 0];
S = solve(eqns, [x,y]);

% Two out of this 4 points are the images of the circular points
s1 = [double(S.x(1));double(S.y(1));1];
s2 = [double(S.x(2));double(S.y(2));1];
s3 = [double(S.x(3));double(S.y(3));1];
s4 = [double(S.x(4));double(S.y(4));1];

% Complex conjugates points are projected on complex conjugate points

% Computation and plot of the two candidate horizons for evaluating which
% one of them is the correct one
% The imaginary part of the two lines, h and hor, is very small so it can
% be eliminated using the function real.

h = null([s1'; s2']);
h = real(h);
h = h/norm(h);

figure(1)
fh = @(x,y) h(1)*x + h(2)*y + h(3);
fimplicit(fh,Color='green',LineWidth=1.5);
hold on;
horizon_text_position = 1300;
text(horizon_text_position, -(h(1)/h(2))*horizon_text_position-h(3)/h(2)-50, 'horizon','Color','green')
hold on;

hor = null([s3.'; s4.']);
hor = real(hor);
hor = hor/norm(hor);

figure(1)
fhor = @(x,y) hor(1)*x + hor(2)*y + hor(3);
fimplicit(fhor,Color='green',LineWidth=0.5);
hold on;
horizon_text_position = 1000;
text(horizon_text_position, -(hor(1)/hor(2))*horizon_text_position-hor(3)/hor(2)+60, 'other horizon candidate','Color','green')
hold on;

% Analyzing the image, evaluating where the parallel lines converge we can
% state that the true horizon is the line h

%saveas(1,'homework_image_horizon.jpg')

%% Question 2
% From l1,l2, C1, C2, find the image projection a of the cone axis

% Finding the image of the vertex of the cone: the axis must go through 
% this point because the projection preserve colinearity
vertex = cross(l1, l2);
vertex = vertex / vertex(3);

figure(1);
scatter(vertex(1), vertex(2), 30, 'filled','red');
hold on;
text(vertex(1), vertex(2) + 30, 'vertex','Color','red')
hold on;

% Finding the image of the centre of the first circular cross section
% The centre of a circumference has as polar line the line at infinity. 
% For this reason the polar line of the the projection of the centre of the
% circumferences is the horizon.
% The image of the centre of the first cross section is given by the
% inverse of the first conic times the horizon.

centre_C1 = inv(C1) * h;
centre_C1 = centre_C1 / centre_C1(3);

% Plotting the image of the centre 
figure(1);
scatter(centre_C1(1), centre_C1(2), 30, 'filled', 'red');
hold on;
text(centre_C1(1) - 270, centre_C1(2) + 30, 'centre 1','Color','red');
hold on;

% Computing the image of the axis of the cone as the line going through the
% point vertex and the image of the centre of the first cross section

axis = cross(vertex, centre_C1);
axis = axis / norm(axis);

% Plotting the image of the axis of the cone
figure(1);
fa = @(x,y) axis(1)*x + axis(2)*y + axis(3);
fimplicit(fa,Color='yellow',LineWidth=1.5);
hold on;
axis_text_position = 1000;
text(-(axis(2)/axis(1))*axis_text_position-axis(3)/axis(1)+50, axis_text_position, 'axis','Color','yellow')
hold on;

% Finding the image of the centre of the second circular cross section
% As double check I evaluate whether the axis goes through the image of the
% centre of the second cross section.

centre_C2 = inv(C2) * h;
centre_C2 = centre_C2 / centre_C2(3);

% Plotting the image of the centre
figure(1);
scatter(centre_C2(1), centre_C2(2), 30, 'filled', 'red');
hold on;
text(centre_C2(1) + 30, centre_C2(2) + 40, 'centre 2','Color','red');
hold on;

%saveas(1,'homework_image_axis.jpg')

%% Question 3
% From l1,l2, C1, C2 (and possibly h and a), find the calibration matrix K.

% The shape of K, since the camera is natural and has zero skew, is:
% K = [f 0 u 
%      0 f v
%      0 0 1]

% For computing K of the camera, and so , for calibrating the camera, it is
% possible to exploit its relation between K and the image of the absolute
% conic w. w^-1 = K * K.'

% The image of the absolute conic is needed and it is possible tp compute
% it solving the system of equations given by the equation of  belonging of
% the images of the circular points to the image of the absolute conic and 
% the equation given by the orthogonality, in the real scene, of two
% directions. If two directions are orthogonal in the real scene, the
% vanishig point on this directions v1, v2 satisfy the equation v1.'*w*v2 =
% 0.

% As v1 I choose arbitrarly a point on the horizon imposing x of v1 and
% then computing the other coordinate y; all the vanishing points are
% placed on the horizon.

x_v1 = 128;
y_v1 = -(h(3)+h(1)*x_v1)/h(2);
v1 = [x_v1;y_v1;1];

% Plotting the vanishing point v1
figure(1)
scatter(v1(1), v1(2), 30, 'filled','blue');
hold on;
text(v1(1) + 30, v1(2) + 40, 'v1','Color','blue');
hold on;

% Finding the polar line of the point v1 with respect to the conic C1
% The direction identified by the vanishing point v1 and the direction of
% the polar line of the point v1 with respect to the circular cross section
% are orthogonal in the real scene.

line_orthogonal_to_direction_v1 = C1 * v1;
line_orthogonal_to_direction_v1 = line_orthogonal_to_direction_v1 / norm(line_orthogonal_to_direction_v1);

% Plotting the image of the line orthogonal to direction of the vanishing
% point v1
fline_orthogonal_to_direction_v1 = @(x,y) line_orthogonal_to_direction_v1(1)*x + line_orthogonal_to_direction_v1(2)*y + line_orthogonal_to_direction_v1(3);
fimplicit(fline_orthogonal_to_direction_v1,Color='blue',LineWidth=1.5);
hold on;

% Finding the vanishing point in the direction,in the real scene,
% orthogonal to v1 as point of intersection between the line 
% line_orthogonal_to_direction_v1 and the horizon
v2 =  cross(line_orthogonal_to_direction_v1, h);
v2 = v2 / v2(3);

% Plotting the vanishing point in the direction orthogonal to the vanishing
% point v1
figure(1)
scatter(v2(1), v2(2), 30, 'filled','blue');
hold on;
text(v2(1) + 30, v2(2) + 30, 'v2','Color','blue');
hold on;

% Finding the image of the absolute conic solving the system of equations
% given by the equation of  belonging of the images of the circular points 
% to the image of the absolute conic and the equation given by the
% orthogonality, in the real scene, of the direction associated to v1 and 
% v2

% Defining and solving the system of equations
syms 'f';
syms 'U0';
syms 'V0';

% Shape of the image of the absolute conic 
w = [1/f^2, 0, -U0/f^2;
    0, 1/f^2, -V0/f^2;
    -U0/f^2, -V0/f^2, (f^2+U0^2+V0^2)/f^2];

% Equations of the system
eq1 = s1.' * w * s1 == 0;
eq2 = s2.' * w * s2 == 0;
eq3 = v1.' * w * v2 == 0;

% Solving the system of equations
S = solve([eq1 eq2 eq3], [f,U0,V0]);
f = double(S.f);
U0 = double(S.U0);
V0 = double(S.V0);
 
% Calibration matrix
K = [abs(f(2)) 0 abs(U0(2));
    0 abs(f(2)) abs(V0(2));
    0 0 1]

% The three values f, U0 and V0 must be positive.
% The choise of taking the absolute value of the results is due to the fact that
% the solutions would be 2 for each variable, one with the plus sign and
% one with the minus. Using the function abs we can avoid to check which 
% solution is the poistive one and always obtain the correct value.

% It is important to say that for small variation of the points v1 and v2
% the variation of K is huge and it is possible to end up with complex
% parameters which cannot be assumed by the parameters.

%saveas(1,'homework_image_computation_K.jpg')

%% Question 4
% From h and K, determine the orientation of the cone axis wrt to the 
% camera reference.

% Since we are interested in the relative position between the axis of the cone
% and the camera, namely the axis orientation in the camera frame, we can
% assume that the camera projection matrix has a shape P = [K 0], so with R
% = I and t = 0 (where I is the identity matrix and 0 is the null vector).

% The axis direction in the real scene is orthogonal to the planes of both
% the two circular cross sections. So, computing a plane parallel to these
% planes, it is possible to obtain the vector of the axis since a plane is
% identified by the vector normal to it.

% The plane of the cross sections can be obtained as backprojection of the
% image of their line at infinity, namely the already computed horizon.

% The backprojection of a line is given by the multiplication between the
% transpose of the camera projection matrix K and the considered line. The
% result is a plane phi_h.

% Definition of the camera projection matrix P
P = [K, zeros(3,1)];

% Backprojection of the horizon
phi = P.'*h;

% The part of phi in which we are really interested is composed by the 
% first three coordinates
phi = phi/phi(3);
phi = phi(1:3);

fprintf(['Angle with respect to x ' num2str((acos(phi(2)/norm(phi))/pi)*180) ' degree\n']);
fprintf(['Angle with respect to y ' num2str((acos(phi(1)/norm(phi))/pi)*180) ' degree\n']);
fprintf(['Angle with respect to z ' num2str((acos(phi(3)/norm(phi))/pi)*180) ' degree\n']);

figure(8)
plot3([0,phi(1)],[0,phi(2)],[0,phi(3)],'Color','red','LineStyle','-','LineWidth',2);
text(phi(1) + 0.1, phi(2) + 0.1, phi(3) + 0.1, 'axis orientation','Color','red');
hold on;
plot3([0,1],[0,0],[0,0],'Color','green','LineStyle','-','LineWidth',2);
hold on;
plot3([0,0],[0,1],[0,0],'Color','blue','LineStyle','-','LineWidth',2);
hold on;
plot3([0,0],[0,0],[0,1],'Color','yellow','LineStyle','-','LineWidth',2);
xlabel('x')
ylabel('y')
zlabel('z')
grid on;

%saveas(8,'axis_orientation_camera_frame.jpg')

%% Function definition for ransac execution

function model=fit_ellipse(points)
    A = [points(:,1).^2 points(:,1).*points(:,2) points(:,2).^2 points(:,1) points(:,2) ones(height(points))];
    N = null(A);
    cc = N(:, 1);
    [a,b,c,d,e,f] = deal(cc(1),cc(2),cc(3),cc(4),cc(5),cc(6));
    model = [a,b,c,d,e,f];
end

function distances=evaluate_ellipse(model, points)
    distances = zeros(height(points),1);
    k = 100;
    for i=1:height(points)
        if (model(2)*points(i,1) + model(5))^2-4*model(3)*(model(1)*(points(i,1).^2)+model(6)+model(4)*points(i,1)) >= 0
            y1 = (-(model(2)*points(i,1) + model(5)) + sqrt((model(2)*points(i,1) + model(5))^2-4*model(3)*(model(1)*(points(i,1)).^2+model(6)+model(4)*points(i,1))))/(2*model(3));
            y2 = (-(model(2)*points(i,1) + model(5)) - sqrt((model(2)*points(i,1) + model(5))^2-4*model(3)*(model(1)*(points(i,1)).^2+model(6)+model(4)*points(i,1))))/(2*model(3));
            
            if (y1-points(i,2))^2 <(y2-points(i,2))^2
                distances(i,1) = (y1-points(i,2)).^2;
            else
                 distances(i,1) = (y2-points(i,2)).^2;
            end
        elseif (model(2)*points(i,2) + model(4))^2-4*model(1)*(model(3)*(points(i,2)).^2+model(6)+model(5)*points(i,2)) >= 0
            x1 = (-(model(2)*points(i,2) + model(5)) + sqrt((model(2)*points(i,2) + model(4))^2-4*model(1)*(model(3)*(points(i,2)).^2+model(6)+model(5)*points(i,2))))/(2*model(1));
            x2 = (-(model(2)*points(i,2) - model(5)) + sqrt((model(2)*points(i,2) + model(4))^2-4*model(1)*(model(3)*(points(i,2)).^2+model(6)+model(5)*points(i,2))))/(2*model(1));
            if (x1-points(i,1))^2 <(x2-points(i,1))^2
                distances(i,1) = (x1-points(i,1)).^2;
            else
                 distances(i,1) = (x2-points(i,1)).^2;
            end
        else
            distances(i,1) = k;
        end
    end
end

function isValid = validate_ellipse(model,varargin)
    C = [model(1), model(2)/2, model(4)/2;
        model(2)/2 model(3) model(5)/2;
        model(4)/2 model(5)/2 model(6)];
    determinant_C = C(3,3)*(C(1,1)*C(2,2)-C(1,2)*C(2,1))-C(2,3)*(C(1,1)*C(3,2)-C(1,2)*C(3,1))+C(1,3)*(C(2,1)*C(3,2)-C(1,2)*C(3,1));
    if determinant_C == 0
        isValid = 0;
    else
        determinant_C33 = C(1,1)*C(2,2)-C(1,2)*C(2,1);
        if determinant_C33 > 0
            isValid = 1;
        else
            isValid = 0;
        end
    end
end