% rigidTransform3D(points1, points2) Fit a rigid transformation between two sets of 3-D points
%  [R, t] = rigidTransform3D(points1, poitns2) computes a rigid
%  trasnsormation between two sets of corresponding 3-D points. points1 and
%  points2 are M-by-3 arrays of [x,y,z] coordinates. R is a 3-by-3 rotation
%  matrix and t is a 1-by-3 translation vector. 
%
%  If the number of points is greater than 3, the rigidTransform3D computes
%  a least-squares fit.
%
%  Note: the function returns t as a column vector, and R in the
%  post-multiply convention (matrix times column vector). R and t must be
%  transposed to be used in most CVST functions.

% Copyright 2016 MathWorks, Inc.

%#codegen

function [R, t] = rigidTransform3D(p, q)

n = cast(size(p, 1), 'like', p);
m = cast(size(q, 1), 'like', q);

% Find data centroid and deviations from centroid
pmean = sum(p,1)/n;
p2 = bsxfun(@minus, p, pmean);

qmean = sum(q,1)/m;
q2 = bsxfun(@minus, q, qmean);

% Covariance matrix
C = p2'*q2;

[U,~,V] = svd(C);

% Handle the reflection case
R = V*diag([1 1 sign(det(U*V'))])*U';

% Compute the translation
t = qmean' - R*pmean';
