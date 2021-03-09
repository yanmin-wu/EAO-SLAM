function distMat = euclideanDistanceTwoPointClouds(reference,sample)
% calculate the euclidean distance for every point in sample point cloud to 
% the closest point in the reference point cloud (number of columns must
% match)

% INPUT: 
%     reference = M x N matrix
%     sample    = P x N matrix

% OUTPUT:
%     distMat   = P x N matrix

distMat = zeros(size(sample,1),1);
for row_idx = 1:size(sample,1)
    diff = reference-repmat(sample(row_idx,:),size(reference,1),1);
    D = sqrt(sum(diff.^2,2));
    distMat(row_idx) = min(D);
end
