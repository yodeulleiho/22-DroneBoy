clc; clear all;
% 1. 임의의 세 개의 3차원 벡터를 키보드 입력으로 받는다.
disp('3개의 3차원 벡터를 입력하시오 [x1 y1 z1; x2 y2 z2; x3 y3 z3]:');
vectors = input('');

% 2. 각각 유클라디안 거리와 코사인 유사도를 계산한다.
distence_12 = norm(vectors(1,:) - vectors(2,:));
distence_23 = norm(vectors(2,:) - vectors(3,:));
distence_13 = norm(vectors(1,:) - vectors(3,:));

cos_sim_12 = dot(vectors(1,:), vectors(2,:)) / (norm(vectors(1,:)) * norm(vectors(2,:)));
cos_sim_23 = dot(vectors(2,:), vectors(3,:)) / (norm(vectors(2,:)) * norm(vectors(3,:)));
cos_sim_13 = dot(vectors(1,:), vectors(3,:)) / (norm(vectors(1,:)) * norm(vectors(3,:)));

% 3. 유클라디안 거리로 보았을 때 가장 가까운 벡터 2개와 코사인 유사도로 보았을 때 가장 유사한 벡터 2개를 출력한다.

distances = [distence_12 distence_23 distence_13];  %벡터간 거리 행렬
similarities = [cos_sim_12 cos_sim_23 cos_sim_13];  %벡터간 코사인 행렬

[dmin, dmin_index] = min(distances);      %가장 가까운 거리값 인덱스 추출
[imin, imin_index] = max(similarities);   %가장 코사인 유사도가 높은 값 인덱스 추출


%각 인덱스 값에 해당하는 벡터 추출
if dmin_index == 1
    closest_vectors = vectors(1:2, :);
elseif dmin_index == 2
    closest_vectors = vectors(2:3, :);
elseif dmin_index == 3
    closest_vectors = [vectors(1, :); vectors(3, :)];
end


if imin_index == 1
    most_similar_indices = vectors(1:2, :);
elseif imin_index == 2
    most_similar_indices = vectors(2:3, :);
elseif imin_index == 3
    most_similar_indices = [vectors(1, :); vectors(3, :)];
end


%벡터 출력
fprintf('유클라디안 거리로 보았을 때 가장 유사한 벡터:\n');
disp(closest_vectors);
fprintf('코사인 유사도로 보았을 때 가장 유사한 벡터:\n');
disp(most_similar_indices);
