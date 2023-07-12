clc; clear all;

[x, y] = findcenter("문제1.png")

function [centerX, centerY] = findcenter(input)
    
image = imread(input);
    th_down = 0.30;
    th_up = 0.3666;

    hsv_img = rgb2hsv(image);                     %rgb를 hsv로 변환
    h = hsv_img(:,:,1);
    s = hsv_img(:,:,2);
    binaryImage = (th_down<h)&(h<th_up)&(s>0.50); %이미지 초록색을 기준으로 이진화

    se = strel('disk', 3);  
    cleanedImage = imopen(binaryImage, se);       %노이즈 제거

    [B, ~] = bwboundaries(cleanedImage);          %이미지 경계선 따기

    x1 = 0;
    y1 = 0;
    x2 = 960;
    y2 = 720;

    for k = 1:length(B)
        boundary = B{k};
        minX = min(boundary(:, 2));
        maxX = max(boundary(:, 2));
        minY = min(boundary(:, 1));
        maxY = max(boundary(:, 1));
        if minX == x1 || maxX == x2
            continue
        elseif minY == y1 || maxY == y2
            continue                              %구멍이 아닌 경계선 제거
        
        else
            X = [minX, maxX];
            Y = [minY, maxY];
            centerX = mean(X);
            centerY = mean(Y);                    %구멍의 중심좌표 찾기
        end
    end
end