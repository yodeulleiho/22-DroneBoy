clear;
drone=ryze();
cam=camera(drone);
preview(cam);
idealX = 480;
idealY = 200;
past_red = 0;
current_red = 0;
move = 0;
a=0;
b=0;

%%%%1단계%%%%%%%%%%%%%%%%%%%%
takeoff(drone);
moveup(drone,'Distance', 0.3,'Speed',1);
%moveback(drone,'Distance', 0.5,'Speed',1);

while 1
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res_red = ((0.95<h)&(h<1.0))&((0.645<s)&(s<0.925));
    if sum(binary_res_red,'all') > 50 && sum(binary_res_red,'all') < 1000
        stats = regionprops('table',binary_res_red,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)    %가장 큰 영역 추출
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        if abs(idealX - centerX) < 70           %표식과 링의 중앙의 오차 계산 및 제어
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.3,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.3,'Speed',1);
        end
        if abs(idealY - centerY) < 50
            b=1;
        elseif idealY - centerY < 0 && b == 0
            movedown(drone,'distance',0.3,'Speed',1);
        elseif idealY - centerY > 0 && b == 0
            moveup(drone,'distance',0.3,'Speed',1);
        end
        if a==0 || b == 0
            continue
        end
    end

    past_red = current_red;
    current_red = sum(binary_res_red, 'all');
    if past_red > 0 & past_red - 50 > current_red  %표식의 픽셀 수 계산 및 회전
        turn(drone, deg2rad(90));
        break
    end
    if current_red >= 1700
        turn(drone, deg2rad(90));
        break
    end
    
    moveforward(drone,'Distance', 0.9,'Speed',1);
end

moveback(drone, 'Distance', 0.2, 'Speed', 1);

%%%2단계%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
past_red = 0;
current_red = 0;
a=0;
b=0;
move = 0;

while 1  %링 인식
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    subplot(2,1,2), subimage(binary_res);
    disp(sum(binary_res,'all'));                            
    fillimg = imfill(binary_res,'holes');
    %링 찾으면 탈출
    if sum(fillimg,'all') > 30000
        break
    end
    if move == 0
        moveleft(drone,'Distance', 0.3,'speed',1);
        move = 1;
    elseif move == 1
        moveleft(drone,'distance', 0.3,'speed',1);
        move = 2;
    elseif move == 2
        moveright(drone,'Distance',3,'Speed',1);
        movedown(drone,'distance',0.5,'Speed',1);
        move = 0;
    end
end


while 1  % 구멍찾기
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    binary_res_red = ((0.95<h)&(h<1.0))&((0.645<s)&(s<0.925));
    subplot(2,1,2), subimage(binary_res);
    
    %표식이 보이면 탈출
    if sum(binary_res_red,'all') > 50
        break   
    end
    
    %이미지 채우기
    fillimg = imfill(binary_res,'holes');
    result = fillimg - binary_res;
    disp(sum(result,'all'));
    %구멍이 보이면 탈출
    if sum(result,'all') > 20000
        break
    elseif sum(result,'all') < 20000
        stats = regionprops('table',binary_res,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        
        if abs(idealX - centerX) < 40
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.2,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.2,'Speed',1);
        end
        if abs(idealY - centerY) < 20
            b=1;
        elseif idealY - centerY < 0
            b=0;
            movedown(drone,'distance',0.2, 'Speed',1);
        elseif idealY - centerY > 0
            b=0;
            moveup(drone,'distance',0.2,'Speed',1);
        end
        if a==1 && b==1
            break
        end
    end
end

while 1 %중심찾기
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    binary_res_red = ((0.95<h)&(h<1.0))&((0.645<s)&(s<0.925));
    
    %표식이 보이면 탈출
    if sum(binary_res_red,'all') > 50
        break
    end
    %이미지 채우기
    fillimg = imfill(binary_res,'holes');
    result = fillimg - binary_res; 
    subplot(2,1,1), subimage(result);
    subplot(2,1,2), subimage(binary_res);
    stats = regionprops('table',result,'Centroid','MajorAxisLength','MinorAxisLength');
    for i = 1:size(stats)
        if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
            maxI=i;
            break;
        end
    end
    centerX = max(stats.Centroid(maxI,1));
    centerY = max(stats.Centroid(maxI,2));
    
    if abs(idealX - centerX) < 40
        a=1;
    elseif idealX - centerX < 0
        a=0;
        moveright(drone,'distance',0.2,'Speed',1);
    elseif idealX - centerX > 0
        a=0;
        moveleft(drone,'distance',0.2,'Speed',1);
    end
    if abs(idealY - centerY) < 20
        b=1;
    elseif idealY - centerY < 0
        b=0;
        movedown(drone,'distance',0.2,'Speed',1);
    elseif idealY - centerY > 0
        b=0;
        moveup(drone,'distance',0.2,'Speed',1);
    end
    
    if a==1 && b==1
        break
    end
end


while 1  %전진
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res_red = ((0.95<h)&(h<1.0))&((0.645<s)&(s<0.925));
    subplot(2,1,1), subimage(binary_res_red);
    disp(sum(binary_res_red,'all'));
    if sum(binary_res_red,'all') > 50 && sum(binary_res_red,'all') < 1000
        stats = regionprops('table',binary_res_red,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        disp(centerX);
        disp(centerY);
        if abs(idealX - centerX) < 40
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.2,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.2,'Speed',1);
        end
        if abs(idealY - centerY) < 20
            b=1;
        elseif idealY - centerY < 0 && b == 0
            movedown(drone,'distance',0.2,'Speed',1);
        elseif idealY - centerY > 0 && b == 0
            moveup(drone,'distance',0.2,'Speed',1);
        end
        if a==0 || b == 0
            continue
        end
    end

    past_red = current_red;
    current_red = sum(binary_res_red, 'all');
    if past_red > 0 & past_red - 50 > current_red 
        turn(drone, deg2rad(90));
        break
    end
    if current_red >= 2000
        turn(drone, deg2rad(90));
        break
    end
    
    moveforward(drone,'Distance', 0.5,'Speed',1);
end

moveback(drone, 'Distance', 0.2, 'Speed', 1);
%%%3단계%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
past_gre = 0;
current_gre = 0;
a=0;
b=0;
move = 0;

while 1  %링 인식
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    subplot(2,1,2), subimage(binary_res);
    disp(sum(binary_res,'all'));                            
    fillimg = imfill(binary_res,'holes');
    %링 찾으면 탈출
    if sum(fillimg,'all') > 30000
        break
    end
    if move == 0
        moveleft(drone,'Distance', 0.3,'speed',1);
        move = 1;
    elseif move == 1
        moveleft(drone,'distance', 0.3,'speed',1);
        move = 2;
    elseif move == 2
        moveright(drone,'Distance',3,'Speed',1);
        movedown(drone,'distance',0.5,'Speed',1);
        move = 0;
    end
end


while 1  % 구멍찾기
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    gre_h_min = 0.34; gre_h_max = 0.45; gre_s_min = 0.4; gre_s_max = 1;
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    binary_res_gre = ((0.32<h)&(h<0.47))&((0.4<s)&(s<1));
    subplot(2,1,2), subimage(binary_res);
    
    %표식이 보이면 탈출
    if sum(binary_res_gre,'all') > 50
        break   
    end
    
    %이미지 채우기
    fillimg = imfill(binary_res,'holes');
    result = fillimg - binary_res;
    disp(sum(result,'all'));
    %구멍이 보이면 탈출
    if sum(result,'all') > 20000
        break
    elseif sum(result,'all') < 20000
        stats = regionprops('table',binary_res,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        
        if abs(idealX - centerX) < 40
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.2,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.2,'Speed',1);
        end
        if abs(idealY - centerY) < 20
            b=1;
        elseif idealY - centerY < 0
            b=0;
            movedown(drone,'distance',0.2, 'Speed',1);
        elseif idealY - centerY > 0
            b=0;
            moveup(drone,'distance',0.2,'Speed',1);
        end
        if a==1 && b==1
            break
        end
    end
end

while 1 %중심찾기
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    binary_res_gre = ((0.95<h)&(h<1.0))&((0.645<s)&(s<0.925));
    
    %표식이 보이면 탈출
    if sum(binary_res_gre,'all') > 50
        break
    end
    %이미지 채우기
    fillimg = imfill(binary_res,'holes');
    result = fillimg - binary_res; 
    subplot(2,1,1), subimage(result);
    subplot(2,1,2), subimage(binary_res);
    stats = regionprops('table',result,'Centroid','MajorAxisLength','MinorAxisLength');
    for i = 1:size(stats)
        if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
            maxI=i;
            break;
        end
    end
    centerX = max(stats.Centroid(maxI,1));
    centerY = max(stats.Centroid(maxI,2));
    
    if abs(idealX - centerX) < 40
        a=1;
    elseif idealX - centerX < 0
        a=0;
        moveright(drone,'distance',0.2,'Speed',1);
    elseif idealX - centerX > 0
        a=0;
        moveleft(drone,'distance',0.2,'Speed',1);
    end
    if abs(idealY - centerY) < 20
        b=1;
    elseif idealY - centerY < 0
        b=0;
        movedown(drone,'distance',0.2,'Speed',1);
    elseif idealY - centerY > 0
        b=0;
        moveup(drone,'distance',0.2,'Speed',1);
    end
    
    if a==1 && b==1
        break
    end
end


while 1  %전진
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res_gre = ((0.95<h)&(h<1.0))&((0.645<s)&(s<0.925));
    subplot(2,1,1), subimage(binary_res_gre);
    disp(sum(binary_res_gre,'all'));
    if sum(binary_res_gre,'all') > 50 && sum(binary_res_gre,'all') < 1000
        stats = regionprops('table',binary_res_gre,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        disp(centerX);
        disp(centerY);
        if abs(idealX - centerX) < 40
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.2,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.2,'Speed',1);
        end
        if abs(idealY - centerY) < 20
            b=1;
        elseif idealY - centerY < 0 && b == 0
            movedown(drone,'distance',0.2,'Speed',1);
        elseif idealY - centerY > 0 && b == 0
            moveup(drone,'distance',0.2,'Speed',1);
        end
        if a==0 || b == 0
            continue
        end
    end

    past_gre = current_gre;
    current_gre = sum(binary_res_gre, 'all');
    if past_gre > 0 & past_gre - 50 > current_gre
        turn(drone, deg2rad(45));
        break
    end
    if current_gre >= 2000
        turn(drone, deg2rad(45));
        break
    end
    
    moveforward(drone,'Distance', 0.7,'Speed',1);
end




%%%%%%%%4단계%%%%%%%%%%%

past_pur = 0;
current_pur = 0;
a=0;
b=0;
move = 0;

while 1
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    subplot(2,1,2), subimage(binary_res);
    disp(sum(binary_res,'all'));
    fillimg = imfill(binary_res,'holes');
    %링 찾으면 탈출
    if sum(fillimg,'all') > 30000
        break
    end
    if move == 0
        moveleft(drone,'Distance',0.3,'speed',1);
        move = 1;
    elseif move == 1
        moveleft(drone,'distance',0.3,'speed',1);
        move = 2;
    elseif move == 2
        moveright(drone,'Distance',3,'Speed',1);
        movedown(drone,'distance',0.5,'Speed',1);
        move = 0;
    end
end


while 1
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    pur_h_min = 0.7; pur_h_max = 0.85; pur_s_min = 0.5; pur_s_max = 1;
    binary_res_pur = ((0.68<h)&(h<0.87))&((0.48<s)&(s<1));
    subplot(2,1,2), subimage(binary_res);
    
    %표식이 보이면 탈출
    if sum(binary_res_pur,'all') > 50
        break
    end
    
    %이미지 채우기
    fillimg = imfill(binary_res,'holes');
    result = fillimg - binary_res;
    disp(sum(result,'all'));
    %구멍이 보이면 탈출
    if sum(result,'all') > 20000
        break
    elseif sum(result,'all') < 20000
        stats = regionprops('table',binary_res,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        
        if abs(idealX - centerX) < 40
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.3,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.3,'Speed',1);
        end
        if abs(idealY - centerY) < 20
            b=1;
        elseif idealY - centerY < 0
            b=0;
            movedown(drone,'distance',0.3,'Speed',1);
        elseif idealY - centerY > 0
            b=0;
            moveup(drone,'distance',0.3,'Speed',1);
        end
        if a==1 && b==1
            break
        end
    end
end


while 1
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res = ((0.615<h)&(h<0.685))&((0.43<s)&(s<0.85));
    binary_res_pur = ((0.68<h)&(h<0.87))&((0.48<s)&(s<1));
    
    %표식이 보이면 탈출
    if sum(binary_res_pur,'all') > 50
        break
    end
    %이미지 채우기
    fillimg = imfill(binary_res,'holes');
    result = fillimg - binary_res; 
    subplot(2,1,1), subimage(result);
    subplot(2,1,2), subimage(binary_res);
    stats = regionprops('table',result,'Centroid','MajorAxisLength','MinorAxisLength');
    for i = 1:size(stats)
        if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
            maxI=i;
            break;
        end
    end
    centerX = max(stats.Centroid(maxI,1));
    centerY = max(stats.Centroid(maxI,2));
    
    if abs(idealX - centerX) < 40
        a=1;
    elseif idealX - centerX < 0
        a=0;
        moveright(drone,'distance',0.3,'Speed',1);
    elseif idealX - centerX > 0
        a=0;
        moveleft(drone,'distance',0.3,'Speed',1);
    end
    if abs(idealY - centerY) < 20
        b=1;
    elseif idealY - centerY < 0
        b=0;
        movedown(drone,'distance',0.3,'Speed',1);
    elseif idealY - centerY > 0
        b=0;
        moveup(drone,'distance',0.3,'Speed',1);
    end
    
    if a==1 && b==1
        break
    end
end


while 1
    frame =snapshot(cam);
    hsv = rgb2hsv(frame);
    h = hsv(:,:,1);
    s = hsv(:,:,2);
    binary_res_pur = ((0.68<h)&(h<0.87))&((0.48<s)&(s<1));
    subplot(2,1,1), subimage(binary_res_pur);
    disp(sum(binary_res_pur,'all'));
    if sum(binary_res_pur,'all') > 50 && sum(binary_res_pur,'all') < 1000
        stats = regionprops('table',binary_res_pur,'Centroid','MajorAxisLength','MinorAxisLength');
        for i = 1:size(stats)
            if stats.MajorAxisLength(i)==max(stats.MajorAxisLength)
                maxI=i;
                break;
            end
        end
        centerX = max(stats.Centroid(maxI,1));
        centerY = max(stats.Centroid(maxI,2));
        disp(centerX);
        disp(centerY);
        if abs(idealX - centerX) < 40
            a=1;
        elseif idealX - centerX < 0
            a=0;
            moveright(drone,'distance',0.2,'Speed',1);
        elseif idealX - centerX > 0
            a=0;
            moveleft(drone,'distance',0.2,'Speed',1);
        end
        if abs(idealY - centerY) < 20
            b=1;
        elseif idealY - centerY < 0 && b == 0
            movedown(drone,'distance',0.2,'Speed',1);
        elseif idealY - centerY > 0 && b == 0
            moveup(drone,'distance',0.2,'Speed',1);
        end
        if a==0 || b == 0
            continue
        end
    end

    past_pur = current_pur;
    current_pur = sum(binary_res_pur, 'all');
    %if past_pur > 0 & past_pur - 50 > current_pur 
        %land(drone);
        %break
    %end
    if current_pur >= 1000
        land(drone);
        break
    end
    
    moveforward(drone,'Distance', 0.4,'Speed',1);
end