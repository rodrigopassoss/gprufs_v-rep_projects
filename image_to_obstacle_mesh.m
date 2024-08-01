clc 
clear all
close all

%image_to_obstacle('mapa_com_obstaculos.png','modelo_dos_obstáculos.obj',0.1,2)
image_to_obstacle('g1092.png','mapa01.obj',0.1,2)



function image_to_obstacle(image_path, output_mesh_path, pixel_to_meter_ratio, obstacle_height)
    if nargin < 4
        obstacle_height = 1.0;
    end
    if nargin < 3
        pixel_to_meter_ratio = 1.0;
    end

    % Carregar a imagem binária
    I_r = imread(image_path);
    Igray = convert_cinza(I_r);
    binary_image = ~convert_bin2(Igray,otsu(Igray));
    [ISeg,Reg,pontos]=segmentador(binary_image,55);
    imshow(ISeg)
    % Filtragem e obtenção dos cantos com o algoritmo de Douglas Peucker
    [cantos] = filtragem(Reg,pontos,0.3);
    hold on

    % Inicializar as listas de vértices e faces
    vertices = [];
    faces = [];
    
    for i=1:length(Reg)
         xc = mean(cantos(i).x);
         yc = mean(cantos(i).y);
         cantos2_x = 0.95*(cantos(i).x-xc) + xc;
         cantos2_y = 0.95*(cantos(i).y-yc) + yc;
         %%--Degug
         plot(cantos(i).x,cantos(i).y,'.r','MarkerSize',20)
         plot(cantos2_x,cantos2_y,'.g','MarkerSize',20)
         % Adequação
         cantos1_x = [cantos(i).x cantos(i).x(1)].*pixel_to_meter_ratio;
         cantos1_y = [cantos(i).y cantos(i).y(1)].*pixel_to_meter_ratio;
         cantos2_x = [cantos2_x cantos2_x(1)].*pixel_to_meter_ratio;
         cantos2_y = [cantos2_y cantos2_y(1)].*pixel_to_meter_ratio;
         for j = 1:(numel(cantos1_x)-1)
             v1 = [cantos1_x(j), cantos1_y(j),0];
             v2 = [cantos2_x(j), cantos2_y(j),0];
             v3 = [cantos2_x(j+1), cantos2_y(j+1),0];
             v4 = [cantos1_x(j+1), cantos1_y(j+1),0];
             v5 = [cantos1_x(j), cantos1_y(j),obstacle_height];
             v6 = [cantos2_x(j), cantos2_y(j),obstacle_height];
             v7 = [cantos2_x(j+1), cantos2_y(j+1),obstacle_height];
             v8 = [cantos1_x(j+1), cantos1_y(j+1),obstacle_height];
             vertices = [vertices; v1; v2; v3; v4; v5; v6; v7; v8];
             idx = size(vertices, 1) - 7;
             faces = [faces;
                     idx, idx+1, idx+5, idx+4; % Face lateral 1
                     idx+1, idx+2, idx+6, idx+5; % Face lateral 2
                     idx+2, idx+3, idx+7, idx+6; % Face lateral 3
                     idx+3, idx, idx+4, idx+7; % Face lateral 4
                     idx+4, idx+5, idx+6, idx+7; % Face superior
                     idx, idx+1, idx+2, idx+3]; % Face inferior
         end
    end

%     % Remover vértices duplicados e atualizar as faces
%     [vertices, ~, indexn] = unique(vertices, 'rows');
%     faces = indexn(faces);

    % Salvar a malha em um arquivo OBJ
    fileID = fopen(output_mesh_path, 'w');
    fprintf(fileID, 'v %.4f %.4f %.4f\n', vertices');
    fprintf(fileID, 'f %d %d %d %d\n', faces');
    fclose(fileID);
end

% Segmentador
function [ImSeg,Reg,ordem]=segmentador(im,limiar)

    f = double(im);
    f = [0.*f(1,:); f]; f = [0.*f(:,1) f];
    f = [f; 0.*f(1,:)]; f = [f 0.*f(:,1)];
    [M,N] = size(f);
    LNBD = 0; 
    Reg = 2;
    directions = [0 -1;-1 -1;-1 0;-1 1; 0 1; 1 1; 1 0; 1 -1];
    angs = [180;225;270;315;0;45;90;135];
    i1=0;j1=0;i2=0;j2=0;
    i3=0;j3=0;i4=0;j4=0;
    
    for i = 1:M
        for j = 1:N
            x = []; y = [];
            if f(i,j)==1 && f(i,j-1)==0
                i2 = i; j2 = j-1; K = round(atan2d(i2-i,j2-j))-45;
                for k = 1:size(directions,1)
                    K = convertTo360(K+45);
                    i1 = i+directions(find(angs==K),1); 
                    j1 = j+directions(find(angs==K),2);
                    if (i1>0) && (j1>0) && (i1 <= M) && (j1 <= N)
                        if f(i1,j1)~=0
                            i2 = i1; j2 = j1;
                            i3 = i; j3 = j;
                            currDir = round(atan2d(i2-i3,j2-j3));
%                             display('Entrou no While')
                            while true
                                for l = 1:size(directions,1)
                                    currDir = convertTo360(currDir-45);
                                    I = find(angs==currDir);
                                    i4 = i3+directions(I,1); 
                                    j4 = j3+directions(I,2);
                                    if (i4>0) && (j4>0) && (i4<=M) && (j4<=N)
                                        if f(i4,j4)~=0
                                            if(f(i3,j3+1)==0)
                                                f(i3,j3) = -Reg;
                                                x = [x j3-1]; y = [y i3-1]; 
                                            elseif (f(i3,j3+1)~=0)&&(f(i3,j3)==1)
                                                f(i3,j3) = Reg;
                                                x = [x j3-1]; y = [y i3-1];
                                            end
                                            break;
                                        end
                                    end
                                end
                                if (i4==i) && (j4==j) && (i3 == i1) && (j3 == j1)
                                    break;
                                else
                                    i2 = i3; j2 = j3;
                                    i3 = i4; j3 = j4;
                                    currDir = round(atan2d(i2-i3,j2-j3));
                                    LNBD = f(i3,j3);
                                end
                            end
%                             display('Saiu do While')
                            ordem(Reg-1) = struct('Reg',Reg,'x',x,'y',y);
                            LNBD = 0; Reg = Reg + 1;

                            break;
                        end
                    end
                end
            end
        end
    end
    ImSeg = abs(f);
    ImSeg = ImSeg(2:end-1,2:end-1);
    ImSeg(ImSeg==1)=0;
    
    Reg = unique(ImSeg(:));
    for r = 1:length(Reg)
      n = sum(ImSeg(:)==Reg(r));
      if(n < limiar)
        ImSeg(ImSeg==Reg(r)) = 0;
      end
    end

%     Reg = sort(unique(ImSeg(:)));
%     for r = 1:length(Reg)
%       ImSeg(ImSeg==Reg(r)) = r-1;
%     end
    Reg = sort(unique(ImSeg(:)));
    Reg = Reg(2:end);      
end


function [cantos] = filtragem(Reg,pontos,limiar);
    cnt = 0;
    for i = 1:length(Reg)
        points = [pontos(Reg(i)-1).x' pontos(Reg(i)-1).y'];
        simplifiedPoints = DouglasPeucker(points,2);
%         if size(simplifiedPoints,1)<4 continue; end
%         simplifiedPoints = DouglasPeuckerPosteriori(simplifiedPoints,limiar);
        if size(simplifiedPoints,1)<4
            continue; 
        else
            cnt = cnt + 1;
            cantos(cnt) = struct('x',simplifiedPoints(:,1)','y',simplifiedPoints(:,2)');
        end
    end
    if cnt==0, cantos=[]; end

end

%% Simplificação
function simplifiedPoints = DouglasPeucker(points, epsilon)
    % Douglas-Peucker algorithm for simplifying a polyline
    if size(points, 1) < 3
        simplifiedPoints = points;
        return;
    end

    [index, d] = perpendicularDistance(points, points(1, :), points(end, :));
    dmax = d(index);
    if dmax > epsilon
        left = DouglasPeucker(points(1:index, :), epsilon);
        right = DouglasPeucker(points(index:end, :), epsilon);
        simplifiedPoints = [left(1:end-1, :); right];
    else
        simplifiedPoints = [points(1, :); points(end, :)];
    end
end

function [index, distance] = perpendicularDistance(points, init, fim)
    % Calculate perpendicular distance from each point to the line formed by init and end
    distance = zeros(size(points, 1), 1);

    for i = 1:size(points, 1)
        distance(i) = abs((fim(2) - init(2)) * points(i, 1) - (fim(1) - init(1)) * points(i, 2) + fim(1) * init(2) - fim(2) * init(1)) / norm(fim - init);
    end

    % Find the point with the maximum distance
    [~, index] = max(distance);
%     perpendicular = points(index, :);
end

function simplifiedPoints = DouglasPeuckerPosteriori(points,limiar);
    dpoints = points(2:end,1:2)-points(1:end-1,1:2);
    dists = sqrt(dpoints(:,1).^2 + dpoints(:,2).^2);
    dmax = max(dists);
    simplifiedPoints = points(1,:);
    recyclePoints = [];
    for i = 2:size(points,1)
        if size(simplifiedPoints,1)<4
            dist = min(sqrt((simplifiedPoints(:,1)-(points(i,1))).^2 + (simplifiedPoints(:,2)-(points(i,2))).^2));
            dist_norm = dist/dmax;        
            if dist_norm > limiar
                simplifiedPoints = [simplifiedPoints; points(i,:)];  
            else
                if i==2
                    recyclePoints = [recyclePoints; points(i,:)];
                else
                    dist1 = min(sqrt((simplifiedPoints(1:end-1,1)-(points(i,1))).^2 ...
                         + (simplifiedPoints(1:end-1,2)-(points(i,2))).^2));
                    dist2 = min(sqrt((simplifiedPoints(1:end-1,1)-(simplifiedPoints(end,1))).^2 ...
                          + (simplifiedPoints(1:end-1,2)-(simplifiedPoints(end,2))).^2));
                    if dist1 > dist2
                        simplifiedPoints(end,:) = points(i,:);
                    end
                end
            end
        else 
            dist1 = min(sqrt((simplifiedPoints(2:end,1)-(points(i,1))).^2 ...
                 + (simplifiedPoints(2:end,2)-(points(i,2))).^2));
            dist2 = min(sqrt((simplifiedPoints(2:end,1)-(simplifiedPoints(1,1))).^2 ...
                  + (simplifiedPoints(2:end,2)-(simplifiedPoints(1,2))).^2));
            if dist1 > dist2
                simplifiedPoints(1,:) = points(i,:);
            end
        end
    end 
    for j = 1:size(recyclePoints,1)
        dist1 = min(sqrt((simplifiedPoints(2:end,1)-(recyclePoints(j,1))).^2 ...
             + (simplifiedPoints(2:end,2)-(recyclePoints(j,2))).^2));
        dist2 = min(sqrt((simplifiedPoints(2:end,1)-(simplifiedPoints(1,1))).^2 ...
              + (simplifiedPoints(2:end,2)-(simplifiedPoints(1,2))).^2));
        if dist1 > dist2
            simplifiedPoints(1,:) = recyclePoints(j,:);
        end
    end
end

% Conversão em tons de cinza
function imgray = convert_cinza(im);
% Insira aqui o seu código
  [~,~,Z] = size(im);
  if Z>1
      y_coefs = [0.299 0.587 0.114];
      im_double = double(im);
      imgray = uint8(y_coefs(1)*im_double(:,:,1) + ...
                     y_coefs(2)*im_double(:,:,2) + ...
                     y_coefs(3)*im_double(:,:,3));
  else
      imgray = im;
  end
end
% Conversão em preto e branco
function imbin = convert_bin(im,window,c);
    im = double(im);
    [lin,col]=size(im);

    mask = ones(window,window)/(window^2);

    i = 1:lin;
    j = 1:col;

    % Calcule o tamanho da saída
    M = zeros(lin,col);
    k = linspace(-floor(window/2),floor(window/2),window);
    for i_ = 1:window
        for j_ = 1:window
            I = i + k(i_); idc_i = I>0 & I<=lin;
            J = j + k(j_); idc_j = J>0 & J<=col;
            M(i(idc_i),j(idc_j)) = M(i(idc_i),j(idc_j)) + im(I(idc_i),J(idc_j))*mask(i_,j_);
        end
    end
      
    %     filtro = fspecial('gaussian', window, 2);
    %     M2 = imfilter(im, filtro, 'replicate');
    
    imbin = true(size(im));
    imbin(im>(M-c)) = false;

end
% Binarização por limiar global
function imbin = convert_bin2(im,limiar);
    imbin = false(size(im));
    imbin(im>limiar) = true;
end
% Limiar de Otsu
function limiar = otsu(imgray);
    Pr = histg(imgray);
    CumPr = cumsum(Pr);
    i = [0:255];
    mg = sum(i.*Pr);
    for k = 1:256
        mk = sum(i(1:k).*Pr(1:k));
        sigma(k) = ((mg*CumPr(k)-mk)^2)/(CumPr(k)*(1-CumPr(k)));
    end
    [~,K]=max(sigma);
    limiar = K-1;
%     #limiar_groundTruth = graythresh(imgray)*255
end
% Histograma usado no Limiar de Otsu
function [hist] = histg(im);
    [lin,col]=size(im);
    for k = 1:256
       hist(k) = numel(find(im==(k-1)));
    end
    hist = hist/(lin*col);
end

% Para a segmentação
function angle360 = convertTo360(angle)
    % Usa a função mod para garantir que o ângulo esteja no intervalo de 0 a 360 graus
    angle360 = mod(angle, 360);

    % Certifica-se de que o resultado seja positivo
    if angle360 < 0
        angle360 = angle360 + 360;
    end
end
