% Camera Parameters
% Camera 1
R1 = [9.6428667991264605e-01 -2.6484969138677328e-01 -2.4165916859785336e-03;
    -8.9795446022112396e-02 -3.1832382771611223e-01 -9.4371961862719200e-01;
    2.4917459103354755e-01 9.1023325674273947e-01 -3.3073772313234923e-01];
t1 = [1.3305621037591506e-01; -2.5319578738559911e-01; 2.2444637695699150e+00];
calibration_matrix_1 = [8.7014531487461625e+02 0 9.4942001822880479e+02;
    0 8.7014531487461625e+02 4.8720049852775117e+02;
    0 0 1];
i1 = R1(1, :);
j1 = R1(2, :);
k1 = R1(3, :);
camPose1 = -inv(R1) * t1;

% Camera 2
R2 = [9.4962278945631540e-01 3.1338395965783683e-01 -2.6554800661627576e-03;
    1.1546856489995427e-01 -3.5774736713426591e-01 -9.2665194751235791e-01;
    -2.9134784753821596e-01 8.7966318277945221e-01 -3.7591104878304971e-01];
t2 = [-4.2633372670025989e-02; -3.5441906393933242e-01; 2.2750378317324982e+00];
calibration_matrix_2 = [8.9334367240024267e+02 0 9.4996816131377727e+02;
    0 8.9334367240024267e+02 5.4679562177577259e+02;
    0 0 1];
i2 = R2(1, :);
j2 = R2(2, :);
k2 = R2(3, :);
camPose1 = -inv(R2) * t2;

% Camera 3
R3 = [-9.9541881789113029e-01 3.8473906154401757e-02 -8.7527912881817604e-02;
    9.1201836523849486e-02 6.5687400820094410e-01 -7.4846426926387233e-01;
    2.8698466908561492e-02 -7.5301812454631367e-01 -6.5737363964632056e-01];
t3 = [-6.0451734755080713e-02; -3.9533167111966377e-01; 2.2979640654841407e+00];
calibration_matrix_3 = [8.7290852997159800e+02 0 9.4445161471037636e+02;
    0 8.7290852997159800e+02 5.6447334036925656e+02;
    0 0 1];
i3 = R3(1, :);
j3 = R3(2, :);
k3 = R3(3, :);
camPose3 = -inv(R3) * t3;

filelist = readtable('FileList.csv', 'ReadVariableNames', false);
filelist = filelist.Variables;

rows =  size(filelist(:,1));
rows = rows(1);

for a = 1: rows % change this to rows later
    
    file1 = get_files(filelist, a, 1);
    file2 = get_files(filelist, a, 2);
    file3 = get_files(filelist, a, 3);
    filerows =  size(file1(:,1));
    filerows = filerows(1);
    
    validrows = 0;
    for b = 2: filerows
        u1 = file1(b, 2);
        u2 = file2(b, 2);
        u3 = file3(b, 2);
        if(u1{1} ~= "" && u2{1} ~= "" && u3{1} ~= "")
            validrows = validrows + 1;
        end
    end
    
    coords = zeros(validrows, 3);
    
    for c = 2: filerows
        u1 = file1(c, 2);
        u2 = file2(c, 2);
        u3 = file3(c, 2);
        
        v1 = file1(c, 3);
        v2 = file2(c, 3);
        v3 = file3(c, 3);
        
        if(u1{1} ~= "" && u2{1} ~= "" && u3{1} ~= "")
            u1 = str2double(u1{1});
            u2 = str2double(u2{1});
            u3 = str2double(u3{1});
            v1 = str2double(v1{1});
            v2 = str2double(v2{1});
            v3 = str2double(v3{1});
            
            A = [(u1-calibration_matrix_1(1,3))*k1(1)-1920*i1(1) (u1-calibration_matrix_1(1,3))*k1(2)-1920*i1(2) (u1-calibration_matrix_1(1,3))*k1(3)-1920*i1(3);
                (u2-calibration_matrix_2(1,3))*k2(1)-1920*i2(1) (u2-calibration_matrix_2(1,3))*k2(2)-1920*i2(2) (u2-calibration_matrix_2(1,3))*k2(3)-1920*i2(3);
                (u3-calibration_matrix_3(1,3))*k3(1)-1920*i3(1) (u3-calibration_matrix_3(1,3))*k3(2)-1920*i3(2) (u3-calibration_matrix_3(1,3))*k3(3)-1920*i3(3);
                (v1-calibration_matrix_1(2,3))*k1(1)-1080*i1(1) (v1-calibration_matrix_1(2,3))*k1(2)-1080*i1(2) (v1-calibration_matrix_1(2,3))*k1(3)-1080*i1(3);
                (v2-calibration_matrix_2(2,3))*k2(1)-1080*i2(1) (v2-calibration_matrix_2(2,3))*k2(2)-1080*i2(2) (v2-calibration_matrix_2(2,3))*k2(3)-1080*i2(3);
                (v3-calibration_matrix_3(2,3))*k3(1)-1080*i3(1) (v3-calibration_matrix_3(2,3))*k3(2)-1080*i3(2) (v3-calibration_matrix_3(2,3))*k3(3)-1080*i3(3)];
            
            B = [(u1-calibration_matrix_1(1,3))*dot(t1, k1)-dot(t1,i1)*1920; (u2-calibration_matrix_2(1,3))*dot(t2, k2)-dot(t2,i2)*1920; (u3-calibration_matrix_3(1,3))*dot(t3, k3)-dot(t3,i3)*1920;
                (v1-calibration_matrix_1(2,3))*dot(t1, k1)-dot(t1,j1)*1080; (v2-calibration_matrix_2(2,3))*dot(t2, k2)-dot(t2,j2)*1080; (v3-calibration_matrix_3(2,3))*dot(t3, k3)-dot(t3,j3)*1080];

            x = inv(transpose(A) * A)*transpose(A)*B;
            coords(c-1, :) = x;
        end
    end
    %pcshow(coords, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 50);
    filename = strcat('Trajectories\', int2str(a));
    filename = strcat(filename, '.csv');
    csvwrite(filename, coords);
end

function file = get_files(filelist, row, col)

filename = filelist(row, col);
filename = filename{1};
baseName = filename(1:find(filename=='.')-1);
baseName = strcat(baseName, '.csv');
fullfileName = fullfile('C:\Users\Halo3\OneDrive\CS4243\Matlab\Project\CS4243\Annotation\Annotation', baseName); % Change to the folder containing the Annotation files
file = readtable(fullfileName, 'ReadVariableNames', false);
file = file.Variables;

end