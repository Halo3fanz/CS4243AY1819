% Camera Parameters
% Camera 1
R1 = [9.6428667991264605e-01 -2.6484969138677328e-01 -2.4165916859785336e-03;
    -8.9795446022112396e-02 -3.1832382771611223e-01 -9.4371961862719200e-01;
    2.4917459103354755e-01 9.1023325674273947e-01 -3.3073772313234923e-01];
t1 = [1.3305621037591506e-01; -2.5319578738559911e-01; 2.2444637695699150e+00];
calibration_matrix_1 = [8.7014531487461625e+02 0 9.4942001822880479e+02;
    0 8.7014531487461625e+02 4.8720049852775117e+02;
    0 0 1];
temp = [R1(1,1) R1(1,2), t1(1); R1(2, 1) R1(2, 2) t1(2); R1(3, 1) R1(3, 2) t1(3)];
H_1 = dot(calibration_matrix_1, temp)
focal_length_u_1 = 8.7014531487461625e+02;
focal_length_v_1 = 8.7014531487461625e+02;
u0_1 = 9.4942001822880479e+02;
v0_1 = 4.8720049852775117e+02;
% Camera 2
R2 = [9.4962278945631540e-01 3.1338395965783683e-01 -2.6554800661627576e-03;
    1.1546856489995427e-01 -3.5774736713426591e-01 -9.2665194751235791e-01;
    -2.9134784753821596e-01 8.7966318277945221e-01 -3.7591104878304971e-01];
t2 = [-4.2633372670025989e-02; -3.5441906393933242e-01; 2.2750378317324982e+00];
calibration_matrix_2 = [8.9334367240024267e+02 0 9.4996816131377727e+02;
    0 8.9334367240024267e+02 5.4679562177577259e+02;
    0 0 1];
focal_length_u_2 = 8.9334367240024267e+02;
focal_length_v_2 = 8.9334367240024267e+02;
u0_2 = 9.4996816131377727e+02;
v0_2 = 5.4679562177577259e+02;
% Camera 3
R3 = [-9.9541881789113029e-01 3.8473906154401757e-02 -8.7527912881817604e-02;
    9.1201836523849486e-02 6.5687400820094410e-01 -7.4846426926387233e-01;
    2.8698466908561492e-02 -7.5301812454631367e-01 -6.5737363964632056e-01];
t3 = [-6.0451734755080713e-02; -3.9533167111966377e-01; 2.2979640654841407e+00];
calibration_matrix_3 = [8.7290852997159800e+02 0 9.4445161471037636e+02;
    0 8.7290852997159800e+02 5.6447334036925656e+02;
    0 0 1];
focal_length_u_3 = 8.7290852997159800e+02;
focal_length_v_3 = 8.7290852997159800e+02;
u0_3 = 9.4445161471037636e+02;
v0_3 = 5.6447334036925656e+02; 

filelist = readtable('FileList.csv', 'ReadVariableNames', false);
filelist = filelist.Variables;

rows =  size(filelist(:,1));
rows = rows(1);

u_matrix = [];
v_matrix = [];

for a = 1: 2 % change this to rows later
    for b = 1: 3
        filename = filelist(a, b);
        filename = filename{1};
        baseName = filename(1:find(filename=='.')-1);
        baseName = strcat(baseName, '.csv');
        fullfileName = fullfile('C:\Users\Halo3\OneDrive\CS4243\Matlab\Project\CS4243\Annotation\Annotation', baseName); % Change to the folder containing the Annotation files
        file = readtable(fullfileName, 'ReadVariableNames', false);
        file = file.Variables;
        filerows =  size(file(:,1));
        filerows = filerows(1);
        
        for c = 2: filerows
            u = file(c, 2);
            if(u{1} == "")
                continue
            end
            u = str2num(u{1});
            u_matrix(a, c-1) = u;
            
            v = file(c, 3);
            v = str2num(v{1});
            
            p = [u v 1];
            projection = dot(H_1, p);
        end
        
    end
    
end