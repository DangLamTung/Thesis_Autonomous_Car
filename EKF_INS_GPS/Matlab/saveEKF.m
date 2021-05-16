function saveEKF(nStates)
%% Load Data
fileName = strcat('SymbolicOutput',int2str(nStates),'.mat');
load(fileName);

%% Open output file
fileName = strcat('SymbolicOutput',int2str(nStates),'.txt');
fid = fopen(fileName,'wt');

%% Write equation for state transition matrix
if exist('SF','var')
    
    fprintf(fid,'SF = zeros(%d,1);\n',numel(SF));
    for rowIndex = 1:numel(SF)
        string = char(SF(rowIndex,1));
        fprintf(fid,'SF(%d) = %s;\n',rowIndex,string);
    end
    
    % fprintf(fid,'\n');
    % fprintf(fid,'F = zeros(%d,%d);\n',nStates,nStates);
    % for rowIndex = 1:nStates
    %     for colIndex = 1:nStates
    %         string = char(F(rowIndex,colIndex));
    %         % don't write out a zero-assignment
    %         if ~strcmpi(string,'0')
    %             fprintf(fid,'F(%d,%d) = %s;\n',rowIndex,colIndex,string);
    %         end
    %     end
    % end
    % fprintf(fid,'\n');
    
end
%% Write equations for control influence (disturbance) matrix
if exist('SG','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SG = zeros(%d,1);\n',numel(SG));
    for rowIndex = 1:numel(SG)
        string = char(SG(rowIndex,1));
        fprintf(fid,'SG(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    % fprintf(fid,'\n');
    % fprintf(fid,'G = zeros(%d,%d);\n',nStates,numel([da;dv]));
    % for rowIndex = 1:nStates
    %     for colIndex = 1:numel([da;dv])
    %         string = char(G(rowIndex,colIndex));
    %         % don't write out a zero-assignment
    %         if ~strcmpi(string,'0')
    %             fprintf(fid,'G(%d,%d) = %s;\n',rowIndex,colIndex,string);
    %         end
    %     end
    % end
    % fprintf(fid,'\n');
    
end
%% Write equations for state error matrix
if exist('SQ','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SQ = zeros(%d,1);\n',numel(SQ));
    for rowIndex = 1:numel(SQ)
        string = char(SQ(rowIndex,1));
        fprintf(fid,'SQ(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
    % fprintf(fid,'\n');
    % fprintf(fid,'Q = zeros(%d,%d);\n',nStates,nStates);
    % for rowIndex = 1:nStates
    %     for colIndex = 1:nStates
    %         string = char(Q(rowIndex,colIndex));
    %         % don't write out a zero-assignment
    %         if ~strcmpi(string,'0')
    %             fprintf(fid,'Q(%d,%d) = %s;\n',rowIndex,colIndex,string);
    %         end
    %     end
    % end
    % fprintf(fid,'\n');
    
end
%% Write equations for covariance prediction
% Only write out upper diagonal (matrix is symmetric)
if exist('SPP','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'SPP = zeros(%d,1);\n',numel(SPP));
    for rowIndex = 1:numel(SPP)
        string = char(SPP(rowIndex,1));
        fprintf(fid,'SPP(%d) = %s;\n',rowIndex,string);
    end
    fprintf(fid,'\n');
    
end

if exist('PP','var')
    
    fprintf(fid,'\n');
    fprintf(fid,'P = zeros(%d,%d);\n',nStates,nStates);
    for colIndex = 1:nStates
        for rowIndex = 1:colIndex
            string = char(PP(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'P(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
end
%%
if exist('x','var')
    [nRow,nCol] = size(x);
    fprintf(fid,'\n');
%     fprintf(fid,'x = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(x(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'x(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
end
%% Write equations for velocity and position data fusion
if exist('H_a','var')
    [nRow,nCol] = size(Ha_b);
    fprintf(fid,'\n');
    fprintf(fid,'Ha_b = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(Ha_b(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'Ha_b(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    [nRow,nCol] = size(H_a);
    fprintf(fid,'\n');
    fprintf(fid,'H_a = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_a(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_a(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    
    
end
%% magnet
if exist('H_m','var')
    [nRow,nCol] = size(Hm_b);
    fprintf(fid,'\n');
    fprintf(fid,'Hm_b = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(Hm_b(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'Hm_b(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    
    [nRow,nCol] = size(H_m);
    fprintf(fid,'\n');
    fprintf(fid,'H_m = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(H_m(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'H_a(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    

    fprintf(fid,'\n');
    
    
    %% K
      [nRow,nCol] = size(SK);
    fprintf(fid,'\n');
    fprintf(fid,'SK = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(SK(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'SK(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end

    fprintf(fid,'\n');
    [nRow,nCol] = size(K);
    fprintf(fid,'\n');
    fprintf(fid,'K = zeros(%d,%d);\n',nRow,nCol);
    for rowIndex = 1:nRow
        for colIndex = 1:nCol
            string = char(K(rowIndex,colIndex));
            % don't write out a zero-assignment
            if ~strcmpi(string,'0')
                fprintf(fid,'K(%d,%d) = %s;\n',rowIndex,colIndex,string);
            end
        end
    end
    fprintf(fid,'\n');
    


%% Close output file
fclose(fid);

end
