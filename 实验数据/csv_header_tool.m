function csv_header_tool
% CSV header normalization tool for experiment data.
% Open this file in MATLAB and press F5 (Run).

clc;

scriptDir = fileparts(mfilename('fullpath'));
fprintf('CSV header tool\n');
fprintf('Script location: %s\n\n', scriptDir);
fprintf('Supported suffix mapping:\n');
fprintf('  D5  -> omega_ref / omega_pll / Iq_ref / Iq_meas\n');
fprintf('  D7  -> raw21 / omega_pll / Iq_ref / Iq_meas\n');
fprintf('  D8  -> omega_pll / Iq_ref / Iq_comp / Iq_cmd\n');
fprintf('  D9  -> i_low_window_a_ticks / i_low_window_b_ticks / i_low_window_c_ticks / i_valid_mask\n');
fprintf('  D10  -> i_pair_active / i_pair_next / i_pair_valid / svm_sector\n');
fprintf('  D11 -> theta_e_meas / theta_e_ctrl / delta_theta_deg / u_mag_pu\n');
fprintf('  D12 -> ud_pu / uq_pu / u_mag_pu / delta_theta_deg\n\n');

modeChoice = menu( ...
    'Choose processing mode', ...
    'Batch: all CSV files in one folder', ...
    'Single: one CSV file only', ...
    'Cancel');

if modeChoice == 0 || modeChoice == 3
    fprintf('Canceled.\n');
    return;
end

backupChoice = menu( ...
    'Create .bak backup before overwrite?', ...
    'Yes (recommended)', ...
    'No', ...
    'Cancel');

if backupChoice == 0 || backupChoice == 3
    fprintf('Canceled.\n');
    return;
end

createBackup = backupChoice == 1;

switch modeChoice
    case 1
        targetDir = uigetdir(scriptDir, 'Select the folder to process');
        if isequal(targetDir, 0)
            fprintf('No folder selected. Canceled.\n');
            return;
        end

        recursiveChoice = menu( ...
            'Include subfolders?', ...
            'Current folder only', ...
            'Include subfolders', ...
            'Cancel');

        if recursiveChoice == 0 || recursiveChoice == 3
            fprintf('Canceled.\n');
            return;
        end

        includeSubfolders = recursiveChoice == 2;
        fileList = collectCsvFiles(targetDir, includeSubfolders);

    case 2
        targetDir = uigetdir(scriptDir, 'Select the folder containing the CSV');
        if isequal(targetDir, 0)
            fprintf('No folder selected. Canceled.\n');
            return;
        end

        [fileName, filePath] = uigetfile(fullfile(targetDir, '*.csv'), 'Select one CSV file');
        if isequal(fileName, 0)
            fprintf('No file selected. Canceled.\n');
            return;
        end

        fileList = {fullfile(filePath, fileName)};

    otherwise
        fprintf('Canceled.\n');
        return;
end

if isempty(fileList)
    fprintf('No CSV files found.\n');
    return;
end

fprintf('Files to process: %d\n', numel(fileList));
confirmText = sprintf('About to process %d CSV file(s). Continue?', numel(fileList));
confirmChoice = questdlg(confirmText, 'Confirm', 'Continue', 'Cancel', 'Continue');
if ~strcmp(confirmChoice, 'Continue')
    fprintf('Canceled.\n');
    return;
end

updated = {};
unchanged = {};
skipped = {};
failed = {};

for idx = 1:numel(fileList)
    filePath = fileList{idx};
    fprintf('\n[%d/%d] %s\n', idx, numel(fileList), filePath);

    [status, message] = processSingleCsv(filePath, createBackup);
    fprintf('  %s\n', message);

    switch status
        case 'updated'
            updated{end + 1} = filePath; %#ok<AGROW>
        case 'unchanged'
            unchanged{end + 1} = filePath; %#ok<AGROW>
        case 'skipped'
            skipped{end + 1} = sprintf('%s | %s', filePath, message); %#ok<AGROW>
        case 'failed'
            failed{end + 1} = sprintf('%s | %s', filePath, message); %#ok<AGROW>
    end
end

fprintf('\nDone.\n');
fprintf('  Updated:   %d\n', numel(updated));
fprintf('  Unchanged: %d\n', numel(unchanged));
fprintf('  Skipped:   %d\n', numel(skipped));
fprintf('  Failed:    %d\n', numel(failed));

printResultGroup('Skipped details', skipped);
printResultGroup('Failed details', failed);
end

function fileList = collectCsvFiles(targetDir, includeSubfolders)
if includeSubfolders
    fileList = collectCsvFilesRecursive(targetDir);
else
    entries = dir(fullfile(targetDir, '*.csv'));
    fileList = arrayfun(@(item) fullfile(item.folder, item.name), entries, 'UniformOutput', false);
end

fileList = sort(fileList);
end

function fileList = collectCsvFilesRecursive(targetDir)
entries = dir(targetDir);
fileList = {};

for idx = 1:numel(entries)
    item = entries(idx);

    if item.isdir
        if strcmp(item.name, '.') || strcmp(item.name, '..')
            continue;
        end

        nestedFiles = collectCsvFilesRecursive(fullfile(item.folder, item.name));
        fileList = [fileList, nestedFiles]; %#ok<AGROW>
        continue;
    end

    [~, ~, ext] = fileparts(item.name);
    if strcmpi(ext, '.csv')
        fileList{end + 1} = fullfile(item.folder, item.name); %#ok<AGROW>
    end
end
end

function [status, message] = processSingleCsv(filePath, createBackup)
[suffix, suffixOk] = extractSuffixFromFileName(filePath);
if ~suffixOk
    status = 'skipped';
    message = 'Missing trailing suffix like _D5 or _D11 in file name.';
    return;
end

[headerNames, headerOk] = headerNamesForSuffix(suffix);
if ~headerOk
    status = 'skipped';
    message = sprintf('No target header defined for suffix %s.', suffix);
    return;
end

[rawBytes, readOk, readMessage] = readFileAsBytes(filePath);
if ~readOk
    status = 'failed';
    message = readMessage;
    return;
end

[bomBytes, contentBytes] = splitBom(rawBytes);
[firstLineBytes, lineEndingBytes, bodyBytes] = splitFirstLine(contentBytes);

firstLine = strtrim(char(firstLineBytes.'));
expectedHeader = strjoin(headerNames, ',');
expectedColumnCount = numel(headerNames);
actualColumnCount = countCsvColumns(firstLine);

if actualColumnCount ~= expectedColumnCount
    status = 'skipped';
    message = sprintf( ...
        'First line has %d column(s), but %s expects %d.', ...
        actualColumnCount, suffix, expectedColumnCount);
    return;
end

if strcmp(firstLine, expectedHeader)
    status = 'unchanged';
    message = sprintf('Header already matches %s.', suffix);
    return;
end

if createBackup
    backupPath = [filePath, '.bak'];
    if exist(backupPath, 'file') ~= 2
        copyOk = copyfile(filePath, backupPath);
        if ~copyOk
            status = 'failed';
            message = sprintf('Failed to create backup: %s', backupPath);
            return;
        end
    end
end

newHeaderBytes = uint8(expectedHeader(:));
newBytes = [bomBytes; newHeaderBytes; lineEndingBytes; bodyBytes];
[writeOk, writeMessage] = writeBytesToFile(filePath, newBytes);
if ~writeOk
    status = 'failed';
    message = writeMessage;
    return;
end

status = 'updated';
message = sprintf('Header updated for %s: %s', suffix, expectedHeader);
end

function [suffix, ok] = extractSuffixFromFileName(filePath)
[~, fileName, ~] = fileparts(filePath);
tokens = regexp(fileName, '_(D\d+)$', 'tokens', 'once', 'ignorecase');

if isempty(tokens)
    suffix = '';
    ok = false;
    return;
end

suffix = upper(tokens{1});
ok = true;
end

function [headerNames, ok] = headerNamesForSuffix(suffix)
switch upper(suffix)
    case 'D5'
        headerNames = {'omega_ref', 'omega_pll', 'Iq_ref', 'Iq_meas'};
    case 'D7'
        headerNames = {'raw21', 'omega_pll', 'Iq_ref', 'Iq_meas'};
    case 'D8'
        headerNames = {'omega_pll', 'Iq_ref', 'Iq_comp', 'Iq_cmd'};
    case 'D9'
        headerNames = {'i_low_window_a_ticks', 'i_low_window_b_ticks', 'i_low_window_c_ticks', 'i_valid_mask'};
    case 'D10'
        headerNames = {'i_pair_active', 'i_pair_next', 'i_pair_valid', 'svm_sector'};
    case 'D11'
        headerNames = {'theta_e_meas', 'theta_e_ctrl', 'delta_theta_deg', 'u_mag_pu'};
    case 'D12'
        headerNames = {'ud_pu', 'uq_pu', 'u_mag_pu', 'delta_theta_deg'};
    otherwise
        headerNames = {};
        ok = false;
        return;
end

ok = true;
end

function [rawBytes, ok, message] = readFileAsBytes(filePath)
fid = fopen(filePath, 'rb');
if fid == -1
    rawBytes = uint8([]);
    ok = false;
    message = sprintf('Failed to open file: %s', filePath);
    return;
end

rawBytes = fread(fid, Inf, '*uint8');
fclose(fid);
ok = true;
message = '';
end

function [bomBytes, contentBytes] = splitBom(rawBytes)
utf8Bom = uint8([239; 187; 191]);

if numel(rawBytes) >= 3 && isequal(rawBytes(1:3), utf8Bom)
    bomBytes = rawBytes(1:3);
    contentBytes = rawBytes(4:end);
else
    bomBytes = uint8([]);
    contentBytes = rawBytes;
end
end

function [firstLineBytes, lineEndingBytes, bodyBytes] = splitFirstLine(contentBytes)
lineBreakIdx = find(contentBytes == 10 | contentBytes == 13, 1, 'first');

if isempty(lineBreakIdx)
    firstLineBytes = contentBytes;
    lineEndingBytes = uint8([]);
    bodyBytes = uint8([]);
    return;
end

firstLineBytes = contentBytes(1:lineBreakIdx - 1);

if contentBytes(lineBreakIdx) == 13 && lineBreakIdx < numel(contentBytes) && contentBytes(lineBreakIdx + 1) == 10
    lineEndingBytes = contentBytes(lineBreakIdx:lineBreakIdx + 1);
    bodyBytes = contentBytes(lineBreakIdx + 2:end);
else
    lineEndingBytes = contentBytes(lineBreakIdx);
    bodyBytes = contentBytes(lineBreakIdx + 1:end);
end
end

function columnCount = countCsvColumns(firstLine)
firstLine = strtrim(firstLine);

if isempty(firstLine)
    columnCount = 0;
    return;
end

columnCount = numel(strsplit(firstLine, ','));
end

function [ok, message] = writeBytesToFile(filePath, rawBytes)
fid = fopen(filePath, 'wb');
if fid == -1
    ok = false;
    message = sprintf('Failed to write file: %s', filePath);
    return;
end

writtenCount = fwrite(fid, rawBytes, 'uint8');
fclose(fid);
if writtenCount ~= numel(rawBytes)
    ok = false;
    message = sprintf('Incomplete write: %s', filePath);
    return;
end

ok = true;
message = '';
end

function printResultGroup(groupName, items)
if isempty(items)
    return;
end

fprintf('\n%s:\n', groupName);
for idx = 1:numel(items)
    fprintf('  - %s\n', items{idx});
end
end
