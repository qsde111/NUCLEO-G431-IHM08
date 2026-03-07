% Build MT6835 angle correction LUT from constant-speed D7 logs.
%
% Method:
%   1) Count raw21 occupancy in equally spaced raw bins.
%   2) Convert the occupancy histogram to a cumulative distribution.
%   3) Use the cumulative distribution as a monotonic raw->corrected mapping.
%
% This is the same core idea as the older AS5047P PowerShell script, but
% adapted to 21-bit MT6835 raw counts and D7 log files.

clear; clc; close all;

%% User config
file_pattern = 'V*_D7.csv';
lut_sizes = [1024 2048];
min_cmd_speed = 150;      % default: build from higher-speed logs first
max_cmd_speed = inf;
skip_seconds = 0.0;
Fs = 2000;                % D7 logging rate, 0.5 ms
raw_column = 'raw21';
full_scale_bits = 21;
smooth_window_bins = 9;   % odd number recommended, 1 disables smoothing
export_header = true;
export_csv = true;
export_mat = true;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

all_files = dir(fullfile(script_dir, file_pattern));
if isempty(all_files)
    error('No files match pattern "%s" under "%s".', file_pattern, script_dir);
end

selected = false(numel(all_files), 1);
cmd_speeds = nan(numel(all_files), 1);
for i = 1:numel(all_files)
    cmd_speeds(i) = local_parse_speed_from_name(all_files(i).name);
    if isnan(cmd_speeds(i))
        selected(i) = true;
    else
        selected(i) = (cmd_speeds(i) >= min_cmd_speed) && (cmd_speeds(i) <= max_cmd_speed);
    end
end

files = all_files(selected);
if isempty(files)
    error('No files remain after speed filtering. Adjust min_cmd_speed/max_cmd_speed.');
end

fprintf('Using %d D7 files to build LUT:\n', numel(files));
for i = 1:numel(files)
    if isnan(local_parse_speed_from_name(files(i).name))
        fprintf('  - %s\n', files(i).name);
    else
        fprintf('  - %s (Vcmd=%.3f rad/s)\n', files(i).name, local_parse_speed_from_name(files(i).name));
    end
end
fprintf('\n');

full_scale = 2 ^ full_scale_bits;

for s = 1:numel(lut_sizes)
    lut_size = lut_sizes(s);
    bits = round(log2(lut_size));
    if (2 ^ bits) ~= lut_size
        error('lut_size=%d must be a power of two.', lut_size);
    end

    bin_shift = full_scale_bits - bits;
    if bin_shift < 0
        error('lut_size=%d is too large for %d-bit raw counts.', lut_size, full_scale_bits);
    end

    hist_counts = zeros(lut_size, 1);
    total_samples = 0;

    for i = 1:numel(files)
        filename = fullfile(files(i).folder, files(i).name);
        data = readtable(filename);
        if ~ismember(raw_column, data.Properties.VariableNames)
            error('File %s is missing column "%s".', files(i).name, raw_column);
        end

        raw = data.(raw_column);
        skip_n = max(0, round(skip_seconds * Fs));
        if skip_n >= numel(raw)
            error('skip_seconds is too large for file %s.', files(i).name);
        end

        raw = raw(skip_n + 1:end);
        raw = uint32(round(raw));
        raw = bitand(raw, uint32(full_scale - 1));

        idx = double(bitshift(raw, -bin_shift)) + 1;  % 1..lut_size
        hist_counts = hist_counts + accumarray(idx, 1, [lut_size 1], @sum, 0);
        total_samples = total_samples + numel(idx);
    end

    if total_samples == 0
        error('No valid samples collected for LUT size %d.', lut_size);
    end

    hist_smooth = local_circular_movmean(hist_counts, smooth_window_bins);

    lut_raw21 = zeros(lut_size + 1, 1);
    lut_raw21(1) = 0;
    cdf_counts = cumsum(hist_smooth);
    lut_raw21(2:end) = round((cdf_counts / cdf_counts(end)) * full_scale);
    lut_raw21(end) = full_scale;
    lut_raw21 = local_enforce_monotonic(lut_raw21, full_scale);

    ideal_raw21 = round((0:lut_size).' * (full_scale / lut_size));
    corr_error_counts = lut_raw21 - ideal_raw21;
    corr_error_deg = corr_error_counts * (360.0 / full_scale);

    out_base = sprintf('mt6835_angle_lut_%d', lut_size);
    out_mat = fullfile(script_dir, [out_base '.mat']);
    out_csv = fullfile(script_dir, [out_base '.csv']);
    out_header = fullfile(script_dir, [out_base '.h']);

    figure('Name', sprintf('MT6835 LUT build - %d', lut_size), 'Position', [100, 100, 1200, 820]);

    subplot(2,2,1);
    stem(hist_counts, 'Marker', 'none', 'Color', [0.60 0.60 0.60]); hold on;
    plot(hist_smooth, 'b', 'LineWidth', 1.1);
    grid on;
    xlabel('Raw bin index');
    ylabel('Counts');
    title(sprintf('Histogram, LUT=%d, total=%d', lut_size, total_samples));
    legend('raw', 'smoothed', 'Location', 'best');

    subplot(2,2,2);
    plot(0:lut_size, lut_raw21, 'b', 'LineWidth', 1.2); hold on;
    plot(0:lut_size, ideal_raw21, '--', 'Color', [0.35 0.35 0.35], 'LineWidth', 1.0);
    grid on;
    xlabel('LUT boundary index');
    ylabel('Corrected raw21');
    title('CDF mapping');
    legend('measured CDF', 'ideal linear', 'Location', 'best');

    subplot(2,2,3);
    plot(0:lut_size, corr_error_counts, 'Color', [0.85 0.25 0.15], 'LineWidth', 1.1);
    grid on;
    xlabel('LUT boundary index');
    ylabel('Error (raw counts)');
    title('Correction relative to ideal linear mapping');

    subplot(2,2,4);
    plot(0:lut_size, corr_error_deg, 'Color', [0.10 0.45 0.15], 'LineWidth', 1.1);
    grid on;
    xlabel('LUT boundary index');
    ylabel('Error (deg mech)');
    title('Correction relative to ideal linear mapping');

    sgtitle(sprintf('MT6835 angle LUT build, size=%d', lut_size));

    if export_mat
        files_used = string({files.name}).';
        save(out_mat, 'lut_raw21', 'ideal_raw21', 'corr_error_counts', 'corr_error_deg', ...
             'hist_counts', 'hist_smooth', 'lut_size', 'bin_shift', 'full_scale', ...
             'full_scale_bits', 'smooth_window_bins', 'files_used');
    end

    if export_csv
        lut_index = (0:lut_size).';
        T = table(lut_index, lut_raw21, ideal_raw21, corr_error_counts, corr_error_deg);
        writetable(T, out_csv);
    end

    if export_header
        local_write_header(out_header, lut_raw21, lut_size, bin_shift, full_scale);
    end

    fprintf('Built LUT size %d\n', lut_size);
    fprintf('  bin_shift        : %d\n', bin_shift);
    fprintf('  total_samples    : %d\n', total_samples);
    fprintf('  output MAT       : %s\n', out_mat);
    fprintf('  output CSV       : %s\n', out_csv);
    if export_header
        fprintf('  output header    : %s\n', out_header);
    end
    fprintf('\n');
end

function speed = local_parse_speed_from_name(filename)
    tokens = regexp(filename, 'V([0-9]+(?:\.[0-9]+)?)_D7', 'tokens', 'once');
    if isempty(tokens)
        speed = NaN;
    else
        speed = str2double(tokens{1});
    end
end

function y = local_circular_movmean(x, window_len)
    x = x(:);
    n = numel(x);
    if (window_len <= 1) || (n == 0)
        y = x;
        return;
    end

    window_len = max(1, round(window_len));
    if mod(window_len, 2) == 0
        window_len = window_len + 1;
    end

    pad = floor(window_len / 2);
    xpad = [x(end - pad + 1:end); x; x(1:pad)];
    kernel = ones(window_len, 1) / window_len;
    y = conv(xpad, kernel, 'same');
    y = y(pad + 1:pad + n);
end

function lut = local_enforce_monotonic(lut, full_scale)
    lut = lut(:);
    lut(1) = 0;
    for i = 2:numel(lut)
        if lut(i) < lut(i - 1)
            lut(i) = lut(i - 1);
        end
    end
    lut(end) = full_scale;
end

function local_write_header(filename, lut_raw21, lut_size, bin_shift, full_scale)
    fid = fopen(filename, 'w');
    if fid < 0
        error('Failed to open %s for writing.', filename);
    end
    cleaner = @(s) upper(regexprep(s, '[^a-zA-Z0-9]', '_'));
    [~, name, ext] = fileparts(filename);
    guard = cleaner([name ext]);
    array_name = sprintf('mt6835_angle_lut_raw21_%d', lut_size);

    fprintf(fid, '#ifndef %s\n', guard);
    fprintf(fid, '#define %s\n\n', guard);
    fprintf(fid, '#include <stdint.h>\n\n');
    fprintf(fid, '#define MT6835_ANGLE_LUT_SIZE (%dU)\n', lut_size);
    fprintf(fid, '#define MT6835_ANGLE_LUT_SHIFT (%dU)\n', bin_shift);
    fprintf(fid, '#define MT6835_ANGLE_FULL_SCALE (%uUL)\n\n', uint32(full_scale));
    fprintf(fid, 'static const uint32_t %s[MT6835_ANGLE_LUT_SIZE + 1U] =\n{\n', array_name);

    for i = 1:numel(lut_raw21)
        if mod(i - 1, 8) == 0
            fprintf(fid, '    ');
        end
        fprintf(fid, '%uU', uint32(lut_raw21(i)));
        if i ~= numel(lut_raw21)
            fprintf(fid, ', ');
        end
        if mod(i, 8) == 0 || i == numel(lut_raw21)
            fprintf(fid, '\n');
        end
    end

    fprintf(fid, '};\n\n');
    fprintf(fid, '#endif /* %s */\n', guard);
    fclose(fid);
end
