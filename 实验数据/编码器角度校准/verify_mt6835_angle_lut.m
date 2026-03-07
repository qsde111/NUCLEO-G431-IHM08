% Verify MT6835 angle correction LUT offline on D7 logs.
%
% Workflow:
%   1) Load a generated LUT MAT file.
%   2) Apply raw21 -> corrected_raw21 mapping sample by sample.
%   3) Recompute speed from corrected raw counts using plain wrapped-diff.
%   4) Compare speed ripple before/after correction in time and frequency.

clear; clc; close all;

%% User config
file_pattern = 'V*_D7.csv';
lut_sizes_to_verify = [1024 2048];
Fs = 2000;                 % 0.5 ms logging interval
preview_seconds = 0.12;
fmin_hz = 1.0;
fmax_hz = 400.0;
top_n_peaks = 8;
use_hann_window = true;

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

data_files = dir(fullfile(script_dir, file_pattern));
if isempty(data_files)
    error('No files match pattern "%s" under "%s".', file_pattern, script_dir);
end

for s = 1:numel(lut_sizes_to_verify)
    lut_size = lut_sizes_to_verify(s);
    lut_mat = fullfile(script_dir, sprintf('mt6835_angle_lut_%d.mat', lut_size));
    if ~isfile(lut_mat)
        error('Missing LUT MAT file: %s. Run build_mt6835_angle_lut first.', lut_mat);
    end

    S = load(lut_mat);
    lut_raw21 = double(S.lut_raw21(:));
    bin_shift = double(S.bin_shift);
    full_scale = double(S.full_scale);
    fprintf('=== Verifying LUT size %d ===\n', lut_size);
    fprintf('  LUT file: %s\n\n', lut_mat);

    for k = 1:numel(data_files)
        filename = fullfile(data_files(k).folder, data_files(k).name);
        data = readtable(filename);

        if ~ismember('raw21', data.Properties.VariableNames)
            error('File %s is missing raw21 column.', data_files(k).name);
        end

        raw21 = uint32(round(data.raw21(:)));
        omega_raw = local_speed_from_raw21(raw21, Fs, full_scale);

        corrected_raw21 = local_apply_lut(raw21, lut_raw21, bin_shift, full_scale);
        omega_corr = local_speed_from_raw21(corrected_raw21, Fs, full_scale);

        have_pll = false;
        omega_pll = [];
        if ismember('omega_pll_rad_s', data.Properties.VariableNames)
            omega_pll = data.omega_pll_rad_s(:);
            have_pll = true;
        elseif ismember('omega_pll', data.Properties.VariableNames)
            omega_pll = data.omega_pll(:);
            have_pll = true;
        end

        dir_sign = sign(mean(omega_raw));
        if have_pll && (sign(mean(omega_pll)) ~= 0)
            dir_sign = sign(mean(omega_pll));
        elseif dir_sign == 0
            dir_sign = 1;
        end
        omega_raw = omega_raw * dir_sign;
        omega_corr = omega_corr * dir_sign;
        if have_pll
            omega_pll = omega_pll * sign(mean(omega_pll));
        end

        n = numel(omega_raw);
        t = (0:n - 1).' / Fs;
        preview_n = min(n, max(16, round(preview_seconds * Fs)));

        mean_raw = mean(omega_raw);
        mean_corr = mean(omega_corr);
        mech_base_hz = abs(mean_corr) / (2 * pi);

        ac_raw = omega_raw - mean_raw;
        ac_corr = omega_corr - mean_corr;

        amp_raw = local_single_sided_amp(ac_raw, use_hann_window);
        amp_corr = local_single_sided_amp(ac_corr, use_hann_window);
        f = (0:floor(n / 2)).' * (Fs / n);

        valid = (f >= fmin_hz) & (f <= min(fmax_hz, Fs / 2));
        peak_idx_raw = local_pick_peaks(amp_raw, f, valid, top_n_peaks);
        ratio = amp_corr ./ max(amp_raw, eps);
        ratio_db = 20 * log10(ratio);

        figure('Name', sprintf('Verify LUT %d - %s', lut_size, data_files(k).name), ...
               'Position', [100, 100, 1280, 920]);

        subplot(2,2,1);
        plot(t(1:preview_n), omega_raw(1:preview_n), 'Color', [0.80 0.25 0.15], 'LineWidth', 1.0); hold on;
        plot(t(1:preview_n), omega_corr(1:preview_n), 'Color', [0.10 0.55 0.15], 'LineWidth', 1.0);
        if have_pll
            plot(t(1:preview_n), omega_pll(1:preview_n), 'Color', [0.10 0.35 0.85], 'LineWidth', 0.9);
            legend('raw diff', 'corrected diff', 'logged pll', 'Location', 'best');
        else
            legend('raw diff', 'corrected diff', 'Location', 'best');
        end
        grid on;
        xlabel('Time (s)');
        ylabel('Speed (rad/s)');
        title(sprintf('%s time preview', data_files(k).name));

        subplot(2,2,2);
        plot(f(valid), amp_raw(valid), 'Color', [0.80 0.25 0.15], 'LineWidth', 1.0); hold on;
        plot(f(valid), amp_corr(valid), 'Color', [0.10 0.55 0.15], 'LineWidth', 1.0);
        local_mark_peaks(f, amp_raw, peak_idx_raw, [0.80 0.25 0.15]);
        grid on;
        xlabel('Frequency (Hz)');
        ylabel('Amplitude (rad/s)');
        title('Speed spectrum before/after correction');
        legend('raw diff', 'corrected diff', 'Location', 'best');

        subplot(2,2,3);
        if mech_base_hz > 1e-6
            order_axis = f(valid) / mech_base_hz;
            plot(order_axis, amp_raw(valid), 'Color', [0.80 0.25 0.15], 'LineWidth', 1.0); hold on;
            plot(order_axis, amp_corr(valid), 'Color', [0.10 0.55 0.15], 'LineWidth', 1.0);
            grid on;
            xlabel('Order (f / f_{mech})');
            ylabel('Amplitude (rad/s)');
            title(sprintf('Order spectrum, f_{mech}=%.3f Hz', mech_base_hz));
            legend('raw diff', 'corrected diff', 'Location', 'best');
        else
            text(0.1, 0.5, 'Mean speed too small to build order spectrum.', 'FontSize', 11);
            axis off;
        end

        subplot(2,2,4);
        plot(f(valid), ratio_db(valid), 'k', 'LineWidth', 1.0);
        grid on;
        xlabel('Frequency (Hz)');
        ylabel('20log10(|corr| / |raw|) (dB)');
        title('Correction gain ratio');
        yline(0, '--', 'Color', [0.45 0.45 0.45]);

        sgtitle(sprintf('Verify LUT %d on %s', lut_size, strrep(data_files(k).name, '_', '\_')));

        fprintf('%s\n', data_files(k).name);
        fprintf('  mean speed    : raw=%9.4f, corr=%9.4f rad/s\n', mean_raw, mean_corr);
        fprintf('  std(ac)       : raw=%9.4f, corr=%9.4f rad/s\n', std(ac_raw), std(ac_corr));
        fprintf('  pkpk(ac)      : raw=%9.4f, corr=%9.4f rad/s\n', max(ac_raw) - min(ac_raw), max(ac_corr) - min(ac_corr));
        fprintf('  peak reduction at raw-diff peaks:\n');
        local_print_reduction(f, amp_raw, amp_corr, peak_idx_raw, mech_base_hz);
        fprintf('\n');
    end
end

function corrected_raw21 = local_apply_lut(raw21, lut_raw21, bin_shift, full_scale)
    raw = double(raw21(:));
    raw = mod(raw, full_scale);

    lut_size = numel(lut_raw21) - 1;
    bin_size = 2 ^ bin_shift;

    idx = floor(raw / bin_size) + 1;     % 1..lut_size
    idx(idx < 1) = 1;
    idx(idx > lut_size) = lut_size;

    frac = (raw - ((idx - 1) * bin_size)) / bin_size;
    y0 = lut_raw21(idx);
    y1 = lut_raw21(idx + 1);
    corrected_raw21 = y0 + ((y1 - y0) .* frac);
    corrected_raw21 = mod(corrected_raw21, full_scale);
end

function omega = local_speed_from_raw21(raw21, Fs, full_scale)
    raw = double(raw21(:));
    raw = mod(raw, full_scale);

    d = diff(raw);
    half_scale = full_scale / 2;
    d(d > half_scale) = d(d > half_scale) - full_scale;
    d(d < -half_scale) = d(d < -half_scale) + full_scale;

    if isempty(d)
        omega = zeros(size(raw));
        return;
    end

    omega = [d(1); d] * ((2 * pi * Fs) / full_scale);
end

function amp = local_single_sided_amp(x, use_hann_window)
    x = x(:);
    n = numel(x);
    if use_hann_window
        w = hann(n, 'periodic');
    else
        w = ones(n, 1);
    end

    cg = mean(w);
    y = fft(x .* w);
    p2 = abs(y) / (n * cg);
    amp = p2(1:floor(n / 2) + 1);
    if numel(amp) > 2
        amp(2:end-1) = 2 * amp(2:end-1);
    end
end

function peak_idx = local_pick_peaks(amp, f, valid_mask, top_n)
    idx = find(valid_mask);
    if numel(idx) < 3
        peak_idx = idx;
        return;
    end

    a = amp(idx);
    local_max = false(size(a));
    local_max(2:end-1) = (a(2:end-1) >= a(1:end-2)) & (a(2:end-1) > a(3:end));
    peak_idx = idx(local_max);

    if isempty(peak_idx)
        peak_idx = idx;
    end

    [~, order] = sort(amp(peak_idx), 'descend');
    keep_n = min(top_n, numel(order));
    peak_idx = peak_idx(order(1:keep_n));

    [~, order_f] = sort(f(peak_idx), 'ascend');
    peak_idx = peak_idx(order_f);
end

function local_mark_peaks(f, amp, peak_idx, color_rgb)
    if isempty(peak_idx)
        return;
    end

    plot(f(peak_idx), amp(peak_idx), 'o', ...
         'Color', color_rgb, ...
         'MarkerFaceColor', color_rgb, ...
         'MarkerSize', 4);

    for i = 1:numel(peak_idx)
        idx = peak_idx(i);
        text(f(idx), amp(idx), sprintf(' %.2f', f(idx)), ...
             'Color', color_rgb, ...
             'FontSize', 8, ...
             'VerticalAlignment', 'bottom');
    end
end

function local_print_reduction(f, amp_raw, amp_corr, peak_idx_raw, mech_base_hz)
    if isempty(peak_idx_raw)
        fprintf('    (no peaks found)\n');
        return;
    end

    for i = 1:numel(peak_idx_raw)
        idx = peak_idx_raw(i);
        ratio = amp_corr(idx) / max(amp_raw(idx), eps);
        ratio_db = 20 * log10(ratio);
        if mech_base_hz > 1e-6
            order = f(idx) / mech_base_hz;
        else
            order = NaN;
        end
        fprintf('    %2d) f=%8.3f Hz, order=%7.3f, |corr/raw|=%8.4f (%7.2f dB)\n', ...
                i, f(idx), order, ratio, ratio_db);
    end
end
