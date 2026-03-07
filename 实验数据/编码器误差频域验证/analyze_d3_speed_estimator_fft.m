% Compare raw-difference speed and PLL speed from D3 logs.
%
% Expected CSV columns:
%   dbg_id_a, dbg_iq_a, omega_diff_rad_s, omega_pll_rad_s
%
% Default workflow:
%   1) Put this script in the D3 data folder.
%   2) Run it directly in MATLAB.
%   3) It scans V*_D3.csv in the current folder.
%
% Output:
%   - time-domain overlay (first preview window)
%   - single-sided amplitude spectrum in Hz
%   - order spectrum normalized by mechanical base frequency
%   - PLL/diff spectrum ratio
%   - command-window summary of dominant peaks

clear; clc; close all;

%% User config
Fs = 2000;                 % sample rate, 0.5 ms logging interval
preview_seconds = 0.20;    % time-domain preview window
fmin_hz = 1.0;             % ignore near-DC leakage
fmax_hz = 400.0;           % focus range for peak picking / display
top_n_peaks = 8;           % report strongest peaks
use_hann_window = true;    % recommended for amplitude comparison
file_pattern = 'V*_D3.csv';

script_dir = fileparts(mfilename('fullpath'));
if isempty(script_dir)
    script_dir = pwd;
end

files = dir(fullfile(script_dir, file_pattern));
if isempty(files)
    error('No files match pattern "%s" under "%s".', file_pattern, script_dir);
end

fprintf('Found %d D3 files.\n\n', numel(files));

for k = 1:numel(files)
    filename = fullfile(files(k).folder, files(k).name);
    data = readtable(filename);

    required_vars = {'omega_diff_rad_s', 'omega_pll_rad_s'};
    for i = 1:numel(required_vars)
        if ~ismember(required_vars{i}, data.Properties.VariableNames)
            error('File %s is missing column: %s', filename, required_vars{i});
        end
    end

    omega_diff = data.omega_diff_rad_s(:);
    omega_pll = data.omega_pll_rad_s(:);

    n = min(numel(omega_diff), numel(omega_pll));
    omega_diff = omega_diff(1:n);
    omega_pll = omega_pll(1:n);

    t = (0:n-1).' / Fs;
    preview_n = min(n, max(16, round(preview_seconds * Fs)));

    mean_diff = mean(omega_diff);
    mean_pll = mean(omega_pll);
    mean_speed = mean_pll;
    mech_base_hz = abs(mean_speed) / (2 * pi);

    x_diff = omega_diff - mean_diff;
    x_pll = omega_pll - mean_pll;

    amp_diff = local_single_sided_amp(x_diff, Fs, use_hann_window);
    amp_pll = local_single_sided_amp(x_pll, Fs, use_hann_window);
    f = (0:floor(n / 2)).' * (Fs / n);

    valid = (f >= fmin_hz) & (f <= min(fmax_hz, Fs / 2));
    peak_idx_diff = local_pick_peaks(amp_diff, f, valid, top_n_peaks);
    peak_idx_pll = local_pick_peaks(amp_pll, f, valid, top_n_peaks);

    ratio = amp_pll ./ max(amp_diff, eps);
    ratio_db = 20 * log10(ratio);

    fig = figure('Name', ['D3 compare - ' files(k).name], 'Position', [80, 80, 1250, 900]);

    subplot(2,2,1);
    plot(t(1:preview_n), omega_diff(1:preview_n), 'Color', [0.80 0.25 0.15], 'LineWidth', 1.0); hold on;
    plot(t(1:preview_n), omega_pll(1:preview_n),  'Color', [0.10 0.35 0.85], 'LineWidth', 1.0);
    grid on;
    xlabel('Time (s)');
    ylabel('Speed (rad/s)');
    title(sprintf('%s time-domain preview (first %.0f ms)', files(k).name, 1000 * t(preview_n)));
    legend('omega\_diff', 'omega\_pll', 'Location', 'best');

    subplot(2,2,2);
    plot(f(valid), amp_diff(valid), 'Color', [0.80 0.25 0.15], 'LineWidth', 1.0); hold on;
    plot(f(valid), amp_pll(valid),  'Color', [0.10 0.35 0.85], 'LineWidth', 1.0);
    local_mark_peaks(f, amp_diff, peak_idx_diff, [0.80 0.25 0.15]);
    local_mark_peaks(f, amp_pll, peak_idx_pll, [0.10 0.35 0.85]);
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('Amplitude (rad/s)');
    title(sprintf('Single-sided spectrum, mean speed %.2f rad/s', mean_speed));
    legend('omega\_diff', 'omega\_pll', 'Location', 'best');

    subplot(2,2,3);
    if mech_base_hz > 1e-6
        order_axis = f(valid) / mech_base_hz;
        plot(order_axis, amp_diff(valid), 'Color', [0.80 0.25 0.15], 'LineWidth', 1.0); hold on;
        plot(order_axis, amp_pll(valid),  'Color', [0.10 0.35 0.85], 'LineWidth', 1.0);
        grid on;
        xlabel('Order (f / f_{mech})');
        ylabel('Amplitude (rad/s)');
        title(sprintf('Order spectrum, f_{mech}=%.3f Hz', mech_base_hz));
        xlim([0, max(order_axis)]);
        legend('omega\_diff', 'omega\_pll', 'Location', 'best');
    else
        text(0.1, 0.5, 'Mean speed too small to build order spectrum.', 'FontSize', 11);
        axis off;
    end

    subplot(2,2,4);
    plot(f(valid), ratio_db(valid), 'k', 'LineWidth', 1.0);
    grid on;
    xlabel('Frequency (Hz)');
    ylabel('20log10(|PLL| / |diff|) (dB)');
    title('Observer attenuation/amplification vs raw difference');
    yline(0, '--', 'Color', [0.45 0.45 0.45]);

    sgtitle(strrep(files(k).name, '_', '\_'));

    fprintf('=== %s ===\n', files(k).name);
    fprintf('Samples: %d, Fs: %.1f Hz, duration: %.2f s\n', n, Fs, n / Fs);
    fprintf('Mean speed: diff=%.4f rad/s, pll=%.4f rad/s, f_mech=%.4f Hz\n', ...
            mean_diff, mean_pll, mech_base_hz);
    fprintf('AC RMS:     diff=%.4f rad/s, pll=%.4f rad/s\n', rms(x_diff), rms(x_pll));
    fprintf('Std:        diff=%.4f rad/s, pll=%.4f rad/s\n', std(x_diff), std(x_pll));

    fprintf('Top diff peaks:\n');
    local_print_peaks(f, amp_diff, peak_idx_diff, mech_base_hz);

    fprintf('Top pll peaks:\n');
    local_print_peaks(f, amp_pll, peak_idx_pll, mech_base_hz);

    fprintf('Matched diff->pll peak ratio:\n');
    local_print_ratios(f, amp_diff, amp_pll, peak_idx_diff, mech_base_hz);
    fprintf('\n');
end

function amp = local_single_sided_amp(x, Fs, use_hann_window)
    n = numel(x);
    if use_hann_window
        w = hann(n, 'periodic');
    else
        w = ones(n, 1);
    end

    cg = mean(w);  % coherent gain for amplitude correction
    xw = x .* w;
    y = fft(xw);

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

    % Re-sort by frequency for easier reading.
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

function local_print_peaks(f, amp, peak_idx, mech_base_hz)
    if isempty(peak_idx)
        fprintf('  (no peaks found)\n');
        return;
    end

    for i = 1:numel(peak_idx)
        idx = peak_idx(i);
        if mech_base_hz > 1e-6
            order = f(idx) / mech_base_hz;
        else
            order = NaN;
        end

        fprintf('  %2d) f=%8.3f Hz, order=%7.3f, amp=%9.5f rad/s\n', ...
                i, f(idx), order, amp(idx));
    end
end

function local_print_ratios(f, amp_diff, amp_pll, peak_idx_diff, mech_base_hz)
    if isempty(peak_idx_diff)
        fprintf('  (no peaks found)\n');
        return;
    end

    for i = 1:numel(peak_idx_diff)
        idx = peak_idx_diff(i);
        ratio = amp_pll(idx) / max(amp_diff(idx), eps);
        ratio_db = 20 * log10(ratio);
        if mech_base_hz > 1e-6
            order = f(idx) / mech_base_hz;
        else
            order = NaN;
        end

        fprintf('  %2d) f=%8.3f Hz, order=%7.3f, |pll/diff|=%8.4f (%7.2f dB)\n', ...
                i, f(idx), order, ratio, ratio_db);
    end
end
