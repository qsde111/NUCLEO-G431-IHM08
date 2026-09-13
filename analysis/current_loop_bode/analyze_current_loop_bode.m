clear; clc; close all;

fs = 2000;                 % D6 stream rate: MOTORAPP_CTRL_HZ / MOTORAPP_STREAM_DIV
motorR = 0.112;            % ohm, 2204-2300KV phase resistance
motorL = 9e-6;             % H, 2204-2300KV phase inductance
kp = 0.1131;               % motor_app.c MOTORAPP_ICTRL_KP
ki = 1407.4;               % motor_app.c MOTORAPP_ICTRL_KI

scriptDir = fileparts(mfilename('fullpath'));
repoRoot = fileparts(fileparts(scriptDir));
dataDir = fullfile(repoRoot, '实验数据', '系统辨识', '开环');
outDir = scriptDir;

files = dir(fullfile(dataDir, 'I_B=*_A=*_*.csv'));
if isempty(files)
    error('No sweep CSV files found in %s', dataDir);
end

allRows = table();
figure('Color', 'w', 'Position', [100 100 1100 720]);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
axMag = nexttile; hold(axMag, 'on'); grid(axMag, 'on'); set(axMag, 'XScale', 'log');
ylabel(axMag, '|Iq_{meas}/Iq_{cmd}| (dB)');
title(axMag, 'Current-loop swept-sine estimate, D6 CSV');

axPh = nexttile; hold(axPh, 'on'); grid(axPh, 'on'); set(axPh, 'XScale', 'log');
xlabel(axPh, 'Frequency (Hz)');
ylabel(axPh, 'Phase (deg)');

for k = 1:numel(files)
    name = files(k).name;
    p = parseSweepName(name);
    if isempty(p)
        warning('Skip unrecognized file name: %s', name);
        continue;
    end

    T = readtable(fullfile(files(k).folder, files(k).name), 'VariableNamingRule', 'preserve');
    active = find(T.iq_sweep_active > 0.5);
    if isempty(active)
        warning('Skip file without active sweep samples: %s', name);
        continue;
    end

    if active(1) ~= 1
        startIdx = active(1);
    else
        % The sweep start/phase is missing if capture begins while active is already 1.
        warning('Skip %s: capture starts while iq_sweep_active=1, so chirp phase origin is unknown.', name);
        continue;
    end

    nExpected = round(p.duration_s * fs);
    stopIdx = min(startIdx + nExpected - 1, height(T));
    if stopIdx <= startIdx + 100
        warning('Skip too-short active segment: %s', name);
        continue;
    end

    u = T.dbg_iq_cmd_a(startIdx:stopIdx);
    y = T.dbg_iq_a(startIdx:stopIdx);
    [freqHz, magDb, phaseDeg] = estimateChirpResponse(u, y, fs, p.f_start_hz, p.f_end_hz, p.duration_s);

    fileCol = repmat({name}, numel(freqHz), 1);
    oneFile = table(fileCol, freqHz(:), magDb(:), phaseDeg(:), ...
        'VariableNames', {'file', 'frequency_hz', 'gain_db', 'phase_deg'});
    allRows = [allRows; oneFile]; %#ok<AGROW>

    label = sprintf("I_B=%.2g A, A=%.2g A, %.0f-%.0f Hz, %.0fs", ...
        p.bias_a, p.amp_a, p.f_start_hz, p.f_end_hz, p.duration_s);
    plot(axMag, freqHz, magDb, 'LineWidth', 1.4, 'DisplayName', label);
    plot(axPh, freqHz, phaseDeg, 'LineWidth', 1.4, 'DisplayName', label);
end

if isempty(allRows)
    error('No usable sweep files were analyzed.');
end

fTheory = logspace(log10(1), log10(3000), 600);
s = 1i * 2*pi*fTheory;
H = (kp*s + ki) ./ (motorL*s.^2 + (motorR + kp)*s + ki);
plot(axMag, fTheory, 20*log10(abs(H)), 'k--', 'LineWidth', 1.2, 'DisplayName', 'RL+PI theory, no delay');
plot(axPh, fTheory, unwrap(angle(H))*180/pi, 'k--', 'LineWidth', 1.2, 'DisplayName', 'RL+PI theory, no delay');
yline(axMag, -3, ':', '-3 dB', 'HandleVisibility', 'off');
xline(axMag, kp/motorL/(2*pi), ':', '2 kHz PI design', 'HandleVisibility', 'off');
xline(axPh, kp/motorL/(2*pi), ':', '2 kHz PI design', 'HandleVisibility', 'off');
legend(axMag, 'Location', 'southwest', 'Interpreter', 'none');
legend(axPh, 'Location', 'southwest', 'Interpreter', 'none');
xlim(axMag, [1 3000]);
xlim(axPh, [1 3000]);
ylim(axMag, [-6 2]);
ylim(axPh, [-45 10]);

summaryPath = fullfile(outDir, 'current_loop_freq_response_summary.csv');
plotPath = fullfile(outDir, 'current_loop_freq_response_bode.png');
figPath = fullfile(outDir, 'current_loop_freq_response_bode.fig');
writetable(allRows, summaryPath);
exportgraphics(gcf, plotPath, 'Resolution', 180);
savefig(gcf, figPath);

fprintf("Wrote %s\n", summaryPath);
fprintf("Wrote %s\n", plotPath);
fprintf("Wrote %s\n", figPath);
fprintf("PI design check with R=%.6g ohm, L=%.6g H:\n", motorR, motorL);
fprintf("  Kp/L = %.2f Hz\n", kp / motorL / (2*pi));
fprintf("  Ki/R = %.2f Hz\n", ki / motorR / (2*pi));

function p = parseSweepName(name)
    tok = regexp(name, 'I_B=([0-9.]+)_A=([0-9.]+)_([0-9.]+)-([0-9.]+)HZ_D=([0-9.]+)s', 'tokens', 'once');
    if isempty(tok)
        p = [];
        return;
    end
    p = struct();
    p.bias_a = str2double(tok{1});
    p.amp_a = str2double(tok{2});
    p.f_start_hz = str2double(tok{3});
    p.f_end_hz = str2double(tok{4});
    p.duration_s = str2double(tok{5});
end

function [freqHz, magDb, phaseDeg] = estimateChirpResponse(u, y, fs, f0, f1, duration_s)
    n = numel(u);
    steps = duration_s * fs;
    idx = (0:n-1).';
    fInst = f0 * exp(log(f1/f0) * idx / steps);
    phi = zeros(n, 1);
    phi(2:end) = cumsum(2*pi*fInst(1:end-1) / fs);

    % Keep away from sweep boundaries where the local fitting window is one-sided.
    freqHz = logspace(log10(f0*1.25), log10(f1*0.82), 36).';
    magDb = nan(size(freqHz));
    phaseDeg = nan(size(freqHz));
    t = idx / fs;

    for i = 1:numel(freqHz)
        f = freqHz(i);
        center_t = duration_s * log(f/f0) / log(f1/f0);
        window_s = min(max(8/f, 0.20), min(6.0, duration_s * 0.45));
        keep = abs(t - center_t) <= window_s/2;
        if nnz(keep) < 20
            continue;
        end

        ii = find(keep);
        w = 0.5 - 0.5*cos(2*pi*(0:numel(ii)-1)'/max(numel(ii)-1, 1));
        X = [sin(phi(ii)), cos(phi(ii)), ones(numel(ii), 1)];
        cu = (X .* w) \ (u(ii) .* w);
        cy = (X .* w) \ (y(ii) .* w);

        ampU = hypot(cu(1), cu(2));
        ampY = hypot(cy(1), cy(2));
        phaseU = atan2d(cu(2), cu(1));
        phaseY = atan2d(cy(2), cy(1));

        magDb(i) = 20*log10(ampY / ampU);
        phaseDeg(i) = wrapTo180Local(phaseY - phaseU);
    end

    good = isfinite(magDb) & isfinite(phaseDeg);
    freqHz = freqHz(good);
    magDb = magDb(good);
    phaseDeg = phaseDeg(good);
end

function y = wrapTo180Local(x)
    y = mod(x + 180, 360) - 180;
end
