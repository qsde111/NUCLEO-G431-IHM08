clear;
clc;
close all;

cfg.dataDir = fileparts(mfilename("fullpath"));
cfg.filePattern = "TCOMP_SCALE=*_V*_D*.csv";
cfg.saveFigures = true;
cfg.outputDir = fullfile(cfg.dataDir, "analysis_out");

% Optional bench current readback from XL4015 display.
cfg.extCurrent = table( ...
    [500; 500; 500; 500; 500; 700; 700; 700; 700; 700], ...
    [0.00; 0.75; 0.90; 1.00; 1.10; 0.00; 0.75; 0.90; 1.00; 1.10], ...
    [0.16; 0.15; 0.16; 0.17; 0.15; 0.21; 0.20; 0.21; 0.22; 0.21], ...
    'VariableNames', {'speed_cmd_rad_s', 'tcomp_scale', 'bench_current_a'});

files = dir(fullfile(cfg.dataDir, cfg.filePattern));
if isempty(files)
    error("No files matched %s in %s.", cfg.filePattern, cfg.dataDir);
end

if ~exist(cfg.outputDir, "dir")
    mkdir(cfg.outputDir);
end

metaRows = repmat(emptyMetaRow(), numel(files), 1);
for k = 1:numel(files)
    metaRows(k) = parseFile(files(k));
end
meta = struct2table(metaRows);

pairKeys = unique(meta(:, ["speed_cmd_rad_s", "tcomp_scale"]), "rows");
summaryRows = repmat(emptySummaryRow(), height(pairKeys), 1);

for k = 1:height(pairKeys)
    speedCmd = pairKeys.speed_cmd_rad_s(k);
    tcompScale = pairKeys.tcomp_scale(k);
    mask = meta.speed_cmd_rad_s == speedCmd & meta.tcomp_scale == tcompScale;
    sub = meta(mask, :);

    row = emptySummaryRow();
    row.speed_cmd_rad_s = speedCmd;
    row.tcomp_scale = tcompScale;

    pathD11 = pickPath(sub, 11);
    pathD12 = pickPath(sub, 12);
    pathD5 = pickPath(sub, 5);

    if strlength(pathD11) > 0
        T11 = readtable(pathD11, "VariableNamingRule", "preserve");
        row.delta_theta_deg_mean = mean(T11.delta_theta_deg);
        row.delta_theta_deg_std = std(T11.delta_theta_deg, 1);
        row.u_mag_d11_mean = mean(T11.u_mag_pu);
    end

    if strlength(pathD12) > 0
        T12 = readtable(pathD12, "VariableNamingRule", "preserve");
        ud = T12.ud_pu;
        uq = T12.uq_pu;
        um = T12.u_mag_pu;
        row.ud_mean_pu = mean(ud);
        row.ud_abs_mean_pu = mean(abs(ud));
        row.ud_rms_pu = calcRms(ud);
        row.uq_mean_pu = mean(uq);
        row.uq_rms_pu = calcRms(uq);
        row.u_mag_mean_pu = mean(um);
        row.u_mag_rms_pu = calcRms(um);
    end

    if strlength(pathD5) > 0
        T5 = readtable(pathD5, "VariableNamingRule", "preserve");
        omegaRef = T5.omega_ref;
        omegaPll = T5.omega_pll;
        iqRef = T5.Iq_ref;
        iqMeas = T5.Iq_meas;

        row.omega_ref_mean = mean(omegaRef);
        row.omega_pll_mean = mean(omegaPll);
        row.omega_std = std(omegaPll, 1);
        row.omega_err_rms = calcRms(omegaPll - omegaRef);
        row.iq_ref_abs_mean = mean(abs(iqRef));
        row.iq_ref_rms = calcRms(iqRef);
        row.iq_meas_abs_mean = mean(abs(iqMeas));
        row.iq_meas_rms = calcRms(iqMeas);
    end

    extMask = cfg.extCurrent.speed_cmd_rad_s == speedCmd & abs(cfg.extCurrent.tcomp_scale - tcompScale) < 1e-9;
    if any(extMask)
        row.bench_current_a = cfg.extCurrent.bench_current_a(find(extMask, 1, "first"));
    end

    summaryRows(k) = row;
end

summary = struct2table(summaryRows);
summary = sortrows(summary, ["speed_cmd_rad_s", "tcomp_scale"]);

summary = addRelativeColumns(summary, "ud_abs_mean_pu");
summary = addRelativeColumns(summary, "u_mag_mean_pu");
summary = addRelativeColumns(summary, "omega_std");
summary = addRelativeColumns(summary, "omega_err_rms");
summary = addRelativeColumns(summary, "iq_ref_abs_mean");
summary = addRelativeColumns(summary, "iq_meas_abs_mean");
summary = addRelativeColumns(summary, "bench_current_a");

writetable(summary, fullfile(cfg.outputDir, "summary.csv"));
makeFigures(summary, cfg);
printDiagnosis(summary);

disp(" ");
disp("Analysis finished.");
disp(summary);
disp("Output folder:");
disp(cfg.outputDir);

function out = pickPath(metaTable, page)
    out = "";
    idx = find(metaTable.page == page, 1, "first");
    if ~isempty(idx)
        out = metaTable.path(idx);
    end
end

function summary = addRelativeColumns(summary, metricName)
    relName = metricName + "_rel_pct";
    summary.(relName) = nan(height(summary), 1);
    speeds = unique(summary.speed_cmd_rad_s);
    for i = 1:numel(speeds)
        mask = summary.speed_cmd_rad_s == speeds(i);
        idx0 = find(mask & abs(summary.tcomp_scale) < 1e-9, 1, "first");
        if isempty(idx0) || ~isfinite(summary.(metricName)(idx0)) || abs(summary.(metricName)(idx0)) < eps
            continue;
        end
        base = summary.(metricName)(idx0);
        summary.(relName)(mask) = 100 * (summary.(metricName)(mask) / base - 1);
    end
end

function makeFigures(summary, cfg)
    speeds = unique(summary.speed_cmd_rad_s);
    colors = lines(max(numel(speeds), 2));

    fig1 = figure("Color", "w", "Name", "tcomp_voltage_summary", "Position", [100 100 1400 850]);
    tl = tiledlayout(fig1, 2, 3, "Padding", "compact", "TileSpacing", "compact");
    title(tl, "TCOMP sweep summary");

    ax = nexttile(tl, 1);
    hold(ax, "on");
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        plot(ax, sub.tcomp_scale, sub.delta_theta_deg_mean, "-o", "LineWidth", 1.2, ...
            "Color", colors(i, :), "DisplayName", sprintf("V%d", speeds(i)));
    end
    grid(ax, "on");
    xlabel(ax, "TCOMP\_SCALE");
    ylabel(ax, "\Delta\theta mean (deg)");
    legend(ax, "Location", "best");
    title(ax, "Injected angle compensation");

    ax = nexttile(tl, 2);
    hold(ax, "on");
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        plot(ax, sub.tcomp_scale, sub.ud_abs_mean_pu, "-o", "LineWidth", 1.2, ...
            "Color", colors(i, :), "DisplayName", sprintf("V%d", speeds(i)));
    end
    grid(ax, "on");
    xlabel(ax, "TCOMP\_SCALE");
    ylabel(ax, "mean |Ud| (pu)");
    title(ax, "D-axis voltage effort");

    ax = nexttile(tl, 3);
    hold(ax, "on");
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        plot(ax, sub.tcomp_scale, sub.u_mag_mean_pu, "-o", "LineWidth", 1.2, ...
            "Color", colors(i, :), "DisplayName", sprintf("V%d", speeds(i)));
    end
    grid(ax, "on");
    xlabel(ax, "TCOMP\_SCALE");
    ylabel(ax, "mean |U| (pu)");
    title(ax, "Total voltage magnitude");

    ax = nexttile(tl, 4);
    hold(ax, "on");
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        plot(ax, sub.tcomp_scale, sub.omega_std, "-o", "LineWidth", 1.2, ...
            "Color", colors(i, :), "DisplayName", sprintf("V%d", speeds(i)));
    end
    grid(ax, "on");
    xlabel(ax, "TCOMP\_SCALE");
    ylabel(ax, "std(\omega\_pll) (rad/s)");
    title(ax, "Steady-state speed ripple");

    ax = nexttile(tl, 5);
    hold(ax, "on");
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        plot(ax, sub.tcomp_scale, sub.iq_ref_abs_mean, "-o", "LineWidth", 1.2, ...
            "Color", colors(i, :), "DisplayName", sprintf("|Iq_ref| V%d", speeds(i)));
        plot(ax, sub.tcomp_scale, sub.iq_meas_abs_mean, "--s", "LineWidth", 1.0, ...
            "Color", colors(i, :), "HandleVisibility", "off");
    end
    grid(ax, "on");
    xlabel(ax, "TCOMP\_SCALE");
    ylabel(ax, "mean |Iq| (A)");
    title(ax, "Current effort");
    legend(ax, "Location", "best");

    ax = nexttile(tl, 6);
    hold(ax, "on");
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        valid = isfinite(sub.bench_current_a);
        if any(valid)
            plot(ax, sub.tcomp_scale(valid), sub.bench_current_a(valid), "-o", "LineWidth", 1.2, ...
                "Color", colors(i, :), "DisplayName", sprintf("XL4015 V%d", speeds(i)));
        end
    end
    grid(ax, "on");
    xlabel(ax, "TCOMP\_SCALE");
    ylabel(ax, "Bench current (A)");
    title(ax, "External supply readback");
    legend(ax, "Location", "best");

    if cfg.saveFigures
        exportgraphics(fig1, fullfile(cfg.outputDir, "summary_overview.png"), "Resolution", 180);
    end
end

function printDiagnosis(summary)
    disp(" ");
    disp("========== TCOMP sweep diagnosis ==========");
    speeds = unique(summary.speed_cmd_rad_s);
    for i = 1:numel(speeds)
        sub = sortrows(summary(summary.speed_cmd_rad_s == speeds(i), :), "tcomp_scale");
        fprintf("V%d:\n", speeds(i));
        printBest(sub, "ud_abs_mean_pu", "lowest mean |Ud|");
        printBest(sub, "u_mag_mean_pu", "lowest mean |U|");
        printBest(sub, "omega_std", "lowest speed ripple std");
        printBest(sub, "iq_ref_abs_mean", "lowest mean |Iq_ref|");
        if any(isfinite(sub.bench_current_a))
            printBest(sub, "bench_current_a", "lowest bench current");
        end
    end
    disp("===========================================");
end

function printBest(sub, metricName, label)
    valid = isfinite(sub.(metricName));
    if ~any(valid)
        return;
    end
    [bestValue, idx] = min(sub.(metricName)(valid));
    validRows = find(valid);
    bestScale = sub.tcomp_scale(validRows(idx));
    fprintf("  - %s: scale=%.2f, %s=%.6f\n", label, bestScale, metricName, bestValue);
end

function row = parseFile(fileInfo)
    row = emptyMetaRow();
    row.path = string(fullfile(fileInfo.folder, fileInfo.name));

    tokens = regexp(fileInfo.name, "TCOMP_SCALE=([0-9.]+)_V([0-9]+)_D(5|11|12)\.csv", "tokens", "once");
    if isempty(tokens)
        return;
    end

    row.tcomp_scale = str2double(tokens{1});
    row.speed_cmd_rad_s = str2double(tokens{2});
    row.page = str2double(tokens{3});
end

function row = emptyMetaRow()
    row = struct( ...
        "path", "", ...
        "tcomp_scale", NaN, ...
        "speed_cmd_rad_s", NaN, ...
        "page", NaN);
end

function row = emptySummaryRow()
    row = struct( ...
        "speed_cmd_rad_s", NaN, ...
        "tcomp_scale", NaN, ...
        "delta_theta_deg_mean", NaN, ...
        "delta_theta_deg_std", NaN, ...
        "u_mag_d11_mean", NaN, ...
        "ud_mean_pu", NaN, ...
        "ud_abs_mean_pu", NaN, ...
        "ud_rms_pu", NaN, ...
        "uq_mean_pu", NaN, ...
        "uq_rms_pu", NaN, ...
        "u_mag_mean_pu", NaN, ...
        "u_mag_rms_pu", NaN, ...
        "omega_ref_mean", NaN, ...
        "omega_pll_mean", NaN, ...
        "omega_std", NaN, ...
        "omega_err_rms", NaN, ...
        "iq_ref_abs_mean", NaN, ...
        "iq_ref_rms", NaN, ...
        "iq_meas_abs_mean", NaN, ...
        "iq_meas_rms", NaN, ...
        "bench_current_a", NaN);
end

function out = calcRms(x)
    x = x(:);
    out = sqrt(mean(x .^ 2));
end
