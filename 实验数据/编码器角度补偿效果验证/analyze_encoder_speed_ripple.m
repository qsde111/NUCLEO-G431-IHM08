clear;
clc;
close all;

cfg.dataDir = fileparts(mfilename("fullpath"));
cfg.filePattern = "*_D3.csv";
cfg.sampleInterval_s = 0.5e-3;
cfg.fs_Hz = 1 / cfg.sampleInterval_s;
cfg.polePairs = 7;
cfg.pllBandwidth_Hz = 130;
cfg.windowSec = 0.5;
cfg.windowOverlap = 0.75;
cfg.previewSec = 0.15;
cfg.freqMin_Hz = 1;
cfg.orderMax = 24;
cfg.numPeaks = 5;
cfg.minPeakSpacing_Hz = 3;
cfg.peakFloor_dB = 24;
cfg.saveFigures = true;
cfg.outputDir = fullfile(cfg.dataDir, "analysis_out");
cfg.useLinearDetrend = false;

files = dir(fullfile(cfg.dataDir, cfg.filePattern));
if isempty(files)
    error("No files matched %s in %s.", cfg.filePattern, cfg.dataDir);
end

if ~exist(cfg.outputDir, "dir")
    mkdir(cfg.outputDir);
end

summaryRows = repmat(emptySummaryRow(), numel(files), 1);
orderSpectra = repmat(struct("file", "", "bandwidth_Hz", NaN, "cmdOmega_rad_s", NaN, "order", [], "pllDb", [], "diffDb", []), numel(files), 1);

for k = 1:numel(files)
    filePath = fullfile(files(k).folder, files(k).name);
    result = analyzeOneFile(filePath, cfg);
    summaryRows(k) = result.summary;
    orderSpectra(k) = result.orderSpectrum;
end

summary = struct2table(summaryRows);
summary = sortrows(summary, ["bandwidth_Hz", "cmdOmega_rad_s"]);
writetable(summary, fullfile(cfg.outputDir, "summary.csv"));

makeSummaryFigure(summary, orderSpectra, cfg);
writeConsoleDiagnosis(summary, cfg);

disp(" ");
disp("Analysis finished.");
disp("Summary table:");
disp(summary);
disp("Output folder:");
disp(cfg.outputDir);

function result = analyzeOneFile(filePath, cfg)
    T = readtable(filePath, "VariableNamingRule", "preserve");
    diffCol = findColumn(T, ["omega_diff", "omegadiff", "diffspeed"]);
    pllCol = findColumn(T, ["omega_pll", "omegapll", "pllspeed"]);
    iqCol = findColumn(T, ["dbg_iq_a", "iq", "iq_a"]);

    xDiff = T.(diffCol);
    xPll = T.(pllCol);
    xIq = [];
    if ~isempty(iqCol)
        xIq = T.(iqCol);
    end

    valid = isfinite(xDiff) & isfinite(xPll);
    if ~isempty(xIq)
        valid = valid & isfinite(xIq);
    end
    xDiff = xDiff(valid);
    xPll = xPll(valid);
    if ~isempty(xIq)
        xIq = xIq(valid);
    end

    if numel(xDiff) < 64
        error("File %s does not have enough valid samples.", filePath);
    end

    t = (0:numel(xDiff)-1).' / cfg.fs_Hz;
    diffRipple = removeTrend(xDiff, cfg.useLinearDetrend);
    pllRipple = removeTrend(xPll, cfg.useLinearDetrend);

    specDiff = welchSpectrum(diffRipple, cfg.fs_Hz, cfg.windowSec, cfg.windowOverlap);
    specPll = welchSpectrum(pllRipple, cfg.fs_Hz, cfg.windowSec, cfg.windowOverlap);

    if ~isempty(xIq)
        iqRipple = removeTrend(xIq, cfg.useLinearDetrend);
        specIq = welchSpectrum(iqRipple, cfg.fs_Hz, cfg.windowSec, cfg.windowOverlap);
    else
        iqRipple = [];
        specIq = emptySpectrum();
    end

    cmdOmega = parseCommandOmega(filePath);
    pllBandwidth = parseBandwidthHz(filePath);
    meanDiff = mean(xDiff);
    meanPll = mean(xPll);
    meanOmegaAbs = max(abs(meanPll), abs(meanDiff));
    mechFreq_Hz = abs(meanOmegaAbs) / (2 * pi);
    elecFreq_Hz = cfg.polePairs * mechFreq_Hz;

    peaksDiff = findDominantPeaks(specDiff.f_Hz, specDiff.psd, cfg);
    peaksPll = findDominantPeaks(specPll.f_Hz, specPll.psd, cfg);
    if ~isempty(specIq.f_Hz)
        peaksIq = findDominantPeaks(specIq.f_Hz, specIq.psd, cfg);
        domFreqIq = peaksIq.freq_Hz(1);
    else
        domFreqIq = NaN;
    end

    domFreqDiff = peaksDiff.freq_Hz(1);
    domFreqPll = peaksPll.freq_Hz(1);

    diffRms = rms(diffRipple);
    pllRms = rms(pllRipple);
    iqRms = rmsOrNaN(iqRipple);

    diffLowRms = bandRms(specDiff.f_Hz, specDiff.psd, cfg.freqMin_Hz, cfg.pllBandwidth_Hz);
    diffHighRms = bandRms(specDiff.f_Hz, specDiff.psd, cfg.pllBandwidth_Hz, cfg.fs_Hz / 2);
    pllLowRms = bandRms(specPll.f_Hz, specPll.psd, cfg.freqMin_Hz, cfg.pllBandwidth_Hz);
    pllHighRms = bandRms(specPll.f_Hz, specPll.psd, cfg.pllBandwidth_Hz, cfg.fs_Hz / 2);

    pllAtDiffPeak = interpSpectrum(specPll.f_Hz, specPll.psd, domFreqDiff);
    diffAtDiffPeak = interpSpectrum(specDiff.f_Hz, specDiff.psd, domFreqDiff);
    attenuation_dB = 10 * log10((pllAtDiffPeak + eps) / (diffAtDiffPeak + eps));

    summary = emptySummaryRow();
    summary.file = string(getFileName(filePath));
    summary.bandwidth_Hz = pllBandwidth;
    summary.cmdOmega_rad_s = cmdOmega;
    summary.samples = numel(xDiff);
    summary.logFs_Hz = cfg.fs_Hz;
    summary.meanOmegaDiff_rad_s = meanDiff;
    summary.meanOmegaPll_rad_s = meanPll;
    summary.meanRpm = meanPll * 60 / (2 * pi);
    summary.mechFreq_Hz = mechFreq_Hz;
    summary.elecFreq_Hz = elecFreq_Hz;
    summary.diffRippleRms_rad_s = diffRms;
    summary.pllRippleRms_rad_s = pllRms;
    summary.iqRippleRms_A = iqRms;
    summary.diffRipplePct = 100 * diffRms / max(abs(meanOmegaAbs), eps);
    summary.pllRipplePct = 100 * pllRms / max(abs(meanOmegaAbs), eps);
    summary.domFreqDiff_Hz = domFreqDiff;
    summary.domFreqPll_Hz = domFreqPll;
    summary.domFreqIq_Hz = domFreqIq;
    summary.domOrderPll_Mech = domFreqPll / max(mechFreq_Hz, eps);
    summary.domOrderPll_Elec = domFreqPll / max(elecFreq_Hz, eps);
    summary.domOrderDiff_Mech = domFreqDiff / max(mechFreq_Hz, eps);
    summary.domOrderDiff_Elec = domFreqDiff / max(elecFreq_Hz, eps);
    summary.pllVsDiffAtDiffPeak_dB = attenuation_dB;
    summary.diffLowBandRms_rad_s = diffLowRms;
    summary.diffHighBandRms_rad_s = diffHighRms;
    summary.pllLowBandRms_rad_s = pllLowRms;
    summary.pllHighBandRms_rad_s = pllHighRms;

    result.summary = summary;
    result.orderSpectrum.file = summary.file;
    result.orderSpectrum.bandwidth_Hz = summary.bandwidth_Hz;
    result.orderSpectrum.cmdOmega_rad_s = summary.cmdOmega_rad_s;
    result.orderSpectrum.order = specPll.f_Hz / max(mechFreq_Hz, eps);
    result.orderSpectrum.pllDb = 10 * log10(specPll.psd + eps);
    result.orderSpectrum.diffDb = 10 * log10(specDiff.psd + eps);

    makeSingleFileFigure(filePath, t, diffRipple, pllRipple, specDiff, specPll, peaksDiff, peaksPll, summary, cfg);
end

function makeSingleFileFigure(filePath, t, diffRipple, pllRipple, specDiff, specPll, peaksDiff, peaksPll, summary, cfg)
    previewCount = min(numel(t), max(64, round(cfg.previewSec * cfg.fs_Hz)));
    orderAxis = specPll.f_Hz / max(summary.mechFreq_Hz, eps);
    orderMask = orderAxis <= cfg.orderMax;
    ratioDb = 10 * log10((specPll.psd + eps) ./ (specDiff.psd + eps));

    fig = figure("Color", "w", "Name", summary.file);
    tl = tiledlayout(fig, 2, 2, "Padding", "compact", "TileSpacing", "compact");
    title(tl, sprintf("%s | BW %.0f Hz | mean PLL = %.1f rad/s | mech = %.2f Hz | elec = %.2f Hz", ...
        summary.file, summary.bandwidth_Hz, summary.meanOmegaPll_rad_s, summary.mechFreq_Hz, summary.elecFreq_Hz), ...
        "Interpreter", "none");

    ax1 = nexttile(tl, 1);
    plot(ax1, t(1:previewCount), diffRipple(1:previewCount), "LineWidth", 1.0, "DisplayName", "omega\_diff ripple");
    hold(ax1, "on");
    plot(ax1, t(1:previewCount), pllRipple(1:previewCount), "LineWidth", 1.2, "DisplayName", "omega\_pll ripple");
    grid(ax1, "on");
    xlabel(ax1, "Time (s)");
    ylabel(ax1, "Ripple (rad/s)");
    legend(ax1, "Location", "best");
    title(ax1, sprintf("Time Preview (first %.3f s)", t(previewCount)));

    ax2 = nexttile(tl, 2);
    plot(ax2, specDiff.f_Hz, 10 * log10(specDiff.psd + eps), "LineWidth", 1.0, "DisplayName", "omega\_diff PSD");
    hold(ax2, "on");
    plot(ax2, specPll.f_Hz, 10 * log10(specPll.psd + eps), "LineWidth", 1.2, "DisplayName", "omega\_pll PSD");
    xline(ax2, cfg.pllBandwidth_Hz, "--", sprintf("PLL BW %.0f Hz", cfg.pllBandwidth_Hz), ...
        "Color", [0.30 0.30 0.30], "LabelVerticalAlignment", "bottom");
    if summary.mechFreq_Hz >= cfg.freqMin_Hz
        xline(ax2, summary.mechFreq_Hz, ":", sprintf("1x mech %.1f Hz", summary.mechFreq_Hz), ...
            "Color", [0.00 0.45 0.74], "LabelVerticalAlignment", "bottom");
    end
    if summary.elecFreq_Hz >= cfg.freqMin_Hz
        xline(ax2, summary.elecFreq_Hz, ":", sprintf("1x elec %.1f Hz", summary.elecFreq_Hz), ...
            "Color", [0.85 0.33 0.10], "LabelVerticalAlignment", "top");
    end
    scatter(ax2, peaksDiff.freq_Hz, 10 * log10(peaksDiff.power + eps), 28, "filled", "DisplayName", "diff peaks");
    scatter(ax2, peaksPll.freq_Hz, 10 * log10(peaksPll.power + eps), 28, "filled", "DisplayName", "pll peaks");
    grid(ax2, "on");
    xlim(ax2, [0, cfg.fs_Hz / 2]);
    xlabel(ax2, "Frequency (Hz)");
    ylabel(ax2, "PSD (dB rel. (rad/s)^2/Hz)");
    legend(ax2, "Location", "best");
    title(ax2, "Absolute Frequency Spectrum");

    ax3 = nexttile(tl, 3);
    plot(ax3, orderAxis(orderMask), 10 * log10(specDiff.psd(orderMask) + eps), "LineWidth", 1.0, "DisplayName", "omega\_diff");
    hold(ax3, "on");
    plot(ax3, orderAxis(orderMask), 10 * log10(specPll.psd(orderMask) + eps), "LineWidth", 1.2, "DisplayName", "omega\_pll");
    xlim(ax3, [0, cfg.orderMax]);
    grid(ax3, "on");
    xlabel(ax3, "Mechanical Order");
    ylabel(ax3, "PSD (dB)");
    legend(ax3, "Location", "best");
    title(ax3, sprintf("Order Spectrum | dom PLL order %.2f mech / %.2f elec", ...
        summary.domOrderPll_Mech, summary.domOrderPll_Elec));

    ax4 = nexttile(tl, 4);
    plot(ax4, specDiff.f_Hz, ratioDb, "LineWidth", 1.1, "Color", [0.20 0.20 0.20]);
    hold(ax4, "on");
    yline(ax4, 0, ":", "0 dB");
    xline(ax4, cfg.pllBandwidth_Hz, "--", sprintf("PLL BW %.0f Hz", cfg.pllBandwidth_Hz), ...
        "Color", [0.30 0.30 0.30], "LabelVerticalAlignment", "bottom");
    if isfinite(summary.domFreqDiff_Hz)
        xline(ax4, summary.domFreqDiff_Hz, ":", sprintf("dom diff %.1f Hz", summary.domFreqDiff_Hz), ...
            "Color", [0.00 0.45 0.74], "LabelVerticalAlignment", "bottom");
    end
    grid(ax4, "on");
    xlim(ax4, [0, cfg.fs_Hz / 2]);
    xlabel(ax4, "Frequency (Hz)");
    ylabel(ax4, "PLL / Diff PSD (dB)");
    title(ax4, sprintf("Observer Filtering | peak attenuation %.1f dB", summary.pllVsDiffAtDiffPeak_dB));

    if cfg.saveFigures
        [~, baseName] = fileparts(filePath);
        baseName = string(baseName);
        exportgraphics(fig, fullfile(cfg.outputDir, baseName + "_analysis.png"), "Resolution", 180);
    end
end

function makeSummaryFigure(summary, orderSpectra, cfg)
    fig = figure("Color", "w", "Name", "speed_ripple_summary");
    tl = tiledlayout(fig, 2, 2, "Padding", "compact", "TileSpacing", "compact");
    title(tl, "Cross-speed speed-ripple diagnosis by PLL bandwidth");

    bwList = unique(summary.bandwidth_Hz(~isnan(summary.bandwidth_Hz)));
    bwList = sort(bwList(:).');
    cmap = turbo(max(numel(bwList), 2));

    ax1 = nexttile(tl, 1);
    hold(ax1, "on");
    for k = 1:numel(bwList)
        mask = summary.bandwidth_Hz == bwList(k);
        sub = sortrows(summary(mask, :), "cmdOmega_rad_s");
        plot(ax1, sub.cmdOmega_rad_s, sub.pllRipplePct, "-o", "LineWidth", 1.2, ...
            "Color", cmap(k, :), "DisplayName", sprintf("BW %.0f Hz", bwList(k)));
    end
    grid(ax1, "on");
    xlabel(ax1, "Command speed (rad/s)");
    ylabel(ax1, "PLL ripple RMS / mean speed (%)");
    legend(ax1, "Location", "best");
    title(ax1, "PLL Ripple vs Speed");

    ax2 = nexttile(tl, 2);
    hold(ax2, "on");
    allBySpeed = sortrows(summary, "cmdOmega_rad_s");
    plot(ax2, allBySpeed.cmdOmega_rad_s, allBySpeed.mechFreq_Hz, ":", "LineWidth", 1.0, ...
        "Color", [0.25 0.25 0.25], "DisplayName", "1x mech");
    plot(ax2, allBySpeed.cmdOmega_rad_s, allBySpeed.elecFreq_Hz, "--", "LineWidth", 1.0, ...
        "Color", [0.45 0.45 0.45], "DisplayName", "1x elec");
    for k = 1:numel(bwList)
        mask = summary.bandwidth_Hz == bwList(k);
        sub = sortrows(summary(mask, :), "cmdOmega_rad_s");
        plot(ax2, sub.cmdOmega_rad_s, sub.domFreqPll_Hz, "-o", "LineWidth", 1.2, ...
            "Color", cmap(k, :), "DisplayName", sprintf("dom PLL | BW %.0f", bwList(k)));
    end
    grid(ax2, "on");
    xlabel(ax2, "Command speed (rad/s)");
    ylabel(ax2, "Dominant frequency (Hz)");
    legend(ax2, "Location", "best");
    title(ax2, "Dominant PLL Ripple Frequency");

    ax3 = nexttile(tl, 3);
    hold(ax3, "on");
    for k = 1:numel(bwList)
        mask = summary.bandwidth_Hz == bwList(k);
        sub = sortrows(summary(mask, :), "cmdOmega_rad_s");
        plot(ax3, sub.cmdOmega_rad_s, sub.pllVsDiffAtDiffPeak_dB, "-o", "LineWidth", 1.2, ...
            "Color", cmap(k, :), "DisplayName", sprintf("BW %.0f Hz", bwList(k)));
    end
    yline(ax3, 0, ":");
    grid(ax3, "on");
    xlabel(ax3, "Command speed (rad/s)");
    ylabel(ax3, "PLL / Diff at diff peak (dB)");
    title(ax3, "How much the PLL suppresses the dominant diff ripple");
    legend(ax3, "Location", "best");

    ax4 = nexttile(tl, 4);
    speedList = unique(summary.cmdOmega_rad_s(~isnan(summary.cmdOmega_rad_s)));
    speedList = sort(speedList(:).');
    rippleGrid = nan(numel(bwList), numel(speedList));
    for ibw = 1:numel(bwList)
        for isp = 1:numel(speedList)
            idx = summary.bandwidth_Hz == bwList(ibw) & summary.cmdOmega_rad_s == speedList(isp);
            if any(idx)
                rippleGrid(ibw, isp) = summary.pllRipplePct(find(idx, 1, "first"));
            end
        end
    end
    imagesc(ax4, speedList, bwList, rippleGrid);
    set(ax4, "YDir", "normal");
    xlabel(ax4, "Command speed (rad/s)");
    ylabel(ax4, "PLL bandwidth (Hz)");
    title(ax4, "PLL ripple RMS / mean speed (%) heatmap");
    cb = colorbar(ax4);
    ylabel(cb, "Ripple (%)");
    for ibw = 1:numel(bwList)
        for isp = 1:numel(speedList)
            if isfinite(rippleGrid(ibw, isp))
                text(ax4, speedList(isp), bwList(ibw), sprintf("%.2f", rippleGrid(ibw, isp)), ...
                    "HorizontalAlignment", "center", "Color", "w", "FontSize", 8);
            end
        end
    end

    if cfg.saveFigures
        exportgraphics(fig, fullfile(cfg.outputDir, "summary_overview.png"), "Resolution", 180);
    end
end

function writeConsoleDiagnosis(summary, cfg)
    domPll = summary.domFreqPll_Hz;
    mechHz = summary.mechFreq_Hz;
    elecHz = summary.elecFreq_Hz;
    bwList = unique(summary.bandwidth_Hz(~isnan(summary.bandwidth_Hz)));
    bwList = sort(bwList(:).');

    [kMech, r2Mech] = fitThroughOrigin(mechHz, domPll);
    [kElec, r2Elec] = fitThroughOrigin(elecHz, domPll);
    iqMatch = mean(abs(summary.domFreqIq_Hz - summary.domFreqPll_Hz) ./ max(summary.domFreqPll_Hz, 1), "omitnan");

    disp(" ");
    disp("========== D3 speed-ripple diagnosis ==========");
    fprintf("Log sample rate           : %.1f Hz\n", cfg.fs_Hz);
    fprintf("Configured PLL bandwidths : %s Hz\n", numArrayToString(bwList));
    fprintf("Dominant PLL ripple ~ mech: order %.2f, R^2 = %.4f\n", kMech, r2Mech);
    fprintf("Dominant PLL ripple ~ elec: order %.2f, R^2 = %.4f\n", kElec, r2Elec);
    fprintf("Mean PLL ripple RMS ratio : %.2f %% of mean speed\n", mean(summary.pllRipplePct, "omitnan"));
    fprintf("Mean diff->PLL attenuation: %.2f dB at diff dominant peak\n", mean(summary.pllVsDiffAtDiffPeak_dB, "omitnan"));
    disp("Per-bandwidth summary:");
    for k = 1:numel(bwList)
        mask = summary.bandwidth_Hz == bwList(k);
        fprintf("- BW %.0f Hz: mean ripple %.2f %% | high-speed(>=100) mean ripple %.2f %% | mean attenuation %.2f dB\n", ...
            bwList(k), ...
            mean(summary.pllRipplePct(mask), "omitnan"), ...
            mean(summary.pllRipplePct(mask & summary.cmdOmega_rad_s >= 100), "omitnan"), ...
            mean(summary.pllVsDiffAtDiffPeak_dB(mask), "omitnan"));
    end

    if r2Elec >= 0.90 || r2Mech >= 0.90
        disp("Diagnosis:");
        disp("- The dominant ripple scales with speed, so it is more likely an order-related disturbance than a fixed PLL oscillation.");
        if r2Elec > r2Mech
            fprintf("- It matches electrical order better (about %.2f x electrical frequency).\n", kElec);
            disp("- First priority: keep improving encoder angle compensation and inspect electrical-periodic sources such as magnetic asymmetry, cogging/current ripple, phase-current sampling, and commutation harmonics.");
        else
            fprintf("- It matches mechanical order better (about %.2f x mechanical frequency).\n", kMech);
            disp("- First priority: inspect mechanical 1x/2x effects such as encoder eccentricity, shaft runout, mounting tilt, and residual angle LUT error.");
        end
        disp("- Lowering PLL bandwidth only suppresses the reported ripple above the new bandwidth. It does not remove the underlying source.");
    else
        disp("Diagnosis:");
        disp("- The dominant ripple does not scale strongly with speed. That is more consistent with PLL tuning, loop interaction, or a fixed-frequency artifact.");
        disp("- In that case, retuning the observer bandwidth/damping is a valid first move.");
    end

    if numel(bwList) >= 2
        [bestOverall, bestHighSpeed] = summarizeBandwidthTradeoff(summary, bwList);
        fprintf("- Lowest mean overall ripple in this data set: BW %.0f Hz.\n", bestOverall);
        fprintf("- Lowest mean high-speed (>=100 rad/s) ripple in this data set: BW %.0f Hz.\n", bestHighSpeed);
    end

    if mean(summary.pllRipplePct, "omitnan") < 1.0
        disp("- The PLL ripple is below 1% RMS of mean speed on average. This may look obvious in time domain but often does not justify more filtering unless it disturbs the speed loop.");
    end

    if ~isnan(iqMatch) && iqMatch < 0.20
        disp("- The dominant PLL ripple is close to the dominant Iq ripple in several files, so some of the ripple may be physically real rather than pure estimation noise.");
    else
        disp("- The dominant PLL ripple does not consistently line up with Iq ripple, so measurement/estimation artifacts remain the more likely cause.");
    end

    if mean(summary.domFreqPll_Hz < 0.8 * cfg.pllBandwidth_Hz, "omitnan") > 0.5
        disp("- Most dominant PLL ripple components sit inside the current 130 Hz observer passband. Lowering bandwidth can hide them, but it will directly add feedback lag.");
    else
        disp("- A large part of the dominant ripple is already near or above 130 Hz, so the present PLL is mainly acting as a smoother, not the source.");
    end
    disp("===============================================");
end

function y = removeTrend(x, useLinearDetrend)
    x = x(:);
    if useLinearDetrend
        y = detrend(x);
    else
        y = x - mean(x);
    end
end

function spec = welchSpectrum(x, fs_Hz, windowSec, overlap)
    x = x(:);
    n = numel(x);
    winLen = round(windowSec * fs_Hz);
    winLen = min(winLen, n);
    winLen = 2 ^ floor(log2(max(winLen, 64)));
    winLen = min(winLen, n);

    if winLen < 32
        winLen = n;
    end

    overlapSamples = min(winLen - 1, round(overlap * winLen));
    step = max(1, winLen - overlapSamples);
    starts = 1:step:(n - winLen + 1);
    if isempty(starts)
        starts = 1;
        winLen = n;
    end

    nfft = 2 ^ nextpow2(winLen);
    w = hannLocal(winLen);
    u = sum(w .^ 2) / winLen;
    halfCount = floor(nfft / 2) + 1;
    psd = zeros(halfCount, 1);

    for idx = starts
        seg = x(idx:idx + winLen - 1);
        seg = seg - mean(seg);
        seg = seg .* w;
        X = fft(seg, nfft);
        P = (abs(X(1:halfCount)) .^ 2) / (fs_Hz * winLen * u);
        if rem(nfft, 2) == 0
            P(2:end-1) = 2 * P(2:end-1);
        else
            P(2:end) = 2 * P(2:end);
        end
        psd = psd + P;
    end

    psd = psd / numel(starts);
    spec.f_Hz = (0:halfCount-1).' * fs_Hz / nfft;
    spec.psd = psd;
end

function peaks = findDominantPeaks(f_Hz, psd, cfg)
    peaks.freq_Hz = NaN(cfg.numPeaks, 1);
    peaks.power = NaN(cfg.numPeaks, 1);

    valid = isfinite(f_Hz) & isfinite(psd) & (f_Hz >= cfg.freqMin_Hz);
    f = f_Hz(valid);
    p = psd(valid);
    if isempty(f)
        return;
    end

    idx = find(p(2:end-1) >= p(1:end-2) & p(2:end-1) > p(3:end)) + 1;
    if isempty(idx)
        [~, idxMax] = max(p);
        idx = idxMax;
    end

    peakFloor = max(p) / (10 ^ (cfg.peakFloor_dB / 10));
    idx = idx(p(idx) >= peakFloor);
    if isempty(idx)
        [~, idxMax] = max(p);
        idx = idxMax;
    end

    [~, order] = sort(p(idx), "descend");
    idxSorted = idx(order);
    selected = [];

    for ii = idxSorted.'
        if isempty(selected) || all(abs(f(ii) - f(selected)) >= cfg.minPeakSpacing_Hz)
            selected(end+1) = ii; %#ok<AGROW>
        end
        if numel(selected) >= cfg.numPeaks
            break;
        end
    end

    selected = selected(:);
    if isempty(selected)
        [~, idxMax] = max(p);
        selected = idxMax;
    end

    nSel = numel(selected);
    peaks.freq_Hz(1:nSel) = f(selected);
    peaks.power(1:nSel) = p(selected);
end

function val = interpSpectrum(f_Hz, psd, target_Hz)
    if isempty(f_Hz) || ~isfinite(target_Hz)
        val = NaN;
        return;
    end
    val = interp1(f_Hz, psd, target_Hz, "linear", "extrap");
end

function out = bandRms(f_Hz, psd, f1_Hz, f2_Hz)
    mask = (f_Hz >= f1_Hz) & (f_Hz <= f2_Hz);
    if nnz(mask) < 2
        out = NaN;
        return;
    end
    out = sqrt(trapz(f_Hz(mask), psd(mask)));
end

function w = hannLocal(n)
    if n <= 1
        w = ones(n, 1);
        return;
    end
    k = (0:n-1).';
    w = 0.5 - 0.5 * cos(2 * pi * k / (n - 1));
end

function colName = findColumn(T, aliases)
    names = string(T.Properties.VariableNames);
    keys = normalizeNames(names);
    aliasKeys = normalizeNames(aliases);
    colName = "";
    for k = 1:numel(aliasKeys)
        idx = find(keys == aliasKeys(k), 1, "first");
        if ~isempty(idx)
            colName = names(idx);
            return;
        end
    end
    if isempty(colName)
        error("Could not find any matching column for aliases: %s", strjoin(cellstr(aliases), ", "));
    end
end

function keys = normalizeNames(names)
    keys = lower(string(names));
    keys = replace(keys, "_", "");
    keys = replace(keys, " ", "");
    keys = replace(keys, "-", "");
end

function omegaCmd = parseCommandOmega(filePath)
    [~, name] = fileparts(filePath);
    token = regexp(name, "V[^0-9+-]*([-+]?\d+(?:\.\d+)?)_D3", "tokens", "once");
    if isempty(token)
        omegaCmd = NaN;
    else
        omegaCmd = str2double(token{1});
    end
end

function bwHz = parseBandwidthHz(filePath)
    [~, name] = fileparts(filePath);
    token = regexp(name, "BD([0-9]+(?:\.[0-9]+)?)", "tokens", "once");
    if isempty(token)
        bwHz = NaN;
    else
        bwHz = str2double(token{1});
    end
end

function out = getFileName(filePath)
    [~, name, ext] = fileparts(filePath);
    out = string(name) + string(ext);
end

function [k, r2] = fitThroughOrigin(x, y)
    x = x(:);
    y = y(:);
    valid = isfinite(x) & isfinite(y) & (abs(x) > eps);
    x = x(valid);
    y = y(valid);

    if numel(x) < 2
        k = NaN;
        r2 = NaN;
        return;
    end

    k = (x' * y) / (x' * x);
    yHat = k * x;
    sse = sum((y - yHat) .^ 2);
    sst0 = sum(y .^ 2);
    if sst0 <= eps
        r2 = 1;
    else
        r2 = 1 - sse / sst0;
    end
end

function out = rmsOrNaN(x)
    if isempty(x)
        out = NaN;
    else
        out = rms(x);
    end
end

function out = numArrayToString(x)
    if isempty(x)
        out = "n/a";
        return;
    end
    out = strjoin(compose("%.0f", x), ", ");
end

function [bestOverall, bestHighSpeed] = summarizeBandwidthTradeoff(summary, bwList)
    meanOverall = nan(size(bwList));
    meanHigh = nan(size(bwList));
    for k = 1:numel(bwList)
        mask = summary.bandwidth_Hz == bwList(k);
        meanOverall(k) = mean(summary.pllRipplePct(mask), "omitnan");
        meanHigh(k) = mean(summary.pllRipplePct(mask & summary.cmdOmega_rad_s >= 100), "omitnan");
    end

    [~, idxOverall] = min(meanOverall);
    [~, idxHigh] = min(meanHigh);
    bestOverall = bwList(idxOverall);
    bestHighSpeed = bwList(idxHigh);
end

function spec = emptySpectrum()
    spec.f_Hz = [];
    spec.psd = [];
end

function row = emptySummaryRow()
    row = struct( ...
        "file", "", ...
        "bandwidth_Hz", NaN, ...
        "cmdOmega_rad_s", NaN, ...
        "samples", NaN, ...
        "logFs_Hz", NaN, ...
        "meanOmegaDiff_rad_s", NaN, ...
        "meanOmegaPll_rad_s", NaN, ...
        "meanRpm", NaN, ...
        "mechFreq_Hz", NaN, ...
        "elecFreq_Hz", NaN, ...
        "diffRippleRms_rad_s", NaN, ...
        "pllRippleRms_rad_s", NaN, ...
        "iqRippleRms_A", NaN, ...
        "diffRipplePct", NaN, ...
        "pllRipplePct", NaN, ...
        "domFreqDiff_Hz", NaN, ...
        "domFreqPll_Hz", NaN, ...
        "domFreqIq_Hz", NaN, ...
        "domOrderPll_Mech", NaN, ...
        "domOrderPll_Elec", NaN, ...
        "domOrderDiff_Mech", NaN, ...
        "domOrderDiff_Elec", NaN, ...
        "pllVsDiffAtDiffPeak_dB", NaN, ...
        "diffLowBandRms_rad_s", NaN, ...
        "diffHighBandRms_rad_s", NaN, ...
        "pllLowBandRms_rad_s", NaN, ...
        "pllHighBandRms_rad_s", NaN);
end
