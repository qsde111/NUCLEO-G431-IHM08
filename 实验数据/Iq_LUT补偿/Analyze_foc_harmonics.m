% FOC 速度环数据谐波分析脚本
% 采样频率：2000 Hz (0.5 ms)
% 数据长度：50000 点

clear; clc; % close all;

% Auto-discover D7 files and parse Vcmd from filename.
files = dir('V=300_D7.csv');
if isempty(files)
    error('No V=*_D7.csv files found in current folder.');
end

filenames = {files.name};
V_refs = zeros(size(filenames));
for i = 1:numel(filenames)
    tok = regexp(filenames{i}, 'V=([\d.]+)_D7\.csv', 'tokens', 'once');
    if isempty(tok)
        error('Failed to parse Vcmd from filename: %s', filenames{i});
    end
    V_refs(i) = str2double(tok{1});
end

[V_refs, order_idx] = sort(V_refs, 'ascend');
filenames = filenames(order_idx);

% 采样参数
Ts = 0.0005;            % 采样周期 0.5ms
Fs = 1 / Ts;            % 采样频率 2000Hz
L = 50000;              % 数据长度

% Create one figure and auto layout subplots.
n_files = numel(filenames);
n_cols = ceil(sqrt(n_files));
n_rows = ceil(n_files / n_cols);
figure('Name', '反馈速度(omega_pll) FFT 频谱分析', 'Position', [100, 100, 1200, 800]);

for i = 1:length(filenames)
    filename = filenames{i};
    
    % 读取CSV数据
    % 假设CSV第一行为表头: omega_ref,omega_pll,Iq_ref,Iq_meas
    opts = detectImportOptions(filename);
    data = readtable(filename, opts);
    
    % 提取反馈速度信号
    % omega_pll = data.Iq_ref;
    omega_pll = data.omega_pll;
    
    % 去除直流偏置（去除平均值），只分析波动（谐波）成分
    omega_ac = omega_pll - mean(omega_pll);
    
    % 执行 FFT
    Y = fft(omega_ac);
    
    % 计算双边频谱 P2 和 单边频谱 P1
    P2 = abs(Y / L);
    P1 = P2(1 : floor(L/2)+1);
    P1(2 : end-1) = 2 * P1(2 : end-1);
    
    % 构造频率轴
    f = Fs * (0 : floor(L/2)) / L;
    
    % 绘制频谱子图
    subplot(n_rows, n_cols, i);
    plot(f, P1, 'b', 'LineWidth', 1.2);
    
    % 计算基频（机械频率）
    % 机械角速度 W = V_refs(i) rad/s，则机械频率 fm = W / (2*pi) Hz
    fm = V_refs(i) / (2 * pi);
    
    title(sprintf('给定速度 V=%d rad/s (机械基频~%.2f Hz)', V_refs(i), fm));
    xlabel('频率 (Hz)');
    ylabel('幅值 (rad/s)');
    
    % 限制 X 轴显示范围，通常机械偏心或电气谐波主要集中在低频段(0~100Hz)
    xlim([0, max(fm*10, 50)]); 
    grid on;
    
    % 自动在图上标出最高的谐波峰值频率
    [max_amp, max_idx] = max(P1);
    hold on;
    plot(f(max_idx), max_amp, 'ro', 'MarkerSize', 6, 'LineWidth', 2);
    text(f(max_idx) + 1, max_amp, sprintf('%.2f Hz', f(max_idx)), 'Color', 'r');
end
