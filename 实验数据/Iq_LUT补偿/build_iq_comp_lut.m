% 按机械角度(全周)生成 Iq 补偿 LUT（用于抑制角度同步的转矩/测速纹波）
%
% 期望的 CSV 列（推荐用固件 D7 页采集）：
%   raw21, omega_pll, Iq_ref, Iq_meas
% 也支持用 pos_mech_rad 代替 raw21（精度略差，但可用）。
%
% 输出：
%   - 角度分桶后的 Iq_ripple LUT（零均值）
%   - 可选：做谐波域滤波（保留指定阶次或截止阶次）
%   - 导出 C 头文件（float 数组）

clear; clc; % close all;

%% ===== 用户配置 =====
filename = 'V=100_D7.csv';  % 替换为你的文件名
N = 1024;                 % LUT 点数（建议 1024 起步，必要时 2048）

match = regexp(filename, 'V=([\d.]+)', 'tokens');

if ~isempty(match)
    % match{1}{1} 提取出来的是字符 '100'，用 str2double 转为数字 100
    V_value = str2double(match{1}{1}); 
    fprintf('成功提取 V 的值为: %.2f\n', V_value);
else
    error('文件名格式不正确，未找到 V= 值');
end

skip_seconds = 0.0;       % 丢掉前面若干秒的过渡段（按采样率估算）
Fs = 2000;                % 采样率（和上位机记录一致：2kHz 或 5kHz）

iq_source = 'Iq_ref';     % 'Iq_ref' 或 'Iq_meas'（通常用 Iq_ref）

% 'none'：不滤波
% 'hmax'：保留 0..Hmax 的谐波（更平滑）
% 'orders'：只保留指定阶次（更干净，适合只想补 2/4/12/14/84 这类）
filter_mode = 'hmax';
Hmax = 200;               % filter_mode='hmax' 时有效（N=1024 下 200 阶足够覆盖 84 阶）
orders = [2 4 12 14 36];  % filter_mode='orders' 时有效

export_c_header = false;
out_header = sprintf('iq_comp_lut_v%d.h', V_value);
array_name = sprintf('iq_comp_lut_a_v%d',V_value);
%% ===== 读数据 =====
data = readtable(filename);

% vars 里面存了 CSV 文件的所有表头列名
vars = data.Properties.VariableNames;

% 选角度输入：raw21 优先
use_raw21 = ismember('raw21', vars);    %查vars中表头列名、有返回1(true)，无返回0(false)
use_pos = ismember('pos_mech_rad', vars);
if ~(use_raw21 || use_pos)
    error('CSV 需要包含 raw21 或 pos_mech_rad 列。');
end

% 选 Iq 源
if ismember(iq_source, vars)
    iq = data.(iq_source);
else
    error('CSV 缺少列：%s', iq_source);
end

% 丢掉过渡段
skip_n = max(0, round(skip_seconds * Fs));
if skip_n >= height(data)
    error('skip_seconds 太大：数据长度不足。');
end
iq = iq(skip_n+1:end);

if use_raw21
    raw21 = data.raw21(skip_n+1:end);
    
     % raw21 在 float 里也可精确表示到 2^21，将读取的浮点数强制转换为无符号32位整型 %
     % 编码器（比如 MT6835 之类的高分辨率磁编）输出的原始数据是整数，
     % 但在 MATLAB 读入 CSV 时默认是 double 浮点数，这里要把它转回纯整数。
    raw21 = uint32(raw21);
    bits = round(log2(N));  % 计算 N 是 2 的多少次方，round()四舍五入函数
    if (2^bits) ~= N
        warning('N 不是 2 的幂；将使用除法分桶，速度慢一些。');
        idx = floor(double(raw21) / (2^21 / N)) + 1; % 1..N
    else
        % 如果 N 是 2 的幂次（比如 1024），用移位操作，更优雅且符合底层逻辑
        shift = 21 - bits;

        % 整数的右移操作是直接丢弃低位，不会产生小数
        % MATLAB 默认所有的数字都是双精度浮点数，后续用的accumarray()要求传入的数据类型必须是浮点型或特定的整型
        idx = double(bitshift(raw21, -shift)) + 1; % 1..N，二进制`raw21 / 2^11`等价于`raw21 >> 11`
        % MATLAB 的数组下标是从 1 开始的。所以要在算出的索引上加 1，保证桶的编号是 1 到 1024
    end
else
    pos = data.pos_mech_rad(skip_n+1:end);
    two_pi = 2*pi;
    pos = mod(pos, two_pi);
    idx = floor((pos / two_pi) * N) + 1;
end

% 防御：把边界压回 1..N
idx(idx < 1) = 1;
idx(idx > N) = N;

%% ===== 按角度分桶求平均 =====
% 把iq数组中与idx一一对应的每个值，放进它对应的[1, N]的区间中，然后再把本区间中全部的iq取均值
% [N 1]防止iq最后几位不在第1024位的区间导致accumarray()函数返回长度小于N的数组
% @mean-取均值 NaN-空区间返回NaN
iq_mean_bin = accumarray(idx, iq, [N 1], @mean, NaN);
% NaN 补齐（一般不会出现，除非数据太短或转速变化大）
if any(isnan(iq_mean_bin))  % isnan 会检查数组里哪些位置是 NaN；any 判断是不是存在至少一个 NaN
    x = (1:N).';
    nan_mask = isnan(iq_mean_bin);  % 一维插值函数，将NaN的空桶左右有数据的桶，估算NaN空桶数据并补齐
    iq_mean_bin(nan_mask) = interp1(x(~nan_mask), iq_mean_bin(~nan_mask), x(nan_mask), 'linear', 'extrap');
end

% 去直流：只保留周期纹波
iq_lut = iq_mean_bin - mean(iq_mean_bin);

%% ===== 可选：谐波域滤波/重建 =====
switch lower(filter_mode)   % lower() 把字符串全变成小写
    case 'none'
        iq_lut_f = iq_lut;
    case 'hmax'
        % 对iq_lut位置相关的数组做FFT，输出位置的阶次-电机转一圈，它振动几次
        % 第一个点是直流(0阶)，第13个点代表电机转1圈振动12次(对应定子齿槽转矩的影响)
        Y = fft(iq_lut);
        Yf = zeros(size(Y));    % zeros()生成全零数组
        H = min(Hmax, floor(N/2)-1);
        Yf(1:H+1) = Y(1:H+1);
        Yf(end-H+1:end) = Y(end-H+1:end);

        % ifft()-Inverse FFT，计算机浮点数的精度误差ifft()后的小数可能带极小的虚数尾巴，用real()去除
        iq_lut_f = real(ifft(Yf));  
    case 'orders'
        Y = fft(iq_lut);
        Yf = zeros(size(Y));
        for k = orders  % 遍历orders，例如 orders = [2 4 12 14 84];只会for循环5次 % 
            if (k >= 0) && (k <= floor(N/2)-1)
                Yf(k+1) = Y(k+1);
                if k ~= 0   % 不处理直流分量
                    Yf(N-k+1) = Y(N-k+1);
                end
            end
        end
        iq_lut_f = real(ifft(Yf));
    otherwise
        error('未知 filter_mode: %s', filter_mode);
end

%% ===== 可视化 =====
% 0:N-1：产生一个从 0 到 1023 的行向量 [0, 1, 2, ..., 1023]。
% .'：转置符号，把横着的行向量，竖起来变成列向量
% 由于使用accumarray()，iq_lut已经强制变为列向量，将 theta 制作位列向量后续绘图时作横轴
% 把机械角度 0 ~ 2pi 切成 1024 份
theta = (0:N-1).' * (2*pi/N);
figure('Name', 'Iq LUT (angle-binned)', 'Position', [100, 100, 1100, 700]);
subplot(2,1,1);     % 第一个 2 分行，第一个 1 分列，最后的 1 表示在第几个格子
plot(theta, iq_lut, 'Color', [0.6 0.6 0.6]); hold on;
plot(theta, iq_lut_f, 'b', 'LineWidth', 1.2);
grid on;
xlabel('\theta_{mech} (rad)');
ylabel('Iq ripple (A)');
legend('raw binned', 'filtered', 'Location', 'best');
title(sprintf('LUT N=%d, source=%s, filter=%s, Vel=%d', N, iq_source, filter_mode, V_value));

subplot(2,1,2);
Y = fft(iq_lut - mean(iq_lut));
P2 = abs(Y / N);    % 做 FFT 后，算出来的数值会被放大 N 倍，所以要除以 N 还原出真实的物理幅值
P1 = P2(1:floor(N/2)+1);    % 后半段（514~1024）只是前半段的镜像，画图时只看前半段
P1(2:end-1) = 2 * P1(2:end-1);  % 能量补偿，砍掉后半段频率图，用 *2 将砍掉的幅值补偿回来
harm = (0:floor(N/2)).';
stem(harm, P1, 'Marker', 'none');   % stem()画火柴棍图，会在每个坐标上竖起一根杆子
grid on;
xlabel('harmonic order (per mech rev)');
ylabel('|FFT|');
xlim([0, min(200, floor(N/2))]);
title('Harmonics of binned Iq ripple');

%% ===== 导出 C 头文件 =====
if export_c_header
    fid = fopen(out_header, 'w');   % 在电脑硬盘上，以写模式（'w'）新建一个文件
    if fid < 0
        error('无法写入文件：%s', out_header);
    end
    guard = upper(regexprep(out_header, '[^a-zA-Z0-9]', '_'));
    fprintf(fid, '#ifndef %s\n', guard);    % 把字符串格式化输出到 fid 所指向的文件里
    fprintf(fid, '#define %s\n\n', guard);
    fprintf(fid, '#define IQ_COMP_LUT_SIZE (%dU)\n\n', N);

    fprintf(fid, 'static const float %s[IQ_COMP_LUT_SIZE] =\n{\n', array_name);
    
    for i = 1:N
        if mod(i-1, 8) == 0
            fprintf(fid, '    ');
        end
        fprintf(fid, '%.9gf', iq_lut_f(i));
        if i ~= N
            fprintf(fid, ', ');
        end
        if mod(i, 8) == 0 || i == N
            fprintf(fid, '\n');
        end
    end
    fprintf(fid, '};\n\n');
    fprintf(fid, '#endif /* %s */\n', guard);
    fclose(fid);
    fprintf('Exported: %s\n', out_header);
end

