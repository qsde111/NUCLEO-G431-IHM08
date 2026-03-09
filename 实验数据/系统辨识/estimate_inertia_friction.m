% clc;
clear;
close all;
% === 1. 读取并提取有效数据 ===
filename = 'I_B=0.3_A=0.1_1-50HZ_D=20s.csv'; % 替换为你的文件名
data = readtable(filename);
valid_indices = data.iq_sweep_active == 1;

% 提取输入输出信号 (记得去均值，消除稳态直流偏置的影响)
% 使用 detrend 函数去掉稳态工作点，让系统在 0 附近做纯动态辨识
omega_out = detrend(data.omega_pll_rad_s(valid_indices)); 
iq_in     = detrend(data.dbg_iq_a(valid_indices));            

% === 2. 构建辨识数据对象 ===
% 【重要】你需要知道你的记录采样周期 Ts 是多少秒！
% 例如你的速度环是 1kHz，那么 Ts = 0.001
Ts = 0.0005;  % <--- 请务必修改为你实际的采样周期！！！

% iddata(输出, 输入, 采样时间)
my_data = iddata(omega_out, iq_in, Ts); 

% === 3. 使用 tfest 估计一阶传递函数 ===
% 我们已知机械系统是 1 个极点 (J s + B)，0 个零点
np = 1; % 极点数量
nz = 0; % 零点数量
sys_est = tfest(my_data, np, nz);

% 显示辨识结果及其拟合度 (Fit percent越高越好，一般大于 70% 就算可用)
disp('辨识得到的传递函数为：');
sys_est

% 绘制波特图，查看辨识出来的频域特性
figure;
bode(sys_est);
grid on;
title('辨识出的机械系统波特图 \omega(s) / Iq(s)');

% === 4. 从传递函数中反推物理参数 J 和 B ===
% 获取分子分母系数： sys_est 形式为 num / (s + den)
[num, den] = tfdata(sys_est, 'v'); 

K_gain = num(2); % 分子常数项
P_pole = den(2); % 分母 s 的系数通常被归一化为1，这里取常数项

Kt = 0.00415; % 反电动势常数/力矩常数 (Nm/A)

% 计算惯量和摩擦系数
J = Kt / K_gain;
B = P_pole * J;

fprintf('===================================\n');
fprintf('计算得到的电机机械参数：\n');
fprintf('转动惯量 J = %e (kg*m^2)\n', J);
fprintf('粘滞摩擦 B = %e (Nm*s/rad)\n', B);
fprintf('===================================\n');