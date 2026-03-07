% 1. 读取CSV文件 (替换为你的实际文件名)
filename = 'B=100_A=0.2_1HZ-100HZ_D=40s.csv';
data = readtable(filename);

% 2. 找到 sweep active 为 1 的索引
% 注意：MATLAB通常会将表头中的点 '.' 替换为下划线 '_'
valid_indices = data.iq_sweep_active == 1;

% 3. 抓取有效数据
omega_pll = data.dbg_omega_pll_rad_s(valid_indices);
iq_fb     = data.dbg_iq_a(valid_indices);
iq_cmd    = data.dbg_iq_cmd_a(valid_indices);

% 4. 简单绘图验证截取是否成功
figure;
subplot(3,1,1); plot(iq_cmd); title('Iq Command (Sweep Part)');
subplot(3,1,2); plot(iq_fb); title('Iq Feedback');
subplot(3,1,3); plot(omega_pll); title('Omega PLL (Feedback Speed)');