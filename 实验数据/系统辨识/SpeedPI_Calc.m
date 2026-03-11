% 速度环 PI 参数计算脚本
clear; % clc;

% --- 1. 填入你辨识出的系统参数 ---
J  = 1.74e-06;      % 转动惯量 (kg*m^2)
B  = 5.00e-06;      % 粘滞摩擦 (Nm*s/rad)
Kt = 0.00415;       % 力矩常数 (Nm/A)

fprintf('====== 电机参数确认 ======\n');
fprintf('转动惯量 J = %e\n', J);
fprintf('粘滞摩擦 B = %e\n', B);
fprintf('力矩常数 Kt= %f\n\n', Kt);

% --- 2. 选择整定方法 ---
disp('请选择 PI 整定方法:');
disp('  1 - 一阶带宽法 (工业最常用, 推荐)');
disp('  2 - 二阶极点配置法 (抗扰动好)');
disp('  3 - 零极点相消法 (带 B 计算, K_i偏小)');
method = input('输入选择 (1/2/3): ');

fprintf('\n====== 计算结果 ======\n');

if method == 1
    % --- 方法 1：一阶带宽法 ---
    bw_hz = input('请输入期望的速度环带宽 (Hz, 例如 20-40): ');
    omega_bw = bw_hz * 2 * pi; % 转换为 rad/s
    
    % 计算 Kp
    Kp = (J * omega_bw) / Kt;
    
    % 计算 Ki (拐点频率选带宽的 1/5)
    omega_pi = omega_bw / 10; 
    Ki = Kp * omega_pi;
    
    fprintf('【一阶带宽法】(带宽: %d Hz)\n', bw_hz);
    fprintf('Kp = %f\n', Kp);
    fprintf('Ki = %f\n', Ki);

elseif method == 2
    % --- 方法 2：二阶极点配置法 ---
    wn_hz = input('请输入自然频率 Wn (Hz, 决定响应速度, 例如 20-40): ');
    zeta  = input('请输入阻尼比 Zeta (通常设为 0.707 或 1.0): ');
    omega_n = wn_hz * 2 * pi;
    
    % 推导时若考虑B：Kp = (2*zeta*omega_n*J - B) / Kt
    % 这里用忽略B的标准公式，因为 B 的影响在几十Hz下微乎其微
    Kp = (2 * zeta * omega_n * J) / Kt;
    Ki = (omega_n^2 * J) / Kt;
    
    fprintf('【二阶极点配置法】(Wn: %d Hz, Zeta: %.3f)\n', wn_hz, zeta);
    fprintf('Kp = %f\n', Kp);
    fprintf('Ki = %f\n', Ki);

elseif method == 3
    % --- 方法 3：零极点相消法 ---
    bw_hz = input('请输入期望的速度环带宽 (Hz, 例如 20-40): ');
    omega_bw = bw_hz * 2 * pi;
    
    Kp = (J * omega_bw) / Kt;
    Ki = (B * omega_bw) / Kt;
    
    fprintf('【零极点相消法(带B)】(带宽: %d Hz)\n', bw_hz);
    fprintf('Kp = %f\n', Kp);
    fprintf('Ki = %f  <-- 注意：此值通常过小，实机表现抗负载极弱\n', Ki);

else
    disp('无效选择，请重新运行。');
end
fprintf('======================\n');