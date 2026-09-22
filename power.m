%% 0) Tabloyu yükle ve zamanı sıfırla
load DrivingCycleTable.mat 
t_raw = seconds(DrivingCycleTable.Time - DrivingCycleTable.Time(1)); 
t     = t_raw / 3600;                      % [h] <--- SAAT OLARAK GÜNCELLENDİ
V     = DrivingCycleTable.Speed_mps;       % [m/s] 

%% 1) Parametreler - ZORLU SÜRÜŞ ŞARTLARI (Düz Zemin, M=125kg, Cd=0.85)
M     = 125;     % kg 
g     = 9.81;    % m/s^2
Crr   = 0.008;   % -
rho   = 1.225;   % kg/m^3
Af    = 0.6;     % m^2
Cd    = 0.85;    % - 
theta = 0;       % rad 
Km    = 1;       % Rotary Mass Factor
windowSize = 10; % s

%% 2) İvme (dV/dt) [m/s^2] - 10s Yumuşatma
a_raw = diff(V) ./ diff(t_raw); 
a_raw = [a_raw; a_raw(end)];
a = movmean(a_raw, windowSize); 

%% 3) Kuvvet bileşenleri [N]
F_inertia = Km * M * a;                        
F_air     = 0.5 * Cd * rho * Af * V.^2;        
F_hill    = M * g * sin(theta);               
F_roll    = M * g * Crr * cos(theta);         
F_total   = F_inertia + F_air + F_hill + F_roll; 

%% 4) Güç hesabı ve Pürüzsüzleştirme
P_W  = V .* F_total;             % Ham Güç [W] 
P_kW = P_W / 1000;               % Ham Güç [kW]
P_W_smooth = movmean(P_W, windowSize); % 10s Yumuşatma

%% 5) Güç grafiği (10s pürüzsüzleştirme)
figure;
plot(t, P_W_smooth, 'LineWidth', 1.5, 'Color', 'b'); 
grid on;
xlabel('Time [h]');             % <--- ETİKET GÜNCELLENDİ
ylabel('Required Power [W] (Smoothed)');
title('E-bike Required Power Cycle (Düz Zemin, Zorlu M & Cd - 10s Smooth)'); 

%% 6) Güç tablosu oluşturma ve çıktı
PowerTable = table(t, DrivingCycleTable.Time, V, a, F_total, P_W, P_kW, P_W_smooth, ...
    'VariableNames', {'Time_h','Time_raw_datetime','Speed_mps','Acceleration_mps2','Total_Force_N','Power_W_Raw','Power_kW_Raw', 'Power_W_Smooth'}); 

disp(PowerTable);