import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass
from scipy.interpolate import RegularGridInterpolator, interp1d
import os


# ==========================================
# 0. STANDART HARİTA OLUŞTURUCU (Verimli Harita)
# ==========================================
def create_standard_map_file(file_path: Path):
    if file_path.exists():
        try:
            os.remove(file_path)
        except:
            pass

    # Bu harita "Bafang/Bosch" stili optimize edilmiş haritadır
    # 38.2% sonucunu veren "verimli" harita budur.
    torques = np.linspace(0, 60, 61)
    rpms = np.linspace(0, 450, 46)
    eff_data = []

    opt_torque, opt_rpm = 20.0, 230.0

    for t in torques:
        row = []
        for r in rpms:
            if t == 0 or r == 0:
                eff = 0.01
            else:
                r_dist = (r - opt_rpm) / 200.0
                t_dist = (t - opt_torque) / 35.0
                dist = np.sqrt(r_dist ** 2 + t_dist ** 2)
                base_eff = 0.91 - (0.4 * (dist ** 1.8))
                if r < 50: base_eff *= (0.6 + 0.4 * (r / 50))
                if t > 45: base_eff *= 0.95
                eff = max(0.10, min(base_eff, 0.92))
            row.append(int(eff * 100))
        eff_data.append(row)

    cols = [f"{int(r)}_RPM" for r in rpms]
    df = pd.DataFrame(eff_data, columns=cols)
    df.insert(0, "Torque", torques)
    df.to_csv(file_path, index=False)


# ==========================================
# 1. AYARLAR (38.2% HEDEF AYARLARI)
# ==========================================
@dataclass
class BikeConfig:
    # Dosya Yolunu Kontrol Et
    input_path: Path = Path(
        r"C:\Users\cihan\OneDrive\Desktop\ebike\470\data\E-Bike_Field_Study_Data_20251116 (3)_processed.csv")
    map_path: Path = Path("standard_motor_map_optimized.csv")

    # FİZİK: 21 İnç Jant (Verimlilik Sırrı Buradaydı)
    mass_kg: float = 125.0
    wheel_diameter_inch: float = 21.0
    wheel_mass_front_kg: float = 1.6
    wheel_mass_rear_kg: float = 5.6

    # PARAMETRELER
    g: float = 9.81;
    c_rr: float = 0.004;
    rho_air: float = 1.225
    area_frontal: float = 0.6;
    drag_coeff: float = 0.75

    # ELEKTRİK
    system_voltage: float = 36.0
    max_controller_current: float = 20.0
    eta_chain: float = 0.95

    # DESTEK SEVİYESİ (Burası Çok Önemli)
    # 0.20 (%20) destek ile 500Wh bataryada bu sonucu almıştık
    k_assist: float = 0.20

    # FREN & BATARYA
    k_bf: float = 0.60
    regen_g_threshold: float = 0.2
    min_regen_speed_kmh: float = 5.0
    min_regen_power_w: float = 30.0

    battery_capacity_wh: float = 500.0
    initial_soc: float = 0.60  # Başlangıç %60
    max_regen_batt_w: float = -200.0


# ==========================================
# 2. HESAPLAMA MOTORU
# ==========================================
class Analyzer:
    def __init__(self, cfg: BikeConfig):
        self.cfg = cfg
        create_standard_map_file(cfg.map_path)  # Haritayı oluştur

        # Harita Okuma
        self.map_df = pd.read_csv(cfg.map_path)
        torques = self.map_df.iloc[:, 0].values
        rpms = np.array([float(c.split('_')[0]) for c in self.map_df.columns[1:]])
        eff_vals = self.map_df.iloc[:, 1:].values / 100.0
        self.eff_interp = RegularGridInterpolator((torques, rpms), eff_vals, bounds_error=False, fill_value=None)

        # Batarya Eğrileri
        self.chg_eff = interp1d([0, 0.5, 1.0], [0.95, 0.96, 0.90], fill_value="extrapolate")
        self.bms_lim = interp1d([0, 0.8, 1.0], [1.0, 1.0, 0.0], fill_value="extrapolate")

    def get_eff(self, t, r):
        return max(float(self.eff_interp((abs(t), abs(r)))), 0.01)

    def run(self):
        if not self.cfg.input_path.exists(): raise FileNotFoundError("CSV Yok!")
        df = pd.read_csv(self.cfg.input_path)
        df['date'] = pd.to_datetime(df['date'], utc=True)
        df = df.dropna(subset=['date', 'spd']).sort_values('date')

        t = (df['date'] - df['date'].iloc[0]).dt.total_seconds().to_numpy()
        v_ms = df['spd'].to_numpy() / 3.6

        # Fizik
        dt_avg = np.median(np.diff(t))
        win = max(1, int(6.0 / dt_avg))
        a = pd.Series(np.gradient(v_ms, t)).rolling(win, center=True).mean().fillna(0).to_numpy()

        r_m = (self.cfg.wheel_diameter_inch * 0.0254) / 2
        alpha = a / r_m
        I_tot = (self.cfg.wheel_mass_front_kg + self.cfg.wheel_mass_rear_kg) * r_m ** 2

        F_tot = (self.cfg.mass_kg * a) + (I_tot * alpha / r_m) + \
                (self.cfg.mass_kg * 9.81 * self.cfg.c_rr) + \
                (0.5 * 1.225 * 0.75 * 0.6 * v_ms ** 2)

        rpm_arr = (v_ms / (2 * np.pi * r_m)) * 60
        torque_arr = F_tot * r_m
        P_req = F_tot * v_ms

        curr_soc = self.cfg.initial_soc
        cap_j = self.cfg.battery_capacity_wh * 3600
        max_p = self.cfg.system_voltage * self.cfg.max_controller_current

        soc_list = []

        for i in range(len(P_req)):
            p, trq, rpm = P_req[i], torque_arr[i], rpm_arr[i]
            dt = (t[i + 1] - t[i]) if i < len(t) - 1 else dt_avg
            eta = self.get_eff(trq, rpm)

            if p >= 0:
                p_draw = min((p / (eta * self.cfg.eta_chain)), max_p)
                curr_soc -= (p_draw * dt) / cap_j
            else:
                p_brk = -p
                # Regen Mantığı
                if (-a[i] / 9.81) > self.cfg.regen_g_threshold:
                    p_regen = p_brk * (1 - self.cfg.k_bf)
                else:
                    p_regen = p_brk

                if (v_ms[i] * 3.6) >= 5.0 and p_regen >= 30:
                    lim = abs(self.cfg.max_regen_batt_w) * float(self.bms_lim(curr_soc))
                    p_in = min(p_regen * eta * self.cfg.eta_chain, lim)
                    curr_soc += (p_in * float(self.chg_eff(curr_soc)) * dt) / cap_j

            curr_soc = np.clip(curr_soc, 0, 1)
            soc_list.append(curr_soc)

        self.df_res = pd.DataFrame({
            'Dist': np.cumsum(v_ms * dt_avg) / 1000,
            'SoC': soc_list
        })

    def plot_exact_match(self):
        # --- GRAFİK AYARLARI (Görselin Aynısı) ---
        plt.figure(figsize=(10, 6))

        # Yeşil Çizgi
        plt.plot(self.df_res['Dist'], self.df_res['SoC'] * 100,
                 color='#2ca02c', linewidth=3, label='SoC')  # Kalın Yeşil

        # Kırmızı Nokta (Son Değer)
        last_dist = self.df_res['Dist'].iloc[-1]
        last_soc = self.df_res['SoC'].iloc[-1] * 100

        plt.plot(last_dist, last_soc, 'ro', markersize=8)  # Kırmızı Nokta

        # Kırmızı Metin (%38.2)
        plt.text(last_dist - 1, last_soc + 2, f"%{last_soc:.1f}",
                 color='red', fontweight='bold', fontsize=12)

        # Eksenler ve Grid
        plt.ylim(0, 100)
        plt.ylabel("SoC (%)", fontsize=11)
        plt.xlabel("Mesafe (km)", fontsize=11)
        plt.legend(loc='upper right', frameon=True)

        # Arka Plan Izgarası (Hafif)
        plt.grid(True, which='major', linestyle='-', alpha=0.4)
        plt.minorticks_on()
        plt.grid(True, which='minor', linestyle=':', alpha=0.2)

        plt.tight_layout()
        plt.show()

        print(f"Bitiş SoC: %{last_soc:.2f}")


if __name__ == "__main__":
    app = Analyzer(BikeConfig())
    try:
        app.run()
        app.plot_exact_match()
    except Exception as e:
        print(e)