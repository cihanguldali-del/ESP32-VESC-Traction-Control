import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass
from scipy.interpolate import RegularGridInterpolator, interp1d
import os


# ==========================================
# 0. HARİTA OLUŞTURUCU (38.2% Veren Optimize Harita)
# ==========================================
def create_optimized_map_file(file_path: Path):
    # Eğer dosya varsa silip yenisini yapalım ki garanti olsun
    if file_path.exists():
        try:
            os.remove(file_path)
        except:
            pass

    torques = np.linspace(0, 60, 61)
    rpms = np.linspace(0, 450, 46)
    eff_data = []

    # Bafang/Bosch Stili Verim Adası
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
# 1. AYARLAR (38.2% KONFİGÜRASYONU)
# ==========================================
@dataclass
class BikeConfig:
    # Dosya Yolunu KONTROL ET
    input_path: Path = Path(
        r"C:\Users\cihan\OneDrive\Desktop\ebike\470\data\E-Bike_Field_Study_Data_20251116 (3)_processed.csv")
    map_path: Path = Path("standard_motor_map_optimized.csv")

    # FİZİK (21 JANT + I*ALPHA)
    mass_kg: float = 125.0
    wheel_diameter_inch: float = 21.0
    wheel_mass_front_kg: float = 1.6
    wheel_mass_rear_kg: float = 5.6

    # ORTAM
    g: float = 9.81;
    c_rr: float = 0.004;
    rho_air: float = 1.225
    area_frontal: float = 0.6;
    drag_coeff: float = 0.75

    # ELEKTRİK & DESTEK
    system_voltage: float = 36.0
    max_controller_current: float = 20.0
    eta_chain: float = 0.95
    k_assist: float = 0.20  # <-- KRİTİK DEĞER

    # FREN & BATARYA
    k_bf: float = 0.60
    regen_g_threshold: float = 0.2
    min_regen_speed_kmh: float = 5.0  # Ölü Bölge Sınırı
    min_regen_power_w: float = 30.0  # Ölü Bölge Sınırı

    battery_capacity_wh: float = 500.0
    initial_soc: float = 0.60
    max_regen_batt_w: float = -200.0


# ==========================================
# 2. HESAPLAMA MOTORU (ANALYZER)
# ==========================================
class Analyzer:
    def __init__(self, cfg: BikeConfig):
        self.cfg = cfg
        self.df = None

        # Harita Hazırla
        create_optimized_map_file(cfg.map_path)

        # Harita Okuyucu
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
        self.df = pd.read_csv(self.cfg.input_path)
        self.df['date'] = pd.to_datetime(self.df['date'], utc=True)
        self.df = self.df.dropna(subset=['date', 'spd']).sort_values('date')

        t = (self.df['date'] - self.df['date'].iloc[0]).dt.total_seconds().to_numpy()
        v_ms = self.df['spd'].to_numpy() / 3.6

        # Fizik Hesaplamaları
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

        # Veri Saklama
        soc_list = []
        P_batt_list = []  # Batarya Gücü (+ Çekilen, - Giren)
        P_mech_waste_list = []  # Mekanik Fren Kaybı

        for i in range(len(P_req)):
            p, trq, rpm = P_req[i], torque_arr[i], rpm_arr[i]
            dt = (t[i + 1] - t[i]) if i < len(t) - 1 else dt_avg
            eta = self.get_eff(trq, rpm)

            p_batt_inst = 0
            p_waste_inst = 0

            if p >= 0:  # ÇEKİŞ
                p_draw = min((p / (eta * self.cfg.eta_chain)), max_p)
                p_batt_inst = p_draw
                curr_soc -= (p_draw * dt) / cap_j
            else:  # FREN / REGEN
                p_brk = -p
                g_val = -a[i] / 9.81

                # Regen vs Mekanik Ayrımı
                if g_val <= self.cfg.regen_g_threshold:
                    p_regen = p_brk;
                    p_mech = 0
                else:
                    p_regen = p_brk * (1 - self.cfg.k_bf)
                    p_mech = p_brk * self.cfg.k_bf

                p_waste_inst += p_mech

                # Regen Limitleri (Ölü Bölgeler)
                if (v_ms[i] * 3.6) >= self.cfg.min_regen_speed_kmh and p_regen >= self.cfg.min_regen_power_w:
                    lim = abs(self.cfg.max_regen_batt_w) * float(self.bms_lim(curr_soc))
                    p_in = min(p_regen * eta * self.cfg.eta_chain, lim)

                    p_batt_inst = -p_in
                    curr_soc += (p_in * float(self.chg_eff(curr_soc)) * dt) / cap_j

                    # Regen verimsizliğinden kaynaklı kayıp
                    p_used_at_wheel = p_in / (eta * self.cfg.eta_chain) if eta > 0 else 0
                    p_waste_inst += (p_regen - p_used_at_wheel)
                else:
                    # Limitlere takıldı, hepsi mekanik frene (ısıya) gitti
                    p_waste_inst += p_regen

            curr_soc = np.clip(curr_soc, 0, 1)
            soc_list.append(curr_soc)
            P_batt_list.append(p_batt_inst)
            P_mech_waste_list.append(p_waste_inst)

        # Sonuçları DataFrame'e kaydet
        self.df['Dist_km'] = np.cumsum(v_ms * dt_avg) / 1000
        self.df['SoC'] = soc_list
        self.df['P_batt'] = P_batt_list
        self.df['P_waste'] = P_mech_waste_list
        self.df['RPM'] = rpm_arr
        self.df['Torque'] = torque_arr
        self.t_arr = t  # Zaman dizisi (İntegral için)

    def print_report(self):
        # Enerji Hesapları (İntegral)
        P_pos = np.clip(self.df['P_batt'], 0, None)  # Çekilen
        P_neg = np.clip(self.df['P_batt'], None, 0)  # Giren (Negatif)

        E_drawn = np.trapz(P_pos, self.t_arr) / 3600
        E_regen = np.trapz(P_neg, self.t_arr) / 3600  # Sonuç negatif çıkar
        E_waste = np.trapz(self.df['P_waste'], self.t_arr) / 3600

        dist = self.df['Dist_km'].iloc[-1]
        start_soc = self.cfg.initial_soc * 100
        end_soc = self.df['SoC'].iloc[-1] * 100
        net_wh = E_drawn + E_regen

        print("\n" + "=" * 50)
        print("   DETAYLI PERFORMANS RAPORU (FİNAL)")
        print("=" * 50)
        print(f"Mesafe             : {dist:.2f} km")
        print(f"SoC Değişimi       : %{start_soc:.1f} -> %{end_soc:.1f}")
        print("-" * 50)
        print(f"Bataryadan Çekilen : {E_drawn:.2f} Wh")
        print(f"Geri Kazanılan     : {abs(E_regen):.2f} Wh (Regen)")
        print(f"Mekanik Fren Kaybı : {E_waste:.2f} Wh")
        print("-" * 50)
        print(f"NET TÜKETİM        : {net_wh:.2f} Wh")
        print(f"VERİMLİLİK         : {net_wh / dist:.2f} Wh/km")
        print("=" * 50)

    def plot_visuals(self):
        # --- GRAFİK 1: SoC ---
        plt.figure(figsize=(10, 6))
        plt.plot(self.df['Dist_km'], self.df['SoC'] * 100, color='#2ca02c', linewidth=3, label='SoC')

        last_km = self.df['Dist_km'].iloc[-1]
        last_soc = self.df['SoC'].iloc[-1] * 100
        plt.plot(last_km, last_soc, 'ro', markersize=8)
        plt.text(last_km - 2, last_soc + 2, f"%{last_soc:.1f}", color='red', fontweight='bold', fontsize=12)

        plt.ylim(0, 100);
        plt.ylabel("SoC (%)");
        plt.xlabel("Mesafe (km)")
        plt.title("Sürüş Boyunca Batarya Durumu")
        plt.grid(True, alpha=0.3);
        plt.legend()
        plt.show()

        # --- GRAFİK 2: ÖLÜ BÖLGELER VE ÇALIŞMA NOKTALARI ---
        plt.figure(figsize=(12, 7))

        # Harita Konturları
        torques = self.map_df.iloc[:, 0].values
        rpms = np.array([float(c.split('_')[0]) for c in self.map_df.columns[1:]])
        eff_vals = self.map_df.iloc[:, 1:].values
        plt.contourf(rpms, torques, eff_vals, levels=20, cmap='viridis', alpha=0.6)
        plt.colorbar(label='Verim (%)')

        # Noktalar
        mask = self.df['spd'] > 0.5
        plt.scatter(self.df.loc[mask, 'RPM'], self.df.loc[mask, 'Torque'], c='red', s=5, alpha=0.5,
                    label='Çalışma Noktaları')

        # Ölü Bölgeler (Gri)
        # 1. Hız Sınırı (< 5 km/h)
        r_m = (self.cfg.wheel_diameter_inch * 0.0254) / 2
        rpm_lim = (self.cfg.min_regen_speed_kmh / 3.6) / (2 * np.pi * r_m) * 60
        plt.axvspan(0, rpm_lim, color='gray', alpha=0.5, label=f'Hız < {self.cfg.min_regen_speed_kmh} km/h (Ölü)')
        plt.axvline(rpm_lim, color='black', linestyle='--')

        # 2. Güç Sınırı (< 30 W)
        rpm_range = np.linspace(rpm_lim, 450, 200)
        w_range = rpm_range * 2 * np.pi / 60
        torque_curve = -self.cfg.min_regen_power_w / w_range
        plt.fill_between(rpm_range, 0, torque_curve, color='gray', alpha=0.5,
                         label=f'Güç < {self.cfg.min_regen_power_w} W (Ölü)')
        plt.plot(rpm_range, torque_curve, 'k--')

        plt.axhline(0, color='black')
        plt.ylim(-40, 40);
        plt.xlim(0, 400)
        plt.title(f"Regen Ölü Bölgeleri - {self.cfg.min_regen_power_w}W & {self.cfg.min_regen_speed_kmh}km/h")
        plt.xlabel("RPM");
        plt.ylabel("Tork (Nm)");
        plt.legend(loc='upper right')
        plt.show()


if __name__ == "__main__":
    app = Analyzer(BikeConfig())
    try:
        app.run()
        app.print_report()
        app.plot_visuals()
    except Exception as e:
        print(f"Hata: {e}")