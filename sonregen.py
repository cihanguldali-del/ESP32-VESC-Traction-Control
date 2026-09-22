import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from dataclasses import dataclass
from scipy.interpolate import RegularGridInterpolator, interp1d
import os

try:
    import mplcursors

    INTERACTIVE = True
except ImportError:
    INTERACTIVE = False


# ==========================================
# 1. AYARLAR (MASTER CONFIG)
# ==========================================
@dataclass
class BikeConfig:
    input_path: Path = Path(
        r"C:\Users\cihan\OneDrive\Desktop\ebike\470\data\E-Bike_Field_Study_Data_20251116 (3)_processed.csv")
    map_path: Path = Path("physics_motor_map_600W.csv")

    mass_kg: float = 125.0
    wheel_diameter_inch: float = 21.0
    wheel_mass_front_kg: float = 1.6
    wheel_mass_rear_kg: float = 5.6
    g: float = 9.81;
    c_rr: float = 0.004;
    rho_air: float = 1.225
    area_frontal: float = 0.6;
    drag_coeff: float = 0.75

    system_voltage: float = 36.0
    max_controller_current: float = 16.66
    eta_chain: float = 0.95

    k_bf: float = 0.70
    regen_g_threshold: float = 0.2
    min_regen_speed_kmh: float = 5.0
    min_regen_power_w: float = 10.0

    battery_capacity_wh: float = 500.0
    initial_soc: float = 0.60
    max_regen_batt_w: float = -200.0


# ==========================================
# 2. MOTOR HARİTASI
# ==========================================
def create_physics_based_map(file_path: Path):
    if not file_path.exists():
        torques = np.linspace(0, 50, 51)
        rpms = np.linspace(0, 450, 46)
        eff_data = [[int(0.85 * 100) for _ in rpms] for _ in torques]
        cols = [f"{int(r)}_RPM" for r in rpms]
        df = pd.DataFrame(eff_data, columns=cols)
        df.insert(0, "Torque", torques)
        df.to_csv(file_path, index=False)


# ==========================================
# 3. ANALİZÖR
# ==========================================
class Analyzer:
    def __init__(self, cfg: BikeConfig):
        self.cfg = cfg
        create_physics_based_map(cfg.map_path)

        self.map_df = pd.read_csv(cfg.map_path)
        torques = self.map_df.iloc[:, 0].values
        rpms = np.array([float(c.split('_')[0]) for c in self.map_df.columns[1:]])
        eff_vals = self.map_df.iloc[:, 1:].values / 100.0
        self.eff_interp = RegularGridInterpolator((torques, rpms), eff_vals, bounds_error=False, fill_value=None)

        self.batt_bms = interp1d([0, 0.8, 0.9, 0.95, 1.0], [1.0, 1.0, 0.35, 0.1, 0.0], fill_value="extrapolate")
        self.batt_chem = interp1d([0, 0.5, 1.0], [0.9, 0.96, 0.9], fill_value="extrapolate")

        self.radius_m = (cfg.wheel_diameter_inch * 0.0254) / 2
        self.I_total = (cfg.wheel_mass_front_kg + cfg.wheel_mass_rear_kg) * self.radius_m ** 2

    def get_eff(self, t, r):
        return max(float(self.eff_interp((abs(t), abs(r)))), 0.01)

    # --- ÇİFT GRAFİKLİ FREN SENARYOSU ---
    def plot_braking_scenario(self):
        t = np.linspace(0, 6, 600)
        dt = t[1] - t[0]
        T_req_arr = np.linspace(0, 90, 600)
        v_kmh = np.linspace(30, 0, 600)
        v_ms = v_kmh / 3.6

        current_sim_soc = self.cfg.initial_soc
        bms_factor = float(self.batt_bms(current_sim_soc))
        chem_eff = float(self.batt_chem(current_sim_soc))

        print(f"[BİLGİ] SoC: %{current_sim_soc * 100:.1f} | BMS Çarpanı: {bms_factor:.2f}")

        T_regen, T_front, T_rear = [], [], []
        P_regen_elec = []  # Bataryaya giren elektriksel güç

        for i in range(len(t)):
            T_req = T_req_arr[i];
            vel = v_ms[i];
            vel_k = v_kmh[i]
            w = vel / self.radius_m
            g_val = (T_req / self.radius_m) / self.cfg.mass_kg / 9.81

            # 1. LİMİTLER
            mech_limit = 45.0
            effective_power_limit = abs(self.cfg.max_regen_batt_w) * bms_factor
            batt_power_limit_torque = effective_power_limit / w if w > 1.0 else mech_limit
            max_hardware_regen = min(mech_limit, batt_power_limit_torque)

            # 2. KONTROL LOJİĞİ
            if vel_k > self.cfg.min_regen_speed_kmh:
                target_regen = T_req if g_val <= self.cfg.regen_g_threshold else T_req * (1 - self.cfg.k_bf)
                act_regen = min(target_regen, max_hardware_regen)
            else:
                act_regen = 0.0

                # 3. MEKANİK FRENLER
            remaining_torque = max(0, T_req - act_regen)
            if remaining_torque > 0:
                if g_val <= self.cfg.regen_g_threshold:
                    tf = remaining_torque * 0.5;
                    tr = remaining_torque * 0.5
                else:
                    tf = T_req * self.cfg.k_bf;
                    tr = max(0, T_req - tf - act_regen)
            else:
                tf = 0;
                tr = 0

            # 4. GÜÇ VE VERİM HESABI
            rpm = w * 60 / (2 * np.pi)
            eta_motor = self.get_eff(act_regen, rpm) if act_regen > 0 else 0
            # P_mekanik = T * w
            # P_elektrik = P_mekanik * verimler
            p_elec_inst = (act_regen * w) * eta_motor * self.cfg.eta_chain * chem_eff

            T_regen.append(act_regen);
            T_front.append(tf);
            T_rear.append(tr)
            P_regen_elec.append(p_elec_inst)

        # --- ENERJİ HESAPLAMALARI (Kümülatif) ---
        # 1. Toplam Kaybedilen Kinetik Enerji (Araç Durana Kadar)
        # E_k = 0.5 * m * v^2
        initial_ke = 0.5 * self.cfg.mass_kg * v_ms[0] ** 2
        current_ke = 0.5 * self.cfg.mass_kg * v_ms ** 2
        cumulative_kinetic_loss = initial_ke - current_ke

        # 2. Toplam Geri Kazanılan Regen Enerjisi (Bataryaya Giren)
        # E_regen = Integral(P_elec * dt)
        cumulative_regen_energy = np.cumsum(np.array(P_regen_elec) * dt)

        # --- GRAFİK 1: TORK DAĞILIMI ---
        plt.figure(figsize=(10, 6))
        plt.plot(t, T_regen, 'g-', linewidth=3, label=f'Regen (BMS Factor: {bms_factor:.1f})')
        plt.plot(t, T_front, 'r--', linewidth=2, label='Front Hydraulic')
        plt.plot(t, T_rear, 'b:', linewidth=2, label='Rear Hydraulic')
        threshold_torque = 0.2 * 9.81 * self.cfg.mass_kg * self.radius_m
        plt.axhline(threshold_torque, color='orange', linestyle='-.', alpha=0.7, label='0.2G Threshold')
        plt.title(f"Scenario 1: Torque Distribution Dynamics (SoC %{current_sim_soc * 100:.0f})", fontsize=11)
        plt.ylabel("Torque (Nm)");
        plt.xlabel("Time (s)")
        plt.grid(True, alpha=0.3);
        plt.legend(loc='upper left')
        if bms_factor < 0.1: plt.text(2, 10, "BMS LİMİTİ AKTİF!\nREGEN KAPALI", color='red', fontsize=14,
                                      fontweight='bold', bbox=dict(facecolor='white', alpha=0.8))
        plt.tight_layout();
        plt.show()

        # --- GRAFİK 2: KÜMÜLATİF ENERJİ (YENİ EKLENEN) ---
        plt.figure(figsize=(10, 6))
        # Siyah Çizgi: Toplam Kayıp Kinetik Enerji
        plt.plot(t, cumulative_kinetic_loss, 'k-', linewidth=2, label='Total Braking Energy (Vehicle Kinetic)')
        # Yeşil Çizgi: Geri Kazanılan Regen Enerjisi
        plt.plot(t, cumulative_regen_energy, 'g-', linewidth=2, label='Recovered Regen Energy')
        # Yeşil Alanı Doldur
        plt.fill_between(t, 0, cumulative_regen_energy, color='green', alpha=0.15)

        plt.title(f"Scenario 2: Cumulative Energy Recovery (SoC %{current_sim_soc * 100:.0f})", fontsize=11)
        plt.ylabel("Energy (Joules)");
        plt.xlabel("Time (s)")
        plt.grid(True, alpha=0.3);
        plt.legend(loc='upper left')

        # Toplam Geri Kazanım Oranını Göster
        total_loss = cumulative_kinetic_loss[-1]
        total_recovered = cumulative_regen_energy[-1]
        if total_loss > 0:
            recovery_ratio = (total_recovered / total_loss) * 100
            plt.text(t[-1] * 0.6, total_loss * 0.5, f"Total Recovery:\n%{recovery_ratio:.1f}",
                     fontsize=12, fontweight='bold', color='green', bbox=dict(facecolor='white', alpha=0.8))

        if INTERACTIVE: mplcursors.cursor(hover=True)
        plt.tight_layout();
        plt.show()


if __name__ == "__main__":
    # Test için SoC %60 (Regen Aktif)
    app = Analyzer(BikeConfig(initial_soc=0.60))
    app.plot_braking_scenario()