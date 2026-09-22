import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass


# ==========================================
# 1. AYARLAR (SON FİNAL KONFİGÜRASYON)
# ==========================================
@dataclass
class BikeConfig:
    # Fizik
    mass_kg: float = 125.0
    g: float = 9.81
    c_rr: float = 0.004
    rho_air: float = 1.225
    area_frontal: float = 0.6
    drag_coeff: float = 0.75

    # JANT: 21 İNÇ (Son kararımız)
    wheel_diameter_inch: float = 21.0
    wheel_mass_front_kg: float = 1.6
    wheel_mass_rear_kg: float = 5.6

    # Elektrik
    system_voltage: float = 36.0
    max_controller_current: float = 20.0  # 720W Giriş
    max_motor_torque_nm: float = 45.0  # Motor Tork Limiti
    eta_chain: float = 0.95  # Zincir/Dişli verimi

    # Simülasyon
    sim_step_s: float = 0.1


# ==========================================
# 2. PERFORMANS TEST MOTORU (0-40 KM/H)
# ==========================================
class AccelerationTester:
    def __init__(self, cfg: BikeConfig):
        self.cfg = cfg
        self.radius_m = (cfg.wheel_diameter_inch * 0.0254) / 2

        # Dönel Eylemsizlik (I = m*r^2)
        # Bu, tekerleklerin dönmeye karşı gösterdiği ekstra dirençtir.
        self.I_total = (cfg.wheel_mass_front_kg + cfg.wheel_mass_rear_kg) * (self.radius_m ** 2)

        # Max Mekanik Güç (Ortalama %80 verim kabulüyle)
        # 36V * 20A = 720W (Elektrik) -> ~576W (Mekanik)
        self.P_mech_max = (cfg.system_voltage * cfg.max_controller_current) * 0.80 * cfg.eta_chain

    def get_max_force(self, v_ms):
        if v_ms < 0.1: v_ms = 0.1

        # 1. Tork Limiti (Düşük hızda motorun beli kırılmasın diye limit)
        F_torque = self.cfg.max_motor_torque_nm / self.radius_m

        # 2. Güç Limiti (Yüksek hızda voltaj yetmez, güç sabit kalır)
        F_power = self.P_mech_max / v_ms

        # Motor hangisini verebiliyorsa onu al (Minimumunu)
        return min(F_torque, F_power)

    def run_test(self):
        print("Test Başlıyor: 0 -> 40 km/h Tam Gaz...")

        t = 0
        v = 0  # m/s cinsinden hız

        times = [0]
        vels_kmh = [0]

        dt = self.cfg.sim_step_s

        while v * 3.6 < 50:  # 50 km/h'yi geçene veya süre bitene kadar
            # KUVVETLER
            F_push = self.get_max_force(v)  # Motor İtiş

            F_aero = 0.5 * self.cfg.rho_air * self.cfg.drag_coeff * self.cfg.area_frontal * (v ** 2)
            F_roll = self.cfg.mass_kg * self.cfg.g * self.cfg.c_rr
            F_resist = F_aero + F_roll

            F_net = F_push - F_resist

            # Araç artık hızlanamıyorsa (Son Hız) döngüyü kır
            if F_net <= 0:
                print(f"-> Doygunluk Hızı: {v * 3.6:.1f} km/h (Daha fazla hızlanamıyor)")
                break

            # İVME HESABI (I*Alpha Dahil Gerçek Formül)
            # F = m*a + I*(a/r^2)  =>  a = F / (m + I/r^2)
            effective_mass = self.cfg.mass_kg + (self.I_total / self.radius_m ** 2)
            a = F_net / effective_mass

            # İNTEGRASYON
            v += a * dt
            t += dt

            times.append(t)
            vels_kmh.append(v * 3.6)

            if t > 60:  # 60 saniyeyi geçerse durdur
                print("-> Zaman Aşımı (60sn)")
                break

        return times, vels_kmh


# ==========================================
# 3. ÇALIŞTIR VE GRAFİK ÇİZ
# ==========================================
if __name__ == "__main__":
    tester = AccelerationTester(BikeConfig())
    times, vels = tester.run_test()

    # 0-40 km/h Süresini Bul
    try:
        # Hızın 40'ı geçtiği İLK anı bul
        idx = next(i for i, v in enumerate(vels) if v >= 40.0)
        t_40 = times[idx]
        print("\n" + "=" * 40)
        print(f"SONUÇ: 0-40 km/h Süresi = {t_40:.2f} saniye")
        print("=" * 40)

        # GRAFİK
        plt.figure(figsize=(10, 6))
        plt.plot(times, vels, linewidth=3, color='#d62728')

        # 40 km/h Noktasını İşaretle
        plt.plot([0, t_40], [40, 40], 'k--', alpha=0.5)
        plt.plot([t_40, t_40], [0, 40], 'k--', alpha=0.5)
        plt.plot(t_40, 40, 'ko', markersize=8)
        plt.text(t_40 + 1, 38, f"{t_40:.2f} sn", fontweight='bold', fontsize=12)

        plt.title("Hızlanma Eğrisi (21 inç Jant)")
        plt.xlabel("Zaman (sn)")
        plt.ylabel("Hız (km/h)")
        plt.grid(True, alpha=0.3)
        plt.show()

    except StopIteration:
        print("\n" + "=" * 40)
        print("SONUÇ: Araç 40 km/h hıza ULAŞAMADI!")
        print(f"Görülen Maksimum Hız: {max(vels):.2f} km/h")
        print("Sebepler: Motor gücü yetersiz veya rüzgar direnci fazla.")
        print("=" * 40)