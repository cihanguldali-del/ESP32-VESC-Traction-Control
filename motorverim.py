import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# AYARLAR (Senin Sistem Limitlerin)
# ==========================================
MAX_BATTERY_POWER = 720.0  # 36V * 20A = 720W (Tüketim Limiti)
MAX_REGEN_POWER = -200.0  # BMS Şarj Limiti (Regen Limiti)

# Verim Kabulleri (Görselin gerçekçi durması için ortalama verim)
ETA_MOTOR = 0.85
ETA_CHAIN = 0.95
ETA_TOTAL = ETA_MOTOR * ETA_CHAIN


# ==========================================
# HESAPLAMA MOTORU
# ==========================================
def generate_power_map():
    # Grid Oluştur (RPM: 0-400, Tork: -50Nm ile +50Nm arası)
    rpms = np.linspace(1, 400, 500)
    torques = np.linspace(-50, 50, 500)
    R, T = np.meshgrid(rpms, torques)

    # Açısal Hız (rad/s)
    omega = R * 2 * np.pi / 60

    # Mekanik Güç (P = Tork * w)
    P_mech = T * omega

    # Batarya Gücü Hesabı (Verim Dahil)
    P_batt = np.zeros_like(P_mech)

    # 1. Tüketim Bölgesi (Tork > 0)
    # Bataryadan çekilen = Mekanik Güç / Verim
    mask_cons = T > 0
    P_batt[mask_cons] = P_mech[mask_cons] / ETA_TOTAL

    # 2. Regen Bölgesi (Tork < 0)
    # Bataryaya giren = Mekanik Güç * Verim
    mask_regen = T < 0
    P_batt[mask_regen] = P_mech[mask_regen] * ETA_TOTAL

    # 3. Limitleri Uygula (Clipping)
    # 720W üstünü ve -200W altını kes
    P_batt_clipped = np.clip(P_batt, MAX_REGEN_POWER, MAX_BATTERY_POWER)

    return R, T, P_batt, P_batt_clipped


# ==========================================
# ÇİZİM (MATPLOTLIB)
# ==========================================
def plot_power_map():
    R, T, P_raw, P_clipped = generate_power_map()

    plt.figure(figsize=(12, 8), dpi=100)

    # Renk Haritası (Kırmızı-Beyaz-Mavi)
    # levels: Renk geçiş aralıkları
    levels = np.linspace(-250, 800, 22)

    # Kontur Çizimi (Dolgulu)
    cmap = plt.cm.RdBu_r  # Red-Blue Reverse (Kırmızı Tüketim, Mavi Regen)
    cntr = plt.contourf(R, T, P_clipped, levels=levels, cmap=cmap, alpha=0.9)

    # İnce Çizgiler (Contour Lines)
    line_levels = [-200, -150, -100, -50, 0, 50, 150, 250, 350, 450, 550, 650, 720]
    lines = plt.contour(R, T, P_clipped, levels=line_levels, colors='black', linewidths=0.5, alpha=0.5)
    plt.clabel(lines, inline=True, fontsize=8, fmt='%1.0f W')

    # Renk Barı
    cbar = plt.colorbar(cntr, extend='both')
    cbar.set_label('Batarya Gücü (Watt)\n[+] Tüketim / [-] Şarj', fontweight='bold')

    # --- ETİKETLER VE SÜSLEMELER ---

    # 0 Tork Çizgisi (Kalın Siyah)
    plt.axhline(0, color='black', linewidth=2)

    # Bölge İsimleri (Kutulu Yazılar)
    plt.text(200, 30, "TÜKETİM BÖLGESİ\n(Batarya Boşalıyor)",
             ha='center', va='center', color='white', fontweight='bold',
             bbox=dict(facecolor='darkred', alpha=0.7, edgecolor='none'))

    plt.text(200, -30, "REGEN BÖLGESİ\n(Batarya Doluyor)",
             ha='center', va='center', color='white', fontweight='bold',
             bbox=dict(facecolor='darkblue', alpha=0.7, edgecolor='none'))

    # Limit Açıklamaları
    plt.text(50, 45, "Controller Limiti\n(Akım Kesme)", fontsize=8, ha='center')
    plt.text(300, -15, "BMS Limiti\n(Şarj Kesme)", fontsize=8, ha='center')

    # Eksenler
    plt.xlabel('Hız (RPM)', fontsize=12, fontweight='bold')
    plt.ylabel('Tork (Nm)', fontsize=12, fontweight='bold')
    plt.title('Batarya Güç Tüketim Haritası\n(Limitler: +720W Tüketim / -200W Regen)', fontsize=14)

    plt.grid(True, alpha=0.3, linestyle='--')
    plt.xlim(1, 400)
    plt.ylim(-50, 50)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_power_map()