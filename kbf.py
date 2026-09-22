import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. PARAMETRELER (Grafikteki Değerler)
# ==========================================
h = 0.6     # Ağırlık Merkezi Yüksekliği (m)
L = 1.15    # Tekerlekler Arası Mesafe (Wheelbase) (m)

# Ağırlık Merkezinin Yeri (Rollover Limitinden Çıkartıldı)
# Devrilme Limiti z = a / h => 1.25 = a / 0.6 => a = 0.75m
a = 0.75    # Ön aksa olan mesafe (m)
b = L - a   # Arka aksa olan mesafe (m) (0.40m)

mu_dry = 0.8  # Kuru Zemin Sürtünme Katsayısı
mu_wet = 0.4  # Islak Zemin Sürtünme Katsayısı

# Fren Dağılımı (0 = Sadece Arka, 1 = Sadece Ön)
K_bf = np.linspace(0.01, 0.99, 200)

# ==========================================
# 2. FORMÜLLER (Araç Dinamiği)
# ==========================================
def get_front_lock_z(mu, k_bf):
    # Ön Teker Kilitleme Limiti (Deceleration z)
    # Formül: z = (mu * b) / (K_bf * L - mu * h)
    denominator = (k_bf * L - mu * h)
    z = (mu * b) / denominator
    # Negatif veya anlamsız değerleri filtrele
    z[denominator <= 0] = np.nan
    z[z < 0] = np.nan
    return z

def get_rear_lock_z(mu, k_bf):
    # Arka Teker Kilitleme Limiti (Deceleration z)
    # Formül: z = (mu * a) / ((1 - K_bf) * L + mu * h)
    denominator = ((1 - k_bf) * L + mu * h)
    z = (mu * a) / denominator
    return z

# --- HESAPLAMA ---
# Kuru Zemin
z_dry_front = get_front_lock_z(mu_dry, K_bf)
z_dry_rear  = get_rear_lock_z(mu_dry, K_bf)

# Islak Zemin
z_wet_front = get_front_lock_z(mu_wet, K_bf)
z_wet_rear  = get_rear_lock_z(mu_wet, K_bf)

# Devrilme Limiti (Rollover)
z_rollover = a / h # 0.75 / 0.6 = 1.25 g

# ==========================================
# 3. ÇİZİM
# ==========================================
plt.figure(figsize=(10, 7))

# Sınır Çizgileri
plt.plot(K_bf, z_dry_front, 'r-', linewidth=2, label=f'Dry Front Lock ($\mu={mu_dry}$)')
plt.plot(K_bf, z_dry_rear,  'r--', linewidth=2, label=f'Dry Rear Lock ($\mu={mu_dry}$)')

plt.plot(K_bf, z_wet_front, 'b-', linewidth=2, label=f'Wet Front Lock ($\mu={mu_wet}$)')
plt.plot(K_bf, z_wet_rear,  'b--', linewidth=2, label=f'Wet Rear Lock ($\mu={mu_wet}$)')

# Devrilme Çizgisi
plt.axhline(z_rollover, color='black', linewidth=2.5, label=f'Rollover Limit ({z_rollover:.2f}g)')

# Optimum Noktalar (Kesişimler)
# Kuru için optimum (Ön ve Arka aynı anda kilitlenir = Ideal Dağılım)
# Bu nokta z = mu olduğu yerdir.
opt_dry_idx = np.nanargmin(np.abs(z_dry_front - z_dry_rear))
plt.plot(K_bf[opt_dry_idx], z_dry_front[opt_dry_idx], 'p', markersize=14, markerfacecolor='yellow', markeredgecolor='black', label='Optimum Point')

# Safe Region Oku
plt.annotate('Safe Region\n(No Lockup)', xy=(0.4, 0.2), xytext=(0.2, 0.3),
             arrowprops=dict(facecolor='green', shrink=0.05),
             fontsize=10, fontweight='bold', color='green')

# KBF Referans Çizgileri (Opsiyonel - Grafikteki dikey çizgiler)
plt.axvline(0.21, color='blue', linestyle='-', alpha=0.8) # Islakta ideal dağılım
plt.axvline(0.42, color='red', linestyle='-', alpha=0.8)  # Kuruda ideal dağılım

# Ayarlar
plt.title(f'E-Bike Optimum Lockup Diagram (h={h}m, L={L}m)', fontsize=14)
plt.xlabel('Brake Force Distribution ($K_{bf}$) [0=Rear, 1=Front]', fontsize=12)
plt.ylabel('Deceleration (z = a/g)', fontsize=12)
plt.ylim(0, 1.4)
plt.xlim(0, 1.0)
plt.grid(True, which='both', linestyle='--', alpha=0.7)
plt.legend(loc='center left', fontsize=10)

plt.tight_layout()
plt.show()