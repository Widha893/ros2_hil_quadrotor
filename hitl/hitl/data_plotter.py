import pandas as pd
import matplotlib.pyplot as plt

# Load data
file_path = 'pitch step.csv'  # Ganti dengan nama file CSV yang sesuai
df = pd.read_csv('Yaw step.csv')

# Plot data
plt.figure(figsize=(10, 5))
plt.plot(df["Time"], df["Yaw (sim)"], label="Respon Sudut Yaw (Derajat)", color="orange")  # Mengubah warna menjadi hijau
# plt.plot(df["Time"], df["Yaw (real)"], label="Respon pada Lingkungan Nyata (Derajat)", color="green")  # Mengubah warna menjadi biru

# Tambahkan batas toleransi atas dan bawah
plt.axhline(y=2.5, color="red", linestyle="--", label="Batas Toleransi Atas")
plt.axhline(y=-2.5, color="red", linestyle="--", label="Batas Toleransi Bawah")
plt.axhline(y=0.0, color="black", linestyle="--", label="Setpoint")

# Label dan judul
plt.xlabel("Waktu (Detik)")
plt.ylabel("Sudut Yaw (Derajat)")
plt.title("Respon Sudut Yaw pada Lingkungan Simulasi")
plt.legend()
plt.grid()

# Tampilkan plot
plt.show()
