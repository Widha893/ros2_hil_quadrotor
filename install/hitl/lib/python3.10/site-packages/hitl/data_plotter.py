import pandas as pd
import matplotlib.pyplot as plt

# Load data
file_path = "data.csv"  # Ganti dengan nama file CSV yang sesuai
df = pd.read_csv('Yaw response.csv')

# Plot data
plt.figure(figsize=(10, 5))
plt.plot(df["Time"], df["Yaw (sim)"], label="Respon pada Simulasi", color="orange")  # Mengubah warna menjadi hijau
plt.plot(df["Time"], df["Yaw (real)"], label="Respon pada Dunia Nyata", color="green")  # Mengubah warna menjadi biru

# Tambahkan batas toleransi atas dan bawah
plt.axhline(y=2.5, color="red", linestyle="--", label="Batas Toleransi Atas")
plt.axhline(y=-2.5, color="red", linestyle="--", label="Batas Toleransi Bawah")
plt.axhline(y=0, color="black", linestyle="--", label="Setpoint")

# Label dan judul
plt.xlabel("Waktu (detik)")
plt.ylabel("Sudut Yaw (Derajat)")
plt.title("Perbandingan Respon Sudut Yaw pada Simulasi dan Dunia Nyata")
plt.legend()
plt.grid()

# Tampilkan plot
plt.show()
