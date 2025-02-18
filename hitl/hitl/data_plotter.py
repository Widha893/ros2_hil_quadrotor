import pandas as pd
import matplotlib.pyplot as plt

# Load data
file_path = 'pitch step.csv'  # Ganti dengan nama file CSV yang sesuai
df = pd.read_csv('pitch step.csv')

# Plot data
plt.figure(figsize=(10, 5))
plt.plot(df["Time"], df["Pitch (Sim)"], label="Sudut Pitch (Derajat)", color="orange")  # Mengubah warna menjadi hijau
plt.plot(df["Time"], df["Pitch (Real)"], label="Respon Pitch (Derajat)", color="green")  # Mengubah warna menjadi biru

# Tambahkan batas toleransi atas dan bawah
plt.axhline(y=1.5, color="red", linestyle="--", label="Batas Toleransi Atas")
plt.axhline(y=-1.5, color="red", linestyle="--", label="Batas Toleransi Bawah")
plt.axhline(y=0.0, color="black", linestyle="--", label="Setpoint")

# Label dan judul
plt.xlabel("Waktu (Detik)")
plt.ylabel("Sudut Pitch (Derajat)")
plt.title("Rspon Sudut Pitch pada Pengujian di Lingkungan Nyata")
plt.legend()
plt.grid()

# Tampilkan plot
plt.show()
