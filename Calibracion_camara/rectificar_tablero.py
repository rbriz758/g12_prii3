import cv2
import numpy as np
import os
import glob

# 1. Cargar los datos de calibración
if not os.path.exists('calib_data.npz'):
    print("Error: No se encontró el archivo 'calib_data.npz'.")
    exit()

with np.load('calib_data.npz') as X:
    mtx, dist = [X[i] for i in ('mtx', 'dist')]

print("Matriz y coeficientes cargados correctamente.")

# 2. Configurar directorios
input_folder = './ImagenesCalibracion'
output_folder = './ImagenesRectificadas'

if not os.path.exists(output_folder):
    os.makedirs(output_folder)
    print(f"Directorio creado: {output_folder}")

# Obtener lista de imágenes
images_path = os.path.join(input_folder, '*.png')
images = glob.glob(images_path)

print(f"Procesando {len(images)} imágenes...")

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"No se pudo leer: {fname}")
        continue
        
    h, w = img.shape[:2]
    filename = os.path.basename(fname)

    # 3. Obtener nueva matriz de cámara
    # Usamos alpha=0.5 para un recorte intermedio (balance entre bordes negros y mantener imagen)
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0.5, (w,h))

    # 4. Rectificar
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # Recortar usando la ROI
    x, y, w_roi, h_roi = roi
    dst = dst[y:y+h_roi, x:x+w_roi]

    # 5. Guardar
    output_path = os.path.join(output_folder, filename)
    cv2.imwrite(output_path, dst)
    print(f"Guardado: {output_path}")

print("\nProceso completado.")