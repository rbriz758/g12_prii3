import numpy as np
import cv2
import glob
import os

# Definir las dimensiones del tablero de ajedrez (número de esquinas internas)
# IMPORTANTE: El usuario indicó 10x14 cuadros, por lo que las esquinas internas son (10-1) x (14-1) = 9x13
CHECKBOARD = (9, 13)

# Criterios de terminación para el refinamiento de esquinas (subpixel)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Preparar puntos de objeto 3D, como (0,0,0), (1,0,0), (2,0,0) ...
objp = np.zeros((CHECKBOARD[0] * CHECKBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKBOARD[0], 0:CHECKBOARD[1]].T.reshape(-1, 2)

# Arrays para almacenar puntos de objeto y puntos de imagen de todas las imágenes
objpoints = [] # Puntos 3d en el espacio del mundo real
imgpoints = [] # Puntos 2d en el plano de la imagen

# Ruta de la carpeta con las fotos
# Se asume que la carpeta 'ImagenesCalibracion' está en el mismo directorio que este script
folder_path = './ImagenesCalibracion'
images_path = os.path.join(folder_path, '*.png')

images = glob.glob(images_path)

print(f"Buscando imágenes en: {images_path}")
print(f"Imágenes encontradas: {len(images)}")

if not images:
    print("No se encontraron imágenes. Verifica la ruta y la extensión de los archivos.")
    exit()

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"No se pudo leer la imagen: {fname}")
        continue
        
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Encontrar las esquinas del tablero de ajedrez
    ret, corners = cv2.findChessboardCorners(gray, CHECKBOARD, None)

    # Si se encuentran, añadir puntos de objeto y puntos de imagen
    if ret == True:
        objpoints.append(objp)

        # Refinar la precisión de las esquinas
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Opcional: Dibujar y mostrar las esquinas
        # img = cv2.drawChessboardCorners(img, CHECKBOARD, corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(100)
        print(f"Esquinas encontradas en: {fname}")
    else:
        print(f"No se encontraron esquinas en: {fname}")

cv2.destroyAllWindows()

# Calibración de la cámara
if len(objpoints) > 0:
    print("\nRealizando calibración...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\nCalibración exitosa.")
    print("Matriz de la cámara (mtx):\n", mtx)
    print("Coeficientes de distorsión (dist):\n", dist)

    # Calcular el error de reproyección
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print(f"\nError total de reproyección: {mean_error/len(objpoints)}")

    # Guardar los datos de calibración
    np.savez('calib_data.npz', mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs)
    print("Datos guardados en 'calib_data.npz'")

else:
    print("\nNo se pudo calibrar: No se detectaron suficientes esquinas en las imágenes.")