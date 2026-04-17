import numpy as np
import math
import csv
import os
from pathlib import Path

def quaternion_to_rotation_matrix(qx, qy, qz, qw):
    """Convierte un quaternion a matriz de rotación 3x3"""
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R

def utm_to_latlon(easting, northing, zone=30, northern=True):
    """
    Convierte coordenadas UTM a latitud/longitud
    """
    # Constantes WGS84
    a = 6378137.0  # semi-eje mayor
    f = 1/298.257223563  # aplanamiento
    k0 = 0.9996  # factor de escala
    
    # Parámetros derivados
    e = math.sqrt(2*f - f*f)  # primera excentricidad
    e_sq = e * e
    
    # Falso Este y Norte
    false_easting = 500000.0
    false_northing = 0.0 if northern else 10000000.0
    
    # Remover falsos valores
    x = easting - false_easting
    y = northing - false_northing
    
    # Meridiano central de la zona
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    
    # Cálculo inverso
    M = y / k0
    mu = M / (a * (1 - e_sq/4 - 3*e_sq*e_sq/64 - 5*e_sq*e_sq*e_sq/256))
    
    e1 = (1 - math.sqrt(1 - e_sq)) / (1 + math.sqrt(1 - e_sq))
    
    phi1 = mu + (3*e1/2 - 27*e1*e1*e1/32) * math.sin(2*mu) + \
           (21*e1*e1/16 - 55*e1*e1*e1*e1/32) * math.sin(4*mu) + \
           (151*e1*e1*e1/96) * math.sin(6*mu)
    
    N1 = a / math.sqrt(1 - e_sq * math.sin(phi1)**2)
    T1 = math.tan(phi1)**2
    C1 = e_sq * math.cos(phi1)**2 / (1 - e_sq)
    R1 = a * (1 - e_sq) / (1 - e_sq * math.sin(phi1)**2)**1.5
    D = x / (N1 * k0)
    
    lat = phi1 - (N1 * math.tan(phi1) / R1) * \
          (D*D/2 - (5 + 3*T1 + 10*C1 - 4*C1*C1 - 9*e_sq) * D*D*D*D/24 + \
           (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*e_sq - 3*C1*C1) * D*D*D*D*D*D/720)
    
    lon = lon0 + (D - (1 + 2*T1 + C1) * D*D*D/6 + \
                  (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*e_sq + 24*T1*T1) * D*D*D*D*D/120) / math.cos(phi1)
    
    return math.degrees(lat), math.degrees(lon)

def transform_tree_to_world(tree_lidar, R_lidar_to_gnss, T_lidar_to_gnss, quaternion, utm_vehicle):
    """
    Transforma un punto de árbol desde coordenadas LiDAR a coordenadas mundiales (lat/lon)
    
    Args:
        tree_lidar: np.array [x, y, z] en coordenadas LiDAR
        R_lidar_to_gnss: matriz 3x3 de rotación LiDAR to GNSS
        T_lidar_to_gnss: np.array [x, y, z] de traslación LiDAR to GNSS
        quaternion: tuple (qx, qy, qz, qw) orientación del vehículo
        utm_vehicle: np.array [x, y, z] posición UTM del vehículo
    
    Returns:
        tuple: (latitud, longitud, altitud, utm_x, utm_y, utm_z)
    """
    # Paso 1: Transformar de LiDAR a GNSS (frame del vehículo)
    point_gnss = R_lidar_to_gnss @ tree_lidar + T_lidar_to_gnss
    
    # Paso 2: Obtener matriz de rotación del vehículo
    qx, qy, qz, qw = quaternion
    R_vehicle_to_world = quaternion_to_rotation_matrix(qx, qy, qz, qw)
    
    # Paso 3: Rotar al frame del mundo
    point_world = R_vehicle_to_world @ point_gnss
    
    # Paso 4: Sumar a la posición UTM del vehículo
    utm_cluster = utm_vehicle + point_world
    
    # Paso 5: Convertir a lat/lon
    lat, lon = utm_to_latlon(utm_cluster[0], utm_cluster[1], zone=30, northern=True)
    
    return lat, lon, utm_cluster[2], utm_cluster[0], utm_cluster[1], utm_cluster[2]

def read_tree_file(filepath):
    """Lee un archivo de árboles y retorna lista de coordenadas"""
    trees = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            # Saltar comentarios y líneas vacías
            if line.startswith('#') or not line:
                continue
            parts = line.split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    trees.append([x, y, z])
                except ValueError:
                    continue
    return trees

def read_gnss_file(filepath):
    """Lee un archivo GNSS CSV y retorna los datos como lista de valores"""
    data = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        # Saltar header
        next(reader, None)
        # Leer primera fila de datos
        for row in reader:
            try:
                data = [float(value) for value in row]
                return data
            except ValueError:
                continue
    return data

def read_calibration_file(filepath):
    """Lee archivo de calibración LiDAR to GNSS
    
    Formato esperado:
    R: 0.9992 -0.0040 0.0400 0.0042 0.9999 -0.0039 -0.0399 0.0041 0.9992
    T: 1.435 0.008 0.574
    """
    R = None
    T = None
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('#') or not line:
                continue
            
            if line.startswith('R:'):
                # Parsear matriz de rotación 3x3
                values = line.replace('R:', '').split()
                if len(values) == 9:
                    R = np.array([float(x) for x in values]).reshape(3, 3)
            
            elif line.startswith('T:'):
                # Parsear vector de traslación
                values = line.replace('T:', '').split()
                if len(values) == 3:
                    T = np.array([float(x) for x in values])
    
    if R is None or T is None:
        raise ValueError(f"Archivo de calibración inválido: {filepath}")
    
    return R, T

def process_all_trees(trees_dir, gnss_dir, output_csv, calib_file):
    """
    Procesa todos los archivos de árboles y genera un CSV con coordenadas geográficas
    
    Args:
        trees_dir: directorio con archivos de árboles (00000.txt, 00001.txt, ...)
        gnss_dir: directorio con subcarpetas fix, imu, utm
        output_csv: ruta del archivo CSV de salida
        calib_file: ruta del archivo de calibración calib_lidar_0_to_gnss.txt
    """
    # Leer transformación LiDAR to GNSS desde archivo de calibración
    try:
        R_lidar_to_gnss, T_lidar_to_gnss = read_calibration_file(calib_file)
        print(f"✓ Calibración cargada desde: {calib_file}")
    except FileNotFoundError:
        print(f"Error: No se encontró archivo de calibración: {calib_file}")
        return
    except ValueError as e:
        print(f"Error: {e}")
        return
    
    # Preparar para escribir CSV
    results = []
    
    # Obtener lista de archivos de árboles
    tree_files = sorted([f for f in os.listdir(trees_dir) if f.endswith('.txt')])
    
    print(f"Encontrados {len(tree_files)} archivos de árboles")
    
    for tree_file in tree_files:
        # Extraer número de frame del nombre del archivo (00000.txt -> 0)
        frame_num = tree_file.replace('.txt', '')
        
        # Leer árboles de este frame
        trees = read_tree_file(os.path.join(trees_dir, tree_file))
        
        if not trees:
            print(f"Frame {frame_num}: No se encontraron árboles")
            continue
        
        # Leer datos GNSS correspondientes (archivos CSV)
        try:
            imu_file = os.path.join(gnss_dir, 'imu', frame_num + '.csv')
            utm_file = os.path.join(gnss_dir, 'utm', frame_num + '.csv')
            
            imu_data = read_gnss_file(imu_file)
            utm_data = read_gnss_file(utm_file)
            
            if len(imu_data) < 4 or len(utm_data) < 3:
                print(f"Frame {frame_num}: Datos GNSS incompletos")
                continue
            
            # Extraer quaternion (qx, qy, qz, qw)
            quaternion = tuple(imu_data[:4])
            
            # Extraer UTM (x, y, z)
            utm_vehicle = np.array(utm_data[:3])
            
            # Procesar cada árbol en este frame
            for tree_idx, tree in enumerate(trees):
                tree_lidar = np.array(tree)
                
                # Transformar a coordenadas mundiales
                lat, lon, alt, utm_x, utm_y, utm_z = transform_tree_to_world(
                    tree_lidar, 
                    R_lidar_to_gnss, 
                    T_lidar_to_gnss, 
                    quaternion, 
                    utm_vehicle
                )
                
                # Guardar resultado
                results.append({
                    'frame': frame_num,
                    'tree_id': tree_idx,
                    'latitude': lat,
                    'longitude': lon
                })
            
            print(f"Frame {frame_num}: {len(trees)} árboles procesados")
            
        except FileNotFoundError as e:
            print(f"Frame {frame_num}: Archivo GNSS no encontrado - {e}")
            continue
        except Exception as e:
            print(f"Frame {frame_num}: Error - {e}")
            continue
    
    # Escribir CSV
    if results:
        with open(output_csv, 'w', newline='') as csvfile:
            fieldnames = ['frame', 'tree_id', 'latitude', 'longitude']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            for row in results:
                writer.writerow(row)
        
        print(f"\n✓ Procesamiento completado!")
        print(f"✓ Total de árboles procesados: {len(results)}")
        print(f"✓ Archivo generado: {output_csv}")
    else:
        print("\n✗ No se procesaron árboles")

if __name__ == "__main__":
    # Configurar rutas
    trees_dir = "/home/scedenilla/pointcloud_test/resultados/ejemplo/centroides"
    gnss_dir = "/home/scedenilla/pointcloud_test/test_sincro/data/gnss"
    calib_file = "/home/scedenilla/pointcloud_test/calibs/calib_lidar_0_to_gnss.txt"
    output_csv = "/home/scedenilla/pointcloud_test/tree_coordinates_ejemplo.csv"
    
    # Verificar que los directorios y archivos existen
    if not os.path.exists(trees_dir):
        print(f"Error: No se encuentra el directorio de árboles: {trees_dir}")
    elif not os.path.exists(gnss_dir):
        print(f"Error: No se encuentra el directorio GNSS: {gnss_dir}")
    elif not os.path.exists(calib_file):
        print(f"Error: No se encuentra el archivo de calibración: {calib_file}")
    else:
        process_all_trees(trees_dir, gnss_dir, output_csv, calib_file)