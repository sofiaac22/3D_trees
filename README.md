# 🌲 GUÍA RÁPIDA DE USO

---

## 1️⃣ CONFIGURACIÓN DE RUTAS

Antes de ejecutar el programa, es necesario revisar y ajustar las rutas definidas en el archivo `main.cpp`.

### 🔧 Variables a modificar:

```cpp
std::string base_path = "../";
std::string sequence = "test_sincro";
std::string camera_name = "camera_0";
std::string lidar_name = "lidar_0";
std::string output_dir = "../resultados/ejemplo";
```

### 📌 Descripción:

- **base_path** → Ruta base donde se encuentran los datos y calibraciones  
- **sequence** → Nombre de la secuencia de datos  
- **camera_name** → Nombre de la carpeta de la cámara  
- **lidar_name** → Nombre de la carpeta del LiDAR  
- **output_dir** → Carpeta donde se guardarán los resultados  

---

## 2️⃣ ESTRUCTURA DE DATOS ESPERADA

Con la configuración anterior, el programa espera encontrar los datos en las siguientes rutas:

### 📐 Calibraciones
```
../calibs/calib_lidar_0_to_camera_0.txt
../calibs/calib_cam_to_camera_0.txt
```

### 📂 Datos
```
../test_sincro/data/camera_0/<timestamp>.png
../test_sincro/data/lidar_0/<timestamp>.bin
../test_sincro/data/annotations/<timestamp>.txt
```

### 📊 Resultados
```
../resultados/ejemplo/
```

> ⚠️ Si tus datos están en otras ubicaciones, modifica las variables indicadas en el punto 1.

---

## 3️⃣ COMPILACIÓN

Desde el directorio `build`:

```bash
make
```

---

## 4️⃣ EJECUCIÓN

Desde el mismo directorio:

```bash
./main
```

---
