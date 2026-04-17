POINTCLOUD TEST - GUÍA RÁPIDA DE USO

1. CONFIGURACIÓN DE RUTAS

Antes de ejecutar el programa, es necesario revisar y ajustar las rutas definidas en el archivo main.cpp.

Las variables a modificar son:

    std::string base_path = "../";
    std::string sequence = "test_sincro";
    std::string camera_name = "camera_0";
    std::string lidar_name = "lidar_0";
    std::string output_dir = "../resultados/ejemplo";

Descripción:

- base_path: ruta base donde se encuentran los datos y calibraciones.
- sequence: nombre de la secuencia de datos.
- camera_name: nombre de la carpeta de la cámara.
- lidar_name: nombre de la carpeta del LiDAR.
- output_dir: carpeta donde se guardarán los resultados.

2. ESTRUCTURA DE DATOS ESPERADA

Con la configuración anterior, el programa espera encontrar los datos en las siguientes rutas:

Calibraciones:
    ../calibs/calib_lidar_0_to_camera_0.txt
    ../calibs/calib_cam_to_camera_0.txt

Datos:
    ../test_sincro/data/camera_0/<timestamp>.png
    ../test_sincro/data/lidar_0/<timestamp>.bin
    ../test_sincro/data/annotations/<timestamp>.txt

Resultados:
    ../resultados/ejemplo/

Si tus datos están en otras ubicaciones, modifica las variables indicadas en el punto 1.

3. COMPILACIÓN

Desde el directorio de build:

    cd /home/scedenilla/pointcloud_test/build
    make

4. EJECUCIÓN

Desde el mismo directorio:

    ./main

