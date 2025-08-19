# Evaluación Cuantitativa de Navegación ROS 2

[![Code Size](https://img.shields.io/github/languages/code-size/{user}/{repo}.svg)](https://github.com/{user}/{repo}) [![Last Commit](https://img.shields.io/github/last-commit/{user}/{repo}.svg)](https://github.com/{user}/{repo}/commits/main) [![GitHub issues](https://img.shields.io/github/issues/{user}/{repo})](https://github.com/{user}/{repo}/issues) [![GitHub pull requests](https://img.shields.io/github/issues-pr/{user}/{repo})](https://github.com/{user}/{repo}/pulls) [![Contributors](https://img.shields.io/github/contributors/{user}/{repo}.svg)](https://github.com/{user}/{repo}/graphs/contributors)

## Descripción

Este proyecto permite evaluar cuantitativamente el desempeño de sistemas de navegación autónoma en ROS 2 a partir de rosbag2. Calcula métricas clave de navegación y genera visualizaciones útiles para el análisis de misiones de robots móviles.

**Métricas calculadas por bag (misión):**
- Tasa de éxito (alcance de meta dentro de tolerancia y antes de timeout)
- Tiempo de misión (segundos)
- RMSE entre trayectoria seguida y plan global (metros)
- Distancia extra recorrida (%)
- Número y duración total de bloqueos (paradas no planificadas)

**Visualizaciones generadas:**
- Trayectoria seguida vs. plan global (2D)
- Velocidades lineal y angular vs. tiempo (con bloqueos resaltados)
- Distancia al goal vs. tiempo

## Sistemas probados y versiones ROS 2

| Sistema        | ROS 2 Distro | Ignition Fortress | 
|---------------|--------------|-------------------|
| Ubuntu 22.04  | Humble       | ✅                | 

## Requisitos

- ROS 2 Humble (u otra versión compatible)
- Python 3.8+
- Paquetes: `rclpy`, `rosbag2_py`, `rosidl_runtime_py`, `numpy`, `pandas`, `matplotlib`

Instala dependencias Python (además de ROS 2):

```sh
pip install numpy pandas matplotlib
```

## Uso

1. **Ejecuta el script de evaluación sobre un rosbag2:**

```sh
python3 evaluation.py --bag /ruta/al/rosbag2
```

### Argumentos principales

- `--bag`: Ruta al directorio del rosbag2 (obligatorio)
- `--use_pose`: Topic de pose a usar (`/odom` o `/amcl_pose`, por defecto `/odom`)
- `--plan_topic`: Topic del plan global (por defecto `/plan`)
- `--cmd_topic`: Topic de comandos de velocidad (por defecto `/cmd_vel`)
- `--goal_topic`: Topic del goal (por defecto `/goal_pose`)
- `--goal_tolerance`: Tolerancia de distancia al goal (por defecto `0.3`)
- `--timeout`: Timeout de la misión en segundos (por defecto `300`)
- `--block_v_thresh`: Umbral de velocidad para detectar bloqueos (por defecto `0.05`)
- `--block_t_thresh`: Duración mínima para considerar un bloqueo (por defecto `3.0`)
- `--out_dir`: Carpeta de salida para resultados (por defecto `./resultados`)

### Ejemplos de uso

**Ejemplo básico:**

```sh
python3 evaluation.py --bag ./bag
```

**Configuración personalizada:**

```sh
python3 evaluation.py --bag ./bag --use_pose /odom --plan_topic /plan --cmd_topic /cmd_vel --goal_topic /goal_pose --goal_tolerance 0.3 --timeout 600 --block_v_thresh 0.02 --block_t_thresh 2.0 --out_dir ./resultados
```

**Evaluación con filtrado avanzado:**

```sh
# Mayor tolerancia para robots grandes
python3 evaluation.py --bag ./robot_grande_bag --goal_tolerance 0.5 --block_v_thresh 0.01

# Timeout más corto para pruebas rápidas
python3 evaluation.py --bag ./prueba_rapida --timeout 60

# Usar pose filtrada de AMCL en lugar de odometría
python3 evaluation.py --bag ./nav_con_localizacion --use_pose /amcl_pose
```

**Analizar múltiples bags (usando shell script):**

```sh
#!/bin/bash
for bag in ./bags/*; do
  echo "Procesando $bag..."
  python3 evaluation.py --bag "$bag" --out_dir "./resultados/$(basename $bag)"
done
```

**Verificar topics disponibles en un bag antes de analizar:**

```sh
# Primero verificamos qué topics están disponibles
ros2 bag info ./bag

# Luego ejecutamos el script con los topics correctos
python3 evaluation.py --bag ./bag --use_pose /robot/odom --plan_topic /robot/plan
```

## Salidas

- `metricas.csv`: Resumen de métricas de la misión.
- `bloqueos.csv`: Detalle de bloqueos detectados (si existen).
- `trayectoria_vs_plan.png`: Gráfica 2D de la trayectoria seguida vs. plan.
- `velocidades.png`: Velocidades lineal y angular vs. tiempo, con bloqueos resaltados.
- `distancia_goal.png`: Distancia al goal vs. tiempo.

Todos los archivos se guardan en el directorio especificado por `--out_dir`.

## Notas

- El script requiere acceso a los mensajes de los topics indicados en el rosbag2.
- Si tienes dudas sobre los nombres de los topics, puedes inspeccionarlos con `ros2 bag info <bag_dir>`.
- El umbral de velocidad (`--block_v_thresh`) debe ajustarse según el robot: valores típicos entre 0.01-0.05 m/s.
- El umbral de tiempo para bloqueos (`--block_t_thresh`) define cuántos segundos debe estar detenido el robot para considerarlo un bloqueo.