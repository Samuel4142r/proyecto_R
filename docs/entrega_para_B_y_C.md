# Material de A (simulación) para B y C

## Para B (CAD y diapositivas)

**Convenciones que debe respetar el CAD:**
- Nombres de los componentes: `base_link`, `front_left_steer`, `front_right_steer`, `front_left_wheel`, `front_right_wheel`, `rear_left_wheel`, `rear_right_wheel`.
- Joints de dirección: revolute en Z, ±18°. Joints de rueda: revolute en el eje de la rueda.
- Parámetros: L = 0.1339 m, W = 0.180 m, r = 0.040 m, ancho de rueda 0.020 m, δmáx = 18°, chasis 0.240 × 0.120 × 0.045 m.
- Meshes del ensamblaje de Fusion ya integrados como apariencia del robot (ver sección 9 del README).

**Para las diapositivas:**
- Diapositiva 6 (de Fusion a Gazebo): `docs/nodes_topics.png` (o `.svg`)
- Diapositiva 7 (controlador): sección 7 del README (lógica de Pure Pursuit, sin código)
- Diapositiva 8 (resultados): `results/plots/xy_fig8_run1.png`, `results/plots/error_fig8_run1.png` y un clip del video de simulación

## Para C (documento IEEE)

**Figuras** (en inglés, listas para el paper):
| Figura | Archivo |
|---|---|
| Trayectoria deseada vs. odometría vs. pose real | `results/plots/xy_fig8_run1.png` |
| Error de seguimiento en el tiempo | `results/plots/error_fig8_run1.png` |
| Deriva de la odometría (posición y rumbo) | `results/plots/drift_fig8_run1.png` |
| Repetibilidad (3 corridas) | `results/plots/error_all_runs.png` |
| Comparación con realimentación ideal | `results/plots/xy_fig8_gt.png` |
| Arquitectura de nodos/tópicos | `docs/nodes_topics.png` |

**Tabla de errores** (figura en 8 de 7 m × 3 m, longitud 19.9 m, v = 0.5 m/s, Ld = 0.6 m):

| Run | Feedback | Real error max | Real error RMS | Estimated error RMS | Final odom drift | Max heading drift |
|---|---|---|---|---|---|---|
| 1 | wheel odometry | 46.8 cm | 23.8 cm | 1.2 cm | 45.6 cm | 4.9° |
| 2 | wheel odometry | 46.8 cm | 23.8 cm | 1.2 cm | 45.6 cm | 4.9° |
| 3 | wheel odometry | 47.0 cm | 23.8 cm | 1.2 cm | 45.6 cm | 4.9° |
| GT | ground-truth pose | 2.5 cm | 1.3 cm | — | — | — |

**Datos del experimento para Métodos/Resultados:**
- Simulador: Gazebo Harmonic (gz-sim 8.15), paso de física de 1 ms, ROS 2 Jazzy.
- Plugins: `AckermannSteering` (tracción trasera y dirección delantera, odometría de ruedas), `JointStatePublisher`, `OdometryPublisher` (pose real).
- Trayectoria: lemniscata de Gerono x = 3.5 sin t, y = 1.5 sin 2t; radio de curvatura mínimo ≈ 1.0 m (radio mínimo de giro del robot: 0.412 m).
- Controlador: Pure Pursuit a 30 Hz, Ld = 0.6 m, δ saturado a ±18°.
- Registro a 20 Hz (CSV): pose deseada, odometría, pose real y comandos.

**Interpretación (para la sección VI y la VII):**
1. El controlador sigue la ruta con ~1.2 cm RMS respecto a la pose que recibe como entrada.
2. El error real (23.8 cm RMS) viene de la **deriva de la odometría de ruedas**: el deslizamiento en las curvas acumula hasta 4.9° de error de rumbo y 45.6 cm de error de posición al final (≈ 2.3 % de la distancia recorrida).
3. Con la pose real como realimentación, el error real baja a 1.3 cm RMS (2.5 cm máx.). La mejora propuesta en la sección VII (EKF que fusione odometría + IMU + GPS con `robot_localization`) ataca justo este problema.
4. Las 3 corridas son casi idénticas (diferencias < 0.5 mm en la deriva): la simulación es repetible.

## Video de simulación (lo graba A)

1. `ros2 launch ackermann_controller run_all.launch.py run_name:=video`
2. Antes de que el robot arranque (~8 s), ordena las ventanas: Gazebo a la izquierda y RViz2 a la derecha.
3. Graba con OBS: *Fuente → Captura de pantalla (XSHM/PipeWire)* y *Archivo → Ajustes → Salida → Formato: mp4*. Detén la grabación cuando el robot se detenga (~40 s).
4. En Gazebo, clic derecho sobre el robot → *Follow* para que la cámara lo siga (opcional).
