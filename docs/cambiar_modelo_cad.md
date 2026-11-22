# Integración del modelo CAD (Fusion 360) en la simulación

> **Estado (2026-11-22): integrado con la Opción A.** Los STL de `ensamblaje_description` están en `src/ackermann_description/meshes/` con sus nombres originales de Fusion, y el Xacro los rota 90° (en el CAD el frente apunta a −Y) y los centra en cada link. No se usó el URDF exportado porque trae masas irreales (chasis 8.7 kg), dirección `continuous` sin límites con ambos pivotes en el centro, y los nombres de las ruedas delanteras cruzados. Nota: en el CAD la distancia entre ejes mide 135.1 mm; la simulación usa 133.87 mm de la tabla.

El robot provisional usa cajas y cilindros con los parámetros de la tabla. Hay dos formas de integrar el modelo de B.

## Opción A (recomendada): solo cambiar los meshes visuales

Las colisiones e inercias del Xacro ya cumplen la tabla de parámetros, así que la física sigue siendo correcta y solo cambia la apariencia. Es la opción más rápida y la que menos se rompe.

1. Copia los STL exportados a `src/ackermann_description/meshes/` con **estos nombres exactos**:
   `base_link.stl`, `front_left_steer.stl`, `front_right_steer.stl`,
   `front_left_wheel.stl`, `front_right_wheel.stl`, `rear_left_wheel.stl`, `rear_right_wheel.stl`
2. Comprueba las unidades: el exportador de fusion2urdf usa mm y aplica `scale="0.001 0.001 0.001"`. Si el robot aparece 1000 veces más grande, agrega ese `scale` a los `<mesh>` del Xacro.
3. El origen de cada STL debe coincidir con el origen de su link:
   - `base_link.stl` (chasis): centrado en el chasis. El Xacro lo coloca a +L/2 del eje trasero.
   - ruedas: centradas en el eje de giro de la rueda.
   - portarruedas: en el eje vertical de dirección (kingpin).

   Si alguno queda desplazado, corrige el `<origin>` del `<visual>` correspondiente en `urdf/ackermann.urdf.xacro`.
4. Compila y lanza con los meshes:
   ```bash
   colcon build --symlink-install && source install/setup.bash
   ros2 launch ackermann_controller run_all.launch.py use_meshes:=true
   ```

## Opción B: usar el URDF completo exportado por fusion2urdf-ros2

1. Copia la carpeta exportada dentro de `src/` (es otro paquete `*_description`).
2. Revisa en el URDF exportado:
   - Los nombres de los joints deben ser **exactamente** los acordados (Fusion los llama `Revolute 1`, `Revolute 2`…, así que hay que renombrarlos):
     `front_left_steer_joint`, `front_right_steer_joint` (revolute, eje Z, límites ±0.5236 rad),
     `front_left_wheel_joint`, `front_right_wheel_joint`, `rear_left_wheel_joint`, `rear_right_wheel_joint` (continuous, eje Y).
   - Los ejes de los joints: rueda = `0 1 0`, dirección = `0 0 1`. Si una rueda gira al revés, cambia el signo del eje.
   - Las inercias deben ser positivas y razonables (masa total ≈ 2.9 kg).
   - Las colisiones de las ruedas conviene reemplazarlas por cilindros (r = 0.05, ancho 0.03); con meshes, el contacto en Gazebo es inestable.
3. Agrega al final del URDF el include de `ackermann.gazebo.xacro` (plugins) y los bloques `<gazebo reference=...>` de fricción de las ruedas, copiados de `ackermann.urdf.xacro`.
4. Cambia la ruta del Xacro en `src/ackermann_gazebo/launch/sim.launch.py`.
5. Vuelve a ajustar el controlador si hace falta (`lookahead_distance`, `linear_speed` en `config/pure_pursuit.yaml`) y corre `bash scripts/run_experiments.sh`.

## Checklist después del cambio
- [ ] `check_urdf` sin errores (`xacro ... > /tmp/r.urdf && check_urdf /tmp/r.urdf`)
- [ ] El robot no se hunde ni salta al aparecer en Gazebo
- [ ] Con teleop, al girar a la izquierda la rueda delantera izquierda gira más que la derecha
- [ ] La figura en 8 se completa y el CSV se guarda
- [ ] Los parámetros coinciden con la tabla (L, W, r, δmáx)
