#!/usr/bin/env bash
# Vuelca todo lo que el robot expone por ROS. Se corre dentro del contenedor.
source /tmp/tb4_env
O=/opt/tb4/info_out; mkdir -p $O
L="--no-daemon --spin-time 6"
ros2 topic list -t $L > $O/topicos.txt
ros2 node list $L > $O/nodos.txt
ros2 service list -t $L > $O/servicios.txt
ros2 action list -t > $O/acciones.txt 2>&1
for t in /battery_state /dock_status /wheel_status /hazard_detection /ir_intensity /imu /mouse \
         /interface_buttons /ip /hmi/display /hmi/led /kidnap_status /slip_status /stop_status \
         /wheel_vels /wheel_ticks /cliff_intensity /robot_description /joint_states /odom /scan \
         /oakd/rgb/preview/camera_info /diagnostics /function_calls /cmd_audio; do
  if grep -q "^$t " $O/topicos.txt; then
    echo "===== $t" >> $O/mensajes.txt
    timeout 8 ros2 topic echo --once --no-arr --qos-reliability best_effort $t 2>&1 | head -60 >> $O/mensajes.txt \
    || timeout 8 ros2 topic echo --once --no-arr $t 2>&1 | head -60 >> $O/mensajes.txt
  fi
done
for t in /scan /odom /imu /oakd/rgb/preview/image_raw/compressed /battery_state /tf; do
  grep -q "^$t " $O/topicos.txt && { echo "== $t"; timeout 6 ros2 topic hz $t 2>&1 | grep average | tail -1; }
done > $O/frecuencias.txt
for n in $(cat $O/nodos.txt); do echo "===== $n"; timeout 10 ros2 param dump $n 2>&1 | head -80; done > $O/parametros.txt
echo listo
