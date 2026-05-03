# 1- Start WSL2 Ubuntu 22.04
    wsl -d Ubuntu-22.04

# 2 - Move t source folder
    cd ~/projects/test_task 
# 3 - Docker run
    cd sim/
    docker compose up --build -d 

# 4 - Launch two simulators for 2 drones, in two different tabs
    docker exec -it sim_candidate bash -lc 'cd ~/workspace/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -I0 --sysid 1 --out=udp:127.0.0.1:14550'
    docker exec -it sim_candidate bash -lc 'cd ~/workspace/ardupilot/ArduCopter && ../Tools/autotest/sim_vehicle.py -v ArduCopter -I1 --sysid 2 --out=udp:127.0.0.1:14551'
# 7 - It another tab. Start second camera 
    gz sim ~/workspace/src/ardupilot_gazebo/worlds/gimbal.sdf -s -r -v 4

# 8 - In another tab. Run GUI app
     cd /opt/workspace/sim

    python gui_main.py \
    --port1 udpin:127.0.0.1:14550 \
    --route1 square_route.json \
    --port2 udpin:127.0.0.1:14551 \
    --route2 octagon_route.json \
    --video_topic1 "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image" \
    --video_topic2 "/world/gimbal/model/mount/model/gimbal/link/pitch_link/sensor/camera/image" 


    