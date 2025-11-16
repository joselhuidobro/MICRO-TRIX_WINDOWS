sudo docker system prune -a
docker system prune -a --volumes
sudo docker-compose build
docker exec -it robot bash

source /opt/esp-idf/export.sh

prender rele desde terminal 



ros2 topic pub --once /relevadores std_msgs/msg/String "data: relay1_on"
ros2 topic pub --once --qos-reliability best_effort /servo_setpoint_deg std_msgs/msg/Float32 "{data: 90.0}"



idf.py fullclean
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
idf.py -p /dev/ttyUSB0 monitor	

sudo docker stop $(sudo docker ps -aq) 
sudo docker stop robot
sudo docker rm robot
sudo docker rmi ubuntu:robot # Or whatever your image name is

# Elimina todos los contenedores y volúmenes
sudo docker rm -f $(docker ps -aq) 2>/dev/null
sudo docker volume rm $(docker volume ls -q) 2>/dev/null

# Limpia el sistema Docker
sudo docker system prune -af
sudo docker system prune -a --volumes


sudo docker-compose up --build -d
sudo docker exec -it robot bash

grep "ESP_WIFI_" build/config/sdkconfig.h
 ls components | grep micro_ros
find components/micro_ros_espidf_component -type f -name '*transport*.h' | head
 idf.py reconfigure | sed -n '/-- Components:/,/--/p' | tail -n +2 | tr -d ','
Compiler supported targets: xtensa-esp32-elf


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

uidobro@gmail.com"
Generating public/private ed25519 key pair.
Enter file in which to save the key (/home/jose/.ssh/id_ed25519): 
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/jose/.ssh/id_ed25519.
Your public key has been saved in /home/jose/.ssh/id_ed25519.pub.
The key fingerprint is:
SHA256:e4fpxYS/zrZ/1dueSAAsfaoyhqPClSTeRfOq/nOCuX4 jhuidobro@gmail.com
The key's randomart image is:
+--[ED25519 256]--+
|                 |
|     o  o        |
|    . o. + .     |
| . . . .. +.     |
|. + o . S....   .|
| . +..  .. *.   o|
|. .o++ .. + =.  +|
|...+oEo. o +o...+|
|..o++.+   .o=+.+.|
+----[SHA256]-----+
jose@jose-desktop:~/Documents/2025master-DRONespui$ eval "$(ssh-agent -s)"


ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOBKp/JP16mTm8aCFe/DnGk78Dt+tUTNSt/Ox9L2u2lw jhuidobro@gmail.com

