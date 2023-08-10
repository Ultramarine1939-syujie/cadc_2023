sudo chmod 777 /dev/ttyACM0
sudo chmod 777 /dev/ttyAMA0
sudo chmod 777 /dev/video0
sudo systemctl restart frpc.service
sudo systemctl status frpc.service
