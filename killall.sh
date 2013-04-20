service hubo-motion stop
service hubo-motion kill
sudo rm /var/lock/hubo/control-daemon
hubo-ach killall
sudo kill -9 $(pidof lt-proto-manip-daemon)
sudo rm /var/lock/hubo/proto-manip-daemon
