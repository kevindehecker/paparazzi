killall server



rm -rf tmp
mkdir -p tmp
cd tmp

telnet 192.168.1.1 <<script2

killall ap.elf
killall IC
mount /dev/sda1 /data/video/stick/
script2

sleep 1

telnet 192.168.1.1 <<script2


rm -f /data/video/stick/data.tar
tar -cf /data/video/stick/data.tar /data/video/drone && mv /data/video/stick/data.tar /data/video/stick/data_up.tar
script2

sleep 10

ftp 192.168.1.1 <<script


cd stick
get data_up.tar
bye
script



cp `ls -rt ~/paparazzi/var/logs/*.data | tail -1` ./
cp `ls -rt ~/paparazzi/var/logs/*.log | tail -1` ./
