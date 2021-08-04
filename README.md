# create virutal camera: 
sudo modprobe v4l2loopback video_nr=1 card_label=VirtualCamera

# use virual camera
ffmpeg -i /dev/video0 -f v4l2 -codec:v rawvideo -pix_fmt yuv420p /dev/video1

#to program to identify and track the elderly 
---run ./run 

#-----to build again--------- 
go to pedestrian_detection folder -> run cmake .. and go to build folder -> run make 

#------ to run WebRTC -----
go to momo folder -> run 

----streaming on intranet: ./momo \--hw-mjpeg-decoder=false \--video-device=/dev/video1 test

----streaming on internet: 
./momo --hw-mjpeg-decoder true --framerate 30 --log-level 2 sora \wss://sora-labo.shiguredo.jp/signaling TungTT1311@TungTT1311 \--video true --audio false \--video-codec-type VP8 --video-bit-rate 15000 \--auto --role sendonly --multistream false \--metadata '{"signaling_key": "Kh4LqMn-ahQsK7lPspeyaimJcpjLlv53W7xa5Q009VOO6WLS"}'