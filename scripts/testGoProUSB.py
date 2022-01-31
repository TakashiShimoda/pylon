## the goprocam python API module and example can be found from
## https://github.com/KonradIT/gopro-py-api

## we just plug the USB directly to pc
## make sure the gopro is using GoPro Connect, not MTP

import cv2
import sys
from goprocam import GoProCamera, constants
import time
import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

## make sure th ethernet inteface name is correctly
## when plugging USB from gopro to pc, the gopro will works as DHCP server
# gopro = GoProCamera.GoPro(ip_address=GoProCamera.GoPro.getWebcamIP("enp0s20f0u1"),camera=constants.gpcontrol, webcam_device="enp0s20f0u4")
gopro = GoProCamera.GoPro(ip_address="172.26.174.51")
gopro.webcamFOV(constants.Webcam.FOV.Wide)

## we use gopro as webcam with slightly low resolution, or can try with 1080 as well
gopro.startWebcam(resolution="480")   #480

## using livestream will end up at error after some times
# gopro.livestream("start")

# gopro.video_settings(res='480p', fps='30')
# gopro.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
cap = cv2.VideoCapture("udp://172.26.174.51:8554", cv2.CAP_FFMPEG)   # , cv2.CAP_FFMPEG
t = time.time()
while True:
	
	nmat, frame = cap.read()
	cv2.imshow("GoPro OpenCV", frame)
	# frame = cv2.resize(frame, (1696, 960))
	print(frame.shape)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()

