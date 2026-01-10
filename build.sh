echo "EAGLE WORKS PROJECT BUILDER V1 de KK7OYV"
echo 
echo "This will build and install our project to the robot."
echo "Run with 'usb' after to install over usb instead."
echo 
echo 
if [[ "$1" == "usb" ]]; then
	echo "disconnecting wifi connections"
	echo "Error is normal!"
	adb disconnect 192.168.43.1

else
	echo "Trying to connect over wifi..."
	echo "Please make sure your network is the robot's network."
	echo "If frozen here check wifi connections to robot."
	echo "No such device error is normal for first connection."
	echo 
	adb disconnect 192.168.43.1
	adb connect 192.168.43.1:5555
fi
echo 
echo "Ready to build. \n Building..."
echo 
./gradlew assembleDebug
echo "build finished. Installing..."
echo 
echo 
adb install -r /home/eagleworks/Desktop/2025Season/FTCRobotController/TeamCode/build/outputs/apk/debug/TeamCode-debug.apk
echo "Done"
