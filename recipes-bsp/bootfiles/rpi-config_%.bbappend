do_deploy:append:raspberrypi3-64() {
	# disable HDMI
    sed -i '/# Enable VC4 Graphics/d' $CONFIG
    sed -i '/dtoverlay=vc4-fkms-v3d/d' $CONFIG
	# disable audio
    sed -i '/# Enable audio/d' $CONFIG
    sed -i '/dtparam=audio=on/d' $CONFIG
}