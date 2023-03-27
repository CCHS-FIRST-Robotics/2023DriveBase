

install:
	[ "`id -u`" = "0" ] || { echo "Must be run as root"; exit 1; }
	install -m 0644 root/etc/udev/rules.d/50-USB_cam.rules /etc/udev/rules.d
	install -m 0755 root/usr/local/bin/stream.sh /usr/local/bin
	install -m 0644 root/etc/systemd/system/stream.service /etc/systemd/system
	udevadm control --reload-rules; 
	systemctl enable stream.service

uninstall:
	[ "`id -u`" = "0" ] || { echo "Must be run as root"; exit 1; }
	systemctl stop stream.service
	systemctl disable stream.service
	test -f /etc/systemd/system/stream.service && rm -f /etc/systemd/stream.service
	test -f /usr/local/bin/stream.sh && rm -f /usr/local/bin/stream.sh
	test -f /etc/udev/rules.d/50-USB_cam.rules && rm -f /etc/udev/rules.d/50-USB_cam.rules
	udevadm control --reload-rules


