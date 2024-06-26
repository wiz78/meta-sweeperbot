FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += "\
	file://wpa_supplicant-wlan0.conf \
"

FILES:${PN} += "${sysconfdir}/wpa_supplicant/wpa_supplicant-wlan0.conf"

do_install:append() {
	install -d ${D}${sysconfdir}/wpa_supplicant
	install -m 0755 ${WORKDIR}/wpa_supplicant-wlan0.conf ${D}${sysconfdir}/wpa_supplicant/

	install -d ${D}${sysconfdir}/systemd/system/multi-user.target.wants
	ln -sf ${systemd_unitdir}/system/wpa_supplicant@.service ${D}${sysconfdir}/systemd/system/multi-user.target.wants/wpa_supplicant@wlan0.service
}
