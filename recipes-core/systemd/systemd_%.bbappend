FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += " file://80-wifi-station.network"

RDEPENDS:${PN}:remove = "systemd-serialgetty"

FILES:${PN} += "${sysconfdir}/systemd/network/80-wifi-station.network"

PACKAGECONFIG:append = " coredump"

do_install:append() {
    # Disable getty@tty1 from starting at boot time.
    sed -i -e "s/enable getty@.service/disable getty@.service/g" ${D}${systemd_unitdir}/system-preset/90-systemd.preset
	
	install -d ${D}${sysconfdir}/systemd/network
	install -m 0755 ${WORKDIR}/80-wifi-station.network ${D}${sysconfdir}/systemd/network
}