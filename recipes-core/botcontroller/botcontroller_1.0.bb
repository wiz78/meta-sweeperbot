DESCRIPTION = "SweeperBot main daemon"
LICENSE = "CLOSED"
DEPENDS = "libeigen libpointmatcher libnabo yaml-cpp fmt boost"
RDEPENDS:${PN} = "bash"
FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

PV = "1.0"
SRC_URI = "file://botcontroller/ \
           file://botcontroller.service \
           file://powersavings.sh \
           file://power_savings.service"

ERROR_QA:remove = "version-going-backwards"

S = "${WORKDIR}/botcontroller"

inherit pkgconfig cmake useradd systemd

USERADD_PACKAGES = "${PN}"
USERADD_PARAM:${PN} = "-m -G dialout -s /bin/false bot"

EXTRA_OECMAKE = "-DCMAKE_BUILD_TYPE=RelWithDebInfo -DTELEMETRY_HOST=${BOT_TELEMETRY_HOST} -DTELEMETRY_PORT=${BOT_TELEMETRY_PORT}"

do_install() {
	install -d ${D}/opt/sweeperbot/bin
	install -m 0755 botcontroller ${D}/opt/sweeperbot/bin/
	install -m 0755 ${WORKDIR}/powersavings.sh ${D}/opt/sweeperbot/bin/

	install -d ${D}/opt/sweeperbot/www
	install -m 0655 ${WORKDIR}/botcontroller/web/*.html ${D}/opt/sweeperbot/www/

	install -d ${D}${systemd_system_unitdir}
	install -m 0644 ${WORKDIR}/botcontroller.service ${D}${systemd_system_unitdir}/
	install -m 0644 ${WORKDIR}/power_savings.service ${D}${systemd_system_unitdir}/
}

FILES:${PN} = " \
	/opt/sweeperbot \
"
SYSTEMD_SERVICE:${PN} = "botcontroller.service power_savings.service"
