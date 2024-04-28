FILESEXTRAPATHS:prepend := "${THISDIR}/files:"

SRC_URI += "\
	file://dropbear_rsa_host_key \
"

do_install:append() {
	install -d -m 0700 ${D}${localstatedir}/lib/dropbear
	install -m 0600 ${WORKDIR}/dropbear_rsa_host_key ${localstatedir}/lib/dropbear/dropbear_rsa_host_key
}

FILES:${PN} += "${localstatedir}/lib/dropbear/dropbear_rsa_host_key"
