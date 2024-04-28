DESCRIPTION = "libnabo is a fast K Nearest Neighbour library for low-dimensional spaces."
AUTHOR = "Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>"
HOMEPAGE = "https://github.com/norlab-ulaval/libnabo"
SECTION = "devel"
LICENSE = "BSD-3-Clause"
LIC_FILES_CHKSUM = "file://LICENSE;md5=3cb41d2f730a1b6a928cdfa87196a614"

DEPENDS = "boost libeigen cmake-native"

SRC_URI = "gitsm://github.com/norlab-ulaval/libnabo.git;protocol=https;branch=master"
SRCREV = "${AUTOREV}"
PV = "git+git${SRCPV}"

ERROR_QA:remove = "version-going-backwards"

S = "${WORKDIR}/git"

inherit cmake