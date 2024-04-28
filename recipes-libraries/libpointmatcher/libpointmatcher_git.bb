DESCRIPTION = "An Iterative Closest Point (ICP) library for 2D and 3D mapping in Robotics"
AUTHOR = "Stéphane Magnenat <stephane.magnenat@mavt.ethz.ch>"
HOMEPAGE = "https://github.com/norlab-ulaval/libnabo"
SECTION = "devel"
LICENSE = "BSD-3-Clause"
LIC_FILES_CHKSUM = "file://LICENSE;md5=a8ff19acd04db62e1193ad06575a7fbb"

DEPENDS = "boost libeigen libnabo yaml-cpp cmake-native"

SRC_URI = "gitsm://github.com/norlab-ulaval/libpointmatcher.git;protocol=https;branch=master"
SRCREV = "${AUTOREV}"
PV = "git+git${SRCPV}"

ERROR_QA:remove = "version-going-backwards"

S = "${WORKDIR}/git"

inherit cmake