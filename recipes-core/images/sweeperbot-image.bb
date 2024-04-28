SUMMARY = "SweeperBot image"

LICENSE = "CLOSED"

inherit core-image extrausers

# sweeper1
EXTRA_USERS_PARAMS = " \
    usermod -p '\$y\$j9T\$3iZOqh2gEyLs5F41FvvMP1\$t0xvRkBckYFuF7o0.PkKx3hF3XMqT/dcxcwr1DgKOr3' root; \
"

# https://docs.yoctoproject.org/4.0.12/dev-manual/common-tasks.html#creating-a-read-only-root-filesystem

IMAGE_FEATURES += " \
    read-only-rootfs \
	ssh-server-dropbear \
	package-management \
"

CORE_IMAGE_EXTRA_INSTALL += " \
	packagegroup-core-full-cmdline \
	gdb \
	strace \
	resize2fs \
	botcontroller \
	botcontroller-dbg \
"

IMAGE_FSTYPES = "wic"
IMAGE_ROOTFS_EXTRA_SPACE = "8097152"