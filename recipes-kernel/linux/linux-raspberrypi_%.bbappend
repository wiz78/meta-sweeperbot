kernel_conf_variable() {
	CONF_SED_SCRIPT="$CONF_SED_SCRIPT /CONFIG_$1[ =]/d;"
	if test "$2" = "n"
	then
		echo "# CONFIG_$1 is not set" >> ${B}/.config
	else
		echo "CONFIG_$1=$2" >> ${B}/.config
	fi
}

do_configure:prepend() {
	kernel_conf_variable MAGIC_SYSRQ n
	kernel_conf_variable MAGIC_SYSRQ_SERIAL n
	kernel_conf_variable MAGIC_SYSRQ_DEFAULT_ENABLE 0
}