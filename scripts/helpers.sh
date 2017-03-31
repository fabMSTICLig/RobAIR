start_quiet_mode() {
	if [ "$#" -gt 0 ]; then
		log="$1"
	else
		log=/dev/null
	fi

	export _robair_quiet_mode=y
	export _robair_quiet_mode_log="$log"

	exec 3>&1
	exec 4>&2
	exec 1>"$log"
	exec 2>&1
}

stop_quiet_mode() {
	[ "$_robair_quiet_mode" != 'y' ] && return 0

	exec 1>&3-
	exec 2>&4-

	unset _robair_quiet_mode
	unset _robair_quiet_mode_log
}

start_job() {
	if [ "$_robair_quiet_mode" = 'y' ]; then
		echo -n "[      ] $1" >&3
	else
		echo -n "[      ] $1" >&2
	fi
}

end_job_success() {
	if [ "$_robair_quiet_mode" = 'y' ]; then
		if [ -n "$TERM" -a "$TERM" != 'dumb' ]; then
			echo -ne "\r[  $(tput setaf 2)OK$(tput sgr0)  ]" >&3
		else
			echo -n ": OK" >&3
		fi
		echo >&3
	else
		if [ -n "$TERM" -a "$TERM" != 'dumb' ]; then
			echo -ne "\r[  $(tput setaf 2)OK$(tput sgr0)  ]" >&2
		else
			echo -n ": OK" >&2
		fi
		echo >&2
	fi
}

end_job_failure() {
	if [ "$_robair_quiet_mode" = 'y' ]; then
		if [ -n "$TERM" -a "$TERM" != 'dumb' ]; then
			echo -ne "\r[$(tput setaf 9)ERREUR$(tput sgr0)]" >&3
		else
			echo -n ": ERREUR" >&3
		fi
		echo >&3

		if [ "$_robair_quiet_mode_log" != "/dev/null" ]; then
			echo >&3
			echo "$(tput setab 1)Consultez le fichier $_robair_quiet_mode_log" \
				"pour déterminer la cause de l'erreur$(tput sgr0)" >&4
		fi

		[ "$1" = 'noexit' ] || exit 1
	else
		if [ -n "$TERM" -a "$TERM" != 'dumb' ]; then
			echo -ne "\r[$(tput setaf 9)ERREUR$(tput sgr0)]" >&2
		else
			echo -n ": ERREUR" >&2
		fi
		echo >&2
		[ "$1" = 'noexit' ] || exit 1
	fi
}

end_job() {
	if [ "$?" -eq 0 ]; then
		end_job_success "$@"
	else
		end_job_failure "$@"
	fi
}
