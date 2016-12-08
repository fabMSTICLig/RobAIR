#!/bin/bash

if [ -z $ROBAIR_HOME ]; then
	echo "La variable ROBAIR_HOME doit être définie."
	exit
fi

if [ ! -z "$1" ]; then
	export ROBAIR_IP=$1
fi

$ROBAIR_HOME/scripts/createDeviceCRT.bash
if [[ ! -f  $ROBAIR_HOME/interface/public/common/js/config.js ]]; then
	cp $ROBAIR_HOME/interface/public/common/js/config.js.default $ROBAIR_HOME/interface/public/common/js/config.js
fi
sed -i -e 's#\(.*serverurl : \).*#\1"'"$ROBAIR_IP"'",#' $ROBAIR_HOME/interface/public/common/js/config.js
sed -i -e 's/\r$//g' $ROBAIR_HOME/interface/public/common/js/config.js
sed -i -e 's#\(.*"url":\).*#\1"https://'"$ROBAIR_IP"':6080/"#' ~/.config/chromium/Default/Bookmarks
