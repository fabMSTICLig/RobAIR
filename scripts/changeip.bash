#!/bin/bash
$ROBAIR_HOME/scripts/createDeviceCRT.bash
sed -i -e 's#\(.*serverurl : \).*#\1"'"$ROBAIR_IP"'",#' $ROBAIR_HOME/interface/public/common/js/config.js
sed -i -e 's/\r$//g' $ROBAIR_HOME/interface/public/common/js/config.js
