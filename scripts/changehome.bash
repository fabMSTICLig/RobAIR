
if [ ! -f $ROBAIR_HOME/interface/config.json ]; then
  cp $ROBAIR_HOME/interface/config.json.default $ROBAIR_HOME/interface/config.json 
fi
sed -i -e 's#\(.*\"key\":\).*#\1"'"$ROBAIR_HOME"'/ssl/device.key",#' $ROBAIR_HOME/interface/config.json
sed -i -e 's#\(.*\"crt\":\).*#\1"'"$ROBAIR_HOME"'/ssl/device.crt",#' $ROBAIR_HOME/interface/config.json
sed -i -e 's#\(.*\"ca\":\).*#\1"'"$ROBAIR_HOME"'/ssl/rootCA.crt"#' $ROBAIR_HOME/interface/config.json
