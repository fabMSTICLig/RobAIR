
if [ ! -f $ROBAIR_HOME/catkin_ws/src/robairmain/launch/robair.launch ]; then
  if [ ! -d $ROBAIR_HOME/catkin_ws/src/robairmain/launch ]; then
    mkdir $ROBAIR_HOME/catkin_ws/src/robairmain/launch
  fi
  cp $ROBAIR_HOME/configs/robair.launch $ROBAIR_HOME/catkin_ws/src/robairmain/launch/
fi
if [ ! -f $ROBAIR_HOME/interface/config.json ]; then
  cp $ROBAIR_HOME/interface/config.json.default $ROBAIR_HOME/interface/config.json 
fi
sed -i -e 's#\(.*\"key\":\).*#\1"'"$ROBAIR_HOME"'/ssl/device.key",#' $ROBAIR_HOME/interface/config.json
sed -i -e 's#\(.*\"crt\":\).*#\1"'"$ROBAIR_HOME"'/ssl/device.crt",#' $ROBAIR_HOME/interface/config.json
sed -i -e 's#\(.*\"ca\":\).*#\1"'"$ROBAIR_HOME"'/ssl/rootCA.crt"#' $ROBAIR_HOME/interface/config.json
