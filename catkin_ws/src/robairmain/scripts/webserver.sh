#!/bin/bash

trap 'kill %1; kill %2' SIGINT
cd $ROBAIR_HOME/signalmaster
nodejs server.js &

cd $ROBAIR_HOME/interface
nodejs app.js &
wait
trap - SIGINT
