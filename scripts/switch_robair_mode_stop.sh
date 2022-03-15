#!/bin/bash

for pid in `ps ax | grep "switch_robair_mode" | grep -v grep |  awk '{print $1}'`
	do kill -SIGINT "$pid"
done

