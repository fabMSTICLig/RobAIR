#!/usr/bin/python
import json
import sys
import os

path = sys.argv[2].split(":")
path.reverse()

with open(sys.argv[1], 'r+') as f:
	data = json.load(f)
	tmp=data

	while len(path)>1:
		tmp = tmp[path.pop()]
	
	tmp[path.pop()]= sys.argv[3]
	f.seek(0) 
	json.dump(data, f, indent=4,sort_keys=True)
