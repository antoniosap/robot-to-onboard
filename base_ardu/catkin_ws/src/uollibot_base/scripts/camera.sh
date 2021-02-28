#!/bin/bash

# -hf --> horizontal flip
# -vf --> vertical flip
raspivid -o - -t 0 -n -w 1920 -h 1080 | cvlc -vvv stream:///dev/stdin --sout '#standard{access=http,mux=ts,dst=:8160}' :demux=h264

