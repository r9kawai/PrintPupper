#!/bin/sh
ps aux | grep python | grep -v grep | awk '{print $2}' | xargs -r kill
ps aux | grep python
