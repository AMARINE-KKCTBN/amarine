#!/bin/bash
echo yourscript.sh called: `date`
HOME=/home/pi/KKCTBN-2022
PYTHONPATH=/usr/bin/python3.9
. /home/pi/KKCTBN-2022/bin/activate
cd /home/pi/KKCTBN-2022
python main.py 2>&1 1>/dev/null
