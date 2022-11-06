#!/bin/bash
echo RUNNING AT: `date`
HOME=/home/pi/KKCTBN-2022
PYTHONPATH=/usr/bin/python3.9
. /home/pi/KKCTBN-2022/bin/activate
cd /home/pi/KKCTBN-2022
python3 main.py
echo SUCCESS RUNNING MAIN.PY IN BACKGROUND
