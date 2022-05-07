system requirement:


ubuntu 18.04 with nvidia-docker with at least 8 gb memory nvidia graphic card

environment:

python 3.6
Baidu apollo 5.0 and 6.0
LGSVL simulator with python API

to run the experiemnt:
######################
1, to start apollo docker cd to apollo.
./docker/scripts/dev_start.sh
./docker/scripts/dev_into.sh
2 in the dock
./apollo.sh build_gpu
./bootstrap.sh
./bridge.sh

3, open lgsvl simulator and select api only mode

4,open another terminal run:
python3 project.py
######################

experiment result are apollo5.0.log and apollo6.0.log
and youtube videos


