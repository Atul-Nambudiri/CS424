#/bin/bash

for x in `seq 1 500`; do
    nice -n 19 ./external &
done

./RobotMain
