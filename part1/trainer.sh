#!/bin/bash


python3 trainer.py ES &
sleep 10
./build/coordinate &
wait

python3 trainer.py CMA &
sleep 10
./build/coordinate &
wait

python3 trainer.py BO &
sleep 10
./build/coordinate &
wait
