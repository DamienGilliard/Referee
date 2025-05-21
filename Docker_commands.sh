#!/bin/bash
echo "Running referee_exec with xvfb-run"
xvfb-run --auto-servernum --server-args="-screen 0 3840x2160x24" ./referee_exec
python3 /home/Referee/plot_probabilities.py 
mkdir -p /home/Referee/src/build/output
mv /home/Referee/src/build/transformations.png /home/Referee/src/build/output/
mv /home/Referee/src/build/rotationAnglesAndStandardDeviations.txt /home/Referee/src/build/output/
mv /home/Referee/normal_distributions.png /home/Referee/src/build/output/
