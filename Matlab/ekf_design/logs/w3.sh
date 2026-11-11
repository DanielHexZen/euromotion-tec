cd /home/user/euromotion-tec/Matlab/ekf_design
octave --no-gui --quiet --eval "addpath(genpath(pwd)); design_sensitivity();" > logs/sens.txt 2>&1
octave --no-gui --quiet --eval "addpath(genpath(pwd)); verify_rate_50_200(1, true);" > logs/rate_833.txt 2>&1
