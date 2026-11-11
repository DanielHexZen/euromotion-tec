cd /home/user/euromotion-tec/Matlab/ekf_design
octave --no-gui --quiet --eval "addpath(genpath(pwd)); verify_monte_carlo(1:5,'rect10','tag','def833');" > logs/mc_def_1_5.txt 2>&1
octave --no-gui --quiet --eval "addpath(genpath(pwd)); ps=ekf_params(); ps.mx_sf_mode=0; verify_monte_carlo(1:5,'rect10','p',ps,'tag','specsf833');" > logs/mc_spec_1_5.txt 2>&1
