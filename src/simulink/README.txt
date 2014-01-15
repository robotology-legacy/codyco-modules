When starting MATLAB remember to add to its path the mex files location and also the models location if you please by doing:
addpath('${CODYCO_DIR}\lib\Release\')
addpath('${CODYCO_ROOT}\simulink\controllers\')

Where CODYCO_DIR points to the location of the CODYCO build directory and CODYCO_ROOT to CODYCO src directory.