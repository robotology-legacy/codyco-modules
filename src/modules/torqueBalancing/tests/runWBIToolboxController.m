function [] = runWBIToolboxController( directory, model )
%runWBIToolboxController run a WBI Toolbox controller
%   This script rely on having a WBITOOLBOX_CONTROLLERS_ROOT set to the 
%   directory where the WBIToolbox-Controllers have been download
addpath([getenv('CODYCO_SUPERBUILD_DIR') '/install/mex'])
addpath([getenv('CODYCO_SUPERBUILD_DIR') '/install/share/WBI-Toolbox'])
addpath([getenv('CODYCO_SUPERBUILD_DIR') '/install/share/WBI-Toolbox/img'])
cd([getenv('WBITOOLBOX_CONTROLLERS_ROOT') '/controllers/' directory ]);
sim(model)

end

