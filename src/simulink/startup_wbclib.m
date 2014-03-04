clear all;
disp(sprintf('\n'));
disp(sprintf('Yarp Based Robotics Whole Body Control Library\n'))
wbclibroot = fileparts(mfilename('fullpath'));
if exist(wbclibroot,'dir')
    controllersDir = [wbclibroot filesep 'controllers'];
    addpath(controllersDir);
    cd(controllersDir);
    libDir = '../../../build/lib';
    imgDir = [controllersDir filesep 'img'];
    addpath(libDir); %Path where mex libs are compiled
    addpath(imgDir);
    delete([pwd filesep 'pathdef.m'])
    if (~savepath([pwd filesep 'pathdef.m']))
        disp(sprintf('ATTENTION: In the current directory you should now see a file called pathdef.m. \nIf this file is not empty please replace your Matlab pathdef.m in'));
        disp([matlabroot '/toolbox/local' ' with this new file']); 
        disp(sprintf('Due to permissions restrictions this step must be done by the user.'));
        disp('After doing this, you will see the WBCLibrary in the Simulink Library Browser');
    else
        disp('There was an error generating pathdef.m To proceed please manually add the contents of variables controllersDir, imgDir and libDir to your matlabpath')
    end
end
