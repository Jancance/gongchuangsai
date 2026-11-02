% Environment smoke tests only. No serial, instrument, external mode or hardware.
out_dir = fileparts(mfilename('fullpath'));
cd(out_dir);
diary(fullfile(out_dir,'matlab_probe.log'));
r = struct;
r.version = version;
r.release = version('-release');
r.root = matlabroot;
r.arch = computer;
r.products = ver;
r.hardware_accessed = false;
fprintf('MATLAB_PROBE_STARTED %s\n',r.release);
features = {'SIMULINK','Control_Toolbox','Identification_Toolbox','Simscape', ...
            'SimMechanics','SimDriveline','Real-Time_Workshop','RTW_Embedded_Coder'};
for k=1:numel(features)
    field=strrep(features{k},'-','_');
    try
        r.license_tests.(field)=license('test',features{k});
    catch ex
        r.license_tests.(field)=ex.message;
    end
end
try
    [t,y]=ode45(@(t,y) -y,[0 1],1);
    r.numeric_test=struct('passed',abs(y(end)-exp(-1))<1e-4,'last',y(end));
catch ex
    r.numeric_test=struct('passed',false,'error',ex.message);
end
try
    sys=tf(1,[1 1]);
    [y,t]=step(sys,linspace(0,1,101));
    r.control_test=struct('passed',abs(y(end)-(1-exp(-1)))<1e-4,'last',y(end));
catch ex
    r.control_test=struct('passed',false,'error',ex.message);
end
try
    u=sin((0:199)'*.1)+cos((0:199)'*.037);
    y=filter(.1,[1 -.9],u);
    data=iddata(y,u,.01);
    fitted=arx(data,[1 1 1]);
    r.identification_test=struct('passed',isa(fitted,'idpoly'),'note','Synthetic data only; not a vehicle fit');
catch ex
    r.identification_test=struct('passed',false,'error',ex.message);
end
try
    mdl='codex_matlab_environment_probe';
    load_system('simulink');
    new_system(mdl);
    add_block('simulink/Sources/Step',[mdl '/Input'],'Time','0');
    add_block('simulink/Continuous/Transfer Fcn',[mdl '/Plant'],'Numerator','[1]','Denominator','[1 1]');
    add_block('simulink/Sinks/To Workspace',[mdl '/Output'],'VariableName','probe_y','SaveFormat','Array');
    add_line(mdl,'Input/1','Plant/1');
    add_line(mdl,'Plant/1','Output/1');
    set_param(mdl,'StopTime','1','Solver','ode45','RelTol','1e-6','AbsTol','1e-8');
    save_system(mdl,fullfile(out_dir,[mdl '.slx']));
    sim(mdl);
    r.simulink_test=struct('passed',abs(probe_y(end)-(1-exp(-1)))<1e-4,'last',probe_y(end));
    close_system(mdl,0);
catch ex
    r.simulink_test=struct('passed',false,'error',ex.message);
    try,close_system('codex_matlab_environment_probe',0);catch,end
end
try
    selected=mex.getCompilerConfigurations('C','Selected');
    installed=mex.getCompilerConfigurations('C','Installed');
    r.c_compiler=struct('selected_count',numel(selected),'installed_count',numel(installed));
    if ~isempty(selected)
        r.c_compiler.selected_name=selected(1).Name;
        source=fullfile(out_dir,'probe_addone.c');
        fid=fopen(source,'w');
        fprintf(fid,'#include "mex.h"\nvoid mexFunction(int nlhs,mxArray **plhs,int nrhs,const mxArray **prhs){plhs[0]=mxCreateDoubleScalar(mxGetScalar(prhs[0])+1);}\n');
        fclose(fid);
        mex('-outdir',out_dir,source);
        r.c_compiler.mex_test_passed=(probe_addone(2)==3);
    else
        r.c_compiler.mex_test_passed=false;
        r.c_compiler.note='No selected C compiler; no mex setup/configuration changes attempted';
    end
catch ex
    r.c_compiler=struct('mex_test_passed',false,'error',ex.message);
end
save(fullfile(out_dir,'environment.mat'),'r');
try
    fid=fopen(fullfile(out_dir,'environment.json'),'w','n','UTF-8');
    fprintf(fid,'%s',jsonencode(r));fclose(fid);
catch ex
    fprintf('JSON_EXPORT_ERROR %s\n',ex.message);
end
disp(r);
fprintf('MATLAB_PROBE_FINISHED\n');
diary off;
