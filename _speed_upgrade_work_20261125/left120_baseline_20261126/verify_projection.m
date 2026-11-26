function verify_projection()
root=fileparts(mfilename('fullpath'));
g=jsondecode(fileread(fullfile(root,'analysis_projection_grid.json')));
m=jsondecode(fileread(fullfile(root,'analysis_metrics.json')));
results=struct('run_id',{},'world_forward_mm',{},'world_left_mm',{},'max_difference_mm',{});
for k=1:numel(g)
    q=g(k); t=q.time_ms(:)/1000; rpm=q.logical_rpm; a=q.relative_yaw_deg(:);
    f=mean(rpm,2)*77*pi/60;
    l=(rpm(:,1)-rpm(:,2)-rpm(:,3)+rpm(:,4))*77*pi/240;
    wf=trapz(t,f.*cosd(a)-l.*sind(a));
    wl=trapz(t,f.*sind(a)+l.*cosd(a));
    if iscell(m), idx=find(cellfun(@(x)strcmp(x.run_id,q.run_id),m)); ref=m{idx};
    else, idx=find(strcmp({m.run_id},q.run_id)); ref=m(idx); end
    d=max(abs([wf-ref.world_forward_mm,wl-ref.world_left_mm]));
    assert(d<1e-8,'Projection mismatch');
    results(end+1)=struct('run_id',q.run_id,'world_forward_mm',wf,'world_left_mm',wl,'max_difference_mm',d);
    fprintf('%s worldF=%+.6f worldL=%+.6f diff=%.3g mm\n',q.run_id,wf,wl,d);
end
fid=fopen(fullfile(root,'matlab_projection_verification.json'),'w');
cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',jsonencode(results));
fprintf('PASS %u same-grid projection checks; MATLAB %s. This verifies arithmetic, not interpolation or physical truth.\n',numel(g),version);
end
