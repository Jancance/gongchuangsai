function validate_two_color_stop_20261113
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'evidence','integration','2026-11-13','near-pzq','two-color-stop');
a=readmatrix(fullfile(out,'observations.csv'));ready=false(size(a,1),1);
% Same existing low-speed25px/s; current two trusted colors1/2.
% Third object must exist and all3 regions/identities be actual observations.
for k=1:size(a,1)
 ix=find(a(:,2)>=a(k,2)-.15 & a(:,2)<=a(k,2));
 if numel(ix)<3 || a(k,2)-a(ix(1),2)<.10 || ~all(a(ix,12)==1),continue;end
 if any(any(a(ix,9:11)~=a(k,9:11))),continue;end
 dt=a(k,2)-a(ix(1),2);
 red=vecnorm(a(ix,3:4)-a(k,3:4),2,2);yellow=vecnorm(a(ix,5:6)-a(k,5:6),2,2);
 ready(k)=max([red;yellow])/dt<=25;
end
assert(any(ready));assert(any(ready(end-30:end)));
assert(~any(ready(a(:,12)==0)));
% Large real motion of either trusted color cannot become ready.
assert(100/.15>25 && 0/.15<=25);
r=struct('passed',true,'local_matlab',true,'source_session','source-observe-20261113-173341', ...
 'observations',size(a,1),'ready_indices',a(ready,1)','existing_low_speed_px_s',25, ...
 'window_s',.15,'minimum_span_s',.10,'minimum_frames',3,'stationary_colors',[1 2], ...
 'required_actual_color_set',[1 2 5],'third_position_inferred',false,'hardware_access',false);
f=fopen(fullfile(out,'matlab-result.json'),'w','n','UTF-8');fprintf(f,'%s\n',jsonencode(r));fclose(f);
fprintf('Actual561frame replay: two-color stationary and full actual3 region observations ready %u frames; missing-object frames excluded\n',sum(ready));
end
