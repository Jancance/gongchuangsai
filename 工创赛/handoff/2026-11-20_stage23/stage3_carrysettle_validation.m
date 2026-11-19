here=fileparts(mfilename('fullpath'));
cases={ ...
 'moving then settled', [1 1 0], [1 1 1], 0, 0, [0 0 1]; ...
 'moving to deadline', [1 1 1], [1 1 1], 0, 3, [0 0 4]; ...
 'position unavailable', [0 0 0], [0 0 1], 0, 0, [0 0 1]; ...
 'bad flags', [1 0 0], [1 1 1], 2, 0, [0 3 3]; ...
 'STOP', [1 0 0], [1 1 1], -2, 0, [0 2 2]};
for i=1:size(cases,1)
    observed=zeros(1,3); terminal=0;
    for k=1:3
        if terminal~=0, observed(k)=terminal; continue; end
        if cases{i,4}==-k, terminal=2;
        elseif cases{i,5}==k, terminal=4;
        elseif cases{i,4}==k, terminal=3;
        elseif cases{i,3}(k) && cases{i,2}(k)==0, terminal=1;
        end
        observed(k)=terminal;
    end
    assert(isequal(observed,cases{i,6}),cases{i,1});
end
report=struct('passed',true,'cases',size(cases,1),'deadline_ms',12000,'physical_acceptance',false);
f=fopen(fullfile(here,'stage3_carrysettle_validation.json'),'w');assert(f>=0);
fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS carry settle specification: %d motion, feedback, deadline and STOP scenarios.\n',size(cases,1));
