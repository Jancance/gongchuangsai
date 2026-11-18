function validate_zone2_stages()
% Navigation dispatch contract; the radar peer supplies all waypoints.
here=fileparts(mfilename('fullpath'));
fixture=jsondecode(fileread(fullfile(here,'p3_path_vectors.json')));
fixture=fixture.legs;
edges=[0 2;2 14;14 10;10 22;22 14;14 10;10 22;22 0];
assert(numel(fixture)==8);
s=blank();
[s,a]=step(s,'start',0,[]); assert(a==0 && s.leg==0);
[s,a]=step(s,'start',2,[]); assert(a==1 && s.leg==0);
[s,a]=step(s,'start',2,[]); assert(a==0 && s.leg==0);
[s,a]=step(s,'map',1,[1 7 0]); assert(a==0 && s.leg==0); % not stowed
[s,a]=step(s,'map',1,[1 7 1]); assert(a==2 && s.leg==1);
stale=s;
fetches=0; hits=0;
for k=1:size(edges,1)
    assert(s.leg==k);
    pair=edges(k,:);
    if k==6 || k==7
        assert(a==3 && s.phase==3); hits=hits+1;
        assert(s.pathId==k && s.paths{pair(1)+1,pair(2)+1}(end)==pair(2));
    else
        assert(a==2 && s.phase==2); fetches=fetches+1;
        nodes=reshape(fixture(k).waypoints,1,[]);
        assert(nodes(end)==pair(2));
        [failed,b]=step(s,'path',s.run,[7 k fliplr(pair) nodes]);
        assert(b==0 && isequal(failed,s));
        [failed,b]=step(s,'path',s.run,[6 k pair nodes]);
        assert(b==0 && isequal(failed,s));
        [failed,b]=step(s,'path',s.run,[7 k+1 pair nodes]);
        assert(b==0 && isequal(failed,s));
        [s,b]=step(s,'path',s.run,[7 k pair nodes]);
        assert(b==3 && s.phase==3 && s.pathId==k);
    end
    [held,b]=step(s,'stop',s.run,[7 k-1 1 1 1]); assert(b==0 && isequal(held,s));
    [held,b]=step(s,'stop',s.run,[7 k 0 1 1]); assert(b==0 && isequal(held,s));
    [held,b]=step(s,'stop',s.run,[7 k 1 0 1]); assert(b==0 && isequal(held,s));
    [held,b]=step(s,'stop',s.run,[7 k 1 1 0]); assert(b==0 && isequal(held,s));
    [s,a]=step(s,'stop',s.run,[7 k 1 1 1]);
end
assert(a==4 && s.phase==4 && fetches==6 && hits==2);
[s,a]=step(s,'start',2,[]); assert(a==1 && s.run==2);
[s,a]=step(s,'map',2,[1 7 1]); assert(a==2 && s.leg==1);
[s,a]=step(s,'path',stale.run,[7 1 edges(1,:) 2]); assert(a==0 && s.phase==2);
[s,a]=step(s,'lost',2,[]); assert(a==5 && s.phase==5 && ~any(~cellfun(@isempty,s.paths),'all'));
[s,a]=step(s,'path',2,[7 1 edges(1,:) 2]); assert(a==0 && s.phase==5);
fprintf('ZONE2_STAGES_PASS: 8 arrivals, 6 radar requests, 2 directed cache hits; stale and incomplete events held\n');
end

function s=blank()
s=struct('phase',0,'run',0,'map',0,'leg',0,'pathId',0,'paths',{cell(25)});
end

function [s,out]=step(s,ev,id,d)
% Outputs: 1 scan; 2 ask radar; 3 prepare from measured pose; 4 done; 5 stop.
edges=[0 2;2 14;14 10;10 22;22 14;14 10;10 22;22 0];
out=0;
if strcmp(ev,'start')
    if id~=2 || ~ismember(s.phase,[0 4]), return; end
    n=s.run+1; s=blank(); s.run=n; s.phase=1; out=1; return;
end
if id~=s.run, return; end
if strcmp(ev,'lost') && ismember(s.phase,1:3)
    s.phase=5; s.paths=cell(25); out=5; return;
end
if strcmp(ev,'map') && s.phase==1 && d(1) && d(2)>0 && d(3)
    s.map=d(2); s.leg=1; s.phase=2; out=2; return;
end
if strcmp(ev,'path') && s.phase==2
    pair=edges(s.leg,:);
    if d(1)~=s.map || d(2)~=s.leg || ~isequal(d(3:4),pair) || numel(d)<5 || d(end)~=pair(2), return; end
    s.paths{pair(1)+1,pair(2)+1}=d(5:end);
    s.phase=3; s.pathId=s.leg; out=3; return;
end
if strcmp(ev,'stop') && s.phase==3 && d(1)==s.map && d(2)==s.pathId && all(d(3:5)==1)
    if s.leg==size(edges,1), s.phase=4; out=4; return; end
    s.leg=s.leg+1; pair=edges(s.leg,:);
    if isempty(s.paths{pair(1)+1,pair(2)+1}), s.phase=2; out=2;
    else, s.pathId=s.leg; out=3; end
end
end
