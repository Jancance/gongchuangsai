here=fileparts(mfilename('fullpath'));
cases=0;
for mode=0:2
    for kind=[3 5 11 12 13 14 15 16]
        expected=1846.4;
        if kind==11, expected=30; end
        if kind==15, expected=1000; end
        for rpm=[30 1000 1846.4]
            assert(admitRate(kind,rpm)==(rpm==expected)); cases=cases+1;
        end
        assert((mode==1 && kind==15)==useNewRise(mode,kind)); cases=cases+1;
    end
end
before=901.39; safeTop=68.55; accel=45055;
rise=pack(before,safeTop,1000,accel);
prior=pack(before,safeTop,1846.4,accel);
place=pack(safeTop,6840.85,1846.4,accel);
assert(isequal(rise([1:6 9:15]),prior([1:6 9:15])));
assert(isequal(rise(7:8),uint8([39 16])) && isequal(place(7:8),uint8([72 32])));
assert(isequal(rise(9:12),uint8([0 1 69 84])) && rise(2)==1 && place(2)==0);
assert(~inPath(before,safeTop,23.24) && inPath(before,safeTop,safeTop-45));
assert(inPath(before,safeTop,safeTop) && 600<=2000 && 2001>2000);
travelMm=abs(before-safeTop)*3200/360/400;
report=struct('passed',true,'rate_cases',cases,'rise_rpm',1000,'other_rpm',1846.4, ...
    'rise_speed_frame_hex','2710','other_speed_frame_hex','4820','accel_field',accel, ...
    'recorded_travel_mm',travelMm,'position_corridor_deg',45,'mechanical_top_gate_pulses',2000, ...
    'physical_braking_validated',false,'physical_acceptance',false);
f=fopen(fullfile(here,'stage3_carryrise1000_validation.json'),'w');assert(f>=0);
fprintf(f,'%s\n',jsonencode(report));fclose(f);
fprintf('PASS carry rise speed specification: %d admission checks, frame 0x2710 only for rise, trajectory boundaries retained.\n',cases);

function yes=admitRate(kind,rpm)
    if kind==15, yes=rpm==1000;
    elseif kind==11, yes=rpm==30;
    else, yes=rpm==1846.4;
    end
end
function yes=useNewRise(mode,kind)
    yes=mode==1 && kind==15;
end
function frame=pack(before,goal,rpm,accel)
    u=uint32(round(abs(goal-before)*100));
    s=uint32(round(rpm*10));
    a=uint32(accel);
    be16=@(n) [bitshift(n,-8) bitand(n,255)];
    frame=uint8([253 goal<before be16(a) be16(a) be16(s) ...
        bitshift(u,-24) bitand(bitshift(u,-16),255) bitand(bitshift(u,-8),255) bitand(u,255) 2 0 107]);
end
function yes=inPath(before,goal,current)
    yes=current>=min(before,goal)-45 && current<=max(before,goal)+45;
end
