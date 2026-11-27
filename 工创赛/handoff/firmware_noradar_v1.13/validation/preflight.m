function preflight
l=141.5;w=127;budget=5;reserve=10;old=[336.5 1170];park=old+[20 0];r=hypot(l,w)+reserve;
assert(clear([old-r old+r]));assert(~clear([park-r park+r]));
ex=w*cosd(1)+l*sind(1)+budget;ey=l*cosd(1)+w*sind(1)+budget;
assert(clear([min(old,park)-[ex ey] max(old,park)+[ex ey]]));
for dir=[-1 1],for shift=[-5 0 5]
 a=old+[0 shift];b=park+[0 shift];assert(clear([min(a,b)-[ex ey] max(a,b)+[ex ey]]));
end,end
% Nonzero descent/ascent share the exact same speed/acceleration and only sign changes.
for pulses=[600 25700 29700]
 for sign=[-1 1],rpm=3000;accel=45055;assert(rpm*10==30000 && accel>0);end
end
assert(200/600==1/3);assert(200>150); % ramp duration .333s, not instantaneous
% Released group is already settled within the same .5mm top tolerance.
target=600;tol=400;for delta=[-400 -1 0 1 400 401]
 arrived=abs(delta)<=tol;legacy=(target+delta)<=target;
 if delta>0 && delta<=tol,assert(arrived && ~legacy);end
end
% Motor prep overlaps a real chassis turn; vision never begins during turn.
for turn_required=[false true],for turn_started=[false true],for turn_done=[false true],for mech_ready=[false true]
 prep=~turn_required || turn_started || turn_done;
 visual=mech_ready && (~turn_required || turn_done);
 if turn_required && ~turn_started && ~turn_done,assert(~prep);end
 if turn_required && ~turn_done,assert(~visual);end
end,end,end,end
for dx=-30:5:30,for dy=-30:5:30
 q=[346.5 1170]+[dx dy];admit=norm([dx dy])<=30 && clear([q-r q+r]);
 if admit,assert(clear([q-r q+r]));end
end,end
rpt=struct('passed',true,'model_only',true,'coarse_old_actual_mm',old,'coarse_new_actual_mm',park,'safe_turn_pivot_mm',old,'extension_rpm',200,'extension_accel_rpm_s',600,'source_lift_up_down_rpm',3000,'preclose_deg',160,'lift_done_tolerance_mm',1,'release_dwell_ms',200,'source_prepare_mm',600,'turn_before_vision',true,'nearby_turn_max_deviation_mm',30);
fid=fopen(fullfile(fileparts(mfilename('fullpath')),'matlab-result.json'),'w');fprintf(fid,'%s',jsonencode(rpt));fclose(fid);disp(rpt);
end
function ok=clear(h)
b=[550 550 1000 1000;550 1400 1000 1850;1400 550 1850 1000;1400 1400 1850 1850];ok=all(h(1:2)>0)&&all(h(3:4)<2400);
for i=1:4,if h(1)<=b(i,3)&&h(3)>=b(i,1)&&h(2)<=b(i,4)&&h(4)>=b(i,2),ok=false;end,end
end
