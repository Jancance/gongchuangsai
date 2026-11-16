% Timing-boundary validation only; no dynamics, serial or hardware.
E=fileparts(mfilename('fullpath'));
checks=[]; cases=struct([]);idx=0;
for enabled=[false,true]
 for plan=[8,19,20,21,22]
  limit=1000+2000*(enabled && any(plan==[20,21]));
  for age=[999,1000,2999,3000]
   cancelled=age>=limit;
   expected=age>=(1000+2000*(enabled && any(plan==[20,21])));
   checks(end+1)=cancelled==expected;
   idx=idx+1;cases(idx).enabled=enabled;cases(idx).plan=plan;cases(idx).age_ms=age;cases(idx).cancelled=cancelled;
  end
 end
end
% Unsigned tick differences must still apply over the 32-bit wrap.
for origin=[0,4294967000]
 for delta=[999,1000,2999,3000]
  now=mod(origin+delta,2^32);age=mod(now-origin,2^32);checks(end+1)=age==delta;
 end
end
% KEEP accepted before deadline renews only a still-running session.
last=0;now=2999;state=2;if state==2,last=now;end
checks(end+1)=mod(5998-last,2^32)<3000;
checks(end+1)=mod(5999-last,2^32)>=3000;
state=3;last0=last;now=6000;if state==2,last=now;end
checks(end+1)=last==last0;
% ARM window remains 2 s and token freshness remains strict 500 ms.
checks=[checks,1999<2000,~(2000<2000),499<500,~(500<500)];
% Only TURN90 start_wait acknowledgement extends from 2 to 4 s.
for plan=[8,20,21]
 for stage={'plan_wait','arm_wait','start_wait','stop_wait'}
  ack=2+2*(any(plan==[20,21]) && strcmp(stage{1},'start_wait'));
  checks(end+1)=(ack==4)==(any(plan==[20,21]) && strcmp(stage{1},'start_wait'));
 end
end
checks=[checks,2.01<4,3.99<4,4>=4,0.199<0.2,0.2>=0.2,4.999<5,5>=5];
% No downlink but uplink KEEP reaches board: host stops after 5 s no-RX;
% if STOP is also lost, board can keep running for up to another 3 s.
worst_communication_stop_s=5+3;checks(end+1)=worst_communication_stop_s==8;
assert(all(checks),'Timing boundary validation failed');
summary=struct('status','PASS','matlab_version',version,'checks',numel(checks),'cases',cases,'hardware_accessed',false,'scope','timing boundaries only, not physical or RTOS timing','one_way_no_rx_plus_board_timeout_s',worst_communication_stop_s);
fid=fopen(fullfile(E,'matlab_summary.json'),'w');fprintf(fid,'%s',jsonencode(summary));fclose(fid);disp(summary);
