function observer_model
checks=0;
for order=perms(1:3)'
    for mask=[0,2^25-1]
        s=initial();
        s=step(s,1,7);
        s=step(s,1,8);
        check(s.run==7 && s.stage==1 && s.scans==0);
        for stage=2:4
            s=step(s,2,stage);
            check(s.stage==stage && ~s.valid);
        end
        for part=order'
            s=step(s,3,[part,mask,1]);
            check(~s.valid);
        end
        check(s.scans==1 && s.stage==5);
        s=step(s,2,6);
        check(~s.valid && ~s.pulses && s.claimed);
        s=step(s,2,8);
        snap=s.snapshot;
        check(s.valid && ~s.claimed && isequal(snap,[7,2,2,2,1,9,1,mask]));
        for pair=[0,24;24,0;1,12;12,1;5,19;19,5]'
            s=step(s,4,[snap,pair']);
            check(s.stage==7 && ~s.claimed && ~s.reply);
            s=step(s,5,[s.ticket,pair(2),pair(1),0,pair(1)]);
            check(~s.reply);
            s=step(s,5,[s.ticket-1,pair',0,pair(2)]);
            check(~s.reply);
            s=step(s,5,[s.ticket,pair',0,pair(1)]);
            check(~s.reply);
            s=step(s,5,[s.ticket,pair',0,pair(2)]);
            check(s.reply && isequal(s.snapshot,snap));
        end
        s=step(s,6,0);
        check(~s.valid && ~s.reply);
        s=step(s,6,1);
        s=step(s,4,[snap,0,24]);
        check(s.ready && ~s.valid && s.stage==8 && s.link==2);
    end
end
for bad=[0,-1,255,26,NaN,Inf]
    s=scanning();
    s=step(s,3,[1,0,1]);
    s=step(s,3,[2,0,1]);
    s=step(s,3,[3,0,bad]);
    check(s.stage==4 && ~s.valid);
    s=step(s,7,3);
    s=step(s,7,4);
    check(s.stage==99 && s.error==3 && s.scans==1 && ~s.valid);
end
for location=[1,4,8,7]
    s=step(initial(),1,7);
    if location~=1, s=scanning(); end
    if location==7 || location==8
        s=complete(s);
        if location==7, s=step(s,4,[s.snapshot,0,24]); end
    end
    s=step(s,7,4);
    s=step(s,3,[3,0,1]);
    s=step(s,5,[s.ticket,0,24,0,24]);
    check(s.stage==99 && ~s.claimed && ~s.valid && ~s.reply);
end
for mode=1:4
    s=initial();
    switch mode
        case 1, s.ready=false;
        case 2, s.safe=false;
        case 3, s.zone=1;
        case 4, s.free=false;
    end
    s=step(s,1,7);
    check(s.stage==0 && ~s.claimed && s.scans==0);
end
s=complete(scanning());
wrong=s.snapshot;
wrong(5)=wrong(5)+1;
s=step(s,4,[wrong,0,24]);
check(s.valid && s.stage==8 && ~s.reply);
s=step(s,4,[s.snapshot,0,24]);
s=step(s,5,[s.ticket,0,24,1,24]);
check(s.stage==7 && ~s.reply);
s=step(s,7,3);
check(~s.valid && s.error==3);
check(fresh(0,2^32-1) && ~fresh(70,70) && ~fresh(69,70));
fprintf('observer_model checks=%u PASS MATLAB %s\n',checks,version('-release'));
    function check(ok)
        assert(ok);
        checks=checks+1;
    end
end

function s=initial()
s=struct('run',0,'stage',0,'ready',true,'safe',true,'free',true,'zone',2, ...
    'claimed',false,'pulses',false,'valid',false,'reply',false,'parts',false(1,3), ...
    'link',1,'generation',0,'ticket',0,'scans',0,'error',0,'goals',[],'snapshot',[]);
end

function s=scanning()
s=step(initial(),1,7);
for stage=2:4
    s=step(s,2,stage);
end
end

function s=complete(s)
for part=1:3
    s=step(s,3,[part,0,1]);
end
s=step(s,2,6);
s=step(s,2,8);
end

function s=step(s,event,input)
switch event
    case 1 % Scan request with externally supplied run ID
        if s.stage==0 && s.ready && s.safe && s.free && s.zone==2
            s.run=input;
            s.stage=1;
            s.claimed=true;
        end
    case 2 % Matching motion completion
        if s.claimed && (input==s.stage+1 || (s.stage==6 && input==8))
            s.stage=input;
            switch input
                case 2, s.pulses=true;
                case {3,6}, s.pulses=false;
                case 4, s.scans=s.scans+1;
                case 8
                    s.claimed=false;
                    s.valid=true;
                    s.generation=s.generation+1;
                    s.snapshot=[s.run,2,2,s.zone,s.generation,9,s.link,s.mask];
            end
        end
    case 3 % Valid ACK, locked STATUS or complete GRID from client
        if s.stage==4 && input(3)==1
            s.parts(input(1))=true;
            if input(1)==3, s.mask=input(2); end
            if all(s.parts)
                s.stage=5;
                s.pulses=true;
            end
        end
    case 4 % Directed PATH submission
        if s.valid && s.stage==8 && isequal(s.snapshot,input(1:8))
            s.reply=false;
            s.stage=7;
            s.ticket=s.ticket+1;
            s.goals=input(9:10);
        end
    case 5 % PATH response
        if s.valid && s.stage==7 && isequal(input,[s.ticket,s.goals,0,s.goals(2)])
            s.reply=true;
            s.stage=8;
        end
    case 6 % READY transition, not a healthy periodic HELLO
        if input && ~s.ready, s.link=s.link+1; end
        s.ready=logical(input);
        if ~s.ready
            s.valid=false;
            s.reply=false;
        end
    case 7 % Bad completion, cancel, or timeout
        if ~s.error, s.error=input; end
        s.stage=99;
        s.valid=false;
        s.reply=false;
        s.claimed=false;
        s.pulses=false;
end
end

function yes=fresh(candidate,last)
delta=mod(candidate-last,2^32);
yes=delta>0 && delta<2^31;
end
