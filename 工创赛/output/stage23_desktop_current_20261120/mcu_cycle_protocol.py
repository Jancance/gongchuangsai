"""MCU-cycle status and observation framing. The Pi never encodes a motor command."""
from binascii import crc_hqx

STATUS_FIELDS=('run','state','pose','prep','item','color0','color1','color2','accepted','done','phase')
STATUS_FIELDS2=STATUS_FIELDS+('scheduled','start_in_ms')

def frame(prefix,values):
    body=(prefix+','+','.join(str(int(x)) for x in values)+',').encode('ascii')
    return body+f'{crc_hqx(body,0xffff):04X}\r\n'.encode('ascii')

def decode_status(line):
    body,crc=line.strip().rsplit(b',',1)
    if len(crc)!=4 or crc_hqx(body+b',',0xffff)!=int(crc,16):
        raise ValueError('status CRC')
    fields=body.decode('ascii').split(',')
    names=STATUS_FIELDS2 if fields[0]=='SC2' else STATUS_FIELDS
    if fields[0] not in ('SC1','SC2') or len(fields)!=len(names)+1:
        raise ValueError('status shape')
    result=dict(zip(names,map(int,fields[1:])))
    if any(x<0 or x>0xffffffff for x in result.values()):raise ValueError('status integer')
    if not 0<=result['state']<=9 or not 0<=result['item']<=3:raise ValueError('status state')
    result['colors']=[result[f'color{i}'] for i in range(3)]
    return result

def observation(run,pose,sequence,valid,right_mm,forward_mm,error_mm,stationary,colors,age_ms):
    if len(colors)!=3:raise ValueError('three regions required')
    return frame('OC1',(run,pose,sequence,int(valid),round(right_mm*1000),round(forward_mm*1000),
        round(error_mm*1000),int(stationary),*colors,age_ms))

def continuous_observation(run,pose,sequence,valid,actual_colors,age_ms,target_color,start_delay_ms):
    if len(actual_colors)!=3 or len(start_delay_ms)!=3:raise ValueError('three actual regions and future crossings')
    return frame('OC2',(run,pose,sequence,int(valid),0,0,0,int(valid),*actual_colors,age_ms,
        target_color,*start_delay_ms))
