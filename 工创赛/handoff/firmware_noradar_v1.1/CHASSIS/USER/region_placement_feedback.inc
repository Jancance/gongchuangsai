static uint16_t SC_Crc(const char *p,uint32_t n);

static uint32_t rp_ticket,rp_item,rp_ring,rp_color,rp_pose,rp_batch,rp_mode;
static uint32_t rp_observed,rp_rx_ms,rp_age_ms,rp_last_byte,rp_used,rp_sequence;
static uint32_t rp_sent;
static char rp_input[128];

static uint32_t RegionPlacement_Mode(uint8_t kind)
{
 switch(kind){
 case APP_ZONE2_SOURCE:return 1U;
 case APP_ZONE2_ROUGH:return 2U;
 case APP_ZONE2_STORAGE_BASE:return 3U;
 case APP_ZONE2_STORAGE_STACK:return 4U;
 default:return 0U;
 }
}

static void RegionPlacement_Line(uint32_t now)
{
 unsigned long run,request,batch,mode,pose,seq,ring,color,valid,occupied,stable,age;
 uint32_t crc=0U,i;
 int n=0;
 if(rp_used<6U || rp_input[rp_used-1U]!='\r')return;
 rp_input[--rp_used]='\0';
 if(sscanf(rp_input,"VO1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%n",
    &run,&request,&batch,&mode,&pose,&seq,&ring,&color,&valid,&occupied,&stable,&age,&n)!=12 ||
    n<=0 || (uint32_t)n+4U!=rp_used)return;
 for(i=(uint32_t)n;i<rp_used;i++){
  char h=rp_input[i];
  if(h>='0' && h<='9')crc=(crc<<4)|(uint32_t)(h-'0');
  else if(h>='A' && h<='F')crc=(crc<<4)|(uint32_t)(h-'A'+10);
  else return;
 }
 if(!rp_sent || run!=rp_ticket || request!=rp_ticket || batch!=rp_batch || mode!=rp_mode ||
    pose!=rp_pose || ring!=rp_ring || color!=rp_color || !seq ||
    (rp_sequence && (int32_t)((uint32_t)seq-rp_sequence)<=0) || valid!=1U ||
    occupied>1U || stable!=1U || age>200U || crc!=SC_Crc(rp_input,(uint32_t)n))return;
 rp_sequence=(uint32_t)seq;
 rp_observed=(uint32_t)occupied;
 rp_rx_ms=now;
 rp_age_ms=(uint32_t)age;
}

static uint32_t RegionPlacement_SharedByte(uint8_t b,uint32_t now,uint32_t binary_active)
{
 if(rp_used && now-rp_last_byte>=200U)rp_used=0U;
 if(!rp_used && (binary_active || b!='V'))return 0U;
 rp_last_byte=now;
 if(b=='\n'){RegionPlacement_Line(now);rp_used=0U;return 1U;}
 if((b<32U && b!='\r') || rp_used>=sizeof(rp_input)-1U){rp_used=0U;return 1U;}
 rp_input[rp_used++]=(char)b;
 return 1U;
}

static uint32_t RegionDispatch_PlacementStable(uint32_t ticket,uint32_t item,uint32_t color,uint32_t ring)
{
 uint32_t now=(uint32_t)xTaskGetTickCount();
 if(rp_ticket!=ticket || rp_item!=item || rp_color!=color || rp_ring!=ring){
  rp_ticket=ticket;rp_item=item;rp_ring=ring;rp_color=color;rp_sent=rp_observed=0U;
  return 0U;
 }
 return rp_observed && now-rp_rx_ms<=200U-rp_age_ms;
}

/* The Pi pairs each station envelope with the status frame immediately following it. */
static uint8_t RegionPlacement_Send(const char *status,uint16_t length,uint32_t run,uint32_t pose,
 uint32_t item,uint32_t now)
{
 AppZone2Station station;
 char wire[224];
 uint32_t batch=0U,mode=0U,ring=0U,color=0U,armed=0U;
 int n;
 (void)now;
 if(AppZone2_StationRead(&station) && station.ticket==run){
  batch=station.batch;mode=RegionPlacement_Mode(station.kind);
  if(mode>1U && rp_ticket==run && rp_item==item){
   ring=rp_ring;color=rp_color;armed=1U;
  }
 }else run=pose=0U;
 n=snprintf(wire,sizeof(wire),"VS1,%lu,%lu,%lu,%lu,%lu,%lu,%lu,",
   (unsigned long)run,(unsigned long)run,(unsigned long)batch,(unsigned long)mode,
   (unsigned long)pose,(unsigned long)ring,(unsigned long)color);
 if(n<=0 || (uint32_t)n+7U+length>sizeof(wire))return 0U;
 snprintf(wire+n,7U,"%04X\r\n",SC_Crc(wire,(uint32_t)n));
 if(mode){
  memcpy(wire+n+6,status,length);
 }else length=0U;
 if(!USART6_VisionSend((const uint8_t*)wire,(uint16_t)((uint32_t)n+6U+length)))return 0U;
 if(armed){
  if(!rp_sent || pose!=rp_pose)rp_observed=0U;
  rp_pose=pose;rp_batch=batch;rp_mode=mode;rp_sent=1U;
 }else rp_sent=rp_observed=0U;
 return 1U;
}
