#include "runtime_route.h"
#include "route_plan.h"
#include <string.h>
#include <math.h>

uint16_t RuntimeRoute_Crc(const char *data,uint32_t length)
{
    uint16_t crc=0xFFFFU;uint32_t i,j;
    for(i=0U;i<length;i++) {
        crc^=(uint16_t)((uint16_t)(uint8_t)data[i]<<8);
        for(j=0U;j<8U;j++)crc=(uint16_t)((crc&0x8000U)?((crc<<1)^0x1021U):(crc<<1));
    }
    return crc;
}
static uint32_t RQ_UInt(const char **cursor,uint32_t *value,uint32_t last)
{
    const char *p=*cursor;uint32_t n=0U,d;
    if(*p<'0'||*p>'9')return 0U;
    while(*p>='0'&&*p<='9') {
        d=(uint32_t)(*p-'0');if(n>(0xFFFFFFFFU-d)/10U)return 0U;
        n=n*10U+d;p++;
    }
    if(*p!=(last?'\0':','))return 0U;
    *cursor=last?p:p+1;*value=n;return 1U;
}
static uint32_t RQ_Int(const char **cursor,int32_t *value)
{
    uint32_t n,negative=0U;
    if(**cursor=='-'){negative=1U;(*cursor)++;}
    if(!RQ_UInt(cursor,&n,0U)||n>0x7FFFFFFFU)return 0U;
    *value=negative?-(int32_t)n:(int32_t)n;return 1U;
}
uint32_t RuntimeRoute_Decode(const char *line,RuntimeRouteMessage *m)
{
    char body[RQ_LINE_MAX];const char *p;uint32_t n=0U,i,crc=0U,d;
    RuntimeRouteMessage empty={0};
    if(!line||!m)return RQ_FORMAT;*m=empty;
    while(n<RQ_LINE_MAX&&line[n])n++;
    if(n<12U||n>=RQ_LINE_MAX||strncmp(line,"RQ1,",4U)||line[n-5U]!=',')return RQ_FORMAT;
    for(i=n-4U;i<n;i++) {
        if(line[i]>='0'&&line[i]<='9')d=(uint32_t)(line[i]-'0');
        else if(line[i]>='A'&&line[i]<='F')d=(uint32_t)(line[i]-'A')+10U;
        else return RQ_FORMAT;
        crc=(crc<<4)|d;
    }
    if(RuntimeRoute_Crc(line,n-4U)!=crc)return RQ_FORMAT;
    memcpy(body,line,n-5U);body[n-5U]='\0';p=body+4;
    if(!RQ_UInt(&p,&m->seq,0)||!RQ_UInt(&p,&m->tick,0)||!RQ_UInt(&p,&m->id,0)||!m->id)return RQ_FORMAT;
    if(!strncmp(p,"BEGIN,",6)) {
        m->op=RQ_BEGIN;p+=6;if(!RQ_UInt(&p,&m->count,1))return RQ_FORMAT;
    } else if(!strncmp(p,"ROW,",4)) {
        m->op=RQ_ROW;p+=4;
        if(!RQ_UInt(&p,&m->index,0)||!RQ_UInt(&p,&m->row.kind,0)||!RQ_Int(&p,&m->row.amount)||
           !RQ_UInt(&p,&m->row.speed,0)||!RQ_UInt(&p,&m->row.timeout_ms,1))return RQ_FORMAT;
    } else if(!strncmp(p,"COMMIT,",7)||!strncmp(p,"GO,",3)) {
        m->op=(*p=='C')?RQ_COMMIT:RQ_GO;p+=m->op==RQ_COMMIT?7:3;
        if(!RQ_UInt(&p,&m->digest,1))return RQ_FORMAT;
    } else if(!strcmp(p,"ABORT"))m->op=RQ_ABORT;
    else return RQ_FORMAT;
    return RQ_OK;
}
static uint32_t RQ_HashWord(uint32_t h,uint32_t word)
{
    uint32_t j;for(j=0U;j<4U;j++){h^=(word>>(8U*j))&0xFFU;h*=16777619U;}return h;
}
uint32_t RuntimeRoute_Digest(uint32_t id,uint32_t count,const RuntimeRouteRow *rows)
{
    uint32_t h=2166136261U,i;h=RQ_HashWord(h,id);h=RQ_HashWord(h,count);
    for(i=0U;i<count;i++) {
        h=RQ_HashWord(h,rows[i].kind);h=RQ_HashWord(h,(uint32_t)rows[i].amount);
        h=RQ_HashWord(h,rows[i].speed);h=RQ_HashWord(h,rows[i].timeout_ms);
    }
    return h;
}
static uint32_t RQ_RowValid(const RuntimeRouteRow *r)
{
    if(r->timeout_ms<1000U||r->timeout_ms>75000U)return 0U;
    if(r->kind==MOTION_TURN)
        return (r->amount==90000||r->amount==-90000)&&r->speed==30000U;
    return r->kind>=MOTION_FORWARD_DISTANCE&&r->kind<=MOTION_RIGHT_DISTANCE&&
        r->amount>=1000&&r->amount<=21000&&r->speed>=13U&&r->speed<=200U;
}
void RuntimeRoute_Init(RuntimeRoute *q){memset(q,0,sizeof(*q));}
void RuntimeRoute_Tick(RuntimeRoute *q,uint32_t now)
{
    if(q->staging&&(uint32_t)(now-q->stage_ms)>=RQ_UPLOAD_MS) {
        q->staging=0U;q->ready=0U;q->last_result=RQ_EXPIRED;
    }
}
uint32_t RuntimeRoute_Stage(RuntimeRoute *q,const RuntimeRouteMessage *m,uint32_t now)
{
    uint32_t bit;
    RuntimeRoute_Tick(q,now);
    if(m->op==RQ_BEGIN) {
        if(!m->count||m->count>MOTION_ACTION_CAPACITY)return RQ_RANGE;
        if(q->staging&&q->stage_id==m->id)
            return q->stage_count==m->count?RQ_DUPLICATE:RQ_ID;
        if(m->id==q->committed.id&&q->committed.count)return RQ_DUPLICATE;
        if(q->seen_id&&((uint32_t)(m->id-q->seen_id)==0U||(uint32_t)(m->id-q->seen_id)>=0x80000000U))return RQ_ID;
        q->seen_id=q->stage_id=m->id;q->stage_count=m->count;q->stage_mask=0U;
        q->stage_ms=now;q->staging=1U;q->ready=0U;return RQ_OK;
    }
    if(!q->staging||q->stage_id!=m->id)return RQ_ID;
    if(m->op==RQ_ABORT){q->staging=q->ready=0U;return RQ_OK;}
    if(m->op!=RQ_ROW)return RQ_FORMAT;
    if(m->index>=q->stage_count||!RQ_RowValid(&m->row))return RQ_RANGE;
    bit=1UL<<m->index;
    if(q->stage_mask&bit)return memcmp(&q->staged[m->index],&m->row,sizeof(m->row))?RQ_ID:RQ_DUPLICATE;
    q->staged[m->index]=m->row;q->stage_mask|=bit;return RQ_OK;
}
uint32_t RuntimeRoute_Prepare(const RuntimeRoute *q,const RuntimeRouteMessage *m,
    uint32_t now,RuntimeRoutePrepared *p)
{
    RouteRow rows[MOTION_ACTION_CAPACITY];RouteCheck check;uint32_t i,budget=5000U,min_ms;
    if(m->op!=RQ_COMMIT)return RQ_FORMAT;
    if(q->committed.count&&m->id==q->committed.id)
        return m->digest==q->committed.digest?RQ_DUPLICATE:RQ_DIGEST;
    if(!q->staging||q->stage_id!=m->id)return RQ_ID;
    if((uint32_t)(now-q->stage_ms)>=RQ_UPLOAD_MS)return RQ_EXPIRED;
    if(q->stage_mask!=((1UL<<q->stage_count)-1UL))return RQ_MISSING;
    if(RuntimeRoute_Digest(m->id,q->stage_count,q->staged)!=m->digest)return RQ_DIGEST;
    memset(p,0,sizeof(*p));p->id=m->id;p->count=q->stage_count;p->digest=m->digest;
    for(i=0U;i<p->count;i++) {
        const RuntimeRouteRow *r=&q->staged[i];if(!RQ_RowValid(r))return RQ_RANGE;
        p->rows[i]=*r;rows[i].kind=r->kind;
        rows[i].amount=(float)r->amount/(r->kind==MOTION_TURN?1000.0f:100.0f);
        rows[i].speed=(float)r->speed/(r->kind==MOTION_TURN?1000.0f:10.0f);
        budget+=r->timeout_ms+1000U;
    }
    if(budget>APP_PERIMETER_GROUP_MS||!RoutePlan_Convert(rows,p->count,&g_chassis_geometry,p->actions,&check))return RQ_PLAN;
    for(i=0U;i<p->count;i++) {
        const RuntimeRouteRow *r=&p->rows[i];
        min_ms=r->kind==MOTION_TURN?(uint32_t)(((r->amount<0?-(float)r->amount:(float)r->amount)/(float)r->speed)*1000.0f)+3000U:p->actions[i].run_ms;
        if(r->timeout_ms<min_ms)return RQ_RANGE;
    }
    for(i=0U;i<p->count;i++) {
        float x=p->local_targets[i][0],y=p->local_targets[i][1],h=p->local_targets[i][2];
        float angle=h*3.14159265358979323846f/180.0f,d=p->actions[i].distance_mm;
        if(p->rows[i].kind==MOTION_TURN)h+=p->actions[i].relative_deg;
        else if(p->rows[i].kind==MOTION_FORWARD_DISTANCE){x+=d*cosf(angle);y+=d*sinf(angle);}
        else if(p->rows[i].kind==MOTION_LEFT_DISTANCE){x-=d*sinf(angle);y+=d*cosf(angle);}
        else {x+=d*sinf(angle);y-=d*cosf(angle);}
        while(h>180.0f)h-=360.0f;while(h<=-180.0f)h+=360.0f;
        p->local_targets[i+1U][0]=x;p->local_targets[i+1U][1]=y;p->local_targets[i+1U][2]=h;
    }
    p->budget_ms=budget;return RQ_OK;
}
void RuntimeRoute_Install(RuntimeRoute *q,const RuntimeRoutePrepared *p)
{q->committed=*p;q->ready=1U;q->consumed=0U;q->staging=0U;q->last_result=RQ_OK;}
uint32_t RuntimeRoute_Start(RuntimeRoute *q,uint32_t id,uint32_t digest)
{
    if(q->committed.id!=id||q->committed.digest!=digest)return RQ_ID;
    if(q->consumed)return RQ_DUPLICATE;
    if(!q->ready||q->staging)return RQ_MISSING;
    q->consumed=1U;q->ready=0U;return RQ_OK;
}
