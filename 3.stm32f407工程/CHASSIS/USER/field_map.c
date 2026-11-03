#include "field_map.h"
#include "zdt_emm_config.h"
#include <string.h>
static uint32_t inside(FieldPoint p)
{ return p.x_mm>=0.0f && p.x_mm<FIELD_SIZE_MM && p.y_mm>=0.0f && p.y_mm<FIELD_SIZE_MM; }
uint32_t FieldMap_CellCenter(uint32_t id,FieldPoint *p)
{
    uint32_t row,col;
    if(!p || id>24U || FIELD_GRID_SIZE_MM*5.0f!=FIELD_SIZE_MM) return 0U;
    row=id/5U;col=id%5U;if(row&1U) col=4U-col;
    p->x_mm=((float)col+0.5f)*FIELD_GRID_SIZE_MM;
    p->y_mm=((float)row+0.5f)*FIELD_GRID_SIZE_MM;return inside(*p);
}
uint32_t FieldMap_Park(uint32_t id,FieldPoint *p)
{
    if(!FieldMap_CellCenter(id,p)) return 0U;
    if(id==4U) {p->x_mm=FIELD_START1_X_MM;p->y_mm=FIELD_START1_Y_MM;}
    else if(id==0U) {p->x_mm=FIELD_START2_X_MM;p->y_mm=FIELD_START2_Y_MM;}
    else if(id==14U) {p->x_mm=FIELD_RAW_X_MM;p->y_mm=FIELD_RAW_Y_MM;}
    else if(id==10U) {p->x_mm=FIELD_COARSE_X_MM;p->y_mm=FIELD_COARSE_Y_MM;}
    else if(id==22U) {p->x_mm=FIELD_STORAGE_X_MM;p->y_mm=FIELD_STORAGE_Y_MM;}
    return inside(*p);
}
uint32_t FieldMap_AxisClear(FieldPoint a,FieldPoint b,float half)
{
    float lo_x,hi_x,lo_y,hi_y;uint32_t i,j;
    /* 四块黄色区：450mm方形，中央十字通道400mm；只是名义尺寸。 */
    static const float lows[2]={550.0f,1400.0f};
    static const float highs[2]={1000.0f,1850.0f};
    if(!inside(a) || !inside(b) || !(half>0.0f && half<=150.0f) ||
        (a.x_mm!=b.x_mm && a.y_mm!=b.y_mm)) return 0U;
    lo_x=(a.x_mm<b.x_mm?a.x_mm:b.x_mm)-half;
    hi_x=(a.x_mm>b.x_mm?a.x_mm:b.x_mm)+half;
    lo_y=(a.y_mm<b.y_mm?a.y_mm:b.y_mm)-half;
    hi_y=(a.y_mm>b.y_mm?a.y_mm:b.y_mm)+half;
    if(lo_x<0.0f || lo_y<0.0f || hi_x>FIELD_SIZE_MM || hi_y>FIELD_SIZE_MM) return 0U;
    for(i=0U;i<2U;i++)for(j=0U;j<2U;j++)
        if(lo_x<=highs[i] && hi_x>=lows[i] && lo_y<=highs[j] && hi_y>=lows[j]) return 0U;
    return 1U;
}
static uint32_t hash_word(uint32_t hash,uint32_t word)
{ uint32_t i;for(i=0U;i<4U;i++){hash^=(word>>(8U*i))&255U;hash*=16777619U;}return hash; }
uint32_t FieldMap_Stage1(FieldStage1Plan *p)
{
    uint32_t i,bits,hash=2166136261U;FieldPoint q;
    if(!p || !FieldMap_Park(4U,&p->start)) return 0U;
    p->stop.x_mm=FIELD_SCAN_X_MM;p->stop.y_mm=FIELD_SCAN_Y_MM;
    p->start_grid=4U;p->goal_grid=2U;
    p->via.x_mm=p->start.x_mm;p->via.y_mm=p->stop.y_mm;
    p->forward_cm=(p->stop.y_mm-p->start.y_mm)/10.0f;
    p->left_cm=(p->start.x_mm-p->stop.x_mm)/10.0f;
    p->entry_speed_cm_s=VEHICLE_STAGE1_ENTRY_SPEED_CM_S;
    p->speed_cm_s=VEHICLE_STAGE1_SPEED_CM_S;
    p->nominal_scan_gap_mm=p->stop.y_mm-FIELD_SCANNER_REAR_OFFSET_MM-FIELD_QR_PLANE_Y_MM;
    if(!(p->nominal_scan_gap_mm>=VEHICLE_SCAN_MIN_STANDOFF_MM && p->nominal_scan_gap_mm<FIELD_SIZE_MM) ||
        !inside(p->stop) || !(p->forward_cm>=10.0f && p->forward_cm<=210.0f) ||
        !(p->entry_speed_cm_s>=1.21f && p->entry_speed_cm_s<=20.0f) ||
        !(p->left_cm>=10.0f && p->left_cm<=210.0f) ||
        !(p->speed_cm_s>=1.21f && p->speed_cm_s<=20.0f) ||
        p->stop.y_mm>=FIELD_GRID_SIZE_MM || p->stop.x_mm<2.0f*FIELD_GRID_SIZE_MM ||
        p->stop.x_mm>=3.0f*FIELD_GRID_SIZE_MM ||
        !FieldMap_AxisClear(p->start,p->via,FIELD_BODY_HALF_MM) ||
        !FieldMap_AxisClear(p->via,p->stop,FIELD_BODY_HALF_MM)) return 0U;
    for(i=0U;i<25U;i++) {
        if(!FieldMap_Park(i,&q)) return 0U;
        memcpy(&bits,&q.x_mm,4U);hash=hash_word(hash,bits);
        memcpy(&bits,&q.y_mm,4U);hash=hash_word(hash,bits);
    }
    memcpy(&bits,&p->stop.x_mm,4U);hash=hash_word(hash,bits);
    memcpy(&bits,&p->stop.y_mm,4U);hash=hash_word(hash,bits);
    memcpy(&bits,&p->entry_speed_cm_s,4U);hash=hash_word(hash,bits);
    memcpy(&bits,&p->speed_cm_s,4U);hash=hash_word(hash,bits);
    hash=hash_word(hash,VEHICLE_MAP_VERSION);
    hash=hash_word(hash,VEHICLE_STAGE1_MOVE_TIMEOUT_MS);
    hash=hash_word(hash,VEHICLE_STAGE1_SCAN_TIMEOUT_MS);
    hash=hash_word(hash,VEHICLE_STAGE1_SCAN_FRESH_MS);
    hash=hash_word(hash,VEHICLE_STAGE1_SCAN_AFTER_STOP);
    hash=hash_word(hash,VEHICLE_STAGE1_QR_STOP_TIMEOUT_MS);
    hash=hash_word(hash,VEHICLE_SCAN_BYPASS);
    {float d=VEHICLE_SCAN_MIN_STANDOFF_MM;memcpy(&bits,&d,4U);hash=hash_word(hash,bits);}
    {float d=FIELD_SCANNER_REAR_OFFSET_MM;memcpy(&bits,&d,4U);hash=hash_word(hash,bits);}
    {float d=FIELD_QR_PLANE_Y_MM;memcpy(&bits,&d,4U);hash=hash_word(hash,bits);}
    {float d=VEHICLE_BODY_LENGTH_MM;memcpy(&bits,&d,4U);hash=hash_word(hash,bits);}
    {float d=VEHICLE_BODY_WIDTH_MM;memcpy(&bits,&d,4U);hash=hash_word(hash,bits);}
    /* 协议/缩放/加速度/确认状态也参与指纹，避免同路线混淆固件。 */
    hash=hash_word(hash,0x454D4D31U);
    hash=hash_word(hash,ZDT_EMM_SPEED_INPUT_DIV10);
    hash=hash_word(hash,ZDT_EMM_MOTION_ACCELERATION);
    hash=hash_word(hash,ZDT_EMM_APP_MAX_SPEED_RAW);
    hash=hash_word(hash,ZDT_EMM_SETTINGS_CONFIRMED);
    p->config_id=hash;return 1U;
}
uint32_t FieldMap_TaskCodeValid(const char *code)
{
    uint32_t i,masks[4]={0U,0U,0U,0U},group,digit;
    if(!code) return 0U;
    for(i=0U;i<15U;i++) {
        if(i==3U || i==7U || i==11U) {if(code[i]!='+') return 0U;continue;}
        group=i/4U;
        if(code[i]<'1' || code[i]>'6') return 0U;
        digit=(uint32_t)(code[i]-'0');
        if((group&1U) && digit>3U) return 0U;
        if(masks[group]&(1UL<<digit)) return 0U;
        masks[group]|=1UL<<digit;
    }
    return code[15]=='\0' && masks[1]==14U && masks[3]==14U && masks[0]==masks[2];
}

uint32_t FieldMap_LocalMode(void) { return VEHICLE_STAGE1_ENABLE; }
