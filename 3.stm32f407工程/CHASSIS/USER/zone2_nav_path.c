#include "zone2_nav_path.h"
#include "field_map.h"
#include <math.h>
#include <string.h>

#define ZONE2_GRID_CELLS 25U
#define ZONE2_GRID_COLUMNS 5U
#define ZONE2_GRID_MASK ((1UL<<ZONE2_GRID_CELLS)-1UL)
#define ZONE2_START_AREA_MM 300.0f
#define ZONE2_NAV_PI 3.14159265358979323846f

static uint32_t context_valid(const Zone2NavContext *c)
{
    return c && c->run_id && c->map_generation && c->layout_id && c->link_generation &&
        c->start_id==2U && c->node_id==1U && c->zone==2U;
}

static uint32_t same_context(const Zone2NavContext *a,const Zone2NavContext *b)
{
    return a->run_id==b->run_id && a->map_generation==b->map_generation &&
        a->layout_id==b->layout_id && a->link_generation==b->link_generation &&
        a->scan_sequence==b->scan_sequence && a->start_id==b->start_id &&
        a->node_id==b->node_id && a->zone==b->zone;
}

uint32_t Zone2NavPath_Leg(uint32_t index,Zone2NavLeg *leg)
{
    static const Zone2NavLeg legs[ZONE2_NAV_LEGS]={
        {0,2,90,90},{2,14,90,90},{14,10,90,-90},{10,22,-90,180},
        {22,14,180,90},{14,10,90,-90},{10,22,-90,180},{22,0,180,90}
    };
    if(!leg || index>=ZONE2_NAV_LEGS) return 0U;
    *leg=legs[index];
    return 1U;
}

uint32_t Zone2NavPath_Station(uint8_t id,FusionPoint *point)
{
    FieldPoint p;
    if(!point) return 0U;
    switch(id) {
    case 2U: p.x_mm=FIELD_SCAN_X_MM;p.y_mm=FIELD_SCAN_Y_MM;break;
    case 0U: case 10U: case 14U: case 22U:
        if(!FieldMap_Park(id,&p)) return 0U;
        break;
    default: return 0U;
    }
    point->x_mm=p.x_mm;point->y_mm=p.y_mm;
    return 1U;
}

uint32_t Zone2NavPath_SetMap(const Zone2NavContext *context,const radar_client_grid_result_t *grid,Zone2NavMap *map)
{
    uint32_t i,mask=0U;
    if(!map) return 0U;
    memset(map,0,sizeof(*map));
    if(!context_valid(context) || !grid || grid->scan_sequence!=context->scan_sequence ||
       grid->count>RADAR_CLIENT_MAX_GRID_IDS) return 0U;
    for(i=0U;i<grid->count;i++) {
        uint32_t id=grid->grid_ids[i];
        if(id>=ZONE2_GRID_CELLS) return 0U;
        mask|=1UL<<id;
    }
    map->context=*context;
    map->occupied_mask=mask;
    map->valid=1U;
    return 1U;
}

uint32_t Zone2NavPath_Matches(const Zone2NavRequest *q,const Zone2NavReply *r)
{
    return q && r && context_valid(&q->context) && same_context(&q->context,&r->context) &&
        q->start==r->path.start && q->goal==r->path.goal;
}

static uint32_t cell_of(FusionPoint p)
{
    uint32_t row,col;
    if(!(p.x_mm>=0.0f && p.x_mm<FIELD_SIZE_MM && p.y_mm>=0.0f && p.y_mm<FIELD_SIZE_MM)) return ZONE2_GRID_CELLS;
    row=(uint32_t)(p.y_mm/FIELD_GRID_SIZE_MM);
    col=(uint32_t)(p.x_mm/FIELD_GRID_SIZE_MM);
    if(row&1U) col=ZONE2_GRID_COLUMNS-1U-col;
    return row*ZONE2_GRID_COLUMNS+col;
}

static uint32_t append(FusionInput *input,FusionPoint point)
{
    uint32_t count=input->count;
    if(count && point.x_mm==input->points[count-1U].x_mm && point.y_mm==input->points[count-1U].y_mm) return 1U;
    if(count>=FUSION_POINTS) return 0U;
    input->points[count]=point;
    input->count++;
    return 1U;
}

static Zone2NavPathResult check_span(FieldPoint a,FieldPoint b,uint32_t occupied,uint32_t *visited)
{
    float dx=b.x_mm-a.x_mm,dy=b.y_mm-a.y_mm;
    FusionPoint p={a.x_mm,a.y_mm};
    if(dx!=0.0f && dy!=0.0f) return ZONE2_NAV_NON_AXIS;
    if(dx==0.0f && dy==0.0f) return ZONE2_NAV_REPEAT;
    dx=dx==0.0f?0.0f:copysignf(FIELD_GRID_SIZE_MM,dx);
    dy=dy==0.0f?0.0f:copysignf(FIELD_GRID_SIZE_MM,dy);
    while(p.x_mm!=b.x_mm || p.y_mm!=b.y_mm) {
        uint32_t id,bit;
        p.x_mm+=dx;p.y_mm+=dy;
        id=cell_of(p);
        if(id>=ZONE2_GRID_CELLS) return ZONE2_NAV_FORMAT;
        bit=1UL<<id;
        if(*visited&bit) return ZONE2_NAV_REPEAT;
        if(occupied&bit) return ZONE2_NAV_OCCUPIED;
        *visited|=bit;
    }
    return ZONE2_NAV_OK;
}

static Zone2NavPathResult convert(const Zone2NavMap *map,const Zone2NavRequest *q,
    const radar_client_path_result_t *path,const Zone2NavGeometry *g,Zone2NavRoute *route)
{
    FusionInput *in=&route->reference;
    FieldPoint center,next;
    uint32_t i;
    if(q->start>=ZONE2_GRID_CELLS || q->goal>=ZONE2_GRID_CELLS || path->waypoint_count>RADAR_CLIENT_MAX_WAYPOINTS)
        return ZONE2_NAV_FORMAT;
    for(i=0U;i<path->waypoint_count;i++) {
        if(path->waypoints[i]>=ZONE2_GRID_CELLS) return ZONE2_NAV_FORMAT;
    }
    if(q->start==q->goal ? path->waypoint_count!=0U :
       !path->waypoint_count || path->waypoints[path->waypoint_count-1U]!=q->goal) return ZONE2_NAV_TERMINAL;
    if(cell_of(g->actual)!=q->start || cell_of(g->terminal)!=q->goal) return ZONE2_NAV_POSE_CELL;
    route->visited_mask=1UL<<q->start;
    if(map->occupied_mask&route->visited_mask) return ZONE2_NAV_OCCUPIED;
    if(!append(in,g->actual)) return ZONE2_NAV_CAPACITY;
    if(q->start!=q->goal) {
        if(!FieldMap_CellCenter(q->start,&center)) return ZONE2_NAV_FORMAT;
        if(hypotf(g->actual.x_mm-center.x_mm,g->actual.y_mm-center.y_mm)>=.01f &&
           !append(in,(FusionPoint){center.x_mm,center.y_mm})) return ZONE2_NAV_CAPACITY;
        for(i=0U;i<path->waypoint_count;i++) {
            Zone2NavPathResult result;
            if(!FieldMap_CellCenter(path->waypoints[i],&next)) return ZONE2_NAV_FORMAT;
            result=check_span(center,next,map->occupied_mask,&route->visited_mask);
            if(result!=ZONE2_NAV_OK) return result;
            if(!append(in,(FusionPoint){next.x_mm,next.y_mm})) return ZONE2_NAV_CAPACITY;
            center=next;
        }
    }
    if(!append(in,g->terminal)) return ZONE2_NAV_CAPACITY;
    return in->count==1U?ZONE2_NAV_COINCIDENT:ZONE2_NAV_OK;
}

static float wrap(float heading)
{
    float angle=fmodf(heading+180.0f,360.0f);
    if(angle<0.0f) angle+=360.0f;
    return angle-180.0f;
}

Zone2NavPathResult Zone2NavPath_Prepare(const Zone2NavMap *map,const Zone2NavRequest *q,
    const Zone2NavReply *r,const Zone2NavGeometry *g,Zone2NavRoute *route,FusionPlan *plan)
{
    FusionInput *in;
    FusionCheck check;
    Zone2NavPathResult result;
    uint32_t id,i;
    if(plan) plan->valid=0U;
    if(!route || !plan || !g || !q || !r || !map || !context_valid(&q->context) || !q->path_id ||
       q->from_cache>1U || !(g->speed_mm_s>0.0f && g->speed_mm_s<=400.0f)) return ZONE2_NAV_INPUT;
    memset(route,0,sizeof(*route));
    if(map->valid!=1U || (map->occupied_mask&~ZONE2_GRID_MASK) ||
       !same_context(&q->context,&map->context)) return ZONE2_NAV_STALE_MAP;
    if(!Zone2NavPath_Matches(q,r) || (!q->from_cache && q->request_sequence!=r->path.request_sequence))
        return ZONE2_NAV_IDENTITY;
    if(r->path.status) return ZONE2_NAV_STATUS;
    result=convert(map,q,&r->path,g,route);
    if(result!=ZONE2_NAV_OK) return result;

    in=&route->reference;
    in->identity=(FusionIdentity){q->context.run_id,q->context.start_id,q->context.map_generation,
        q->context.layout_id,q->path_id,q->start>q->goal?1U:0U};
    in->model_only=1U;
    in->config=g->config;
    in->config.body_length_mm=VEHICLE_BODY_LENGTH_MM;
    in->config.body_width_mm=VEHICLE_BODY_WIDTH_MM;
    in->config.geometry=g_chassis_geometry;
    for(i=0U;i+1U<in->count;i++) in->speed_mm_s[i]=g->speed_mm_s;
    for(id=0U;id<ZONE2_GRID_CELLS;id++) {
        if(map->occupied_mask&(1UL<<id)) {
            const float half=FIELD_GRID_SIZE_MM*0.5f;
            FieldPoint p;
            if(in->box_count>=FUSION_BOXES) return ZONE2_NAV_CAPACITY;
            if(!FieldMap_CellCenter(id,&p)) return ZONE2_NAV_FORMAT;
            in->boxes[in->box_count++]=(FusionBox){p.x_mm-half,p.y_mm-half,p.x_mm+half,p.y_mm+half};
        }
    }
    if(!FusionReference_Prepare(plan,in,&check))
        return check.reason==FUSION_GEOMETRY?ZONE2_NAV_GEOMETRY:ZONE2_NAV_INPUT;
    route->preferred.travel_heading_deg=plan->hold_heading_deg;
    if(plan->turn_direction==-1) {
        route->preferred.first_deg=route->preferred.second_deg=-90;
        /* The turn disk and 180-degree-symmetric rectangle prove identical clearance. */
        route->alternate=(Zone2NavTurns){wrap(plan->hold_heading_deg+180.0f),90,90};
        route->has_alternate=1U;
    } else {
        route->preferred.first_deg=(int32_t)roundf(wrap(in->config.goal_heading_deg-in->config.start_heading_deg));
    }
    return ZONE2_NAV_OK;
}

uint32_t Zone2NavPath_InsideReturn(FusionPoint p,float heading,float pos_error,float heading_error,uint32_t stopped)
{
    float deviation,r,c,s,ex,ey;
    if(stopped!=1U || !(heading>=-3600.0f && heading<=3600.0f) ||
       !(pos_error>=0.0f && pos_error<=100.0f) || !(heading_error>=0.0f && heading_error<=5.0f)) return 0U;
    deviation=fabsf(wrap(heading-90.0f));
    if(deviation>1.0f) return 0U;
    /* Both support extents increase over this interval (0 to 6 degrees from +Y). */
    r=(deviation+heading_error)*(ZONE2_NAV_PI/180.0f);
    c=cosf(r);s=sinf(r);
    ex=(VEHICLE_BODY_WIDTH_MM*c+VEHICLE_BODY_LENGTH_MM*s)*0.5f+pos_error;
    ey=(VEHICLE_BODY_LENGTH_MM*c+VEHICLE_BODY_WIDTH_MM*s)*0.5f+pos_error;
    return p.x_mm>=ex && p.x_mm<=ZONE2_START_AREA_MM-ex &&
        p.y_mm>=ey && p.y_mm<=ZONE2_START_AREA_MM-ey;
}
