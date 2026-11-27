#include "zone2_nav_path.h"
#include "field_map.h"
#include "zone2_fixed_routes_20261121.h"
#include <math.h>
#include <string.h>
#define ZONE2_START_AREA_MM 300.0f
#define ZONE2_NAV_PI 3.14159265358979323846f

uint32_t Zone2NavPath_Leg(uint8_t start_id,uint32_t index,Zone2NavLeg *leg)
{
    static const Zone2NavLeg legs[ZONE2_NAV_LEGS]={
        {0,2,90,90},{2,14,90,90},{14,10,90,-90},{10,22,-90,180},
        {22,14,180,90},{14,10,90,-90},{10,22,-90,180},{22,0,180,90}
    };
    if(!leg || index>=ZONE2_NAV_LEGS || (start_id!=1U && start_id!=2U)) return 0U;
    *leg=legs[index];
    if(start_id==1U) {
        if(index==0U) leg->start=4U;
        if(index+1U==ZONE2_NAV_LEGS) leg->goal=4U;
    }
    return 1U;
}
uint32_t Zone2NavPath_Station(uint8_t id,FusionPoint *point)
{
    FieldPoint p;
    if(!point) return 0U;
    if(id==2U) p=(FieldPoint){FIELD_SCAN_X_MM,FIELD_SCAN_Y_MM};
    else if((id!=0U && id!=4U && id!=10U && id!=14U && id!=22U) ||
       !FieldMap_Park(id,&p)) return 0U;
    *point=(FusionPoint){p.x_mm,p.y_mm};
    return 1U;
}
static uint32_t append(FusionInput *in,FusionPoint point)
{
    if(in->count && hypotf(point.x_mm-in->points[in->count-1U].x_mm,
        point.y_mm-in->points[in->count-1U].y_mm)<0.01f) return 1U;
    if(in->count>=FUSION_POINTS) return 0U;
    in->points[in->count++]=point;
    return 1U;
}
static float wrap(float heading)
{
    float angle=fmodf(heading+180.0f,360.0f);
    if(angle<0.0f) angle+=360.0f;
    return angle-180.0f;
}
static Zone2NavPathResult prepare_local(const Zone2NavMap *map,uint32_t index,
    const Zone2NavGeometry *g,Zone2NavRoute *route,FusionPlan *plan,uint32_t force_return)
{
    Zone2NavLeg leg;
    FusionInput *in;
    FusionCheck check;
    FusionPoint return_park;
    uint32_t i;
    const Zone2FixedRoute *fixed;
    if(plan) plan->valid=0U;
    if(!map || !route || !plan || !g || map->valid!=1U || !map->context.run_id ||
       !map->context.layout_id || !map->context.map_generation ||
       !Zone2NavPath_Leg(map->context.start_id,index,&leg) ||
       !(g->speed_mm_s>0.0f && g->speed_mm_s<=2000.0f) ||
       (force_return && index!=2U && index!=5U)) return ZONE2_NAV_INPUT;
    memset(route,0,sizeof(*route));
    in=&route->reference;
    in->identity=(FusionIdentity){map->context.run_id,map->context.start_id,map->context.map_generation,
        map->context.layout_id,index+1U,leg.start>leg.goal?1U:0U};
    in->model_only=1U;
    in->config=g->config;
    in->config.body_length_mm=VEHICLE_BODY_LENGTH_MM;
    in->config.body_width_mm=VEHICLE_BODY_WIDTH_MM;
    in->config.geometry=g_chassis_geometry;
    fixed=&s_zone2_fixed_routes[map->context.start_id-1U][index];
    if(fixed->count<2U || fixed->count>3U) return ZONE2_NAV_FORMAT;
    if(leg.start==10U && g->actual.x_mm>336.5f+0.01f) {
        route->entry_needed=1U;route->entry_from=g->actual;
        route->entry_to=(FusionPoint){336.5f,g->actual.y_mm};
        if(!append(in,route->entry_to))return ZONE2_NAV_CAPACITY;
    } else if(!append(in,g->actual)) return ZONE2_NAV_CAPACITY;
    /* Try the true measured pose directly first. Only geometric refusal of
     * this valid raw candidate may select the configured-return fallback. */
    if(!force_return && (index==2U || index==5U)) {
        if(!append(in,g->terminal)) return ZONE2_NAV_CAPACITY;
        if(in->count<2U) return ZONE2_NAV_COINCIDENT;
        in->speed_mm_s[0]=g->speed_mm_s;
        if(FusionReference_Prepare(plan,in,&check)) {
            route->raw_direct=1U;
            goto prepared;
        }
        if(check.reason!=FUSION_GEOMETRY) return ZONE2_NAV_INPUT;
        in->count=1U;
    }
    /* The fallback returns to named parking, then follows its safe suffix.
     * An already-arrived pose keeps its actual coordinates; never rebase it. */
    if(index==2U || index==5U) {
        if(!Zone2NavPath_Station(leg.start,&return_park)) return ZONE2_NAV_TERMINAL;
        if(!(g->config.position_tolerance_mm>=0.0f &&
             g->config.position_tolerance_mm<=g->config.position_budget_mm)) return ZONE2_NAV_INPUT;
        if(hypotf(g->actual.x_mm-return_park.x_mm,g->actual.y_mm-return_park.y_mm)>
             g->config.position_tolerance_mm) {
            if(!append(in,return_park)) return ZONE2_NAV_CAPACITY;
            route->return_stop_node=1U;
        }
    }
    for(i=1U;i+1U<fixed->count;i++)
        if(!append(in,fixed->points[i])) return ZONE2_NAV_CAPACITY;
    if(!append(in,g->terminal)) return ZONE2_NAV_CAPACITY;
    if(in->count<2U) return ZONE2_NAV_COINCIDENT;
    for(i=0U;i+1U<in->count;i++) in->speed_mm_s[i]=g->speed_mm_s;
    if(!FusionReference_Prepare(plan,in,&check))
        return check.reason==FUSION_GEOMETRY?ZONE2_NAV_GEOMETRY:ZONE2_NAV_INPUT;
prepared:
    route->preferred.travel_heading_deg=plan->hold_heading_deg;
    if(plan->turn_direction==-1) {
        route->preferred.first_deg=route->preferred.second_deg=-90;
        route->alternate=(Zone2NavTurns){wrap(plan->hold_heading_deg+180.0f),90,90};
        route->has_alternate=1U;
    } else route->preferred.first_deg=(int32_t)roundf(wrap(in->config.goal_heading_deg-in->config.start_heading_deg));
    return ZONE2_NAV_OK;
}

Zone2NavPathResult Zone2NavPath_PrepareLocal(const Zone2NavMap *map,uint32_t index,
    const Zone2NavGeometry *g,Zone2NavRoute *route,FusionPlan *plan)
{
    return prepare_local(map,index,g,route,plan,0U);
}

Zone2NavPathResult Zone2NavPath_PrepareReturn(const Zone2NavMap *map,uint32_t index,
    const Zone2NavGeometry *g,Zone2NavRoute *route,FusionPlan *plan)
{
    return prepare_local(map,index,g,route,plan,1U);
}

uint32_t Zone2NavPath_RawDirectValid(const Zone2NavRoute *route,const FusionPlan *plan)
{
    Zone2NavLeg leg;
    uint32_t path;
    if(!route || !plan || plan->valid!=1U || route->raw_direct!=1U ||
       route->return_stop_node || route->reference.count!=2U || plan->input.count!=2U ||
       memcmp(&route->reference.identity,&plan->input.identity,sizeof(FusionIdentity))) return 0U;
    path=plan->input.identity.path_id;
    return (path==3U || path==6U) &&
        Zone2NavPath_Leg((uint8_t)plan->input.identity.start_id,path-1U,&leg) &&
        leg.start==14U && leg.goal==10U;
}

uint32_t Zone2NavPath_ReturnStopValid(const Zone2NavRoute *route,const FusionPlan *plan)
{
    Zone2NavLeg leg;
    FusionPoint park;
    uint32_t path;
    if(!route || !plan || plan->valid!=1U) return 0U;
    if(route->raw_direct) return Zone2NavPath_RawDirectValid(route,plan);
    if(!route->return_stop_node) return 1U;
    path=plan->input.identity.path_id;
    if(route->return_stop_node!=1U || (path!=3U && path!=6U) || plan->input.count<3U ||
       route->reference.count!=plan->input.count ||
       memcmp(&route->reference.identity,&plan->input.identity,sizeof(FusionIdentity)) ||
       !Zone2NavPath_Leg((uint8_t)plan->input.identity.start_id,path-1U,&leg) ||
       leg.start!=14U || leg.goal!=10U || !Zone2NavPath_Station(leg.start,&park)) return 0U;
    return hypotf(plan->input.points[1].x_mm-park.x_mm,
                  plan->input.points[1].y_mm-park.y_mm)<=0.01f &&
           hypotf(route->reference.points[1].x_mm-park.x_mm,
                  route->reference.points[1].y_mm-park.y_mm)<=0.01f;
}

uint32_t Zone2NavPath_InsideReturn(uint8_t start_id,FusionPoint p,float heading,float pos_error,float heading_error,uint32_t stopped)
{
    float deviation,r,c,s,ex,ey,x;
    if((start_id!=1U && start_id!=2U) || stopped!=1U || !(heading>=-3600.0f && heading<=3600.0f) ||
       !(pos_error>=0.0f && pos_error<=100.0f) || !(heading_error>=0.0f && heading_error<=5.0f)) return 0U;
    deviation=fabsf(wrap(heading-90.0f));
    if(deviation>1.0f) return 0U;
    /* Both support extents increase over this interval (0 to 6 degrees from +Y). */
    r=(deviation+heading_error)*(ZONE2_NAV_PI/180.0f);
    c=cosf(r);s=sinf(r);
    ex=(VEHICLE_BODY_WIDTH_MM*c+VEHICLE_BODY_LENGTH_MM*s)*0.5f+pos_error;
    ey=(VEHICLE_BODY_LENGTH_MM*c+VEHICLE_BODY_WIDTH_MM*s)*0.5f+pos_error;
    x=p.x_mm-(start_id==1U?FIELD_SIZE_MM-ZONE2_START_AREA_MM:0.0f);
    return x>=ex && x<=ZONE2_START_AREA_MM-ex &&
        p.y_mm>=ey && p.y_mm<=ZONE2_START_AREA_MM-ey;
}
