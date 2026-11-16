#include <stdint.h>
typedef struct {float wheel_diameter_mm,track_width_mm,wheelbase_mm;} ChassisGeometry;
extern const ChassisGeometry g_chassis_geometry;
extern const uint8_t g_chassis_forward_direction[4];
#define CHASSIS_FORWARD_DISTANCE_SCALE 1.02182540f
