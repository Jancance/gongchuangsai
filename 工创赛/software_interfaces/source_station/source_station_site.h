#ifndef SOURCE_STATION_SITE_H
#define SOURCE_STATION_SITE_H
/* Offline candidate only. Before field commissioning, set these from the
 * actual camera/arm/chassis installation and verify the metric site profile.
 * pzq's taught points are preserved; current optical-region binding is NOT
 * inferred from the historic stationary three-piece success. */
#define SOURCE_STATION_ENABLE 1U
#define SOURCE_STATION_MANUAL_QR_AFTER_ALIGNMENT 1U
#define SOURCE_STATION_POSE_VERSION 0U
#define SOURCE_STATION_PZQ_POSE_VERIFIED 0U
#define SOURCE_STATION_FRAME_MAX_AGE_MS 150U
/* Legacy CC02 reports colors only: it cannot prove exposure age or instance
 * identity/stationary budget. A real observation provider must attach these. */
/* Runtime evidence is accepted only through SourceStation_OfferTarget; there
 * is no default producer. Register the actual camera/tracker at deployment. */
#endif
