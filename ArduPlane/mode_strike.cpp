// #include "mode.h"
// #include "Plane.h"
// #include <GCS_MAVLink/GCS.h>
// #include <AP_Math/AP_Math.h>
// #include <AP_GPS/AP_GPS.h>

// // Terminal entry distance threshold (meters)
// #define STRIKE_TERMINAL_DISTANCE_M 500.0f
// // Minimum strike distance (meters) - terminate if closer
// #define STRIKE_MIN_DISTANCE_M 2.0f
// // Minimum altitude AGL to continue strike (meters)
// #define STRIKE_MIN_ALT_AGL_M 1.0f
// // Maximum dive angle (centi-degrees) - prevent over-vertical dive
// #define STRIKE_MAX_DIVE_ANGLE_CDEG 8500
// // Minimum dive angle (centi-degrees) - ensure aggressive dive
// #define STRIKE_MIN_DIVE_ANGLE_CDEG 1500
// // Earth radius approximation for flat-earth calculations (meters)
// #define EARTH_RADIUS_M 6371000.0f
// // Meters per degree latitude
// #define METERS_PER_DEG_LAT 111319.5f

// bool ModeStrike::_enter()
// {
//     int32_t lat = plane.aparm.str_lat.get();
//     int32_t lon = plane.aparm.str_lon.get();
    
//     if(lat == 0 && lon == 0) {
//         gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike WP invalid");
//         return false;
//     }

//     // Store target location
//     target_location.lat = lat;
//     target_location.lng = lon;

//     // Initialize strike state
//     in_terminal_dive = false;
//     strike_complete = false;

//     gcs().send_text(MAV_SEVERITY_INFO,"Strike Point Lat: %ld Lon: %ld", (long)lat, (long)lon);
//     gcs().send_text(MAV_SEVERITY_INFO,"Entered Strike Mode - Aggressive Dive");
    
//     // Set as guided waypoint for navigation
//     plane.set_guided_WP(target_location);
    
//     // Initialize small radius for direct aggressive approach
//     active_radius_m = 5.0f;
    
//     return true;
// }

// void ModeStrike::update()
// {
//     // Check for GPS loss - failsafe
//     if (plane.gps.status() < AP_GPS::GPS_OK_FIX_2D) {
//         gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike: GPS lost - FAILSAFE");
//         return;
//     }

//     // Get current position and altitude
//     const Location &current = plane.current_loc;
//     float altitude_agl = plane.relative_altitude; // meters AGL
    
//     // Calculate horizontal distance to target using flat-earth approximation
//     float dx = (target_location.lat - current.lat) * 1e-7f * METERS_PER_DEG_LAT;
//     float curr_lat_rad = radians(current.lat * 1e-7f);
//     float dy = (target_location.lng - current.lng) * 1e-7f * METERS_PER_DEG_LAT * cosf(curr_lat_rad);
//     float horizontal_distance = sqrtf(dx*dx + dy*dy);
    
//     // Vertical distance (current altitude AGL)
//     float vertical_distance = MAX(altitude_agl, 0.1f); // prevent division by zero
    
//     // Check termination conditions
//     if (altitude_agl <= STRIKE_MIN_ALT_AGL_M || 
//         horizontal_distance < STRIKE_MIN_DISTANCE_M) {
//         strike_complete = true;
//         gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Target reached or altitude too low");
//         // Disarm or switch to failsafe mode
//         return;
//     }
    
//     // Check if we should enter terminal dive phase
//     if (!in_terminal_dive && horizontal_distance < STRIKE_TERMINAL_DISTANCE_M) {
//         in_terminal_dive = true;
//         gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Entering terminal dive phase");
//     }
    
//     // Calculate required dive angle (real-time)
//     float dive_angle_rad = atan2f(vertical_distance, horizontal_distance);
//     int32_t dive_angle_cdeg = constrain_int32(
//         degrees(dive_angle_rad) * 100.0f,
//         STRIKE_MIN_DIVE_ANGLE_CDEG,
//         STRIKE_MAX_DIVE_ANGLE_CDEG
//     );
    
//     // Wind compensation for strike accuracy
//     Vector3f wind_vec;
//     float wind_speed = 0.0f;
//     float wind_direction_rad = 0.0f;
//     if (ahrs.wind_estimate(wind_vec)) {
//         wind_speed = wind_vec.length();
//         wind_direction_rad = atan2f(-wind_vec.y, -wind_vec.x); // NED to compass direction
//     }
    
//     // Calculate target bearing
//     float target_bearing_rad = atan2f(dy, dx);
    
//     // Project wind onto strike vector
//     float wind_along_path = 0.0f;
//     if (wind_speed > 0.1f) {
//         float wind_angle_diff = wrap_PI(wind_direction_rad - target_bearing_rad);
//         wind_along_path = wind_speed * cosf(wind_angle_diff);
//     }
    
//     // Get current airspeed and groundspeed
//     float airspeed_ms = 0.0f;
//     bool airspeed_valid = ahrs.airspeed_TAS(airspeed_ms);
//     if (!airspeed_valid) {
//         // Fallback to groundspeed if no airspeed sensor
//         airspeed_ms = ahrs.groundspeed();
//     }
    
//     float effective_ground_speed = airspeed_ms + wind_along_path;
    
//     // Recompute dive angle if wind significantly affects ground speed
//     // (This is a simplified model - in practice, wind affects the trajectory)
//     if (fabsf(wind_along_path) > 2.0f && effective_ground_speed > 0.1f) {
//         // Adjust dive angle slightly based on wind component
//         // More headwind = steeper dive needed, tailwind = shallower
//         float wind_factor = 1.0f + (wind_along_path / effective_ground_speed) * 0.1f;
//         dive_angle_cdeg = constrain_int32(
//             dive_angle_cdeg * wind_factor,
//             STRIKE_MIN_DIVE_ANGLE_CDEG,
//             STRIKE_MAX_DIVE_ANGLE_CDEG
//         );
//     }
    
//     // HEADING CONTROL - Lock heading to target
//     int32_t target_heading_cd = degrees(target_bearing_rad) * 100.0f;
//     target_heading_cd = wrap_360_cd(target_heading_cd);
    
//     // Update navigation controller to point at target
//     plane.prev_WP_loc = current;
//     plane.next_WP_loc = target_location;
//     plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    
//     // Calculate roll for heading alignment (aggressive)
//     plane.calc_nav_roll();
    
//     // PITCH CONTROL - Aggressive dive
//     if (in_terminal_dive) {
//         // Terminal dive phase: Force pitch to computed dive angle
//         plane.nav_pitch_cd = -dive_angle_cdeg; // Negative = pitch down
        
//         // Check for excessive pitch (safety)
//         float current_pitch_deg = ahrs.pitch_sensor * 0.01f;
//         if (current_pitch_deg < -85.0f) {
//             gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Pitch limit exceeded");
//             strike_complete = true;
//             return;
//         }
//     }else {
//         // Approach phase: Use computed dive angle but allow navigation blending
//         // Gradually transition to dive angle as we approach terminal distance
//         float approach_factor = horizontal_distance / STRIKE_TERMINAL_DISTANCE_M;
//         approach_factor = constrain_float(approach_factor, 0.0f, 1.0f);
        
//         // Blend between navigation pitch and dive angle
//         int32_t nav_pitch = plane.TECS_controller.get_pitch_demand();
//         int32_t strike_pitch = dive_angle_cdeg;
//         plane.nav_pitch_cd = -(nav_pitch * approach_factor + strike_pitch * (1.0f - approach_factor));
//     }

//     // THROTTLE CONTROL - Maximum energy
//     if (in_terminal_dive) {
//         // Terminal dive: Force maximum throttle
//         float throttle_max = plane.aparm.throttle_max.get();
//         if (throttle_max <= 0) {
//             throttle_max = 100.0f; // Default to 100% if not configured
//         }
//         SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_max);
//     } else {
//         // Approach phase: Use TECS but push toward max throttle
//         plane.calc_throttle();
//     }

//     if (strike_complete) {
//         SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
//         plane.set_mode(plane.mode_fbwa, ModeReason::STIKE_COMPLETE);
//         return;
//     }
// }

#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

// ---------------- CONFIG ----------------
#define STRIKE_TERMINAL_DISTANCE_M     500.0f
#define STRIKE_MIN_DISTANCE_M           2.0f
#define STRIKE_MIN_ALT_AGL_M             1.0f

#define STRIKE_MAX_DIVE_ANGLE_CDEG   8500   // 85 deg
#define STRIKE_MIN_DIVE_ANGLE_CDEG   1500   // 15 deg

#define METERS_PER_DEG_LAT        111319.5f
// ---------------------------------------

bool ModeStrike::_enter()
{
    int32_t lat = plane.aparm.str_lat.get();
    int32_t lon = plane.aparm.str_lon.get();

    if (lat == 0 && lon == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Strike: Invalid target");
        return false;
    }

    target_location.lat = lat;
    target_location.lng = lon;

    in_terminal_dive = false;
    strike_complete  = false;

    gcs().send_text(MAV_SEVERITY_INFO,
                    "Strike target set: %ld %ld",
                    (long)lat, (long)lon);

    return true;
}

// ------------------------------------------------------

void ModeStrike::update()
{
    // ---------- FAILSAFE ----------
    if (plane.gps.status() < AP_GPS::GPS_OK_FIX_2D) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Strike: GPS LOST");
        return;
    }

    const Location &current = plane.current_loc;

    // ---------- AGL ----------
    float altitude_agl = plane.relative_altitude; // meters

    if (plane.terrain.enabled()) {
        float terr_agl;
        if (plane.terrain.height_above_terrain(terr_agl)) {
            altitude_agl = terr_agl;
        }
    }
    // ---------- DISTANCE ----------
    float dx = (target_location.lat - current.lat) * 1e-7f * METERS_PER_DEG_LAT;
    float lat_rad = radians(current.lat * 1e-7f);
    float dy = (target_location.lng - current.lng) * 1e-7f *
               METERS_PER_DEG_LAT * cosf(lat_rad);

    float horizontal_distance = sqrtf(dx * dx + dy * dy);

    // ---------- TERMINATION ----------
    if (altitude_agl < STRIKE_MIN_ALT_AGL_M ||
        horizontal_distance < STRIKE_MIN_DISTANCE_M) {

        strike_complete = true;

        plane.throttle_nudge = -plane.aparm.throttle_cruise.get();
        plane.set_mode(plane.mode_fbwa,
                       ModeReason::STRIKE_COMPLETE);

        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Strike complete");
        return;
    }

    // ---------- TERMINAL ENTRY ----------
    if (!in_terminal_dive &&
        horizontal_distance < STRIKE_TERMINAL_DISTANCE_M) {

        in_terminal_dive = true;
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "Strike: TERMINAL DIVE");
    }

    // ---------- BASE DIVE GEOMETRY ----------
    float base_dive_rad =
        atan2f(MAX(altitude_agl, 0.1f), horizontal_distance);

    float base_dive_cdeg =
        degrees(base_dive_rad) * 100.0f;

    // ---------- THROTTLE NORMALIZATION ----------
    float throttle_pct = plane.throttle_percentage();
    throttle_pct = constrain_float(throttle_pct, 0.0f, 100.0f);
    float throttle_norm = throttle_pct * 0.01f;

    // ---------- AIRSPEED ----------
    float tas = 0.0f;
    if (!ahrs.airspeed_TAS(tas)) {
        tas = ahrs.groundspeed();
    }

    float cruise_tas = MAX(plane.aparm.airspeed_cruise.get(), 5.0f);
    float speed_norm = constrain_float(tas / cruise_tas, 0.5f, 2.0f);

    // ---------- WIND ----------
    Vector3f wind;
    float wind_along = 0.0f;

    if (ahrs.wind_estimate(wind)) {
        float bearing = atan2f(dy, dx);
        float wind_dir = atan2f(-wind.y, -wind.x);
        wind_along = wind.length() * cosf(wrap_PI(wind_dir - bearing));
    }

    // ---------- AGGRESSIVENESS ----------
    float aggressiveness =
        (0.6f * throttle_norm) +
        (0.4f * speed_norm);

    float wind_factor = 1.0f;
    if (fabsf(wind_along) > 1.0f && tas > 5.0f) {
        wind_factor += (wind_along / tas) * 0.2f;
    }

    float dynamic_dive_cdeg =
        base_dive_cdeg *
        aggressiveness *
        wind_factor;

    dynamic_dive_cdeg = constrain_float(
        dynamic_dive_cdeg,
        STRIKE_MIN_DIVE_ANGLE_CDEG,
        STRIKE_MAX_DIVE_ANGLE_CDEG
    );

    // ---------- DISTANCE BLENDING ----------
    float terminal_factor =
        1.0f - constrain_float(horizontal_distance /
                               STRIKE_TERMINAL_DISTANCE_M,
                               0.0f, 1.0f);

    // smoothstep
    terminal_factor =
        terminal_factor * terminal_factor *
        (3.0f - 2.0f * terminal_factor);

    float blended_dive_cdeg =
        terminal_factor * dynamic_dive_cdeg +
        (1.0f - terminal_factor) * base_dive_cdeg;

    // ---------- HEADING CONTROL ----------
    plane.prev_WP_loc = current;
    plane.next_WP_loc = target_location;
    plane.nav_controller->update_waypoint(
        plane.prev_WP_loc,
        plane.next_WP_loc);
    plane.calc_nav_roll();

    // ---------- PITCH CONTROL (TECS-CORRECT) ----------
    // plane.pitch_limit_min_cd = -STRIKE_MAX_DIVE_ANGLE_CDEG;
    // plane.pitch_limit_max_cd = 0;

    plane.nav_pitch_cd = -blended_dive_cdeg;

    // ---------- THROTTLE CONTROL (TECS-CORRECT) ----------
    float strike_throttle =
        plane.aparm.throttle_cruise.get() +
        throttle_norm *
        (plane.aparm.throttle_max.get() -
         plane.aparm.throttle_cruise.get());

    if (in_terminal_dive) {
        strike_throttle = plane.aparm.throttle_max.get();
    }

    strike_throttle = constrain_float(
        strike_throttle,
        plane.aparm.throttle_cruise.get(),
        plane.aparm.throttle_max.get()
    );

    plane.throttle_nudge =
        strike_throttle - plane.aparm.throttle_cruise.get();

    plane.calc_throttle();
}

// ------------------------------------------------------

// void ModeStrike::_exit()
// {
//     // Restore limits
//     plane.pitch_limit_min_cd = plane.aparm.pitch_limit_min.get();
//     plane.pitch_limit_max_cd = plane.aparm.pitch_limit_max.get();
//     plane.throttle_nudge = 0;
// }

void ModeStrike::navigate()
{
    // Update target waypoint continuously
    plane.set_guided_WP(target_location);
    
    // Use very small loiter radius for direct aggressive approach
    // This makes it go straight to target instead of circling
    plane.update_loiter(active_radius_m);
}

bool ModeStrike::handle_guided_request(Location target_loc)
{
    plane.fix_terrain_WP(target_loc, __LINE__);
    
    // Update target location
    target_location = target_loc;
    
    // Set altitude to zero if not already set
    if (!target_location.terrain_alt) {
        target_location.set_alt_cm(0, Location::AltFrame::ABOVE_HOME);
    }

    plane.set_guided_WP(target_location);

    return true;
}