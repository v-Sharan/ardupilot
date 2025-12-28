#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

// Terminal entry distance threshold (meters)
#define STRIKE_TERMINAL_DISTANCE_M 500.0f
// Minimum strike distance (meters) - terminate if closer
#define STRIKE_MIN_DISTANCE_M 2.0f
// Minimum altitude AGL to continue strike (meters)
#define STRIKE_MIN_ALT_AGL_M 1.0f
// Maximum dive angle (centi-degrees) - prevent over-vertical dive
#define STRIKE_MAX_DIVE_ANGLE_CDEG 8500
// Minimum dive angle (centi-degrees) - ensure aggressive dive
#define STRIKE_MIN_DIVE_ANGLE_CDEG 1500
// Earth radius approximation for flat-earth calculations (meters)
#define EARTH_RADIUS_M 6371000.0f
// Meters per degree latitude
#define METERS_PER_DEG_LAT 111319.5f

bool ModeStrike::_enter()
{
    double lat = plane.aparm.str_lat.get();
    double lon = plane.aparm.str_lon.get();
    
    if(lat == 0 && lon == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike WP invalid");
        return false;
    }

    // Store target location
    target_location.lat = lat;
    target_location.lng = lon;

    // Initialize strike state
    in_terminal_dive = false;
    strike_complete = false;

    gcs().send_text(MAV_SEVERITY_INFO,"Strike Point Lat: %ld Lon: %ld", (long)lat, (long)lon);
    gcs().send_text(MAV_SEVERITY_INFO,"Entered Strike Mode - Aggressive Dive");
    
    // Set as guided waypoint for navigation
    plane.set_guided_WP(target_location);
    
    // Initialize small radius for direct aggressive approach
    active_radius_m = 5.0f;
    
    return true;
}

void ModeStrike::update()
{
    // Check for GPS loss - failsafe
    if (plane.gps.status() < AP_GPS::GPS_OK_FIX_2D) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike: GPS lost - FAILSAFE");
        return;
    }

    // Get current position and altitude
    const Location &current = plane.current_loc;
    float altitude_agl = plane.relative_altitude; // meters AGL
    
    // Calculate horizontal distance to target using flat-earth approximation
    float dx = (target_location.lat - current.lat) * 1e-7f * METERS_PER_DEG_LAT;
    float curr_lat_rad = radians(current.lat * 1e-7f);
    float dy = (target_location.lng - current.lng) * 1e-7f * METERS_PER_DEG_LAT * cosf(curr_lat_rad);
    float horizontal_distance = sqrtf(dx*dx + dy*dy);
    
    // Vertical distance (current altitude AGL)
    float vertical_distance = MAX(altitude_agl, 0.1f); // prevent division by zero
    
    // Check termination conditions
    if (altitude_agl <= STRIKE_MIN_ALT_AGL_M || 
        horizontal_distance < STRIKE_MIN_DISTANCE_M) {
        strike_complete = true;
        gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Target reached or altitude too low");
        // Disarm or switch to failsafe mode
        return;
    }
    
    // Check if we should enter terminal dive phase
    if (!in_terminal_dive && horizontal_distance < STRIKE_TERMINAL_DISTANCE_M) {
        in_terminal_dive = true;
        gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Entering terminal dive phase");
    }
    
    // Calculate required dive angle (real-time)
    float dive_angle_rad = atan2f(vertical_distance, horizontal_distance);
    int32_t dive_angle_cdeg = constrain_int32(
        degrees(dive_angle_rad) * 100.0f,
        STRIKE_MIN_DIVE_ANGLE_CDEG,
        STRIKE_MAX_DIVE_ANGLE_CDEG
    );
    
    // Wind compensation for strike accuracy
    Vector3f wind_vec;
    float wind_speed = 0.0f;
    float wind_direction_rad = 0.0f;
    if (ahrs.wind_estimate(wind_vec)) {
        wind_speed = wind_vec.length();
        wind_direction_rad = atan2f(-wind_vec.y, -wind_vec.x); // NED to compass direction
    }
    
    // Calculate target bearing
    float target_bearing_rad = atan2f(dy, dx);
    
    // Project wind onto strike vector
    float wind_along_path = 0.0f;
    if (wind_speed > 0.1f) {
        float wind_angle_diff = wrap_PI(wind_direction_rad - target_bearing_rad);
        wind_along_path = wind_speed * cosf(wind_angle_diff);
    }
    
    // Get current airspeed and groundspeed
    float airspeed_ms = 0.0f;
    bool airspeed_valid = ahrs.airspeed_TAS(airspeed_ms);
    if (!airspeed_valid) {
        // Fallback to groundspeed if no airspeed sensor
        airspeed_ms = ahrs.groundspeed();
    }
    
    float effective_ground_speed = airspeed_ms + wind_along_path;
    
    // Recompute dive angle if wind significantly affects ground speed
    // (This is a simplified model - in practice, wind affects the trajectory)
    if (fabsf(wind_along_path) > 2.0f && effective_ground_speed > 0.1f) {
        // Adjust dive angle slightly based on wind component
        // More headwind = steeper dive needed, tailwind = shallower
        float wind_factor = 1.0f + (wind_along_path / effective_ground_speed) * 0.1f;
        dive_angle_cdeg = constrain_int32(
            dive_angle_cdeg * wind_factor,
            STRIKE_MIN_DIVE_ANGLE_CDEG,
            STRIKE_MAX_DIVE_ANGLE_CDEG
        );
    }
    
    // HEADING CONTROL - Lock heading to target
    int32_t target_heading_cd = degrees(target_bearing_rad) * 100.0f;
    target_heading_cd = wrap_360_cd(target_heading_cd);
    
    // Update navigation controller to point at target
    plane.prev_WP_loc = current;
    plane.next_WP_loc = target_location;
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    
    // Calculate roll for heading alignment (aggressive)
    plane.calc_nav_roll();
    
    // PITCH CONTROL - Aggressive dive
    if (in_terminal_dive) {
        // Terminal dive phase: Force pitch to computed dive angle
        plane.nav_pitch_cd = -dive_angle_cdeg; // Negative = pitch down
        
        // Check for excessive pitch (safety)
        float current_pitch_deg = ahrs.pitch_sensor * 0.01f;
        if (current_pitch_deg < -85.0f) {
            gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Pitch limit exceeded");
            strike_complete = true;
            return;
        }
    }else {
        // Approach phase: Use computed dive angle but allow navigation blending
        // Gradually transition to dive angle as we approach terminal distance
        float approach_factor = horizontal_distance / STRIKE_TERMINAL_DISTANCE_M;
        approach_factor = constrain_float(approach_factor, 0.0f, 1.0f);
        
        // Blend between navigation pitch and dive angle
        int32_t nav_pitch = plane.TECS_controller.get_pitch_demand();
        int32_t strike_pitch = dive_angle_cdeg;
        plane.nav_pitch_cd = -(nav_pitch * approach_factor + strike_pitch * (1.0f - approach_factor));
    }

    // THROTTLE CONTROL - Maximum energy
    if (in_terminal_dive) {
        // Terminal dive: Force maximum throttle
        float throttle_max = plane.aparm.throttle_max.get();
        if (throttle_max <= 0) {
            throttle_max = 100.0f; // Default to 100% if not configured
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_max);
    } else {
        // Approach phase: Use TECS but push toward max throttle
        plane.calc_throttle();
    }

    if (strike_complete) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
        plane.set_mode(plane.mode_fbwa, ModeReason::STRIKE_COMPLETE);
        return;
    }
}

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