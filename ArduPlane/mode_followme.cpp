#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

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

bool ModeFollowMe::_enter()
{
    int32_t lat = plane.aparm.str_lat.get();
    int32_t lon = plane.aparm.str_lon.get();
    
    if(lat == 0 && lon == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"FollowMe WP invalid");
        return false;
    }

    // Store target location`
    target_location.lat = lat;
    target_location.lng = lon;

    gcs().send_text(MAV_SEVERITY_INFO,"FollowMe Point Lat: %ld Lon: %ld", (long)lat, (long)lon);
    gcs().send_text(MAV_SEVERITY_INFO,"FollowMe Point Lat: %.6f Lon: %.6f", lat / 1e7f, lon / 1e7f);
    
    // Set as guided waypoint for navigation
    plane.set_guided_WP(target_location);
    
    // Initialize small radius for direct aggressive approach
    active_radius_m = 5.0f;
    
    return true;
}

void ModeFollowMe::update()
{
   #if HAL_QUADPLANE_ENABLED
    if (plane.auto_state.vtol_loiter && plane.quadplane.available()) {
        plane.quadplane.guided_update();
        return;
    }
#endif

    // Received an external msg that guides roll within g2.guided_timeout?
    if (plane.guided_state.last_forced_rpy_ms.x > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.x < plane.g2.guided_timeout*1000.0f) {
        plane.nav_roll_cd = constrain_int32(plane.guided_state.forced_rpy_cd.x, -plane.roll_limit_cd, plane.roll_limit_cd);
        plane.update_load_factor();

#if AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    // guided_state.target_heading is radians at this point between -pi and pi ( defaults to -4 )
    // This function is used in Guided and AvoidADSB, check for guided
    } else if ((plane.control_mode == &plane.mode_guided) && (plane.guided_state.target_heading_type != GUIDED_HEADING_NONE) ) {
        uint32_t tnow = AP_HAL::millis();
        float delta = (tnow - plane.guided_state.target_heading_time_ms) * 1e-3f;
        plane.guided_state.target_heading_time_ms = tnow;

        float error = 0.0f;
        if (plane.guided_state.target_heading_type == GUIDED_HEADING_HEADING) {
            error = wrap_PI(plane.guided_state.target_heading - AP::ahrs().get_yaw_rad());
        } else {
            Vector2f groundspeed = AP::ahrs().groundspeed_vector();
            error = wrap_PI(plane.guided_state.target_heading - atan2f(-groundspeed.y, -groundspeed.x) + M_PI);
        }

        float bank_limit = degrees(atanf(plane.guided_state.target_heading_accel_limit/GRAVITY_MSS)) * 1e2f;
        bank_limit = MIN(bank_limit, plane.roll_limit_cd);

        // push error into AC_PID
        const float desired = plane.g2.guidedHeading.update_error(error, delta, plane.guided_state.target_heading_limit);

        // Check for output saturation
        plane.guided_state.target_heading_limit = fabsf(desired) >= bank_limit;

        plane.nav_roll_cd = constrain_int32(desired, -bank_limit, bank_limit);
        plane.update_load_factor();

#endif // AP_PLANE_OFFBOARD_GUIDED_SLEW_ENABLED
    } else {
        plane.calc_nav_roll();
    }

    if (plane.guided_state.last_forced_rpy_ms.y > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.y < plane.g2.guided_timeout*1000.0f) {
        plane.nav_pitch_cd = constrain_int32(plane.guided_state.forced_rpy_cd.y, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    } else {
        plane.calc_nav_pitch();
    }

    // Throttle output
    if (plane.guided_throttle_passthru) {
        // manual passthrough of throttle in fence breach
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));

    }  else if (plane.aparm.throttle_cruise > 1 &&
            plane.guided_state.last_forced_throttle_ms > 0 &&
            millis() - plane.guided_state.last_forced_throttle_ms < plane.g2.guided_timeout*1000.0f) {
        // Received an external msg that guides throttle within g2.guided_timeout?
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.guided_state.forced_throttle);

    } else {
        // TECS control
        plane.calc_throttle();

    }
}

void ModeFollowMe::navigate()
{
    // Update target waypoint continuously
    plane.set_guided_WP(target_location);
    
    // Use very small loiter radius for direct aggressive approach
    // This makes it go straight to target instead of circling
    plane.update_loiter(active_radius_m);
}

bool ModeFollowMe::handle_guided_request(Location target_loc)
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