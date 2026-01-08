#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

bool ModeStrike::_enter()
{
    int32_t lat = plane.aparm.str_lat.get();
    int32_t lon = plane.aparm.str_lon.get();
    
    if(lat == 0 && lon == 0) {
        // gcs().send_text(MAV_SEVERITY_INFO,"Lat: %.6f Lon: %.6f", lat / 1e7f, lon / 1e7f);
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike WP invalid");
        return false;
    }

    // Store target location`
    target_location.lat = lat;
    target_location.lng = lon;

    gcs().send_text(MAV_SEVERITY_INFO,"Strike Point Lat: %ld Lon: %ld", (long)lat, (long)lon);
    gcs().send_text(MAV_SEVERITY_INFO,"Strike Point Lat: %.6f Lon: %.6f", lat / 1e7f, lon / 1e7f);
    // gcs().send_text(MAV_SEVERITY_INFO,"Entered Strike Mode - Aggressive Dive");
    plane.Strike.isStrike = true;
    plane.set_rudder_offset(0,true);
    
    return true;
}

bool ModeStrike::does_auto_navigation() const
{
   return (!plane.Strike.isStrike);
}

bool ModeStrike::does_auto_throttle() const
{
   return (!plane.Strike.isStrike);
}

void ModeStrike::update()
{
    // Update strike mode logic here
    float desired_pitch_deg = plane.aparm.str_min_dis.get(); // Targeting a steep dive angle of -45 degrees
    float pitch_deg = ahrs.get_pitch_deg();
    float roll_deg = ahrs.get_roll_deg();
    // float yaw_deg = ahrs.get_yaw_deg();
    
    float pitch_rate = (desired_pitch_deg - pitch_deg) / 0.5f;
    float roll_rate = (0.0f - roll_deg) / 0.5; // No roll change
    float yaw_rate = 0.0f; // No yaw change
    
    float throttle = plane.throttle_controller();
    float throttle_pct = constrain_float(throttle, plane.aparm.throttle_min, plane.aparm.throttle_max);

    plane.set_target_throttle_rate_rpy_strike(throttle_pct,roll_rate,pitch_rate,yaw_rate);
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

void ModeStrike::_exit(){
    plane.Strike.isStrike = false;
}

void ModeStrike::run()
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        pullup.stabilize_pullup();
        return;
    }
#endif
    
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {

        wiggle_servos();

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleRight);

        // Relax attitude control
        reset_controllers();

    } else {
        // Normal flight, run base class
        Mode::run();

    }
}
