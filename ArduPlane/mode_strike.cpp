#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>

bool ModeStrike::_enter()
{
    Location loc{plane.current_loc};

    int32_t lat = plane.aparm.str_lat.get();
    int32_t lon = plane.aparm.str_lon.get();
    
    if(lat == 0 && lon == 0) {
        // gcs().send_text(MAV_SEVERITY_INFO,"Lat: %.6f Lon: %.6f", lat / 1e7f, lon / 1e7f);
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike WP invalid");
        return false;
    }
    
    // Store target location
    target_location.lat = lat;
    target_location.lng = lon;
    target_location.alt = loc.alt;
    
    plane.set_rudder_offset_strike(0,true);
    gcs().send_text(MAV_SEVERITY_INFO,"Lat: %.6f Lon: %.6f alt: %d",target_location.lat / 1e7f, target_location.lng / 1e7f,target_location.alt);
    plane.guided_throttle_passthru = false;
    plane.set_guided_WP(target_location);
    isStrike = false;
    plane.Strike.isStrikeMode = false;
    return true;
}

void ModeStrike::navigate()
{
    plane.update_loiter(0);
}


bool ModeStrike::does_auto_navigation() const
{
   return (!plane.Strike.isStrikeMode);
}

bool ModeStrike::does_auto_throttle() const
{
   return (!plane.Strike.isStrikeMode);
}

void ModeStrike::update()
{
    float distance = plane.current_loc.get_distance(target_location);
    int16_t terminal_distance = plane.aparm.str_term_dis.get();

    if(distance > terminal_distance){
        plane.mode_guided.update();
    }else {
        if(!isStrike){
            gcs().send_text(MAV_SEVERITY_INFO,"Started Dive");
        }
        plane.Strike.isStrikeMode = true;
        isStrike = true;
        plane.current_loc.change_alt_frame(Location::AltFrame::ABOVE_HOME);
        
        float alt_m = plane.current_loc.alt * 0.01f;
        
        float horizontal_distance = plane.current_loc.get_distance(target_location);

        if (horizontal_distance < 1.0f) {
            horizontal_distance = 1.0f;
        }
        
        // Dive angle
        float dive_angle = degrees(atanf(alt_m / horizontal_distance));
        
        // Target bearing
        float target_bearing = degrees(plane.current_loc.get_bearing(target_location));
        
        // Aircraft attitude
        float pitch_deg = ahrs.get_pitch_deg();
        float roll_deg  = ahrs.get_roll_deg();
        float yaw_deg   = degrees(ahrs.get_yaw());
        
        // Bearing error normalization
        float bearing_error = target_bearing - yaw_deg;
        
        while (bearing_error > 180.0f)  bearing_error -= 360.0f;
        while (bearing_error < -180.0f) bearing_error += 360.0f;
        
        // Desired dive pitch
        float desired_pitch_deg = -dive_angle;
        
        // Smooth bank turning
        float max_bank = 45.0f;
        
        float desired_roll = constrain_float(bearing_error * 0.5f, -max_bank, max_bank);

        float roll_rate = (desired_roll - roll_deg) / 0.5f;

        // Dive only when aligned
        float pitch_rate = (desired_pitch_deg - pitch_deg) / 0.5f;
        
        float yaw_rate = 0.0f;
        
        float throttle = plane.throttle_controller();
        float throttle_pct = constrain_float(throttle, plane.aparm.throttle_min, plane.aparm.throttle_max);
        
        plane.set_target_throttle_rate_rpy_strike(throttle_pct,roll_rate,pitch_rate,yaw_rate);
    }
}

float ModeStrike::compute_dive_angle(float w, float x, float y, float z)
{
    // vertical component of camera forward vector
    float vz = 1.0f - 2.0f * (x*x + y*y);

    // compute dive angle
    float angle_rad = asinf(-vz);

    return fabsf(degrees(angle_rad));
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
    if(isStrike){
        plane.Strike.isStrikeMode = false;
    }
    isStrike = false;
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
