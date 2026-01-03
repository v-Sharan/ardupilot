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

bool ModeStrike::_enter()
{
    int32_t lat = plane.aparm.str_lat.get();
    int32_t lon = plane.aparm.str_lon.get();
    
    if(lat == 0 && lon == 0) {
        gcs().send_text(MAV_SEVERITY_CRITICAL,"Strike WP invalid");
        return false;
    }

    // Store target location`
    target_location.lat = lat;
    target_location.lng = lon;

    // Initialize strike state
    in_terminal_dive = false;
    strike_complete = false;

    gcs().send_text(MAV_SEVERITY_INFO,"Strike Point Lat: %ld Lon: %ld", (long)lat, (long)lon);
    gcs().send_text(MAV_SEVERITY_INFO,"Strike Point Lat: %.6f Lon: %.6f", lat / 1e7f, lon / 1e7f);
    gcs().send_text(MAV_SEVERITY_INFO,"Entered Strike Mode - Aggressive Dive");
    
    // Set as guided waypoint for navigation
    // plane.set_guided_WP(target_location);
    
    // Initialize small radius for direct aggressive approach
    active_radius_m = 5.0f;
    
    return true;
}

// function do_knife_edge(arg1, arg2)
//     -- arg1 is pitch target, arg2 is duration
//     local now = millis():tofloat() * 0.001
//     if not running then
//         running = true
//         height_PI.reset()
//         knife_edge_s = now
//         gcs:send_text(5, string.format("Starting Pitch Change to %d°", arg1))
//     end

//     if (now - knife_edge_s) < arg2 then
//         -- Get current pitch
//         local pitch_deg = math.deg(ahrs:get_pitch_rad())
//         local pitch_error = (arg1 - pitch_deg)

//         -- Compute pitch rate using pitch error
//         local pitch_rate = pitch_error / PITCH_TCONST:get()
        
//         -- Keep roll stable (neutralize roll rate)
//         local roll_rate = 0

//         -- Maintain altitude
//         target_pitch = height_PI.update(initial_height)
        
//         -- Get yaw rate and throttle
//         yaw_rate = 0  -- No yaw adjustment needed
//         throttle = throttle_controller()

//         -- Apply control
//         set_rate_targets(throttle, roll_rate, pitch_rate, yaw_rate)
//     else
//         gcs:send_text(5, "Finished Pitch Change")
//         running = false
//         if vehicle:get_mode() == MODE_AUTO then
//             vehicle:nav_script_time_done(last_id)
//         else
//             vehicle:nav_scripting_enable(255)
//         end
//         return
//     end
// end

void ModeStrike::update()
{
    // Update strike mode logic here
    float desired_pitch_deg = -20.0f; // Targeting a steep dive angle of -45 degrees
    float pitch_deg = ahrs.pitch_sensor * 0.01f;
    float pitch_err = desired_pitch_deg - pitch_deg; // Targeting a steep dive angle of -desired degrees
    gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Desired Pitch: %.2f Current Pitch: %.2f Pitch Error: %.2f", desired_pitch_deg, pitch_deg, pitch_err);
    float pitch_rate = pitch_err / 0.05f;

    float pitch_rate_dps = constrain_float(pitch_rate,-plane.g.acro_pitch_rate,plane.g.acro_pitch_rate); // Time constant of 0.1 seconds for aggressive response
    float roll_rate_dps = constrain_float(0.0, -plane.g.acro_roll_rate, plane.g.acro_roll_rate); // No roll change
    float yaw_rate_dps = constrain_float(0.0, -plane.g.acro_yaw_rate, plane.g.acro_yaw_rate); // No yaw change

    float throttle = plane.throttle_controller();
    gcs().send_text(MAV_SEVERITY_WARNING,"Strike: Pitch Rate: %.2f Roll Rate: %.2f Yaw Rate: %.2f Throttle: %.2f", pitch_rate_dps, roll_rate_dps, yaw_rate_dps, throttle);

    const float speed_scaler = plane.get_speed_scaler();
    const float aileron = plane.rollController.get_rate_out(roll_rate_dps, speed_scaler);
    const float elevator = plane.pitchController.get_rate_out(pitch_rate_dps, speed_scaler);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
    float rudder = 0;
    if (plane.yawController.rate_control_enabled()) {
        rudder = 0 * 45;
        if (true) {
            rudder += plane.yawController.get_rate_out(yaw_rate_dps, speed_scaler, false);
        } else {
            plane.yawController.reset_I();
        }
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
}

// void ModeStrike::update()
// {
//     float desired_pitch_deg = -20.0f;
//     float current_pitch_deg = ahrs.pitch_sensor * 0.01f;

//     // Pitch error (deg)
//     float pitch_error_deg = desired_pitch_deg - current_pitch_deg;
    
//     // Angle error → pitch rate (deg/sec)
//     float pitch_rate_dps = pitch_error_deg / 0.05f;
//     // const float dive_angle_dps = radians(dive_angle_rad);
//     const float speed_scaler = plane.get_speed_scaler();
//     Vector3f body_rates = plane.get_body_rates(pitch_rate_dps);
//     const float aileron = plane.rollController.get_rate_out(body_rates.x, speed_scaler);
//     const float elevator = plane.pitchController.get_rate_out(body_rates.y, speed_scaler);
//     SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
//     SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
//     gcs().send_text(MAV_SEVERITY_WARNING,"Strike: %.4f deg dive elevator: %.4f", body_rates.y,elevator);
//     float rudder = 0;
//     if (plane.yawController.rate_control_enabled()) {
//         rudder = 0 * 45;
//         if (true) {
//             rudder += plane.yawController.get_rate_out(0, speed_scaler, false);
//         } else {
//             plane.yawController.reset_I();
//         }
//     }
//     SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
//     SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);

//     float pitch_rad = ahrs.pitch_sensor * 0.01f * DEG_TO_RAD;
//     float throttle = plane.aparm.throttle_cruise.get() + sin(pitch_rad) * 80.0f;
//     float throttle_con = constrain_float(throttle, 0.0f, 100.0f);
//     float throttle_pct = constrain_float(throttle_con, plane.aparm.throttle_min, plane.aparm.throttle_max);
//     SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle_pct);
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