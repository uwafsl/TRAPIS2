//
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
//
//  DO NOT EDIT this file to adjust your configuration.  Create your own
//  APM_Config.h and use APM_Config.h.example as a reference.
//
// WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING WARNING
///
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// Default and automatic configuration details.
//
// Notes for maintainers:
//
// - Try to keep this file organised in the same order as APM_Config.h.example
//
#pragma once

#include "defines.h"

///
/// DO NOT EDIT THIS INCLUDE - if you want to make a local change, make that
/// change in your local copy of APM_Config.h.
///
#include "APM_Config.h"

// Just so that it's completely clear...
#define ENABLED                 1
#define DISABLED                0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifdef CONFIG_APM_HARDWARE
#error CONFIG_APM_HARDWARE option is depreated! use CONFIG_HAL_BOARD instead.
#endif

#ifndef MAV_SYSTEM_ID
 # define MAV_SYSTEM_ID          1
#endif

//////////////////////////////////////////////////////////////////////////////
// FrSky telemetry support
//

#ifndef FRSKY_TELEM_ENABLED
#define FRSKY_TELEM_ENABLED ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// Optical flow sensor support
//

#ifndef OPTFLOW
#if AP_AHRS_NAVEKF_AVAILABLE
 # define OPTFLOW ENABLED
#else
 # define OPTFLOW DISABLED
#endif
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// Radio channel limits
//
// Note that these are not called out in APM_Config.h.reference.
//
#ifndef CH5_MIN
 # define CH5_MIN        1000
#endif
#ifndef CH5_MAX
 # define CH5_MAX        2000
#endif
#ifndef CH6_MIN
 # define CH6_MIN        1000
#endif
#ifndef CH6_MAX
 # define CH6_MAX        2000
#endif
#ifndef CH7_MIN
 # define CH7_MIN        1000
#endif
#ifndef CH7_MAX
 # define CH7_MAX        2000
#endif
#ifndef CH8_MIN
 # define CH8_MIN        1000
#endif
#ifndef CH8_MAX
 # define CH8_MAX        2000
#endif


#ifndef FLAP_1_PERCENT
 # define FLAP_1_PERCENT 0
#endif
#ifndef FLAP_1_SPEED
 # define FLAP_1_SPEED 0
#endif
#ifndef FLAP_2_PERCENT
 # define FLAP_2_PERCENT 0
#endif
#ifndef FLAP_2_SPEED
 # define FLAP_2_SPEED 0
#endif
//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE
// FLIGHT_MODE_CHANNEL
//
#ifndef FLIGHT_MODE_CHANNEL
 # define FLIGHT_MODE_CHANNEL    8
#endif
#if (FLIGHT_MODE_CHANNEL != 5) && (FLIGHT_MODE_CHANNEL != 6) && (FLIGHT_MODE_CHANNEL != 7) && (FLIGHT_MODE_CHANNEL != 8)
 # error XXX
 # error XXX You must set FLIGHT_MODE_CHANNEL to 5, 6, 7 or 8
 # error XXX
#endif

#if !defined(FLIGHT_MODE_1)
 # define FLIGHT_MODE_1                  RTL
#endif
#if !defined(FLIGHT_MODE_2)
 # define FLIGHT_MODE_2                  RTL
#endif
#if !defined(FLIGHT_MODE_3)
 # define FLIGHT_MODE_3                  FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_4)
 # define FLIGHT_MODE_4                  FLY_BY_WIRE_A
#endif
#if !defined(FLIGHT_MODE_5)
 # define FLIGHT_MODE_5                  MANUAL
#endif
#if !defined(FLIGHT_MODE_6)
 # define FLIGHT_MODE_6                  MANUAL
#endif


//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM
//
#ifndef AUTO_TRIM
 # define AUTO_TRIM                              DISABLED
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY
//
#ifndef GROUND_START_DELAY
 # define GROUND_START_DELAY             0
#endif

//////////////////////////////////////////////////////////////////////////////
// ENABLE ELEVON_MIXING
//
#ifndef ELEVON_MIXING
 # define ELEVON_MIXING          DISABLED
#endif
#ifndef ELEVON_REVERSE
 # define ELEVON_REVERSE     DISABLED
#endif
#ifndef ELEVON_CH1_REVERSE
 # define ELEVON_CH1_REVERSE     DISABLED
#endif
#ifndef ELEVON_CH2_REVERSE
 # define ELEVON_CH2_REVERSE     DISABLED
#endif

#ifndef DSPOILR_RUD_RATE_DEFAULT
 #define DSPOILR_RUD_RATE_DEFAULT 100
#endif

//////////////////////////////////////////////////////////////////////////////
// CAMERA TRIGGER AND CONTROL
//
// uses 1182 bytes of memory
#ifndef CAMERA
 # define CAMERA         ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
// MOUNT (ANTENNA OR CAMERA)
//
// uses 7726 bytes of memory on 2560 chips (all options are enabled)
#ifndef MOUNT
#define MOUNT          ENABLED
#endif

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Altitude measurement and control.
//
#ifndef ALTITUDE_MIX
 # define ALTITUDE_MIX                   1
#endif


//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE
//
#ifndef AIRSPEED_CRUISE
 # define AIRSPEED_CRUISE                12 // 12 m/s
#endif
#define AIRSPEED_CRUISE_CM AIRSPEED_CRUISE*100


//////////////////////////////////////////////////////////////////////////////
// MIN_GNDSPEED
//
#ifndef MIN_GNDSPEED
 # define MIN_GNDSPEED                   0 // m/s (0 disables)
#endif
#define MIN_GNDSPEED_CM MIN_GNDSPEED*100


//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control
//
#ifndef AIRSPEED_FBW_MIN
 # define AIRSPEED_FBW_MIN               9
#endif
#ifndef AIRSPEED_FBW_MAX
 # define AIRSPEED_FBW_MAX               22
#endif

#ifndef ALT_HOLD_FBW
 # define ALT_HOLD_FBW 0
#endif
#define ALT_HOLD_FBW_CM ALT_HOLD_FBW*100


//////////////////////////////////////////////////////////////////////////////
// Servo Mapping
//
#ifndef THROTTLE_MIN
 # define THROTTLE_MIN                   0 // percent
#endif
#ifndef THROTTLE_CRUISE
 # define THROTTLE_CRUISE                45
#endif
#ifndef THROTTLE_MAX
 # define THROTTLE_MAX                   100
#endif

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
#ifndef HEAD_MAX
 # define HEAD_MAX                               45
#endif
#ifndef PITCH_MAX
 # define PITCH_MAX                              20
#endif
#ifndef PITCH_MIN
 # define PITCH_MIN                              -25
#endif
#define HEAD_MAX_CENTIDEGREE HEAD_MAX * 100
#define PITCH_MAX_CENTIDEGREE PITCH_MAX * 100
#define PITCH_MIN_CENTIDEGREE PITCH_MIN * 100

#ifndef RUDDER_MIX
 # define RUDDER_MIX           0.5f
#endif


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Dataflash logging control
//

#ifndef LOGGING_ENABLED
 # define LOGGING_ENABLED                ENABLED
#endif

#define DEFAULT_LOG_BITMASK   0xffff


//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
#ifndef WP_RADIUS_DEFAULT
 # define WP_RADIUS_DEFAULT              90
#endif

#ifndef LOITER_RADIUS_DEFAULT
 # define LOITER_RADIUS_DEFAULT 60
#endif

#ifndef ALT_HOLD_HOME
 # define ALT_HOLD_HOME 100
#endif
#define ALT_HOLD_HOME_CM ALT_HOLD_HOME*100

#ifndef USE_CURRENT_ALT
 # define USE_CURRENT_ALT FALSE
#endif

#ifndef INVERTED_FLIGHT_PWM
 # define INVERTED_FLIGHT_PWM 1750
#endif

#ifndef PX4IO_OVERRIDE_PWM
 # define PX4IO_OVERRIDE_PWM 1750
#endif

//////////////////////////////////////////////////////////////////////////////
// Developer Items
//

#ifndef SCALING_SPEED
 # define SCALING_SPEED          15.0
#endif

// use this to completely disable the CLI. We now default the CLI to
// off on smaller boards.
#ifndef CLI_ENABLED
#define CLI_ENABLED ENABLED
#endif

// use this to disable geo-fencing
#ifndef GEOFENCE_ENABLED
 # define GEOFENCE_ENABLED ENABLED
#endif

// pwm value on FENCE_CHANNEL to use to enable fenced mode
#ifndef FENCE_ENABLE_PWM
 # define FENCE_ENABLE_PWM 1750
#endif

// a digital pin to set high when the geo-fence triggers. Defaults
// to -1, which means don't activate a pin
#ifndef FENCE_TRIGGERED_PIN
 # define FENCE_TRIGGERED_PIN -1
#endif

// if RESET_SWITCH_CH is not zero, then this is the PWM value on
// that channel where we reset the control mode to the current switch
// position (to for example return to switched mode after failsafe or
// fence breach)
#ifndef RESET_SWITCH_CHAN_PWM
 # define RESET_SWITCH_CHAN_PWM 1750
#endif

#define HIL_SUPPORT ENABLED

//////////////////////////////////////////////////////////////////////////////
// Parachute release
#ifndef PARACHUTE
#define PARACHUTE ENABLED
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && !defined(CONFIG_ARCH_BOARD_PX4FMU_V4)
# define HAVE_PX4_MIXER 1
#else
# define HAVE_PX4_MIXER 0
#endif

//UWAFSL START
//Adding Parameters
#ifndef UW_RADIUS      //Defines uw_radius
 # define UW_RADIUS 100 //Sets default value
#endif
#ifndef UW_ALTITUDE      //Defines uw_altitude
 # define UW_ALTITUDE 100  //Sets default value
#endif
#ifndef WSTR_WL_PRO_GAIN     //Defines wstr_wl_pro_gain
 # define WSTR_WL_PRO_GAIN 3 //Sets default value
#endif
#ifndef WSTR_WL_DER_GAIN     //Defines wstr_wl_der_gain
# define WSTR_WL_DER_GAIN 0.5 //Sets default value
#endif
#ifndef WSTR_RD_PRO_GAIN     //Defines wstr_rd_pro_gain
# define WSTR_RD_PRO_GAIN 3 //Sets default value
#endif
#ifndef WSTR_RD_DER_GAIN     //Defines wstr_rd_der_gain
# define WSTR_RD_DER_GAIN 0.5 //Sets default value
#endif
#ifndef WSTR_AH_PRO_GAIN     //Defines wstr_ah_pro_gain
# define WSTR_AH_PRO_GAIN 0.5 //Sets default value
#endif
#ifndef WSTR_TRAPIS_LOC       //Defines wstr_trapis_loc
# define WSTR_TRAPIS_LOC 0  //Sets default value
#endif
#ifndef WSTR_ACTIVATE       //Defines wstr_activate
# define WSTR_ACTIVATE 0  //Sets default value
#endif
#ifndef WSTR_WL_INT_GAIN     //Defines wstr_wl_int_gain
# define WSTR_WL_INT_GAIN 0.01 //Sets default value
#endif
#ifndef WSTR_AL_PRO_GAIN     //Defines uw_pro_forget_factor
# define WSTR_AL_PRO_GAIN 3 //Sets default value
#endif
#ifndef WSTR_AL_DER_GAIN    //Defines uw_der_forget_factor
# define WSTR_AL_DER_GAIN 0.5 //Sets default value
#endif
/*
--------- Old parameters that we no longer use -------------

#ifndef WSTR_HOME       //Defines wstr_home
# define WSTR_HOME 1  //Sets default value
#endif
#ifndef UW_PSIDOTERR_LIM     //Defines uw_psiDotErr_lim
# define UW_PSIDOTERR_LIM 0.3 //Sets default value
#endif
#ifndef UW_PRO_FORGET_FACTOR     //Defines uw_pro_forget_factor
# define UW_PRO_FORGET_FACTOR 0.98 //Sets default value
#endif
#ifndef UW_DER_FORGET_FACTOR    //Defines uw_der_forget_factor
# define UW_DER_FORGET_FACTOR 0.8 //Sets default value
#endif
*/
//UWAFSL END
