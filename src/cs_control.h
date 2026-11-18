/*
 * cs_control.h - constant-speed propeller pitch controller (LPC1768)
 *
 * Call CS_PropRevolution() once per propeller revolution (e.g. from the
 * hall/capture ISR) with the period of that revolution in microseconds.
 * Call CS_Update() every 20 ms from the main loop. It returns the pitch motor
 * command in motor rpm:  > 0 = COARSE (blade angle increasing),
 *                        < 0 = FINE   (blade angle decreasing),
 *                          0 = stop.
 *
 * Sign convention: the measured motor rpm passed in must use the same signs
 * (+ = moving COARSE), whichever way the encoder counts.
 *
 * Blade geometry comes from YOUR function that converts Encoder_Pos to blade
 * angle (cam + lever: angle = 30 + asin(cam travel / lever radius)). The
 * controller differentiates it numerically to find degrees per encoder step
 * at the current position, so no separate geometry parameters are needed.
 */
#ifndef CS_CONTROL_H_
#define CS_CONTROL_H_

#include <stdint.h>
#include <stdbool.h>

#define CS_TICK_S  0.020f            /* CS_Update() call interval */
#define CS_REV_HISTORY  17           /* revolution periods kept (max rate window 16 samples) */

/* stop_flags bits (set by you from the hub controller's stop status) */
#define CS_STOP_FINE    0x01u        /* fine stop active: no further FINE movement     */
#define CS_STOP_COARSE  0x02u        /* coarse stop active: no further COARSE movement */

/* Pitch motor behaviour by hub firmware (CS_SetHubProfile). Claude 13/11/2026 */
typedef enum {
    CS_HUB_LEGACY = 0,  /* hub before 1.62: open-loop DAC, slow correction; customer COARSE ~0.74 of command */
    CS_HUB_LEARNED,     /* hub 1.62 on, raw mode: learned feed-forward, runs at the commanded speed */
    CS_HUB_MAXON        /* Maxon internal speed control (hub sw_flags bit 2): commanded speed, 20000 rpm/s ramp */
} CS_HUB_PROFILE_T;

typedef enum {
    CS_NO_RPM = 0,      /* fewer than 4 revolutions, or rpm data stale: motor stopped */
    CS_HOLD,            /* inside the deadband: motor stopped                         */
    CS_CONTROL,         /* normal PD control                                          */
    CS_OVERSPEED,       /* at or heading for max_rpm: full COARSE                     */
    CS_AT_LIMIT         /* wanted to move but the fine or coarse stop is active       */
} CS_STATE_T;

typedef struct {
    /* ---------------- configuration (set once, see CS_InitDefaults) ---------------- */
    float prop_gear_ratio;          /* engine rpm / prop rpm: 2.54 Rotax 916, 1.0 direct drive */
    float pitch_gear_ratio;         /* pitch motor : gearbox, 190.11 (for reference; only needed to set
                                       encoder_steps_per_motor_rev if steps are counted after the gearbox) */
    float encoder_steps_per_motor_rev; /* Encoder_Pos steps per pitch MOTOR revolution: 4.0   */
    float (*pos_to_angle)(int32_t encoder_pos);   /* YOUR function: Encoder_Pos -> blade deg */
    int32_t slope_span_steps;       /* +/- steps used to measure deg per step (smoothing)     */
    float motor_nominal_rpm;        /* max pitch motor speed, light load: 7000                 */
    float motor_coarse_loaded_rpm;  /* speed the motor reliably reaches driving COARSE under
                                       full aerodynamic load (high prop rpm): ~4000            */
    float motor_min_rpm;            /* slowest speed the motor runs smoothly at                */
    float motor_slew_rpm_per_tick;  /* max change of command per 20 ms                         */

    float target_rpm;               /* ENGINE rpm the pilot/mode selects                       */
    float deadband_rpm;             /* +/- ENGINE rpm: 60 Rotax 916, 30 direct drive           */
    float max_rpm;                  /* ENGINE rpm never to exceed: 5800                        */
    float overspeed_margin_rpm;     /* go full COARSE if heading within this of max_rpm        */
    float overspeed_min_rate_rpm_s; /* ...and only if rising faster than this: 150             */

    float lag_s;                    /* command-to-motor-response lag: 0.10 s                   */
    int32_t rate_window_revs;       /* rpm samples in the least-squares rate fit (3..16): 8   */
    float rate_window_s;            /* ...but no older than this (keeps low-rpm delay short)   */
    float rate_filter;              /* 0..1, extra smoothing of the rate (1 = none)            */
    float stale_s;                  /* rpm data older than this => CS_NO_RPM                   */
    float kp_motor_rpm_per_deg;     /* motor rpm per degree of blade needed: 1560              */
    float above_target_gain;        /* above target, gain rises to this as rpm nears max: 3.0  */
    float high_zone_rpm;            /* target within this of max_rpm => upper band = 0: 200    */
    float overspeed_min_gap_rpm;    /* overspeed level never closer than this to target: 60    */
    float reverse_pause_s;          /* stop for this long before changing direction: 0.15      */
    float overspeed_min_hold_s;     /* overspeed response lasts at least this long: 0.3 s      */
    float motor_decel_rpm_per_tick; /* max reduction of speed per tick (not stops): 1000       */
    float os_speed_filter;          /* 0..1 averaging of graded overspeed speed: 0.3           */
    float os_speed_fall_per_100ms;  /* burst speed may fall this fraction of peak per 100 ms   */
    float os_burst_max_deg;         /* burst ends once rpm falls AND blade moved this: 0.6 deg */
    float os_settle_s;              /* after a burst only a real max_rpm re-triggers: 0.6 s    */
    float os_recover_s;             /* after a burst FINE only at min speed, not while rising: 3 s */
    float os_response_s;            /* rpm response for the overspeed stopping distance...    */
    float os_response_zone_rpm;     /* ...within this below the target: 0.6 s, 400 rpm          */
    float os_quiet_s;               /* ...and only if the blade has been still this long: 1 s   */
    float rise_reach_s;             /* no FINE start if external rise reaches band within: 3 s */
    float start_persist_s;          /* outside the band this long before starting: 0.3 s       */
    float override_rate_deg;        /* rate thresholds in blade degrees (0 = use the rpm/s      */
    float fast_rise_deg;            /*   fields as set). Each tick:  rpm/s = deg x S / response */
    float overspeed_min_rate_deg;   /*   defaults 0.8, 2.15, 0.2 (= 300, 800, 75 at S 130)      */
    float stop_band_frac;           /* moving: stop when error within this x deadband: 0.5     */
    float override_rate_rpm_s;      /* rpm rate treated as a significant external factor       */
    float override_lookahead_s;     /* how far ahead to look when that happens                 */
    float rpm_per_deg_ss;           /* steady-state engine rpm change per deg of blade: 130    */
    float rpm_response_s;           /* how quickly rpm follows a blade change: 0.35 s          */
    float fast_rise_rpm_s;          /* rate above which a rise gets full COARSE at once        */
    /* Take-off run-up move (Claude 17/11/2026): with a take-off target, the rpm rising up through runup_frac of
       it and the blade finer than takeoff_angle_deg + takeoff_margin_deg, one full-speed COARSE move to that angle
       instead of the predicted-overspeed burst. takeoff_angle_deg 0 = not used (old behaviour). */
    float takeoff_angle_deg;        /* static full-throttle blade angle at the take-off rpm    */
    float takeoff_margin_deg;       /* added on the COARSE side: 0.5                            */
    float runup_stop_lead_s;        /* the move stops this x blade rate before the goal: 0.19 s */
    float runup_frac;               /* no COARSE below this fraction of the target: 0.85       */
    float runup_settle_s;           /* after the move, below the band: no predictive burst: 1 s */

    /* ---------------- inputs, update before each CS_Update() ---------------- */
    float motor_rpm;                /* optional, for logging only: blade speed is now measured
                                       from encoder_pos, so its sign no longer matters         */
    int32_t encoder_pos;            /* latest Encoder_Pos from the hub motor controller        */
    uint8_t stop_flags;             /* CS_STOP_FINE / CS_STOP_COARSE                           */

    /* ---------------- outputs / diagnostics (read only) ---------------- */
    float engine_rpm;               /* average of last 2 prop revs x prop_gear_ratio           */
    float engine_rpm_rate;          /* engine rpm per second (least-squares slope)             */
    float rpm_smoothed;             /* least-squares line value at the newest sample           */
    float rpm_change_100ms;         /* engine_rpm_rate x 0.1                                   */
    float predicted_rpm;            /* engine rpm expected when the next command takes effect  */
    float blade_deg;                /* blade angle from pos_to_angle(encoder_pos)              */
    float deg_per_step;             /* local blade degrees per encoder step (magnitude)        */
    float own_rate;                 /* rpm/s expected from our own blade movement              */
    float external_rate;            /* measured rate minus own_rate: throttle, climb, dive...  */
    int32_t motor_cmd_rpm;          /* last command                                            */
    CS_STATE_T state;

    /* ---------------- internal ---------------- */
    volatile uint32_t rev_period_us[CS_REV_HISTORY];   /* [0] = newest revolution */
    volatile uint32_t rev_count;
    uint32_t last_rev_count;
    float rev_age_s;
    bool rate_valid;
    int32_t pos_hist[3];            /* encoder_pos for the last 3 ticks, [0] newest            */
    uint8_t pos_hist_n;
    float blade_f;                  /* blade angle the rpm has caught up with                  */
    bool blade_f_valid;
    int32_t pause_ticks;            /* remaining stop time before a reversal                   */
    int32_t last_dir;               /* direction of the last movement: +1 / -1                 */
    bool rise_hold;                 /* external rise in progress: no FINE movement             */
    int32_t latch_ticks;            /* remaining minimum overspeed time                        */
    int32_t persist_ticks;          /* ticks the rpm has been outside the band while stopped   */
    int32_t relatch_ticks;          /* no re-trigger for this long after a release             */
    float os_speed_f;               /* averaged overspeed speed                                */
    float os_speed_out;             /* burst speed actually commanded                          */
    float os_speed_peak;            /* highest burst speed so far                              */
    float os_burst_start_deg;       /* blade angle when the burst started                      */
    int32_t settle_ticks;           /* remaining settle time after a burst                     */
    int32_t recover_ticks;          /* remaining gentle-recovery time after a burst            */
    int32_t quiet_ticks;            /* ticks since the pitch motor was last commanded          */
    bool overspeed_latched;         /* full COARSE until clearly safe again                    */
    CS_HUB_PROFILE_T hub_profile;   /* set by CS_SetHubProfile                                 */
    bool own_slow_at_takeoff;       /* own_pending with os_response_s near the take-off target */
    float blade_f2;                 /* as blade_f, with os_response_s                          */
    bool runup_armed;               /* take-off target, rpm seen below runup_frac, blade fine   */
    bool runup_move;                /* the run-up move is under way                             */
    int32_t runup_settle_ticks;     /* remaining runup_settle_s after the move                 */
} CS_CTRL_T;

/* Fill in Rotax 916 iS defaults (call once, then change what you need) */
void CS_InitDefaults(CS_CTRL_T *cs);

/* After CS_InitDefaults: select the pitch motor profile from the hub firmware version and Maxon mode */
void CS_SetHubProfile(CS_CTRL_T *cs, int hub_version, bool maxon_mode);

/* Call once per propeller revolution with its period in microseconds (ISR safe) */
void CS_PropRevolution(CS_CTRL_T *cs, uint32_t period_us);

/* Call every 20 ms. Returns the signed pitch motor command in rpm. */
int32_t CS_Update(CS_CTRL_T *cs);

#endif /* CS_CONTROL_H_ */
