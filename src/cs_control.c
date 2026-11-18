/*
 * cs_control.c - constant-speed propeller pitch controller (LPC1768)
 *
 * Normal governing (most of the time) is proportional, as in the original logic:
 *   - stopped: do nothing while the rpm error is inside the deadband
 *   - outside it: drive toward the target at a speed proportional to the blade
 *     angle needed (rpm distance / rpm_per_deg_ss x kp_motor_rpm_per_deg),
 *     never below the minimum speed (700 = 10%)
 *   - keep moving until the error is inside half the deadband, then send ONE
 *     zero (stop with brake). No stop-go-stop-go.
 *
 * The error used is predicted over the short lag (rpm measurement + ~120 ms
 * motor response), including the blade movement already under way, which is
 * measured from the encoder.
 *
 * Significant external factors (throttle, steep climb/descent) are detected
 * by a fast rpm rate (least-squares slope). Then the controller looks further
 * ahead (override_lookahead_s) so it starts moving, or moves faster, before the
 * rpm arrives. Overspeed and fast rises (take-off) get full COARSE at once.
 */

#include "chip.h"          /* __disable_irq / __enable_irq */
#include "cs_control.h"

void CS_InitDefaults(CS_CTRL_T *cs)
{
    cs->prop_gear_ratio          = 2.54f;
    cs->pitch_gear_ratio         = 190.11f;
    cs->encoder_steps_per_motor_rev = 4.0f;  /* one step = 1/4 motor revolution */
    cs->pos_to_angle             = 0;        /* MUST SET to your Encoder_Pos -> angle function */
    cs->slope_span_steps         = 8;
    cs->motor_nominal_rpm        = 7000.0f;
    cs->motor_coarse_loaded_rpm  = 3000.0f;  /* COARSE speed under full load (overspeed prediction); bench log 0792 */
    cs->motor_min_rpm            = 700.0f;   /* 10% of nominal */
    cs->motor_slew_rpm_per_tick  = 7000.0f;  /* no ramp up: straight to the commanded speed (user 17/11/2026). Was
                                                1000 (0 -> full in 140 ms, gentler reversals): reversals already stop
                                                for reverse_pause_s, and the ramp's large steps kept restarting hub
                                                1.63's speed-correction hold-off (~0.12 s lost at every start) */

    cs->target_rpm               = 5400.0f;
    cs->deadband_rpm             = 60.0f;
    cs->max_rpm                  = 5800.0f;
    cs->overspeed_margin_rpm     = 100.0f;
    cs->overspeed_min_rate_rpm_s = 150.0f;   /* predictive overspeed only when rising faster */

    cs->lag_s                    = 0.12f;    /* measured: command -> 50% motor speed, log 0197 */
    cs->rate_window_revs         = 8;        /* least-squares slope over 8 two-rev samples */
    cs->rate_window_s            = 0.25f;
    cs->rate_filter              = 1.0f;     /* no extra smoothing needed with least squares */
    cs->stale_s                  = 0.30f;
    cs->kp_motor_rpm_per_deg     = 1560.0f;  /* motor rpm per degree of blade needed (= 12 per rpm at 130 rpm/deg) */
    cs->above_target_gain        = 3.0f;     /* gain multiplier as rpm nears max_rpm above target */
    cs->high_zone_rpm            = 200.0f;   /* target within this of max_rpm: upper band removed */
    cs->overspeed_min_gap_rpm    = 60.0f;    /* overspeed level at least this above target */
    cs->reverse_pause_s          = 0.15f;    /* stop this long before reversing */
    cs->overspeed_min_hold_s     = 0.30f;    /* overspeed response lasts at least this long */
    cs->motor_decel_rpm_per_tick = 1000.0f;  /* ease down 1000 rpm per tick */
    cs->os_speed_filter          = 0.3f;     /* averaging of the graded overspeed speed */
    cs->os_speed_fall_per_100ms  = 0.15f;    /* burst speed falls at most 15% of its peak per 100 ms */
    cs->os_burst_max_deg         = 0.6f;     /* minimum burst: ~0.5 deg stopped the customer's take-off rise */
    cs->os_settle_s              = 0.6f;     /* let the rpm show the effect before another burst */
    cs->os_recover_s             = 3.0f;     /* customer cycle ~3 s: the fast FINE recovery set up the next burst */
    cs->os_response_s            = 0.6f;     /* take-off power: rpm follows the blade in ~0.6-0.7 s, not 0.3 */
    cs->os_response_zone_rpm     = 400.0f;   /* ...used within 400 rpm below the target (bench run 975) */
    cs->os_quiet_s               = 1.0f;     /* ...and only after the blade has been still 1 s (run 977) */
    cs->rise_reach_s             = 3.0f;     /* no fine if an external rise reaches the band within this */
    cs->override_rate_deg        = 0.8f;     /* = 300 rpm/s at S 130, 0.35 s */
    cs->fast_rise_deg            = 2.15f;    /* = 800 rpm/s */
    cs->overspeed_min_rate_deg   = 0.2f;     /* = 133 rpm/s at S 200 (was 0.4: bench run 961, a 250-300 rpm/s
                                                take-off rise was too slow to trigger before 5764 -> 5801) */
    cs->stop_band_frac           = 0.5f;     /* stop when within half the deadband */
    cs->override_rate_rpm_s      = 300.0f;   /* faster than this = significant external factor */
    cs->override_lookahead_s     = 0.5f;
    cs->rpm_per_deg_ss           = 130.0f;   /* log 0197/0792: 100-180 rpm per degree */
    cs->rpm_response_s           = 0.30f;
    cs->start_persist_s          = 0.30f;    /* rpm must stay outside the band this long before a start
                                                (control.c sets 0.5 after Russell's tests) */
    cs->fast_rise_rpm_s          = 800.0f;   /* e.g. take-off throttle advance */
    cs->takeoff_angle_deg        = 0.0f;     /* parameter BL_TAKEOFF_ANGLE (control.c); 0 = no run-up move */
    cs->takeoff_margin_deg       = 0.5f;     /* user 17/11/2026: 0.7 (19.0 for a static 18.3); 0.5 after bench run 1141 */
    cs->runup_stop_lead_s        = 0.19f;    /* bench run 1141: stopping at blade rate x lag_s (0.12 s) ran 0.2 deg past */
    cs->runup_frac               = 0.85f;    /* user: COARSE below ~5000 for a 5700 take-off is a mistake */
    cs->runup_settle_s           = 1.0f;

    cs->motor_rpm = 0.0f;
    cs->encoder_pos = 0;
    cs->stop_flags = 0;
    cs->blade_deg = 0.0f;
    cs->deg_per_step = 0.0f;

    cs->engine_rpm = cs->engine_rpm_rate = cs->rpm_change_100ms = cs->predicted_rpm = 0.0f;
    cs->rpm_smoothed = 0.0f;
    cs->motor_cmd_rpm = 0;
    cs->state = CS_NO_RPM;

    for (int i = 0; i < CS_REV_HISTORY; i++) cs->rev_period_us[i] = 0;
    cs->rev_count = 0;
    cs->last_rev_count = 0;
    cs->rev_age_s = 1.0f;
    cs->rate_valid = false;
    cs->pos_hist_n = 0;
    cs->own_rate = cs->external_rate = 0.0f;
    cs->overspeed_latched = false;
    cs->blade_f = 0.0f;
    cs->blade_f_valid = false;
    cs->pause_ticks = 0;
    cs->last_dir = 0;
    cs->rise_hold = false;
    cs->persist_ticks = 0;
    cs->latch_ticks = 0;
    cs->relatch_ticks = 0;
    cs->os_speed_f = -1.0f;
    cs->os_speed_out = 0.0f;
    cs->os_speed_peak = 0.0f;
    cs->os_burst_start_deg = 0.0f;
    cs->settle_ticks = 0;
    cs->recover_ticks = 0;
    cs->quiet_ticks = 0;
    cs->hub_profile = CS_HUB_LEGACY;
    cs->own_slow_at_takeoff = false;
    cs->blade_f2 = 0.0f;
    cs->runup_armed = false;
    cs->runup_move = false;
    cs->runup_settle_ticks = 0;
}

/*
 * Claude 13/11/2026. Pitch motor behaviour depends on the hub firmware (version from the start-up handshake):
 *  - before 1.62: open-loop DAC with slow correction; under load COARSE runs well below the commanded speed (customer
 *    logs ~0.74 at 10 %), FINE a little above. The constants above were tuned with such hubs - unchanged.
 *  - 1.62 on (raw mode): learned feed-forward per direction and speed, saved in MRAM; runs at the commanded speed
 *    (bench 0.97-1.02 from 5 to 70 %), 1.63 down to 3 %.
 *  - Maxon internal speed control (hub sw_flags bit 2): commanded speed, but the DEC ramps the set value at
 *    ~20000 rpm/s, so a full-speed burst takes ~0.35 s to reach speed.
 * Profiles other than LEGACY start with the same constants; change them here only, as bench tuning shows.
 */
void CS_SetHubProfile(CS_CTRL_T *cs, int hub_version, bool maxon_mode)
{
    if (maxon_mode)              cs->hub_profile = CS_HUB_MAXON;
    else if (hub_version >= 162) cs->hub_profile = CS_HUB_LEARNED;
    else                         cs->hub_profile = CS_HUB_LEGACY;

    switch (cs->hub_profile) {
    case CS_HUB_LEARNED:
        /* At take-off the rpm follows the blade in ~0.6-0.7 s, not rpm_response_s. With 1.63 the AC300 often
           followed a take-off burst with one long slow COARSE move (instead of the older hubs' second burst) that
           stopped when the 0.3 s prediction reached the band and overshot ~0.13 deg (rpm 5700 -> 5640, then FINE:
           bench 1092-1095, fast_then_boost). A lower motor_min_rpm (560) only made that move longer, same overshoot. */
        cs->own_slow_at_takeoff = true;
        break;
    case CS_HUB_MAXON:
        break;
    default:
        break;
    }
}

void CS_PropRevolution(CS_CTRL_T *cs, uint32_t period_us)
{
    for (int i = CS_REV_HISTORY - 1; i > 0; i--)
        cs->rev_period_us[i] = cs->rev_period_us[i - 1];
    cs->rev_period_us[0] = period_us;
    cs->rev_count++;
}

/* Small helpers so no maths library is needed (works with any Redlib/Newlib variant) */
static float Absf(float v)   { return v < 0.0f ? -v : v; }
static bool  IsNanf(float v) { return v != v; }          /* NaN is the only value not equal to itself */
static int32_t RoundToInt(float v) { return (int32_t)(v >= 0.0f ? v + 0.5f : v - 0.5f); }

/*
 * RPM rate by least squares.
 * Samples are the 2-revolution average rpm ending at each revolution (the
 * same averaging that cancels 4-cylinder direct-drive firing pulses), placed
 * at the centre of their 2-rev window. A straight line is fitted through the
 * newest samples and its slope is the rate. Every sample contributes, so the
 * result is far smoother than a difference between two averages.
 * The fitted line's value at the newest sample is also returned: a smoothed
 * rpm for the proportional term.
 * avail = number of valid periods in p[]. Returns false if < 3 samples.
 */
static bool LeastSquaresRate(const CS_CTRL_T *cs, const uint32_t *p, int avail, float *rate, float *rpm_fit)
{
    int max_samples = cs->rate_window_revs;
    if (max_samples < 3) max_samples = 3;
    if (max_samples > CS_REV_HISTORY - 1) max_samples = CS_REV_HISTORY - 1;
    if (max_samples > avail - 1) max_samples = avail - 1;
    if (max_samples < 3) return false;

    float t_s[CS_REV_HISTORY], y[CS_REV_HISTORY];
    float end = 0.0f;                  /* end time of revolution k, newest = 0, going back (s) */
    int n = 0;
    for (int k = 0; k < max_samples; k++) {
        float pk  = (float)p[k] * 1e-6f;
        float pk1 = (float)p[k + 1] * 1e-6f;
        float win = pk + pk1;
        float mid = end - 0.5f * win;
        if (n >= 3 && -mid > cs->rate_window_s) break;   /* older than the time window */
        t_s[n] = mid;
        y[n]   = 120.0f / win * cs->prop_gear_ratio;      /* engine rpm */
        n++;
        end -= pk;
    }

    float st = 0.0f, sy = 0.0f;
    for (int i = 0; i < n; i++) { st += t_s[i]; sy += y[i]; }
    float tm = st / n, ym = sy / n, sxx = 0.0f, sxy = 0.0f;
    for (int i = 0; i < n; i++) {
        float dt = t_s[i] - tm;
        sxx += dt * dt;
        sxy += dt * (y[i] - ym);
    }
    if (sxx <= 0.0f) return false;
    *rate = sxy / sxx;                 /* engine rpm per second */
    *rpm_fit = ym + *rate * (t_s[0] - tm);
    return true;
}

static float Clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

int32_t CS_Update(CS_CTRL_T *cs)
{
    uint32_t p[CS_REV_HISTORY], count;
    float cmd = 0.0f;

    /* ---- take a consistent copy of the revolution data ---- */
    __disable_irq();
    for (int i = 0; i < CS_REV_HISTORY; i++) p[i] = cs->rev_period_us[i];
    count = cs->rev_count;
    __enable_irq();

    bool new_rev = (count != cs->last_rev_count);
    if (new_rev) {
        cs->last_rev_count = count;
        cs->rev_age_s = 0.5f * CS_TICK_S;   /* arrived somewhere in the last tick */
    } else {
        cs->rev_age_s += CS_TICK_S;
    }

    /* ---- blade angle and local gearing from the user's geometry function ---- */
    bool geometry_ok = false;
    float deg_per_motor_rev = 0.0f;
    if (cs->pos_to_angle != 0) {
        int32_t pos = cs->encoder_pos;
        int32_t k = cs->slope_span_steps > 0 ? cs->slope_span_steps : 1;
        float b  = cs->pos_to_angle(pos);
        float dps = Absf(cs->pos_to_angle(pos + k) - cs->pos_to_angle(pos - k)) / (2.0f * (float)k);
        if (!IsNanf(b)) cs->blade_deg = b;
        if (!IsNanf(dps) && dps > 1e-6f) cs->deg_per_step = dps;   /* else keep the last good value */
        deg_per_motor_rev = cs->deg_per_step * cs->encoder_steps_per_motor_rev;
        geometry_ok = deg_per_motor_rev > 0.0f;
    }

    if (!geometry_ok || count < 4 || p[3] == 0 || cs->rev_age_s > cs->stale_s) {
        cs->state = CS_NO_RPM;
        cs->rate_valid = false;
        cs->blade_f_valid = false;
        cs->overspeed_latched = false;
        cs->runup_armed = cs->runup_move = false;
        cs->runup_settle_ticks = 0;
        cmd = 0.0f;
    }
    else {
        /* ---- rpm from the last 2 revs, rate by least squares over recent revs ---- */
        float win_a = (float)(p[0] + p[1]) * 1e-6f;          /* seconds, newest 2 revs */
        cs->engine_rpm = 120.0f / win_a * cs->prop_gear_ratio;

        if (new_rev) {
            int avail = (count < CS_REV_HISTORY) ? (int)count : CS_REV_HISTORY;
            float raw_rate, rpm_fit;
            if (LeastSquaresRate(cs, p, avail, &raw_rate, &rpm_fit)) {
                if (!cs->rate_valid) { cs->engine_rpm_rate = raw_rate; cs->rate_valid = true; }
                else cs->engine_rpm_rate += cs->rate_filter * (raw_rate - cs->engine_rpm_rate);
                cs->rpm_smoothed = rpm_fit;
            } else {
                cs->rpm_smoothed = cs->engine_rpm;
            }
        }
        cs->rpm_change_100ms = cs->engine_rpm_rate * 0.1f;

        /* ---- measurements ---- */
        float d   = 0.5f * win_a + cs->rev_age_s;   /* age of the rpm measurement (s) */
        float r   = cs->engine_rpm_rate;
        /* guard against impossible values (e.g. 0 from an empty config record):
           both are divided by below */
        if (cs->rpm_per_deg_ss < 1.0f)  cs->rpm_per_deg_ss = 1.0f;
        if (cs->rpm_response_s < 0.05f) cs->rpm_response_s = 0.05f;
        float S   = cs->rpm_per_deg_ss;             /* engine rpm per degree of blade */
        if (!cs->rate_valid) cs->rpm_smoothed = cs->engine_rpm;
        float err = cs->rpm_smoothed - cs->target_rpm;
        float db  = cs->deadband_rpm;

        /* Rate thresholds defined in blade degrees: the blade movement needed to
           cancel that rpm rate is rate x rpm_response_s / S, so threshold (rpm/s)
           = degrees x S / rpm_response_s. This makes them engine-independent.
           A _deg value of 0 means "use the rpm/s field as set". */
        {
            float rpm_s_per_deg = S / cs->rpm_response_s;
            if (cs->override_rate_deg > 0.0f)      cs->override_rate_rpm_s      = cs->override_rate_deg      * rpm_s_per_deg;
            if (cs->fast_rise_deg > 0.0f)          cs->fast_rise_rpm_s          = cs->fast_rise_deg          * rpm_s_per_deg;
            if (cs->overspeed_min_rate_deg > 0.0f) cs->overspeed_min_rate_rpm_s = cs->overspeed_min_rate_deg * rpm_s_per_deg;
        }

        /* blade speed actually under way, measured from the encoder over the last 2 ticks
           (deg/s, + = COARSE). Uses the angle function, so encoder direction doesn't matter. */
        float blade_rate_now = 0.0f;
        if (cs->pos_hist_n >= 3)
            blade_rate_now = (cs->blade_deg - cs->pos_to_angle(cs->pos_hist[1])) / (2.0f * CS_TICK_S);
        if (IsNanf(blade_rate_now)) blade_rate_now = 0.0f;

        /* ---- our own blade movement ----
           The rpm follows the blade with a first-order lag (rpm_response_s) and
           settles at S rpm per degree. blade_f is the angle the rpm has caught up
           with; the rest of the effect of moves already made is still on its way. */
        float k = CS_TICK_S / cs->rpm_response_s;
        if (k > 1.0f) k = 1.0f;
        if (!cs->blade_f_valid) { cs->blade_f = cs->blade_f2 = cs->blade_deg; cs->blade_f_valid = true; }
        cs->blade_f += k * (cs->blade_deg - cs->blade_f);
        /* the same with os_response_s (take-off power), used for own_pending when own_slow_at_takeoff */
        float k2 = (cs->os_response_s > 0.05f) ? CS_TICK_S / cs->os_response_s : 1.0f;
        if (k2 > 1.0f) k2 = 1.0f;
        cs->blade_f2 += k2 * (cs->blade_deg - cs->blade_f2);

        cs->own_rate      = -S * (cs->blade_deg - cs->blade_f) / cs->rpm_response_s;
        cs->external_rate = r - cs->own_rate;          /* throttle, climb, dive ... */

        float own_pending = -S * (cs->blade_deg - cs->blade_f)     /* moves already made        */
                            - S * blade_rate_now * cs->lag_s        /* movement during the lag   */
                            + cs->own_rate * d;                     /* since the rpm was measured */
        /* Claude 13/11/2026 (hub profile LEARNED): at take-off (target within high_zone_rpm of max_rpm) and within
           os_response_zone_rpm below the target the rpm follows the blade in ~os_response_s, so more of the effect
           of moves already made is still to come - a slow COARSE move otherwise stops too late. Used only to shorten
           or prevent a COARSE band move (e_slow below, applied in the band logic as a function of the error alone, so
           starting and stopping use the same value: no stop-go). FINE decisions and the overspeed prediction keep
           own_pending (a bigger own effect there made the controller lazy near max_rpm: fix 2, reverted 12/11). */
        bool slow_active = cs->own_slow_at_takeoff && (cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm &&
                           cs->rpm_smoothed >= cs->target_rpm - cs->os_response_zone_rpm &&
                           cs->os_response_s > cs->rpm_response_s;
        float own_pending_slow = own_pending;
        if (slow_active) {
            float own_rate2 = -S * (cs->blade_deg - cs->blade_f2) / cs->os_response_s;
            own_pending_slow = -S * (cs->blade_deg - cs->blade_f2)
                               - S * blade_rate_now * cs->lag_s
                               + own_rate2 * d;
        }

        /* external rate: look ahead over the lag, plus a further look-ahead blended in
           smoothly (none below override_rate_rpm_s, full at twice that) */
        float w = Clampf((Absf(cs->external_rate) - cs->override_rate_rpm_s) / cs->override_rate_rpm_s,
                         0.0f, 1.0f);
        float e_ext = cs->external_rate * (d + cs->lag_s + w * cs->override_lookahead_s);
        float e_ctl = err + own_pending + e_ext;
        float e_slow = err + own_pending_slow + e_ext;
        cs->predicted_rpm = cs->target_rpm + e_ctl;

        /* ---- overspeed ----
           Trigger level: max_rpm - margin, but at least overspeed_min_gap_rpm above
           the target (take-off 5700 vs 5800 must not look like an overspeed).
           Predicted including the stopping distance. Predictive triggering needs a
           genuine EXTERNAL rise; reaching max_rpm always triggers. Latched until the
           prediction is clearly back below the level. */
        float os_level = cs->max_rpm - cs->overspeed_margin_rpm;
        if (os_level < cs->target_rpm + cs->overspeed_min_gap_rpm) os_level = cs->target_rpm + cs->overspeed_min_gap_rpm;
        if (os_level > cs->max_rpm) os_level = cs->max_rpm;
        /* Stopping distance for an external rise: to cancel a rate r the blade must
           move about r x rpm_response_s / S degrees; at the loaded COARSE speed
           that takes t_stop, during which the rpm climbs a further ~r x t_stop / 2.
           Close below the target the slower response at high power (os_response_s) is
           used: with rpm_response_s a steady ~600 rpm/s take-off rise only triggered at
           5589 and peaked at 5842 (bench run 975, 916 iS power map). Not further below:
           early in the run-up the rise eases by itself and the prediction would be far
           too early (run-up bursts 1000+ rpm below the target). And only once the blade
           has been still for os_quiet_s: soon after our own movement, part of the rise
           is still its effect (the own-movement model uses rpm_response_s too) and looks
           external - after a FINE recovery that set off a burst every 2.7 s at a steady
           5700 (bench run 977). */
        if (cs->motor_cmd_rpm != 0) cs->quiet_ticks = 0;
        else if (cs->quiet_ticks < 1000) cs->quiet_ticks++;
        float os_pred = cs->rpm_smoothed + own_pending + cs->external_rate * (d + cs->lag_s);
        if (cs->external_rate > 0.0f) {
            float coarse_deg_s = cs->motor_coarse_loaded_rpm / 60.0f * deg_per_motor_rev;
            float resp = cs->rpm_response_s;
            if (cs->os_response_s > resp && cs->rpm_smoothed >= cs->target_rpm - cs->os_response_zone_rpm &&
                cs->quiet_ticks * CS_TICK_S >= cs->os_quiet_s)
                resp = cs->os_response_s;
            float t_stop = (cs->external_rate * resp / S) / coarse_deg_s;
            os_pred += 0.5f * cs->external_rate * t_stop;
        }

        /* While the blade is still moving FINE, the first response is simply to stop
           it (the normal logic below does that): stopping alone changes the rpm
           gradient a lot. Only once it is stopped (or already moving COARSE) can a
           predicted overspeed call for full COARSE. Reaching max_rpm always does. */
        float prev_cmd = (float)cs->motor_cmd_rpm;
        bool not_moving_fine = (prev_cmd >= 0.0f);
        /* Near the target the overspeed rules put avoiding max_rpm first (full speed
           at take-off, no release while still rising). Well below the target, e.g.
           during the take-off run-up, a fast rise is just the engine accelerating:
           those rules then made a 3.4 deg run-up burst (bench run 965, 1 s throttle)
           and held the rpm ~500 below the target for 4 s. */
        bool near_target = (cs->rpm_smoothed >= cs->target_rpm - db);
        /* For releasing a burst while the rpm is still rising, "near" reaches further
           at take-off (target within high_zone_rpm of max): os_response_zone_rpm, where
           os_response_s already triggers early. A graded burst started at 5498 was
           released at 5649 with the rpm rising ~500 rpm/s; the 0.3 s relatch hold then
           delayed the full-speed burst to 5728 and the rpm peaked at 5808 (bench run 982,
           fast_then_boost). */
        bool near_release = near_target ||
            ((cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm &&
             cs->rpm_smoothed >= cs->target_rpm - cs->os_response_zone_rpm);

        /* ---- take-off run-up move (Claude 17/11/2026) ----
           Rotax 912 ULS test stand (runs 910 / 912): the run-up rose 2000-3000 rpm/s, the fast-rise rule started a
           full-speed burst at ~2800 rpm, and it only ended once the rpm fell - 3.6 / 4.5 deg COARSE where ~1.8 was
           needed - then 6-9 s of FINE steps back up to the band. Standing still at full throttle the blade angle for
           the take-off rpm is known (takeoff_angle_deg, from a static full-throttle check; hardly changes with
           density altitude). So with a take-off target: no COARSE while the rpm is below runup_frac of the target,
           then ONE full-speed move to takeoff_angle_deg + takeoff_margin_deg (the margin is on the safe, low-rpm
           side; airspeed then raises the rpm and the normal control takes over). Only when the rpm rises up through
           runup_frac from below with the blade finer than the goal - not a touch-and-go (rpm stays above it, blade
           already coarse). Starts earlier only if at the present rate the rpm would reach max_rpm before the move
           could be done. Reaching max_rpm always gives the overspeed response. */
        bool takeoff_target = (cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm;
        float runup_goal = cs->takeoff_angle_deg + cs->takeoff_margin_deg;
        float runup_rpm = cs->runup_frac * cs->target_rpm;
        if (!takeoff_target || cs->takeoff_angle_deg <= 0.0f) {
            cs->runup_armed = cs->runup_move = false;
            cs->runup_settle_ticks = 0;
        } else if (!cs->runup_move) {
            if (cs->blade_deg >= runup_goal - 0.2f) cs->runup_armed = false;   /* already coarse enough */
            else if (cs->rpm_smoothed < runup_rpm) cs->runup_armed = true;
            if (cs->runup_armed) {
                float full_deg_s = 0.85f * cs->motor_nominal_rpm / 60.0f * deg_per_motor_rev;
                float t_move = cs->lag_s + (runup_goal - cs->blade_deg) / full_deg_s;
                bool late = (r > 0.0f) && (cs->max_rpm - cs->rpm_smoothed < r * t_move);
                if (cs->rpm_smoothed >= runup_rpm || late) {
                    cs->runup_armed = false;
                    cs->runup_move = true;
                    cs->overspeed_latched = false;
                }
            }
        }
        if (cs->runup_move) {
            /* stop early by the distance the blade still covers after the stop command */
            float lead = (blade_rate_now > 0.0f) ? blade_rate_now * cs->runup_stop_lead_s : 0.0f;
            if (cs->blade_deg + lead >= runup_goal || (cs->stop_flags & CS_STOP_COARSE) ||
                cs->rpm_smoothed < cs->runup_frac * runup_rpm) {           /* throttle closed again: give up */
                cs->runup_move = false;
                cs->runup_settle_ticks = (int32_t)(cs->runup_settle_s / CS_TICK_S + 0.5f);
            }
        }
        if (cs->runup_settle_ticks > 0) cs->runup_settle_ticks--;
        /* No predictive COARSE while armed or moving, nor for runup_settle_s after the move while below the band:
           just after a fast move our own-movement model makes the remaining rise look external and large (it would
           predict an overspeed at once). In the band or above, the normal prediction applies again. */
        bool runup_block = cs->runup_armed || cs->runup_move ||
                           (cs->runup_settle_ticks > 0 && cs->rpm_smoothed < cs->target_rpm - db);

        if (cs->overspeed_latched) {
            /* release once the prediction (which includes the rpm drop still to come
               from the blade movement already made) is clearly below the level */
            if (cs->latch_ticks > 0) cs->latch_ticks--;
            /* Avoiding the overspeed comes first: the burst goes on until the rpm is
               actually FALLING. Once it is (and the blade has moved at least
               os_burst_max_deg) the burst ends at once - at take-off power the rpm
               follows the blade slowly (customer: ~0.7 s, not rpm_response_s), so the
               prediction would keep calling for more long after enough was done
               (bench run 957: 0.7 deg more after the peak). The rpm may then dip below
               the band; os_recover_s brings it back gently (see the band logic).
               For os_settle_s only a real, rising max_rpm starts another burst. */
            if (cs->os_burst_max_deg > 0.0f && cs->engine_rpm_rate < 0.0f &&
                cs->blade_deg - cs->os_burst_start_deg >= cs->os_burst_max_deg) {
                cs->overspeed_latched = false;
                cs->settle_ticks  = (int32_t)(cs->os_settle_s / CS_TICK_S + 0.5f);
                cs->recover_ticks = (int32_t)(cs->os_recover_s / CS_TICK_S + 0.5f);
            }
            /* release when the prediction is clear AND either the external rise has
               eased (below 2 x override_rate_rpm_s) or the blade has already moved
               far enough to bring the predicted rpm down to the target.
               Not while the rpm is still rising faster than overspeed_min_rate_rpm_s:
               the prediction assumes a quicker rpm response than take-off power gives,
               and avoiding the overspeed comes first. */
            if (cs->overspeed_latched &&
                cs->latch_ticks == 0 && cs->engine_rpm < cs->max_rpm &&
                (!near_release || cs->engine_rpm_rate < cs->overspeed_min_rate_rpm_s) &&
                os_pred < os_level - 0.5f * cs->overspeed_min_gap_rpm &&
                (cs->external_rate < 2.0f * cs->override_rate_rpm_s ||
                 os_pred <= cs->target_rpm)) {
                cs->overspeed_latched = false;
                /* no re-trigger straight after a release (unless genuinely urgent) */
                cs->relatch_ticks = (int32_t)(cs->overspeed_min_hold_s / CS_TICK_S + 0.5f);
                cs->recover_ticks = (int32_t)(cs->os_recover_s / CS_TICK_S + 0.5f);
            }
        } else if (cs->settle_ticks > 0 &&
                   (cs->engine_rpm < cs->max_rpm || cs->engine_rpm_rate < 0.0f)) {
            cs->settle_ticks--;          /* after a burst: wait for its effect
                                            (above max_rpm but falling also waits) */
        } else if (cs->relatch_ticks > 0 && cs->engine_rpm < cs->max_rpm && os_pred < cs->max_rpm) {
            cs->relatch_ticks--;
        } else if (cs->engine_rpm >= cs->max_rpm ||
                   (!runup_block && not_moving_fine && cs->external_rate > cs->overspeed_min_rate_rpm_s && os_pred >= os_level) ||
                   (!runup_block && not_moving_fine && cs->external_rate > cs->fast_rise_rpm_s && os_pred > cs->target_rpm)) {
            cs->runup_move = false;      /* max_rpm reached during the run-up move: the overspeed response takes over */
            cs->overspeed_latched = true;
            cs->latch_ticks = (int32_t)(cs->overspeed_min_hold_s / CS_TICK_S + 0.5f);
            cs->os_speed_f = -1.0f;      /* start the speed filter afresh */
            cs->os_speed_out = 0.0f;
            cs->os_speed_peak = 0.0f;
            cs->os_burst_start_deg = cs->blade_deg;
            cs->settle_ticks = 0;
        }

        if (cs->overspeed_latched) {
            /* COARSE at a speed graded by how far above the target the rpm is heading:
               blade degrees needed = predicted excess / S, times the proportional gain,
               raised toward above_target_gain as the prediction approaches max_rpm.
               Full speed when the prediction reaches max_rpm or the rpm is there.
               (The output stage stops first if we were moving FINE.) */
            cs->state = CS_OVERSPEED;
            float excess = os_pred - cs->target_rpm;
            if (excess < 0.0f) excess = 0.0f;
            float span = cs->max_rpm - cs->target_rpm;
            float g = 1.0f;
            if (span > 0.0f)
                g = 1.0f + (cs->above_target_gain - 1.0f) * Clampf(excess / span, 0.0f, 1.0f);
            float speed = cs->kp_motor_rpm_per_deg * (excess / S) * g;
            speed = Clampf(speed, cs->motor_min_rpm, cs->motor_nominal_rpm);
            /* average the graded speed so it doesn't follow every wobble in the
               prediction; full speed at once if really at / heading for max_rpm.
               Also always full speed when the target is close to max_rpm (take-off):
               with only ~100 rpm between them the graded speed is far too low
               (bench run 963: 22-31% from 5730 to 5795, then 5805) */
            if (cs->os_speed_f < 0.0f) cs->os_speed_f = speed;
            else cs->os_speed_f += cs->os_speed_filter * (speed - cs->os_speed_f);
            if (cs->engine_rpm >= cs->max_rpm || os_pred >= cs->max_rpm ||
                ((cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm && near_target))
                cs->os_speed_f = cs->motor_nominal_rpm;

            /* during a burst the speed may rise freely but only fall slowly
               (os_speed_fall_per_100ms of the burst's peak speed per 100 ms),
               so it doesn't dip mid-burst as our own movement starts to tell */
            if (cs->os_speed_f > cs->os_speed_peak) cs->os_speed_peak = cs->os_speed_f;
            /* ...but once the blade has moved enough (predicted rpm at or below the
               target) there is no reason to keep going: ease off at the normal rate */
            float fall = (os_pred > cs->target_rpm)
                       ? cs->os_speed_fall_per_100ms * cs->os_speed_peak * (CS_TICK_S / 0.1f)
                       : cs->motor_decel_rpm_per_tick;
            float out = cs->os_speed_out - fall;
            if (out < cs->os_speed_f) out = cs->os_speed_f;
            if (out < cs->motor_min_rpm) out = cs->motor_min_rpm;
            cs->os_speed_out = out;
            cmd = out;
        }
        else {
            /* ---- band ----
               Normally target +/- deadband. When the target is close to max_rpm
               (within high_zone_rpm) and nothing external is happening, the upper
               half of the band is removed: hold between target - deadband and target,
               which doubles the room before max_rpm. */
            float db_hi = db;
            if ((cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm &&
                Absf(cs->external_rate) < cs->override_rate_rpm_s)
                db_hi = 0.0f;
            float centre = 0.5f * (db_hi - db);          /* band centre relative to target */
            float half   = 0.5f * (db_hi + db);          /* half-width of the band         */
            float e_c    = e_ctl - centre;
            /* (hub profile LEARNED, take-off) on the COARSE side the smaller of the two predictions, never below 0 */
            if (slow_active && e_c > 0.0f) {
                float e_cs = e_slow - centre;
                if (e_cs < e_c) e_c = (e_cs > 0.0f) ? e_cs : 0.0f;
            }

            /* ---- move / stop decision with hysteresis (no stop-go-stop-go) ----
               Stopped: start only when outside the band.
               Moving:  keep going the same way until inside stop_band_frac of the
                        half-band on our side (or past the centre), then ONE zero. */
            float dir = 0.0f;
            if (prev_cmd == 0.0f) {
                /* Start only once the rpm has been outside the band for start_persist_s
                   (engines wander +/-20-30 rpm over a second or so - log 0745-0748 -
                   which otherwise causes a small move about every second). Start at
                   once if the error is large or an external factor is acting.
                   (Restored 10/11/2026: lost in the 29/10/2026 4:37 pm edit, so the
                   start_persist_s set in control.c had no effect.)
                   Not when the target is close to max_rpm (take-off): there the wait let
                   a slow rise reach the overspeed level, and the full-speed bursts and
                   FINE recoveries brought the ~3 s hunting back (bench run 970). */
                if (Absf(e_c) > half) {
                    if (cs->persist_ticks < 1000) cs->persist_ticks++;
                    bool urgent = (Absf(e_c) > 2.0f * half + cs->deadband_rpm) || (w > 0.0f) ||
                                  ((cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm);
                    if (urgent || cs->persist_ticks * CS_TICK_S >= cs->start_persist_s)
                        dir = (e_c > 0.0f) ? 1.0f : -1.0f;
                } else {
                    cs->persist_ticks = 0;
                }
            } else {
                float prev_dir = (prev_cmd > 0.0f) ? 1.0f : -1.0f;
                if (prev_dir * e_c > cs->stop_band_frac * half) dir = prev_dir;
            }

            /* Don't drive FINE while the rpm is already rising quickly for an external
               reason (throttle, airspeed): it is coming up by itself, and going fine
               only adds to it (this caused the second burst on take-off). */
            /* with hysteresis: set above override_rate_rpm_s, cleared only once the
               external rise has fallen below half of it (no stop-go on noise) */
            if (cs->external_rate > cs->override_rate_rpm_s) cs->rise_hold = true;
            else if (cs->external_rate < 0.5f * cs->override_rate_rpm_s) cs->rise_hold = false;
            if (dir < 0.0f && cs->rise_hold)
                dir = 0.0f;

            /* Don't START a fine movement if an external push is already bringing
               the rpm up to the band by itself within rise_reach_s (e.g. after a
               take-off burst the rpm is still climbing toward the target). A fine
               movement already under way is not interrupted (no stop-go). */
            {
                /* (uses the larger of the measured and external rate: on a real engine
                   the rise near full power can be slow and partly masked by our own
                   blade movement - log 0745 at 22.3 s went fine while still rising.
                   Restored 10/11/2026, lost in the 29/10/2026 4:37 pm edit.) */
                float rise = (r > cs->external_rate) ? r : cs->external_rate;
                /* (Claude 17/11/2026: well below the band only the external rise - our own pending effect is in
                   e_ctl already, and counting it again held each FINE step back: the 912 staircase, see below) */
                if (cs->rpm_smoothed < cs->target_rpm - 2.0f * db) rise = cs->external_rate;
                if (dir < 0.0f && prev_cmd == 0.0f && rise > 0.0f) {
                    float to_band = (cs->target_rpm - db) - cs->rpm_smoothed;
                    if (to_band <= rise * cs->rise_reach_s)
                        dir = 0.0f;
                }
            }

            /* Gentle recovery for os_recover_s after a burst: the rpm may have dipped
               below the band, and that is far better than a quick FINE recovery that
               overshoots into the next burst (customer take-offs, ~3 s cycle). So no
               FINE start while the rpm is already rising, and FINE at minimum speed.
               Also, in recovery and whenever the target is close to max_rpm (take-off),
               a FINE movement stops once the rpm rises faster than overspeed_min_rate_rpm_s:
               at take-off power the rpm follows the blade in ~0.7 s, not rpm_response_s,
               so the band logic kept going FINE to ~5600 with the rpm rising 300-400 rpm/s,
               it carried on past 5700 and the predicted overspeed burst fired - a 2.7 s
               cycle at a steady 5700 (bench runs 977, 979). Restarting FINE (outside the
               recovery) waits until the rise is below half of that (no stop-go). */
            bool recovering = (cs->recover_ticks > 0);
            if (recovering) cs->recover_ticks--;
            if (dir < 0.0f && (recovering || (cs->max_rpm - cs->target_rpm) <= cs->high_zone_rpm)) {
                float rise_lim = (prev_cmd < 0.0f) ? cs->overspeed_min_rate_rpm_s
                               : (recovering ? 0.0f : 0.5f * cs->overspeed_min_rate_rpm_s);
                /* Claude 17/11/2026: not well below the band (more than 2 x deadband). There this rule made a
                   staircase of short FINE moves and pauses - Rotax 912 stand, Hold 4800 -> Takeoff 5700 ~7 s
                   (runs 910 / 912), bench run 1141 8 s. Using only the external rate was not enough (run 1143):
                   as the move tapers off, the own-movement rate drops at once but the measured rate (0.25 s
                   least squares) lags, so a false external rise stopped the move 1 deg short. Well below the band
                   the move's own stop (e_ctl: own moves still to come + external look-ahead) and rise_hold (a real
                   external rise above override_rate_rpm_s) are enough. Close to the band the measured rate still
                   decides (runs 977 / 979: at take-off power the rpm follows the blade more slowly than the model). */
                bool far_below = cs->rpm_smoothed < cs->target_rpm - 2.0f * db;
                if (!far_below && cs->engine_rpm_rate > rise_lim)
                    dir = 0.0f;
            }

            if (dir == 0.0f) {
                cs->state = CS_HOLD;
                cmd = 0.0f;
            } else {
                /* speed = K x (degrees of blade needed to reach the target),
                   degrees needed = rpm distance / rpm per degree.
                   Above the target, the gain rises toward above_target_gain as the
                   rpm approaches max_rpm. */
                float deg_needed = Absf(e_ctl) / S;
                float gain = 1.0f;
                if (dir > 0.0f && cs->rpm_smoothed > cs->target_rpm && cs->max_rpm > cs->target_rpm)
                    gain = 1.0f + (cs->above_target_gain - 1.0f) *
                           Clampf((cs->rpm_smoothed - cs->target_rpm) / (cs->max_rpm - cs->target_rpm), 0.0f, 1.0f);
                float speed = cs->kp_motor_rpm_per_deg * deg_needed * gain;
                speed = Clampf(speed, cs->motor_min_rpm, cs->motor_nominal_rpm);
                if (dir < 0.0f && recovering) speed = cs->motor_min_rpm;
                cs->state = CS_CONTROL;
                cmd = dir * speed;
            }
        }

        /* the run-up move: full speed COARSE (overrides the band, which wants FINE below the target) */
        if (cs->runup_move) {
            cs->state = CS_CONTROL;
            cmd = cs->motor_nominal_rpm;
        }

        /* ---- fine / coarse stops reported by the hub ---- */
        if ((cmd < 0.0f && (cs->stop_flags & CS_STOP_FINE)) ||
            (cmd > 0.0f && (cs->stop_flags & CS_STOP_COARSE))) {
            cmd = 0.0f;
            if (cs->state == CS_CONTROL) cs->state = CS_AT_LIMIT;
        }

        /* ---- motor speed range ---- */
        cmd = Clampf(cmd, -cs->motor_nominal_rpm, cs->motor_nominal_rpm);
        if (cmd != 0.0f && Absf(cmd) < cs->motor_min_rpm)
            cmd = (cmd > 0.0f) ? cs->motor_min_rpm : -cs->motor_min_rpm;
    }

    /* ---- encoder history for measuring the blade speed ---- */
    cs->pos_hist[2] = cs->pos_hist[1];
    cs->pos_hist[1] = cs->pos_hist[0];
    cs->pos_hist[0] = cs->encoder_pos;
    if (cs->pos_hist_n < 3) cs->pos_hist_n++;

    /* ---- output stage ----
       Zero is "stop with brake" and is sent at once. A change of direction is
       never sent directly: the motor is stopped first and held stopped for
       reverse_pause_s (stopping alone changes the rpm gradient a lot); after
       that it may start the other way if still needed. Only an actual overspeed
       (rpm at max_rpm) skips the pause. Speed changes are ramped both ways
       (motor_slew_rpm_per_tick up, motor_decel_rpm_per_tick down); a stop
       (zero) is always immediate. */
    float prev = (float)cs->motor_cmd_rpm;
    if (prev != 0.0f) cs->last_dir = (prev > 0.0f) ? 1 : -1;
    if (cs->pause_ticks > 0) cs->pause_ticks--;

    if (cmd != 0.0f) {
        int dir_new = (cmd > 0.0f) ? 1 : -1;
        bool reversing = (prev != 0.0f && dir_new != cs->last_dir);
        bool in_pause  = (prev == 0.0f && cs->pause_ticks > 0 && dir_new != cs->last_dir);
        if (reversing) {
            cmd = 0.0f;
            cs->pause_ticks = (cs->engine_rpm >= cs->max_rpm) ? 0
                              : (int32_t)(cs->reverse_pause_s / CS_TICK_S + 0.5f);
        } else if (in_pause) {
            cmd = 0.0f;
        } else {
            float sign = (float)dir_new;
            float base = (prev == 0.0f) ? cs->motor_min_rpm : Absf(prev);
            float mag  = Absf(cmd);
            if (mag > base + cs->motor_slew_rpm_per_tick)              /* ramp up */
                mag = base + cs->motor_slew_rpm_per_tick;
            if (prev != 0.0f && mag < base - cs->motor_decel_rpm_per_tick) /* ease down */
                mag = base - cs->motor_decel_rpm_per_tick;
            cmd = sign * mag;
        }
    }

    cs->motor_cmd_rpm = RoundToInt(cmd);
    return cs->motor_cmd_rpm;
}
