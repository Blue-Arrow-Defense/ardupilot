/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for JSBSim
*/

#include "SIM_JSBSim.h"

#if HAL_SIM_JSBSIM_ENABLED

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>   // getenv/atof for the SITL_* launch overrides
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

#define DEBUG_JSBSIM 1

JSBSim::JSBSim(const char *frame_str) :
    Aircraft(frame_str),
    sock_control(false),
    sock_fgfdm(true),
    initialised(false),
    jsbsim_script(nullptr),
    jsbsim_fgout(nullptr),
    catapult_launch(false),
    air_start(false),
    catapult_armed_at_ms(0),
    created_templates(false),
    started_jsbsim(false),
    opened_control_socket(false),
    opened_fdm_socket(false),
    frame(FRAME_NORMAL)
{
    if (strstr(frame_str, "elevon")) {
        frame = FRAME_ELEVON;
    } else if (strstr(frame_str, "vtail")) {
        frame = FRAME_VTAIL;
    } else {
        frame = FRAME_NORMAL;
    }
    const char *model_name = strchr(frame_str, ':');
    if (model_name != nullptr) {
        jsbsim_model = model_name + 1;
    }
    // "-catapult" ground-launches from a rail; "-air" uses the same launch
    // mechanism but from an elevated start (see air_start). Both imply a
    // launch-on-arm sequence and share the "Geran-2" model directory.
    if (strstr(frame_str, "-catapult") || strstr(frame_str, "-air")) {
        catapult_launch = true;
        air_start = (strstr(frame_str, "-air") != nullptr);
        // strip the "-catapult"/"-air" suffix so the model name still
        // matches the aircraft directory on disk
        char *model_copy = strdup(jsbsim_model);
        char *suffix = strstr(model_copy, "-catapult");
        if (suffix == nullptr) {
            suffix = strstr(model_copy, "-air");
        }
        if (suffix != nullptr) {
            *suffix = '\0';
        }
        jsbsim_model = model_copy;
    }
    control_port = 5505 + instance*10;
    fdm_port = 5504 + instance*10;

    printf("JSBSim backend started: control_port=%u fdm_port=%u\n",
           control_port, fdm_port);
}

/*
  catapult force magnitude in pounds-force.

  The stroke is a fixed 0.4 s (see the Catapult Release event), so this is
  simply m*dV/dt for the airframe's own mass and its own desired release
  speed - it is NOT a universal constant. Overridden per airframe via
  SITL_CATAPULT_LBF (set by docker/sim/entrypoint.sh from the airframe
  registry in backend/app/models.py); the default is the Geran-2's value so
  existing setups are unchanged.
 */
float JSBSim::catapult_lbf(void) const
{
    const char *env = getenv("SITL_CATAPULT_LBF");
    if (env != nullptr) {
        const float v = atof(env);
        if (v > 0) {
            return v;
        }
    }
    return 4900.0f;
}

/*
  create template files
 */
bool JSBSim::create_templates(void)
{
    if (created_templates) {
        return true;
    }

    asprintf(&jsbsim_script, "%s/jsbsim_start_%u.xml", autotest_dir, instance);
    asprintf(&jsbsim_fgout,  "%s/jsbsim_fgout_%u.xml", autotest_dir, instance);

    printf("JSBSim_script: '%s'\n", jsbsim_script);
    printf("JSBSim_fgout: '%s'\n", jsbsim_fgout);

    FILE *f = fopen(jsbsim_script, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim script %s", jsbsim_script);
    }

    // A catapult launch starts on the ground exactly like the default
    // case (same Ground Trim, known to work) and fires the aircraft's
    // own external_reactions/catapult force (defined in Geran-2.xml)
    // once the vehicle is armed - i.e. once ArduPilot itself is
    // actually ready to fly it. This avoids guessing a trimmed
    // "already airborne" initial condition (tried and abandoned:
    // JSBSim's own Full Trim solver can't find a valid trim point for
    // this airframe either, and a hand-picked AoA either sank or
    // porpoised into the ground during testing).
    //
    // ArduPilot's generic "start engine" event below only sets
    // engine[0]/set-running=1 with no throttle command, so a
    // catapult_launch aircraft still needs its own event to force
    // full throttle before ArduPilot itself is armed and commanding
    // real throttle. (The engine was originally a piston model needing
    // a magneto/starter sequence too; that model never reached a
    // self-sustaining RPM even in the aircraft's own "validated"
    // reference script and was replaced with an electric model - see
    // Engines/MD550.xml and the "Engine start" event comment below.)
    // send_servos() below withholds ArduPilot's own (zero, pre-arm)
    // throttle command for catapult_launch so this startup throttle
    // isn't immediately overwritten - see the comment there.
    char catapult_events[8192] = "";
    if (catapult_launch) {
        snprintf(catapult_events, sizeof(catapult_events),
"    <property value=\"0\"> sim/armed </property>\n"
"\n"
"    <!-- Rigidly holds the aircraft's position AND attitude until\n"
"         launch, using JSBSim's own forces/hold-down (see\n"
"         FGFDMExec::SetHoldDown / FGAccelerations - checked every\n"
"         frame, not just at the moment it's set). This replaces an\n"
"         earlier attempt that only supported the aircraft's weight\n"
"         with a vertical force at the CG: that left rotation totally\n"
"         unconstrained, and since qbar (and so all aerodynamic\n"
"         stabilizing/damping moments) is ~0 at the near-zero airspeed\n"
"         of the whole 5-second engine warm-up, any small residual\n"
"         moment (propeller gyroscopics, thrust misalignment, even the\n"
"         autopilot's own pre-launch elevator commands acting through a\n"
"         still-airframe with no aerodynamic damping) integrated,\n"
"         completely unopposed, into a wildly wrong attitude by the\n"
"         time the catapult fired (confirmed directly via a high-rate\n"
"         JSBSim CSV log: pitch had already drifted to 60-70 degrees\n"
"         before the catapult ever fired, explaining a post-launch\n"
"         'pitch divergence' that no amount of catapult force/PID/\n"
"         elevator-sign tuning could fix, since the damage was already\n"
"         done before launch). hold-down freezes translational AND\n"
"         rotational velocity every frame, so nothing can accumulate. -->\n"
"    <event name=\"Rail support on\" persistent=\"false\">\n"
"      <condition> simulation/sim-time-sec ge 0.0 </condition>\n"
"      <set name=\"forces/hold-down\" value=\"1\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n"
"    <!-- Engine swapped from a piston model (magneto/starter/mixture,\n"
"         FGPiston) to an electric model (FGElectric, see\n"
"         Engines/MD550.xml) - it never reached self-sustaining RPM\n"
"         and died the instant starter assistance ended, confirmed to\n"
"         be a pre-existing engine-model calibration problem rather\n"
"         than anything specific to this integration. FGElectric has\n"
"         no magneto/starter/mixture state at all - it just outputs\n"
"         PowerWatts*throttle - so magneto_cmd/starter_cmd no longer\n"
"         exist as properties and setting them throws a fatal\n"
"         'No property .../magneto_cmd is defined' exception. Only\n"
"         throttle and set-running are needed now. -->\n"
"    <event name=\"Engine start\" persistent=\"false\">\n"
"      <condition> simulation/sim-time-sec ge 0.0 </condition>\n"
"      <set name=\"fcs/throttle-cmd-norm[0]\" value=\"1.0\"/>\n"
"      <set name=\"propulsion/engine[0]/set-running\" value=\"1\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n"
"    <!-- TRIED AND REVERTED: teleporting the aircraft from \"held down,\n"
"         at rest\" straight to \"just released, at flying speed\" by\n"
"         setting ic/u-fps then simulation/reset=1 (the trick bcoconni\n"
"         describes for a pitched catapult in\n"
"         https://github.com/JSBSim-Team/jsbsim/issues/203, using\n"
"         FGFDMExec::ResetToInitialConditions to re-apply ic/* via\n"
"         RunIC() mid-run instead of simulating the catapult stroke).\n"
"         Confirmed broken for ArduPilot's own JSBSim SITL backend\n"
"         specifically: FGFDMExec::RunIC() (src/FGFDMExec.cpp)\n"
"         unconditionally calls Models[eInput]->InitModel(), which\n"
"         re-binds the TCP console socket ArduPilot is already\n"
"         connected through - this is fine for a standalone JSBSim\n"
"         batch run (where reset means \"restart the whole run\") but\n"
"         fatal here: \"Could not bind to TCP input socket, error = 98\"\n"
"         (EADDRINUSE) followed immediately by \"Fatal: Failed to send\n"
"         on control socket: Broken pipe\", killing the sim outright.\n"
"         There is no mode flag to skip just the Input/Output re-init\n"
"         while still running RunIC() for the position/velocity state,\n"
"         and no other tied property exposes a live, mid-run velocity\n"
"         write (velocities/u-fps etc. are read-only outputs of\n"
"         FGPropagate - checked src/models/FGPropagate.cpp). Do not\n"
"         retry this approach against the vendored JSBSim without\n"
"         first patching JSBSim itself. Reverted to simulating the\n"
"         actual catapult stroke below instead. -->\n"
"    <event name=\"Catapult Fire\" persistent=\"false\">\n"
"      <!-- ArduPilot's own auto_takeoff_check() doesn't confirm\n"
"           \"launched\" (and so doesn't hand off to real TECS/attitude\n"
"           control) instantly on arm - it retries over ~0.2-1s+\n"
"           (repeated \"Bad launch AUTO\"), gated partly on an attitude\n"
"           bounds check. Delaying the physical release relative to arm\n"
"           does NOT help (confirmed directly with delays from 0 to 5\n"
"           seconds - identical divergence every time, always starting\n"
"           at release, never during the hold): the aircraft coasts on\n"
"           a fixed pre-launch elevator/aileron output until launch is\n"
"           confirmed, and if the attitude check keeps failing (which\n"
"           it does once any real disturbance starts diverging), launch\n"
"           never confirms and real control never engages before\n"
"           impact. The actual fix is FLIGHT_OPTIONS bit 2\n"
"           (DISABLE_TOFF_ATTITUDE_CHK, see Geran-2-catapult.parm) so\n"
"           launch confirms on the first timer expiry regardless of\n"
"           attitude, instead of this chicken-and-egg loop. -->\n"
"      <condition> sim/armed eq 1 </condition>\n"
"      <!-- Magnitude sized from the airframe's ACTUAL flying speed, not\n"
"           its theoretical stall. Wing loading is ~42 kg/m2 (200 kg,\n"
"           S=4.69 m2), so at a usable alpha the aircraft needs ~40 m/s\n"
"           to make lift = weight; the theoretical CLmax=1.0 stall\n"
"           (~27 m/s) is only reached at ~40 deg alpha and is not a\n"
"           flyable speed. The original 2810 lbf released the aircraft\n"
"           at only ~29 m/s: ground-truth CSV showed it rotate to\n"
"           alpha~15 deg yet still sink 3m to the ground within ~1.3s\n"
"           because lift (~940 N) was less than half its weight\n"
"           (1961 N). 4900 lbf over the 0.4s stroke imparts\n"
"           ~4900*4.4482*0.9659*0.4/200 ~= 42 m/s (minus stroke losses,\n"
"           ~40 m/s at release) so the wing is genuinely flying the\n"
"           instant it leaves the rail. -->\n"
"           The magnitude is PER-AIRFRAME (SITL_CATAPULT_LBF, default 4900\n"
"           for the Geran-2): it must scale with mass, and a second, much\n"
"           lighter airframe makes that obvious - 4900 lbf on the 34 kg\n"
"           Delta-38 would be a 65 g, 250 m/s launch. See\n"
"           aircraft/Delta-38/Delta-38.xml for that airframe's own sizing. -->\n"
"      <set name=\"external_reactions/catapult/magnitude\" value=\"%.1f\"/>\n"
"      <set name=\"forces/hold-down\" value=\"0\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n"
"    <event name=\"Catapult Release\" persistent=\"false\">\n"
"      <condition> sim/armed eq 1 </condition>\n"
"      <delay> 0.4 </delay>\n"
"      <set name=\"external_reactions/catapult/magnitude\" value=\"0\"/>\n"
"      <notify/>\n"
"    </event>\n", catapult_lbf());
    }

    // The generic "start engine" event below just forces set-running=1,
    // which for a catapult_launch aircraft's more detailed piston
    // engine model actually causes it to seize up once starter
    // assistance ends (confirmed directly against the model). Skip it
    // there - the "Engine warm-up" event above starts the engine
    // properly instead. Non-catapult aircraft are unaffected.
    char start_engine_event[256] = "";
    if (!catapult_launch) {
        snprintf(start_engine_event, sizeof(start_engine_event),
"    <event name=\"start engine\">\n"
"      <condition> simulation/sim-time-sec le 0.01 </condition>\n"
"      <set name=\"propulsion/engine[0]/set-running\" value=\"1\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n");
    }

    // (Removed) A high-rate ground-truth CSV of the launch dynamics
    // (/tmp/geran2_debug.csv, 120 Hz) used to root-cause the early
    // post-launch attitude error. That error is now handled by the DCM
    // launch-acceleration mitigation (see AP_AHRS_DCM::drift_correction
    // and AP_AHRS::_active_EKF_type), and AUTO catapult launches climb
    // out cleanly in testing, so the diagnostic is no longer needed. It
    // wrote an unbounded file for the whole flight, so it is disabled
    // here; to re-enable for debugging, populate debug_csv_output with a
    // JSBSim <output type="CSV"> block.
    char debug_csv_output[1] = "";

    fprintf(f,
"<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
"<?xml-stylesheet type=\"text/xsl\" href=\"http://jsbsim.sf.net/JSBSimScript.xsl\"?>\n"
"<runscript xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\"\n"
"    xsi:noNamespaceSchemaLocation=\"http://jsbsim.sf.net/JSBSimScript.xsd\"\n"
"    name=\"Testing %s\">\n"
"\n"
"  <description>\n"
"    test ArduPlane using %s and JSBSim\n"
"  </description>\n"
"\n"
"  <use aircraft=\"%s\" initialize=\"reset\"/>\n"
"\n"
"  <!-- we control the servos via the jsbsim console\n"
"       interface on TCP 5124 -->\n"
"  <input port=\"%u\"/>\n"
"\n"
"%s"
"  <run start=\"0\" end=\"10000000\" dt=\"%.6f\">\n"
"    <property value=\"0\"> simulation/notify-time-trigger </property>\n"
"\n"
"%s"
"    <event name=\"Trim\">\n"
"      <condition>simulation/sim-time-sec ge 0.01</condition>\n"
"      <set name=\"simulation/do_simple_trim\" value=\"%u\"/>\n"
"      <notify/>\n"
"    </event>\n"
"\n"
"%s"
"  </run>\n"
"\n"
"</runscript>\n"
"",
            jsbsim_model,
            jsbsim_model,
            jsbsim_model,
            control_port,
            debug_csv_output,
            1.0/rate_hz,
            start_engine_event,
            // tGround(2) assumes the aircraft is stationary on the
            // ground and will actively move it there to solve for
            // equilibrium - which for catapult_launch pulls the
            // elevated rail IC straight down onto the ground (gear
            // contacts fire at t=0.01s), soaking the whole warm-up and
            // catapult stroke in ground friction. tNone(6) just runs
            // the given IC forward with no trim solve, matching the
            // aircraft's own validated standalone catapult test.
            catapult_launch ? 6U : 2U,
            catapult_events);
    fclose(f);

    f = fopen(jsbsim_fgout, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim fgout script %s", jsbsim_fgout);
    }
    fprintf(f, "<?xml version=\"1.0\"?>\n"
            "<output name=\"127.0.0.1\" type=\"FLIGHTGEAR\" port=\"%u\" protocol=\"UDP\" rate=\"%f\">\n"
            "  <time type=\"simulation\" resolution=\"1e-6\"/>\n"
            "</output>",
            fdm_port, rate_hz);
    fclose(f);

    char *jsbsim_reset;
    asprintf(&jsbsim_reset, "%s/aircraft/%s/reset.xml", autotest_dir, jsbsim_model);

    printf("JSBSim_reset: '%s'\n", jsbsim_reset);

    f = fopen(jsbsim_reset, "w");
    if (f == nullptr) {
        AP_HAL::panic("Unable to create jsbsim reset script %s", jsbsim_reset);
    }
    float r, p, y;
    dcm.to_euler(&r, &p, &y);

    // Stationary on the ground, ready for a runway takeoff roll. A
    // catapult_launch aircraft instead sits on the belly rail at the
    // exact attitude/height validated in catapult_init.xml (5 deg
    // nose-up, ~3m/10ft) rather than the generic wheeled-takeoff pose
    // (13 deg) - the catapult force direction/sizing in Geran-2.xml
    // was tuned against that rail pose, and using the wrong starting
    // attitude measurably changes how much of the impulse becomes
    // forward speed vs. ground reaction (empirically ~25% slower).
    float ic_alt_m = 1.3;
    float ic_theta_deg = 13.0;
    // <running>-1</running> requests all engines start already running;
    // harmless no-op for the default case, kept for catapult_launch.
    const char *ic_running = "";
    if (catapult_launch) {
        // Launch height. An earlier attempt at 8.0m was reverted back
        // to 3.05m because - UNDER THE OLD ACCELEROMETER BUG - more
        // height just gave the (false-estimate-driven) roll excursion
        // more room to grow before it hit the ground anyway. That bug
        // is fixed now (synthetic specific-force override in recv_fdm),
        // so the attitude estimate is correct off the rail and that
        // objection no longer applies. The real remaining failure is a
        // pure lift deficit during the first ~0.5s: right off the rail
        // the aircraft is at ~0 alpha (the catapult adds forward speed
        // faster than the wing can rotate), so lift is briefly far
        // below weight and it sinks ~1-3m before rotation builds enough
        // alpha to fly. From 3.05m that sink reached the ground; a
        // modest height margin lets the wing "catch" before contact.
        // This is insurance for the transient only - the actual fix is
        // launching above the airframe's real flying speed (see the
        // catapult magnitude below and the airspeed envelope in
        // Geran-2-catapult.parm). Measured: with the ~40 m/s release the
        // transient sink is a stable, reproducible ~3.3m (min ground
        // clearance ~4.6m when launching from 8m, seen identically
        // across runs - it's deterministic, not jitter), after which the
        // wing catches and it climbs away cleanly to the mission
        // altitude. 15m launch height turns that ~4.6m squeak into a
        // comfortable ~11m clearance so run-to-run timing jitter can
        // never turn the transient into a ground strike. Represents an
        // elevated launch position (tower/ramp/hillside), which is
        // realistic for this class of catapult.
        ic_alt_m = 15.0;
        ic_theta_deg = 5.0;
        ic_running = "  <running> -1 </running>\n";
        // Air start: identical launch mechanics, but held at an elevated
        // altitude so the aircraft appears and flies at height rather than
        // ground launching. The transient post-release sink (~3 m) is
        // irrelevant with this much clearance. Height is tunable via
        // SITL_AIR_START_ALT (m AGL) so it can be adjusted without a rebuild.
        if (air_start) {
            const char *alt_env = getenv("SITL_AIR_START_ALT");
            ic_alt_m = alt_env ? atof(alt_env) : 150.0;
        }
    }

    fprintf(f,
            "<?xml version=\"1.0\"?>\n"
            "<initialize name=\"Start up location\">\n"
            "  <latitude unit=\"DEG\" type=\"geodetic\"> %f </latitude>\n"
            "  <longitude unit=\"DEG\"> %f </longitude>\n"
            "  <altitude unit=\"M\"> %f </altitude>\n"
            "  <vt unit=\"FT/SEC\"> 0.0 </vt>\n"
            "  <gamma unit=\"DEG\"> 0.0 </gamma>\n"
            "  <phi unit=\"DEG\"> 0.0 </phi>\n"
            "  <theta unit=\"DEG\"> %f </theta>\n"
            "  <psi unit=\"DEG\"> %f </psi>\n"
            "%s"
            "</initialize>\n",
            home.lat*1.0e-7,
            home.lng*1.0e-7,
            ic_alt_m,
            ic_theta_deg,
            degrees(y),
            ic_running);
    fclose(f);

    created_templates = true;
    return true;
}


/*
  start JSBSim child
 */
bool JSBSim::start_JSBSim(void)
{
    if (started_jsbsim) {
        return true;
    }
    if (!open_fdm_socket()) {
        return false;
    }

    int p[2];
    int devnull = open("/dev/null", O_RDWR|O_CLOEXEC);
    if (pipe(p) != 0) {
        AP_HAL::panic("Unable to create pipe");
    }
    pid_t child_pid = fork();
    if (child_pid == 0) {
        // in child
        setsid();
        dup2(devnull, 0);
        dup2(p[1], 1);
        close(p[0]);
        for (uint8_t i=3; i<100; i++) {
            close(i);
        }
        char *logdirective;
        char *script;
        char *nice;
        char *rate;

        asprintf(&logdirective, "--logdirectivefile=%s", jsbsim_fgout);
        asprintf(&script, "--script=%s", jsbsim_script);
        asprintf(&nice, "--nice=%.8f", 10*1e-9);
        asprintf(&rate, "--simulation-rate=%f", rate_hz);

        if (chdir(autotest_dir) != 0) {
            perror(autotest_dir);
            exit(1);
        }

        int ret = execlp("JSBSim",
                         "JSBSim",
                         "--suspend",
                         rate,
                         nice,
                         logdirective,
                         script,
                         nullptr);
        if (ret != 0) {
            perror("JSBSim");
        }
        exit(1);
    }
    close(p[1]);
    jsbsim_stdout = p[0];

    // read startup to be sure it is running
    char c;
    if (read(jsbsim_stdout, &c, 1) != 1) {
        AP_HAL::panic("Unable to start JSBSim");
    }

    if (!expect("JSBSim Execution beginning")) {
        AP_HAL::panic("Failed to start JSBSim");
    }
    if (!open_control_socket()) {
        AP_HAL::panic("Failed to open JSBSim control socket");
    }

    fcntl(jsbsim_stdout, F_SETFL, fcntl(jsbsim_stdout, F_GETFL, 0) | O_NONBLOCK);

    started_jsbsim = true;
    check_stdout();
    close(devnull);
    return true;
}

/*
  check for stdout from JSBSim
 */
void JSBSim::check_stdout(void) const
{
    char line[100];
    ssize_t ret = ::read(jsbsim_stdout, line, sizeof(line));
    if (ret > 0) {
#if DEBUG_JSBSIM
        write(1, line, ret);
#endif
    }
}

/*
  a simple function to wait for a string on jsbsim_stdout
 */
bool JSBSim::expect(const char *str) const
{
    const char *basestr = str;
    while (*str) {
        char c;
        if (read(jsbsim_stdout, &c, 1) != 1) {
            return false;
        }
        if (c == *str) {
            str++;
        } else {
            str = basestr;
        }
#if DEBUG_JSBSIM
        write(1, &c, 1);
#endif
    }
    return true;
}

/*
  open control socket to JSBSim
 */
bool JSBSim::open_control_socket(void)
{
    if (opened_control_socket) {
        return true;
    }
    if (!sock_control.connect("127.0.0.1", control_port)) {
        return false;
    }
    printf("Opened JSBSim control socket\n");
    sock_control.set_blocking(false);
    opened_control_socket = true;

    char startup[] =
        "info\n"
        "resume\n"
        "iterate 1\n"
        "set atmosphere/turb-type 4\n";
    sock_control.send(startup, strlen(startup));
    return true;
}

/*
  open fdm socket from JSBSim
 */
bool JSBSim::open_fdm_socket(void)
{
    if (opened_fdm_socket) {
        return true;
    }
    if (!sock_fgfdm.bind("127.0.0.1", fdm_port)) {
        check_stdout();
        return false;
    }
    sock_fgfdm.set_blocking(false);
    sock_fgfdm.reuseaddress();
    opened_fdm_socket = true;
    return true;
}


/*
  decode and send servos
*/
void JSBSim::send_servos(const struct sitl_input &input)
{
    char *buf = nullptr;
    float aileron  = filtered_servo_angle(input, 0);
    float elevator = filtered_servo_angle(input, 1);
    float throttle = filtered_servo_range(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    if (frame == FRAME_ELEVON) {
        // fake an elevon plane
        float ch1 = aileron;
        float ch2 = elevator;
        aileron  = (ch2-ch1)/2.0f;
        // the minus does away with the need for RC2_REVERSED=-1
        elevator = -(ch2+ch1)/2.0f;
    } else if (frame == FRAME_VTAIL) {
        // fake a vtail plane
        float ch1 = elevator;
        float ch2 = rudder;
        // this matches VTAIL_OUTPUT==2
        elevator = (ch2-ch1)/2.0f;
        rudder   = (ch2+ch1)/2.0f;
    }
    float wind_speed_fps = input.wind.speed / FEET_TO_METERS;
    // exposes ArduPilot's real arm state to the JSBSim script so a
    // catapult_launch aircraft can fire its catapult once the vehicle
    // is actually armed and ready, rather than at a guessed IC/time
    bool armed = hal.util->get_soft_armed();
    char armed_property[32] = "";
    if (catapult_launch) {
        snprintf(armed_property, sizeof(armed_property),
                 "set sim/armed %d\n", armed ? 1 : 0);
    }
    // A catapult_launch aircraft needs its engine startup sequence's
    // throttle=1.0 (see the run script) to stick, both before arming
    // (so the engine reaches a running state pre-launch) and through
    // the catapult stroke - handing control to ArduPilot's own throttle
    // output right at the moment of arming risks ArduPilot's own
    // throttle-suppression safety logic (it holds commanded throttle
    // near zero until its own launch-detection confirms a launch,
    // which can take a second or more of retries) re-zeroing the engine
    // mid-air right after the catapult releases it. Rather than a fixed
    // timer (which either cuts off too early if detection is slow, or
    // needlessly overrides once ArduPilot is already commanding real
    // throttle), keep forcing full throttle until ArduPilot is itself
    // commanding (nearly) full throttle, then hand control back
    // immediately. A generous hard cap guards against never handing
    // back control if something else is wrong.
    //
    // The threshold is 95%, not "anything above zero". A catapult launch
    // is committed at full power on the rail, and ArduPilot's own throttle
    // does not step straight to 100 on arming - measured in SITL, TECS
    // ramps it up over roughly half a second while it acquires its speed
    // and height errors. Handing back at the first non-zero value therefore
    // fed the engine ~50% throttle in the middle of the catapult stroke.
    // An <electric_engine> shrugs that off (power is PowerWatts*throttle,
    // instantaneous), but a real FGPiston loses rpm and manifold pressure
    // and needs a second or more to spin the propeller back up - so the
    // aircraft left the rail slow and with almost no thrust, decelerated
    // and mushed into the ground. Waiting for ArduPilot to ask for full
    // throttle costs nothing on a takeoff (it wants THR_MAX anyway) and
    // keeps the engine at its rated power right through the launch.
    char throttle_cmd[48];
    bool force_full_throttle = false;
    if (catapult_launch) {
        if (armed && catapult_armed_at_ms == 0) {
            catapult_armed_at_ms = AP_HAL::millis();
        }
        const bool within_hard_cap = catapult_armed_at_ms != 0 &&
            AP_HAL::millis() - catapult_armed_at_ms < 10000;
        force_full_throttle = !armed ||
            (within_hard_cap && throttle < 0.95f);
    }
    if (force_full_throttle) {
        throttle_cmd[0] = '\0';
    } else {
        snprintf(throttle_cmd, sizeof(throttle_cmd),
                 "set fcs/throttle-cmd-norm %f\n", throttle);
    }
    asprintf(&buf,
             "set fcs/aileron-cmd-norm %f\n"
             "set fcs/elevator-cmd-norm %f\n"
             "set fcs/rudder-cmd-norm %f\n"
             "%s"
             "set atmosphere/psiw-rad %f\n"
             "set atmosphere/wind-mag-fps %f\n"
             "set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps %f\n"
             "set atmosphere/turbulence/milspec/severity %f\n"
             "%s"
             "iterate 1\n",
             aileron, elevator, rudder,
             throttle_cmd,
             radians(input.wind.direction),
             wind_speed_fps,
             wind_speed_fps/3,
             input.wind.turbulence,
             armed_property);
    ssize_t buflen = strlen(buf);
    ssize_t sent = sock_control.send(buf, buflen);
    free(buf);
    if (sent < 0) {
        if (errno != EAGAIN) {
            fprintf(stderr, "Fatal: Failed to send on control socket: %s\n",
                    strerror(errno));
            exit(1);
        }
    }
    if (sent < buflen) {
        fprintf(stderr, "Failed to send all bytes on control socket\n");
    }
}

/* nasty hack ....
   JSBSim sends in little-endian
 */
void FGNetFDM::ByteSwap(void)
{
    uint32_t *buf = (uint32_t *)this;
    for (uint16_t i=0; i<sizeof(*this)/4; i++) {
        buf[i] = ntohl(buf[i]);
    }
    // fixup the 3 doubles
    buf = (uint32_t *)&longitude;
    uint32_t tmp;
    for (uint8_t i=0; i<3; i++) {
        tmp = buf[0];
        buf[0] = buf[1];
        buf[1] = tmp;
        buf += 2;
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void JSBSim::recv_fdm(const struct sitl_input &input)
{
    FGNetFDM fdm;
    check_stdout();

    do {
        while (sock_fgfdm.recv(&fdm, sizeof(fdm), 100) != sizeof(fdm)) {
            send_servos(input);
            check_stdout();
        }
        fdm.ByteSwap();
    } while (fdm.cur_time == time_now_us);

    accel_body = Vector3f(fdm.A_X_pilot, fdm.A_Y_pilot, fdm.A_Z_pilot) * FEET_TO_METERS;

    if (catapult_launch && !hal.util->get_soft_armed()) {
        // JSBSim's forces/hold-down (used to rigidly hold a
        // catapult_launch aircraft in place before launch - see the
        // "Rail support on" event above) zeroes the FDM's kinematic
        // acceleration outputs entirely (FGAccelerations::SetHoldDown
        // in the vendored JSBSim source sets vUVWdot/vPQRdot to zero
        // directly), rather than properly computing specific force
        // (what a real accelerometer measures: total acceleration
        // minus gravity - for a genuinely stationary, supported
        // aircraft that's a nonzero ~1g reading along whatever
        // direction gravity projects onto the body axes). Confirmed
        // directly via MAVLink RAW_IMU: a normal aircraft sitting on
        // its own gear correctly reads this (e.g. Rascal shows
        // zacc=-968 at rest), while catapult_launch read exactly zero
        // on all three axes for the entire ~20s pre-launch hold -
        // starving DCM/EKF3 of any tilt reference before the aircraft
        // ever left the rail. That, not anything about the catapult
        // stroke itself, was the actual root cause of every attitude
        // problem chased this session. fdm.phi/fdm.theta themselves
        // are unaffected by the hold-down acceleration bug (confirmed
        // via ground-truth CSV staying exactly at the rail attitude
        // throughout), so synthesize the specific force a real
        // accelerometer would read for that known, fixed attitude -
        // standard aerospace convention, specific force = -(gravity
        // vector) resolved into the body frame via the current
        // roll/pitch (yaw doesn't affect this projection).
        accel_body.x = GRAVITY_MSS * sinf(fdm.theta);
        accel_body.y = -GRAVITY_MSS * sinf(fdm.phi) * cosf(fdm.theta);
        accel_body.z = -GRAVITY_MSS * cosf(fdm.phi) * cosf(fdm.theta);
    }

    double p, q, r;
    SIM::convert_body_frame(degrees(fdm.phi), degrees(fdm.theta),
                             degrees(fdm.phidot), degrees(fdm.thetadot), degrees(fdm.psidot),
                             &p, &q, &r);
    gyro = Vector3f(p, q, r);

    velocity_ef = Vector3f(fdm.v_north, fdm.v_east, fdm.v_down) * FEET_TO_METERS;
    location.lat = RAD_TO_DEG_DOUBLE * fdm.latitude * 1.0e7;
    location.lng = RAD_TO_DEG_DOUBLE * fdm.longitude * 1.0e7;
    location.alt = fdm.agl*100 + home.alt;
    dcm.from_euler(fdm.phi, fdm.theta, fdm.psi);
    airspeed = fdm.vcas * KNOTS_TO_METERS_PER_SECOND;
    airspeed_pitot = airspeed;

    // update magnetic field
    update_mag_field_bf();
    
    rpm[0] = fdm.rpm[0];
    rpm[1] = fdm.rpm[1];
    
    time_now_us = fdm.cur_time;
}

void JSBSim::drain_control_socket()
{
    const uint16_t buflen = 1024;
    char buf[buflen];
    ssize_t received;
    do {
        received = sock_control.recv(buf, buflen, 0);
    } while (received > 0);
}
/*
  update the JSBSim simulation by one time step
 */
void JSBSim::update(const struct sitl_input &input)
{
    while (!initialised) {
        if (!create_templates() ||
            !start_JSBSim() ||
            !open_control_socket() ||
            !open_fdm_socket()) {
            time_now_us = 1;
            return;
        }
        initialised = true;
    }
    send_servos(input);
    recv_fdm(input);
    adjust_frame_time(rate_hz);
    sync_frame_time();
    drain_control_socket();
}

} // namespace SITL

#endif  // HAL_SIM_JSBSIM_ENABLED
