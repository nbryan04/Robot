#pragma once
#include <Arduino.h>
#include "drive_train.h"
#include "ultrasonic.h"
#include "camera.h"
#include "claw.h"
#include "metal_detector.h"
#include "tilt_sensor.h"
#include "LineFollower.h"
#include "ir_sensor.h"

// Non-blocking mission state machine implementing the COLLECT phase of the
// robot FSM: dead-reckon between rock clusters, find/centre/scan/grab each
// rock, sweep for teletubbies, and climb the ramp to the UPPER deck, up to the
// point where phase is set to PANEL.
//
// Navigation model: PER-MOVE RELATIVE dead reckoning. Every hop just issues a
// fresh turn()+driveStraight() from wherever the robot currently is, so error
// resets each cluster and "re-zeroing odometry" is implicit (no global pose).
//
// Call update() every loop(). It expects drivetrain.update() and
// ultrasonic.update() to also be called every loop() by the caller.
// One leg of a dead-reckon hop: turn in place, then drive straight.
// angleDeg: + = clockwise / right, - = counter-clockwise / left.
struct HopLeg {
    float angleDeg;
    float distMM;
};

// All constants for the solar-panel removal "dance". There are two competition
// surfaces that differ ONLY in this sequence, so each surface gets its own set
// (selected at boot by the surface-select pin). Angles: + = right/CW, - = left/CCW.
struct PanelRemoveParams {
    float predriveMM;   // short straight (realign target = beacon trigger + this) before turn1
    float turn1Deg;     // first turn off the beacon spot
    float driveMM;      // straight leg (through the arch) after turn1
    float turn2Deg;     // second turn to finish the sequence
    float turnSpeed;    // speed for the two turns
    float driveSpeed;   // speed for the straight legs
    int   armAngle;     // arm hover angle over the panel (hand stays closed)
    unsigned long armSettleMs;  // time for the arm to reach that angle
};

class Mission {
public:
    static constexpr int MAX_HOP_LEGS = 3;  // max legs any one cluster can use

    enum Phase { COLLECT, PANEL, DONE };
    enum Level { LOWER, UPPER };

    // Where to aim once a rock passes validation.
    enum AimMode {
        AIM_CENTROID,       // average of all on-rock sweep samples (robust to lumps)
        AIM_MIN_DISTANCE,   // angle of the single closest reading
        AIM_EDGE_MIDPOINT   // midpoint of the two detected edges (original)
    };

    enum State {
        ROUTER,           // n18: dispatch on phase
        // --- collection ---
        HOP_TO_CLUSTER,   // n1:  dead-reckon to next cluster (or divert to ramp)
        FIND_ROCK,        // n2:  wide ultrasonic sweep
        FIND_ROCK_RETRY,  // rock-2 failsafe: nudge +SWEEP_RETRY_TURN_DEG, then re-sweep
        FAIL_TELETUBBY_SCAN, // rock-5/6 failsafe: return to arrival pose, scan teletubby, advance
        TRAVEL_TO_ROCK,   // n3:  drive up to grab distance
        CENTRE_ROCK,      // n4:  face the rock, capture metal reference
        TELETUBBY_SWEEP,  // n5:  camera sweep for coloured blobs
        POINT_TELETUBBY,  // n9:  pivot to point at the blob
        RECENTRE_ROCK,    // n24: pivot back onto the rock
        NEED_METAL,       // n22: still need the metal rock?
        LOWER_CLAW,       // n6:  lower claw / detector
        SCAN_METAL,       // n7:  read detector
        ENGAGE_CLAW,      // n8:  close claw
        STORE_ROCK,       // n10: store in basket
        RAISE_CLAW,       // n16: raise claw (decoy)
        ADVANCE_CLUSTER,  // n12: rocks_visited += 1
        ALL_DONE,         // n23: all rocks done? -> maybe phase = PANEL
        SWEEP_TEST_RECENTER, // bench test: return to neutral heading, re-sweep
        // --- panel phase (after collection): crest -> find/follow line to panel -
        CREST,            // n26: on the upper deck; hand off to the panel line
        FIND_LINE,        // rotate (negative/CCW) until the LF sees the tape
        FOLLOW_LINE,      // follow the tape until the IR panel beacon reads high
        PANEL_REMOVE,     // removal dance: lower -> realign+predrive -> turn1 -> drive -> turn2
        // --- ramp climb via line following (COLLECT, after rock 4) ---
        RAMP_FIND_LINE,   // rotate until the LF sees the tape at the ramp foot
        RAMP_FOLLOW_LINE, // follow the tape up the ramp until the crest, then hop to rock 5
        // --- ramp (old dead-reckoned approach; unused now) ---
        RAMP_APPROACH,    // n27: dead-reckon to ramp foot, watch tilt
        RAMP_CLIMB,       // n25: climb until flat (crest)
        RAMP_RECOVERY,    // n30: re-acquire / last-resort assume crest
        // --- terminal (out of scope for this pass) ---
        HOLD              // phase == PANEL/DONE: stop and hold
    };

    Mission(Drivetrain& dt, Ultrasonic& us, Camera& cam, Claw& cl,
            MetalDetector& md, TiltSensor& ts, LineFollower& lf, IR_Sensor& irs);

    void begin();
    void update();

    // ---- Manual / bench-test controls ----
    // Force the FSM into any state (resets its sub-step + timer). Note: states
    // that consume per-rock values (rockBearing, metalReference, ...) assume an
    // earlier state set them, so jumping mid-flow may act on stale values.
    void jumpTo(State s);

    // Bypass the scanning-heavy states so you can exercise just the
    // hop -> find -> travel -> centre navigation spine.
    bool enableRockSearch = true;      // MASTER: false skips find/travel/centre
                                       //        (n2-n4) at EVERY rock; with metal
                                       //        scan on, hop straight into the claw
                                       //        sequence at each rock
    // Per-rock sweep gate, ANDed with enableRockSearch. Indexed by rocks_visited
    // (0-based): [0]=rock1 [1]=rock2 [2]=rock3 [3]=rock4 [4]=rock5 [5]=rock6. A
    // rock left false skips the sweep/travel/centre and goes straight to the scan
    // at the dead-reckoned arrival pose. Default: sweep every rock.
    bool sweepOnRock[6] = { false, false, false, false, true, true };
    bool enableTeletubbySweep = true;  // false: skip the camera sweep (n5)
    bool enableMetalScan = true;       // false: skip lower/scan/grab (n6-n10,n16)

    // Competition-surface select: 0 = surface 1, 1 = surface 2. The two surfaces
    // differ ONLY in the solar-panel removal dance (see panelParams). main.cpp sets
    // this at boot from the surface-select pin.
    int panelSurface = 0;

    // Panel line-acquire spin direction, per rock the robot heads to the panel
    // FROM -- either an EARLY EXIT (both objectives met before rock 6) or rock 6's
    // normal completion. The robot faces the panel line differently at each rock,
    // so it spins CW (true) or CCW (false) to catch it. Indexed 0-based:
    // [0]=rock1 [1]=rock2 [2]=rock3 [3]=rock4 [4]=rock5 [5]=rock6. Rock 1 is unused
    // (can't have both objectives that early).
    bool crestSpinCWByRock[6] = { false, true, false, false, true, false };
    //                            rock1  r2    r3     r4     r5    r6

    // After rock 4, climb the ramp by FOLLOWING THE LINE (not dead reckoning),
    // detect the crest, then start the dead-reckoned hop to rock 5 (HOP_LEGS[4]).
    // false: hop straight over the ramp as before. Requires the IR pins wired
    // (the LF channels ride the IR sensor's DMA scan).
    bool enableRampLineFollow = true;

    // TEMP TEST: stop and hold at the ramp crest instead of starting the hop to
    // rock 5, so the crest detection can be checked in isolation. Set false to
    // resume the normal flow (crest -> HOP_LEGS[4]).
    bool pauseAtCrest = false;

    // TEST hook for the metal decision:
    //   0  = use the real detector
    //   N  = force "metal" only when scanning rock N (1-based)
    //  -1  = force every rock to read as a decoy (no metal)
    // Any non-zero value also skips the real detector's hardware calls.
    int testMetalOnRock = 0;

    // Aim point once a rock is validated (see AimMode). Centroid is most robust
    // for rough rocks; edge-midpoint is the original; min-distance the nearest bump.
    AimMode aimMode = AIM_CENTROID;

    // TEST: after centring, drive back to the position the robot arrived at from
    // the hop (undo the sweep/travel/centre excursion), then hop as normal. Lets
    // you exercise sweep centring without it disturbing the dead-reckoned path.
    // Assumes a single sweep pass (SWEEP_PASSES == 1).
    bool returnAfterCentre = false;

    // TEST MODE (at-rock procedure): simulate the robot just arriving at a rock.
    // Combine with jumpTo(FIND_ROCK) at startup to skip the hop and run the whole
    // per-rock procedure once -- sweep -> travel -> centre -> teletubby -> metal
    // -> claw -- then realign to the arrival pose (honouring returnAfterCentre)
    // and HOLD instead of counting the rock and hopping onward. Lets you observe
    // the at-rock behaviour in isolation, on the bench, over and over.
    bool stopAfterRock = false;

    // BENCH TEST (stationary sweep): jump straight to FIND_ROCK at startup and,
    // instead of travelling to the rock, print the sweep result over Serial then
    // recenter and sweep again -- forever. Lets you watch the edge detection in
    // place. Serial prints every edge (accepted or ignored) with its delta angle.
    bool sweepTestMode = false;

    // Mission variables (Init / Variables block of the FSM).
    Phase phase = COLLECT;         // COLLECT / PANEL / DONE
    Level level = LOWER;           // which deck we are on
    int rock = 0;                  // metal rock collected (max 1)
    int teletubbies = 0;           // confident teletubbies found (max 2)
    int rocks_visited = 0;         // clusters checked so far (max 6)

    State state = ROUTER;

    // ---- Edge-event debug (updated live during the FIND_ROCK sweep) ----
    // Every start/end edge the sweep detects (whether or not the accept logic
    // keeps it) is latched here for debug/inspection.
    // edgeEventSeq bumps on each edge so a caller can tell when a new one fired.
    Ultrasonic::EdgeEvent lastEdgeEvent = Ultrasonic::NONE;
    float         lastEdgeDeltaDeg = 0.0f;  // deg from the start of the sweep arc
    unsigned long edgeEventSeq = 0;         // increments on each detected edge

    // ---- Sweep result summary (published when a FIND_ROCK sweep finishes) ----
    // Angles are degrees INTO the sweep arc (0 at the far-left start, up to
    // SWEEP_ARC at the far-right end). Distance is the closest reading seen.
    // sweepResultSeq bumps once per completed sweep so the caller can redraw.
    float sweepStartAngle = 0.0f;   // first start edge
    float sweepEndAngle   = 0.0f;   // last end edge
    float sweepDistanceCm = 0.0f;   // closest reading during the sweep
    bool  sweepFound = false;       // did the sweep validate a rock
    unsigned long sweepResultSeq = 0;

private:
    void enter(State s);           // transition helper: resets sub-step + timer
    bool driveIdle();              // drivetrain finished its current move
    float sweepHeadingDeg(float arcDeg); // heading during an active sweep turn

    // Global interrupts (checked every loop; placeholders for now).
    bool checkStall();
    bool checkTimeLow();

    // Driver references.
    Drivetrain& drive;
    Ultrasonic& ultra;
    Camera& camera;
    Claw& claw;
    MetalDetector& metal;
    TiltSensor& tilt;
    LineFollower& line;
    IR_Sensor& ir;

    // Per-state bookkeeping.
    int subStep = 0;
    int hopLeg = 0;                // which leg of the current hop we are on
    int centreAttempts = 0;        // distance-correction passes in CENTRE_ROCK
    int sweepPass = 0;             // sweep+centre passes done at this rock
    int approachAttempts = 0;      // sweep->travel tries at this rock (capped so a
                                   // rock we can't close on doesn't loop forever)
    bool rockSweepRetried = false; // rock-2 failsafe: a second sweep (after a +deg
                                   // nudge) has already been tried at this rock
    float clusterHeading = 0.0f;   // net rotation (deg) added by the sweep/approach
                                   // since the hop finished; undone before the next hop
    float excursionOriginMM = 0.0f;// both-wheel odometer reading captured at the
                                   // arrival pose; the baseline the excursion is
                                   // measured from
    float excursionForward = 0.0f; // net forward distance (mm) driven since arrival
                                   // (travel + centre), read from the odometer at
                                   // ADVANCE and reversed to realign
    unsigned long stateTimer = 0;

    // Ramp line-follow bookkeeping.
    float rampClimbOriginMM = 0.0f; // odometer baseline captured when the climb starts
    bool  rampWasTilted = false;    // tilt sensor latched onto the incline during the climb
    // Panel FOLLOW_LINE ramp-PWM crest (an early exit runs the panel line up the
    // ramp): drop from RAMP_BASE_PWM to LINE_BASE_PWM once we crest by tilt OR by
    // climbing RAMP_CLIMB_MAX_MM past the ramp foot (distance fallback if tilt never
    // clears). Latched so we never climb back onto ramp PWM after cresting.
    bool  panelRampWasTilted = false;  // tilt latched onto the incline during the panel follow
    bool  panelRampCrested   = false;  // crest reached -> stay on the slower line PWM
    float panelRampOriginMM  = 0.0f;   // odometer at ramp entry; climb distance measured from here
    float panelRampTime      = 0.0f;   // ms when the ramp was first detected (for RAMP_MIN_TIME)
    bool  earlyExitClimb = false;      // this run early-exited on the LOWER deck (rock <=4),
                                       // so the panel line climbs the ramp -> crest creep applies
    unsigned long panelCrestCreepUntil = 0; // creep at CREST_CREEP_PWM until this ms (0 = none)
    float crestForwardOriginMM = 0.0f; // odometer at the crest; the momentum coast is measured from here
    bool  absorbCrestMomentum = false; // hop-5's first drive rides the ramp momentum, then trims to distance
    bool  crestSpinCW = false;         // active panel line-acquire spin direction (latched per rock)

    // Panel beacon-align bookkeeping.
    float irTriggerMM = 0.0f;       // odometer position where the beacon first crossed threshold
    bool  irTriggerValid = false;   // have we latched a trigger position this FOLLOW_LINE
    bool  irWasAbove = false;       // was the beacon above threshold on the previous window
    unsigned long irConsiderAfterMs = 0; // ignore IR beacon reads until this ms (early-exit cooldown)

    // Per-rock working values.
    float rockBearing = 0.0f;      // deg from post-hop forward
    float startEdgeAngle = 0.0f;
    float endEdgeAngle = 0.0f;
    bool foundStartEdge = false;
    bool foundEndEdge = false;
    float minSweepDistance = 0.0f; // closest valid reading seen during the sweep
    float minSweepAngle = 0.0f;    // heading at that closest reading
    int   metalSampleCount = 0;    // confirmation samples taken in SCAN_METAL
    bool  metalAllAbove = false;   // have all samples so far cleared the threshold
    float sumOnRockAngle = 0.0f;   // running sum of on-rock sample angles (centroid)
    long  onRockCount = 0;         // number of on-rock samples in that sum
    float teletubbyBearing = 0.0f; // deg from rock-forward

    // ---------------- Temporary tuning values ----------------
    // Hardcoded placeholders; each measured on the real field later.
    //
    // Hop path to each cluster, indexed by rocks_visited (0 = hop to rock 1,
    // 1 = hop to rock 2, 2 = hop to rock 3, ...). Each hop is a list of legs;
    // every leg turns (deg: + = right/CW, - = left/CCW) then drives (mm).
    // Measured from the PREVIOUS rock so error resets every cluster.
    // HOP_LEG_COUNT says how many legs of each row are actually used.
    int HOP_LEG_COUNT[6] = {2, 3, 1, 2, 2, 1};  // rock 3 (index 2) uses 2 legs
    HopLeg HOP_LEGS[6][MAX_HOP_LEGS] = {
        { {0,583},{20.5, 175} },                 // -> rock 1
        { {-45, 275},{45, 400},{-65,10} },                 // -> rock 2
        { {39.5, 340}, },    // -> rock 3: two legs (turn right, then left)
        { {-35, 190}, {-30, 285} },                 // -> rock 4
        { {0, 220} },                 // -> (ramp){-50, 350} , {-54.5, 1500}rock 5 (upper deck, after ramp)
        { {90, 120} },                 // -> rock 6 (upper deck)
    };
    float HOP_TURN_SPEED  = 0.15;   // speed for the in-place turn portion of a hop leg
    float HOP_DRIVE_SPEED = 0.20;   // speed for the drive-straight portion of a hop leg

    float SWEEP_ARC        = 65.0f;  // deg, wide arc to cover drift
    float SWEEP_SPEED      = 0.15f;
    int   SWEEP_PASSES     = 1;      // sweep+centre passes per rock (2 = one refine pass)
    float SWEEP_RETRY_TURN_DEG = 20.0f;  // rock-2 failsafe: nudge this far (+ = CW) then
                                         // re-sweep once if the first sweep finds nothing
                                         // (tracked in clusterHeading so realign undoes it)
    float MIN_ROCK_ANGLE   = 3.0f;    // deg between start/end edges to count as a rock
    float GRAB_DISTANCE_CM = 15.0f;   // target ultrasonic distance at the rock
    float CENTRE_MARGIN_CM = 3.0f;    // acceptable +/- error from the target
    float CENTRE_SPEED     = 0.12f;   // slow speed for distance corrections
    unsigned long CENTRE_SETTLE_MS = 250;  // let the filter settle before measuring
    int CENTRE_MAX_TRIES = 7;         // give up correcting after this many passes

    float TRAVEL_MAX_MM = 500.0f;     // give-up distance driving toward a rock
    float TRAVEL_SPEED  = 0.15f;
    int   APPROACH_MAX_TRIES = 3;     // sweep->travel attempts before abandoning
                                      // the rock (prevents an endless re-approach)

    // Metal scan: the baseline is tared at hover (clear of the rear metal) after
    // letting the frequency filter settle there, then we wait for it to settle
    // again at the lowered position before reading the shift.
    unsigned long HOVER_SETTLE_MS = 800;   // filter settle at hover before tare
    unsigned long SCAN_SETTLE_MS  = 500;   // wait after lowering before sampling
    // Confirmation: after settling, require this many readings, spaced apart,
    // to ALL clear the threshold before we grab (rejects the lowering spike).
    int METAL_SAMPLE_COUNT = 3;
    unsigned long METAL_SAMPLE_SPACING_MS = 200;

    unsigned long CAMERA_PRESCAN_DELAY_MS = 500; // settle before triggering the camera
    unsigned long POINT_DWELL_MS = 300; // pause while pointing at a teletubby

    // Panel phase: rotate to find the tape, follow it, then stop on the IR beacon.
    // PWM duty is raw (0..MAX_DUTY = 1023); motors need ~350+ to move.
    int LINE_SEEK_PWM = 600;   // in-place rotation speed while hunting for the tape
    int LINE_BASE_PWM = 600;   // forward speed while following the tape at the PANEL
    int RAMP_BASE_PWM = 800;   // forward speed while following the tape up the RAMP (needs more to climb)
    // Right after cresting on an EARLY-EXIT run that climbed the ramp (both
    // objectives met at/before rock 4), creep at this reduced PWM for CREST_CREEP_MS
    // to settle onto the flat top before resuming LINE_BASE_PWM.
    int CREST_CREEP_PWM = 350;              // gentle PWM just after the crest
    unsigned long CREST_CREEP_MS = 2000;    // how long to hold CREST_CREEP_PWM
    float LINE_RIGHT_SCALE = 1.07f;  // right motor is weaker: scale its PWM up to match
    // The stop condition (IR beacon) uses IR_Sensor::detected(), which applies the
    // per-tone threshold THRESHOLD_1K / THRESHOLD_10K in ir_sensor.h (selected by
    // IR_SELECT_PIN). Tune those two from the [IR] strength printed over Serial.
    // On detection the robot reverses back to where the amplitude FIRST crossed the
    // threshold (it drifts past during the confirm windows + coast), so the final
    // stop is repeatable. Skip the reverse if the overshoot is under this (mm).
    float IR_ALIGN_DEADBAND_MM = 5.0f;

    // On an early-exit-before-ramp run the panel line climbs the ramp before reaching
    // the beacon, so ignore the IR sensor for this long after the search starts (at
    // CREST) to avoid a false trigger during the climb. Only applied on those runs.
    unsigned long IR_COOLDOWN_MS = 5000;

    // Panel removal "dance" (runs once the beacon trips in FOLLOW_LINE): lower the
    // arm (hand stays closed), realign to the beacon trigger + a short pre-drive,
    // then turn1 -> straight -> turn2 to sweep the panel off. The TWO competition
    // surfaces differ ONLY here, so each has its own parameter set; the active one
    // is chosen at boot by panelSurface (0 = surface 1, 1 = surface 2), which
    // main.cpp sets from the surface-select pin. Fields: see PanelRemoveParams.
    //                                  predrive turn1  drive  turn2  tSpd  dSpd  arm settle
    PanelRemoveParams panelParams[2] = {
        /* surface 1 */ {  45.0f, 85.0f, 140.0f, -90.0f, 0.22f, 0.15f, 45, 700 },
        /* surface 2 */ {  50.0f, 85.0f, 120.0f, -90.0f, 0.22f, 0.15f, 45, 700 },
    };

    // The active surface's removal parameters (panelSurface is public config below).
    PanelRemoveParams& panel() { return panelParams[panelSurface]; }

    // Ramp climb via line following (after rock 4). The crest is detected by the
    // tilt sensor (latched onto the incline, then back to flat). Until the IMU is
    // pinned in (tilt.isPresent()==false) that can't fire, so this odometry cap on
    // the forward climb distance is the fallback crest trigger -- and a safety
    // cap even once the IMU works. Tune to just past the ramp length.
    float RAMP_CLIMB_MAX_MM = 1500.0f;

    // Ramp (old dead-reckoned approach; unused now).
    float RAMP_APPROACH_MM = 1000.0f;
    float RAMP_CLIMB_MM    = 1000.0f;
    float RAMP_SPEED       = 0.20f;
    unsigned long RAMP_SEARCH_MAX_MS = 4000;  // no tilt by here -> recovery
    unsigned long CLIMB_MAX_MS       = 6000;  // no crest by here -> recovery
    unsigned long RAMP_MIN_CLIMB_MS  = 800;   // ignore "flat" right after entry

    float RAMP_MIN_TIME = 1000.0f; // ms, ignore tilt events until this long after the climb starts
    float rampTime = 0.0f;
};
