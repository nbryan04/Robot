#pragma once
#include <Arduino.h>
#include "drive_train.h"
#include "ultrasonic.h"
#include "camera.h"
#include "claw.h"
#include "metal_detector.h"
#include "tilt_sensor.h"
#include "LineFollower.h"

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

class Mission {
public:
    static constexpr int MAX_HOP_LEGS = 3;  // max legs any one cluster can use

    enum Phase { COLLECT, PANEL, DONE };
    enum Level { LOWER, UPPER };

    enum State {
        ROUTER,           // n18: dispatch on phase
        // --- collection ---
        HOP_TO_CLUSTER,   // n1:  dead-reckon to next cluster (or divert to ramp)
        FIND_ROCK,        // n2:  wide ultrasonic sweep
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
        // --- ramp ---
        RAMP_APPROACH,    // n27: dead-reckon to ramp foot, watch tilt
        RAMP_CLIMB,       // n25: climb until flat (crest)
        RAMP_RECOVERY,    // n30: re-acquire / last-resort assume crest
        CREST,            // n26: level = UPPER, snap rocks_visited to 4
        // --- terminal (out of scope for this pass) ---
        HOLD              // phase == PANEL/DONE: stop and hold
    };

    Mission(Drivetrain& dt, Ultrasonic& us, Camera& cam, Claw& cl,
            MetalDetector& md, TiltSensor& ts, LineFollower& lf);

    void begin();
    void update();

    // ---- Manual / bench-test controls ----
    // Force the FSM into any state (resets its sub-step + timer). Note: states
    // that consume per-rock values (rockBearing, metalReference, ...) assume an
    // earlier state set them, so jumping mid-flow may act on stale values.
    void jumpTo(State s);

    // Bypass the scanning-heavy states so you can exercise just the
    // hop -> find -> travel -> centre navigation spine.
    bool enableRockSearch = true;      // false: skip find/travel/centre (n2-n4);
                                       //        with metal scan on, hop straight
                                       //        into the claw sequence at each rock
    bool enableTeletubbySweep = true;  // false: skip the camera sweep (n5)
    bool enableMetalScan = true;       // false: skip lower/scan/grab (n6-n10,n16)

    // TEST hook: metal detector isn't real yet. 0 = use the real detector.
    // N (1-based) = force "metal detected" only when scanning rock N.
    int testMetalOnRock = 0;

    // Mission variables (Init / Variables block of the FSM).
    Phase phase = COLLECT;         // COLLECT / PANEL / DONE
    Level level = LOWER;           // which deck we are on
    int rock = 0;                  // metal rock collected (max 1)
    int teletubbies = 0;           // confident teletubbies found (max 2)
    int rocks_visited = 0;         // clusters checked so far (max 6)

    State state = ROUTER;

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

    // Per-state bookkeeping.
    int subStep = 0;
    int hopLeg = 0;                // which leg of the current hop we are on
    unsigned long stateTimer = 0;

    // Per-rock working values.
    float rockBearing = 0.0f;      // deg from post-hop forward
    float startEdgeAngle = 0.0f;
    float endEdgeAngle = 0.0f;
    bool foundStartEdge = false;
    bool foundEndEdge = false;
    float teletubbyBearing = 0.0f; // deg from rock-forward
    float metalReference = 0.0f;

    // ---------------- Temporary tuning values ----------------
    // Hardcoded placeholders; each measured on the real field later.
    //
    // Hop path to each cluster, indexed by rocks_visited (0 = hop to rock 1,
    // 1 = hop to rock 2, 2 = hop to rock 3, ...). Each hop is a list of legs;
    // every leg turns (deg: + = right/CW, - = left/CCW) then drives (mm).
    // Measured from the PREVIOUS rock so error resets every cluster.
    // HOP_LEG_COUNT says how many legs of each row are actually used.
    int HOP_LEG_COUNT[6] = {2, 1, 2, 2, 1, 1};  // rock 3 (index 2) uses 2 legs
    HopLeg HOP_LEGS[6][MAX_HOP_LEGS] = {
        { {0,260},{21, 185} },                 // -> rock 1
        { {-32.7, 570} },                 // -> rock 2
        { {42, 250}, {-30, 202} },    // -> rock 3: two legs (turn right, then left)
        { {-35, 256}, {-30, 240} },                 // -> rock 4
        { {0, 0} },                 // -> rock 5 (upper deck, after ramp)
        { {0, 0} },                 // -> rock 6 (upper deck)
    };
    float HOP_SPEED = 0.150f;

    float SWEEP_ARC        = 120.0f;  // deg, wide arc to cover drift
    float SWEEP_SPEED      = 0.15f;
    float GRAB_DISTANCE_CM = 6.0f;    // ultrasonic reading = at grab distance
    float CENTRE_MARGIN_CM = 3.0f;    // slack when verifying we are centred

    float TRAVEL_MAX_MM = 500.0f;     // give-up distance driving toward a rock
    float TRAVEL_SPEED  = 0.15f;

    float METAL_DELTA = 500.0f;        // |reading - reference| over this = metal

    unsigned long POINT_DWELL_MS = 600; // pause while pointing at a teletubby

    // Ramp.
    float RAMP_APPROACH_MM = 1000.0f;
    float RAMP_CLIMB_MM    = 1000.0f;
    float RAMP_SPEED       = 0.20f;
    unsigned long RAMP_SEARCH_MAX_MS = 4000;  // no tilt by here -> recovery
    unsigned long CLIMB_MAX_MS       = 6000;  // no crest by here -> recovery
    unsigned long RAMP_MIN_CLIMB_MS  = 800;   // ignore "flat" right after entry
};
