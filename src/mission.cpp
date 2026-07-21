#include "mission.h"

Mission::Mission(Drivetrain& dt, Ultrasonic& us, Camera& cam, Claw& cl,
                 MetalDetector& md, TiltSensor& ts, LineFollower& lf)
    : drive(dt), ultra(us), camera(cam), claw(cl),
      metal(md), tilt(ts), line(lf) {}

void Mission::begin() {
    enter(ROUTER);
}

void Mission::jumpTo(State s) {
    enter(s);
}

// -------- helpers ----------------------------------------------------------

void Mission::enter(State s) {
    state = s;
    subStep = 0;
    stateTimer = millis();
}

bool Mission::driveIdle() {
    return drive.state == Drivetrain::Idle;
}

// Heading (deg, relative to the sweep-start forward) during an active +arc turn
// that runs from -arc/2 up to +arc/2.
float Mission::sweepHeadingDeg(float arcDeg) {
    return -arcDeg / 2.0f + arcDeg * drive.moveProgress();
}

// Global interrupts, checked every loop (placeholders for now).
bool Mission::checkStall() {
    // TODO: detect a stall (commanded to move but encoders not changing) and
    // recover, then resume. Raise the threshold while climbing the ramp.
    return false;
}

bool Mission::checkTimeLow() {
    // TODO: when the run clock is low, abandon remaining clusters and head for
    // the panel (via the ramp if still on the LOWER deck).
    return false;
}

// -------- main update ------------------------------------------------------

void Mission::update() {
    // Global interrupts are checked every loop, not drawn as edges.
    if (checkStall())   { /* TODO: recover then resume */ }
    if (checkTimeLow()) { /* TODO: abandon clusters, head for panel */ }

    switch (state) {

    // ---- n18: Phase Router ------------------------------------------------
    case ROUTER:
        if (phase == COLLECT) {
            enter(HOP_TO_CLUSTER);
        } else {
            // PANEL / DONE are out of scope for this pass.
            enter(HOLD);
        }
        break;

    // ---- n1: Dead-reckon to next cluster ----------------------------------
    case HOP_TO_CLUSTER: {
        int idx = rocks_visited;
        if (idx > 5) idx = 5;  // defensive clamp

        if (subStep == 0) {
            // Counter says the ramp is next: divert before hopping.
            if (level == LOWER && rocks_visited == 4) {
                enter(RAMP_APPROACH);
                break;
            }
            // Start the first leg of the hop path.
            hopLeg = 0;
            drive.turn(HOP_LEGS[idx][hopLeg].angleDeg, HOP_SPEED);
            subStep = 1;
        } else if (subStep == 1) {
            // Turn finished: drive this leg's straight segment.
            if (driveIdle()) {
                drive.driveStraight(HOP_LEGS[idx][hopLeg].distMM, HOP_SPEED);
                subStep = 2;
            }
        } else {  // subStep == 2
            if (driveIdle()) {
                hopLeg++;
                if (hopLeg < HOP_LEG_COUNT[idx]) {
                    // More legs to go: turn into the next one.
                    drive.turn(HOP_LEGS[idx][hopLeg].angleDeg, HOP_SPEED);
                    subStep = 1;
                } else {
                    // Whole path done. Tilt cross-check: ramp may show up here.
                    if (level == LOWER && tilt.isOnRamp()) {
                        enter(RAMP_APPROACH);
                    } else if (enableRockSearch) {
                        enter(FIND_ROCK);
                    } else if (enableMetalScan) {
                        // No search, but still run the claw sequence at the rock.
                        enter(NEED_METAL);
                    } else {
                        enter(ADVANCE_CLUSTER);  // nav-only: skip the search
                    }
                }
            }
        }
        break;
    }

    // ---- n2: Find Rock (wide ultrasonic sweep) ----------------------------
    case FIND_ROCK:
        if (subStep == 0) {
            // Rotate to the start of the arc.
            drive.turn(-SWEEP_ARC / 2.0f, SWEEP_SPEED);
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                // Sweep across the full arc while polling the ultrasonic.
                ultra.scanState = Ultrasonic::WAITING_FOR_OBJECT;
                foundStartEdge = false;
                foundEndEdge = false;
                drive.turn(SWEEP_ARC, SWEEP_SPEED);
                subStep = 2;
            }
        } else {  // subStep == 2: sweeping
            Ultrasonic::EdgeEvent e = ultra.checkEdgeEvents();
            float h = sweepHeadingDeg(SWEEP_ARC);
            if (e == Ultrasonic::START_EDGE) {
                foundStartEdge = true;
                startEdgeAngle = h;
            } else if (e == Ultrasonic::END_EDGE && foundStartEdge) {
                foundEndEdge = true;
                endEdgeAngle = h;
            }

            if (driveIdle()) {
                // Sweep finished (robot now at +SWEEP_ARC/2).
                if (foundStartEdge) {
                    if (foundEndEdge) {
                        rockBearing = (startEdgeAngle + endEdgeAngle) / 2.0f;
                    } else {
                        // Object ran to the edge of the arc; use its midpoint.
                        rockBearing = (startEdgeAngle + SWEEP_ARC / 2.0f) / 2.0f;
                    }
                    enter(TRAVEL_TO_ROCK);
                } else {
                    // Rock not found: cluster missed.
                    enter(ADVANCE_CLUSTER);
                }
            }
        }
        break;

    // ---- n3: Travel to Rock -----------------------------------------------
    case TRAVEL_TO_ROCK:
        if (subStep == 0) {
            // Face the rock: from +SWEEP_ARC/2 to rockBearing.
            drive.turn(rockBearing - SWEEP_ARC / 2.0f, SWEEP_SPEED);
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                drive.driveStraight(TRAVEL_MAX_MM, TRAVEL_SPEED);
                subStep = 2;
            }
        } else {  // subStep == 2: closing in
            float d = ultra.filteredDistanceCm;
            bool valid = (d > 0 && d < Ultrasonic::MAX_VALID_DISTANCE);
            if (valid && d <= GRAB_DISTANCE_CM) {
                drive.stop();
                enter(CENTRE_ROCK);
            } else if (driveIdle()) {
                // Reached give-up distance without arriving: rock lost.
                enter(FIND_ROCK);
            }
        }
        break;

    // ---- n4: Centre on Rock (re-zero odometry) ----------------------------
    case CENTRE_ROCK: {
        // Per-move relative odometry: facing the rock IS the fresh origin, so
        // there is nothing global to reset here.
        // TODO: add a fine ±nudge sweep to minimise distance if needed.
        float d = ultra.filteredDistanceCm;
        bool inRange = (d > 0 && d < GRAB_DISTANCE_CM + CENTRE_MARGIN_CM);
        if (inRange) {
            // Claw is upright here: capture the metal reference for this rock.
            metalReference = metal.getReferenceFrequency();
            enter(TELETUBBY_SWEEP);
        } else {
            // Centering fails.
            enter(ADVANCE_CLUSTER);
        }
        break;
    }

    // ---- n5: Teletubby sweep ----------------------------------------------
    case TELETUBBY_SWEEP:
        if (subStep == 0) {
            if (!enableTeletubbySweep) {  // bench-test bypass
                enter(NEED_METAL);
                break;
            }
            drive.turn(-SWEEP_ARC / 2.0f, SWEEP_SPEED);
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                drive.turn(SWEEP_ARC, SWEEP_SPEED);
                subStep = 2;
            }
        } else if (subStep == 2) {  // sweeping, camera scanning
            if (teletubbies < 2 && camera.checkForTeletubby()) {
                // Confident blob: stop here (already pointing at it).
                drive.stop();
                teletubbyBearing = sweepHeadingDeg(SWEEP_ARC);
                enter(POINT_TELETUBBY);
            } else if (driveIdle()) {
                // Sweep finished, no new teletubby: return to rock-forward.
                drive.turn(-SWEEP_ARC / 2.0f, SWEEP_SPEED);
                subStep = 3;
            }
        } else {  // subStep == 3: returning to forward
            if (driveIdle()) {
                enter(NEED_METAL);
            }
        }
        break;

    // ---- n9: Point to teletubby -------------------------------------------
    case POINT_TELETUBBY:
        // We are already pointing at the blob (stopped mid-sweep).
        if (subStep == 0) {
            teletubbies += 1;
            stateTimer = millis();
            subStep = 1;
        } else {
            if (millis() - stateTimer >= POINT_DWELL_MS) {
                enter(RECENTRE_ROCK);
            }
        }
        break;

    // ---- n24: Re-centre on Rock -------------------------------------------
    case RECENTRE_ROCK:
        if (subStep == 0) {
            // Pivot back from the blob to rock-forward.
            drive.turn(-teletubbyBearing, SWEEP_SPEED);
            subStep = 1;
        } else {
            if (driveIdle()) {
                float d = ultra.filteredDistanceCm;
                bool reacquired = (d > 0 && d < GRAB_DISTANCE_CM + CENTRE_MARGIN_CM);
                if (reacquired) {
                    enter(NEED_METAL);
                } else {
                    enter(ADVANCE_CLUSTER);  // re-centre fails
                }
            }
        }
        break;

    // ---- n22: Still need the metal rock? ----------------------------------
    case NEED_METAL:
        if (rock == 1 || !enableMetalScan) {
            enter(ADVANCE_CLUSTER);  // already have it (or scan bypassed)
        } else {
            enter(LOWER_CLAW);
        }
        break;

    // ---- n6: Lower claw ---------------------------------------------------
    case LOWER_CLAW:
        if (subStep == 0) {
            claw.lowerForScan();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            enter(SCAN_METAL);
        }
        break;

    // ---- n7: Scan for metal -----------------------------------------------
    case SCAN_METAL: {
        bool metalDetected;
        if (testMetalOnRock > 0) {
            // TEST override: fake metal only on the chosen rock (1-based).
            metalDetected = (rocks_visited + 1 == testMetalOnRock);
        } else {
            float reading = metal.getReading();
            metalDetected = fabs(reading - metalReference) > METAL_DELTA;
        }

        if (metalDetected) {
            enter(ENGAGE_CLAW);   // metal detected
        } else {
            enter(RAISE_CLAW);    // no metal (decoy)
        }
        break;
    }

    // ---- n8: Engage Claw --------------------------------------------------
    case ENGAGE_CLAW:
        if (subStep == 0) {
            claw.closeHand();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            enter(STORE_ROCK);
        }
        break;

    // ---- n10: Store Rock in Basket ----------------------------------------
    case STORE_ROCK:
        if (subStep == 0) {
            claw.storeToBasket();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            rock = 1;
            enter(ADVANCE_CLUSTER);
        }
        break;

    // ---- n16: Raise Claw --------------------------------------------------
    case RAISE_CLAW:
        if (subStep == 0) {
            claw.raiseToRest();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            enter(ADVANCE_CLUSTER);
        }
        break;

    // ---- n12: Advance to next cluster -------------------------------------
    case ADVANCE_CLUSTER:
        rocks_visited += 1;
        enter(ALL_DONE);
        break;

    // ---- n23: All rocks done? ---------------------------------------------
    case ALL_DONE:
        if (rocks_visited >= 6) {
            phase = PANEL;  // n11: collection complete, head for the panel
        }
        enter(ROUTER);
        break;

    // ---- n27: Approach ramp -----------------------------------------------
    case RAMP_APPROACH:
        if (subStep == 0) {
            drive.driveStraight(RAMP_APPROACH_MM, RAMP_SPEED);
            stateTimer = millis();
            subStep = 1;
        } else {
            line.update();  // keep the line on the ramp run (stub for now)
            if (tilt.isOnRamp()) {
                drive.stop();
                enter(RAMP_CLIMB);
            } else if (millis() - stateTimer > RAMP_SEARCH_MAX_MS || driveIdle()) {
                drive.stop();
                enter(RAMP_RECOVERY);
            }
        }
        break;

    // ---- n25: Climb ramp --------------------------------------------------
    case RAMP_CLIMB:
        if (subStep == 0) {
            drive.driveStraight(RAMP_CLIMB_MM, RAMP_SPEED);
            stateTimer = millis();
            subStep = 1;
        } else {
            line.update();  // follow line straight up (stub for now)
            bool settled = (millis() - stateTimer > RAMP_MIN_CLIMB_MS);
            if (settled && !tilt.isOnRamp()) {
                drive.stop();
                enter(CREST);  // tilt back to flat: crest reached
            } else if (millis() - stateTimer > CLIMB_MAX_MS || driveIdle()) {
                drive.stop();
                enter(RAMP_RECOVERY);
            }
        }
        break;

    // ---- n30: Ramp recovery -----------------------------------------------
    case RAMP_RECOVERY:
        // TODO: re-acquire the line and retry the approach.
        // Last resort per the FSM: assume the crest and continue on odometry.
        enter(CREST);
        break;

    // ---- n26: Crest: re-localize ------------------------------------------
    case CREST:
        level = UPPER;
        if (rocks_visited < 4) rocks_visited = 4;  // snap to 4
        // Per-move relative odometry: distance origin resets on the next move.
        // NOTE: crest fixes DISTANCE origin, not HEADING.
        enter(ROUTER);
        break;

    // ---- terminal: PANEL / DONE out of scope this pass --------------------
    case HOLD:
        drive.stop();
        break;
    }
}
