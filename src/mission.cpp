#include "mission.h"

Mission::Mission(Drivetrain& dt, Ultrasonic& us, Camera& cam, Claw& cl,
                 MetalDetector& md, TiltSensor& ts, LineFollower& lf, IR_Sensor& irs)
    : drive(dt), ultra(us), camera(cam), claw(cl),
      metal(md), tilt(ts), line(lf), ir(irs) {}

void Mission::begin() {
    // Baseline the forward odometer at the spawn pose. A normal run re-baselines
    // at each cluster arrival (HOP_TO_CLUSTER); the at-rock test skips the hop, so
    // this spawn baseline is what its realignment measures against.
    excursionOriginMM = drive.forwardOdometryMM();
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
    if (s == CENTRE_ROCK) centreAttempts = 0;  // fresh distance-correction loop
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
        } else if (phase == PANEL) {
            enter(CREST);   // panel phase: crest -> find line -> follow to panel
        } else {
            enter(HOLD);
        }
        break;

    // ---- n1: Dead-reckon to next cluster ----------------------------------
    case HOP_TO_CLUSTER: {
        int idx = rocks_visited;
        if (idx > 5) idx = 5;  // defensive clamp

        if (subStep == 0) {
            // Before the hop to rock 5 (rocks_visited == 4), climb the ramp by
            // following the line. Once we crest, RAMP_FOLLOW_LINE sets level=UPPER
            // and returns here; the level==LOWER guard is then false so we fall
            // through to the normal dead-reckoned hop (HOP_LEGS[4]) from the top.
            if (idx == 4 && level == LOWER && enableRampLineFollow) {
                enter(RAMP_FIND_LINE);
                break;
            }
            // Start the first leg of the hop path. Skip the turn if it's a
            // zero-angle leg (issuing a 0-magnitude move makes the drivetrain
            // jitter in place instead of finishing).
            hopLeg = 0;
            if (HOP_LEGS[idx][hopLeg].angleDeg != 0.0f) {
                drive.turn(HOP_LEGS[idx][hopLeg].angleDeg, HOP_TURN_SPEED);
            }
            subStep = 1;
        } else if (subStep == 1) {
            // Turn finished (or none issued): drive this leg's straight segment.
            // Skip the drive on a zero-distance leg so a rotation-only leg works.
            if (driveIdle()) {
                if (absorbCrestMomentum) {
                    // First drive off the ramp crest: ride the climb momentum. Wait
                    // for the coast to fully stop, then drive only the REMAINING
                    // distance so the net forward from the crest equals this leg's
                    // distance. A long coast just needs a small (or reverse) trim.
                    float settleThresh = 0.03f;
                    if (fabs(drive.leftMotor.speed())  > settleThresh ||
                        fabs(drive.rightMotor.speed()) > settleThresh) {
                        break;   // still coasting; keep waiting
                    }
                    float coasted = drive.forwardOdometryMM() - crestForwardOriginMM;
                    float move = HOP_LEGS[idx][hopLeg].distMM - coasted;
                    if (fabs(move) > 2.0f) {
                        drive.driveStraight(move, HOP_DRIVE_SPEED);
                    }
                    Serial.printf("[RAMP] crest coast %.0f mm -> trim %.0f (target %.0f)\n",
                                  coasted, move, HOP_LEGS[idx][hopLeg].distMM);
                    absorbCrestMomentum = false;
                } else if (HOP_LEGS[idx][hopLeg].distMM != 0.0f) {
                    drive.driveStraight(HOP_LEGS[idx][hopLeg].distMM, HOP_DRIVE_SPEED);
                }
                subStep = 2;
            }
        } else {  // subStep == 2
            if (driveIdle()) {
                hopLeg++;
                if (hopLeg < HOP_LEG_COUNT[idx]) {
                    // More legs to go: turn into the next one (skip if zero).
                    if (HOP_LEGS[idx][hopLeg].angleDeg != 0.0f) {
                        drive.turn(HOP_LEGS[idx][hopLeg].angleDeg, HOP_TURN_SPEED);
                    }
                    subStep = 1;
                } else {
                    // Whole path done: this arrival heading is the reference the
                    // next hop is measured from. Start tracking excursion rotation.
                    clusterHeading = 0.0f;
                    excursionOriginMM = drive.forwardOdometryMM();  // arrival baseline
                    sweepPass = 0;  // fresh rock: start the sweep-pass count over
                    approachAttempts = 0;  // and its re-approach retry count
                    // NOTE: no tilt->ramp cross-check here. The ramp is entered
                    // deliberately at rocks_visited==4 (subStep 0 -> FIND_LINE), so
                    // a stray isOnRamp() latch (e.g. a grab/turn jolt spiking the
                    // gyro-fused tilt) must NOT divert us mid-collection.
                    if (enableRockSearch && sweepOnRock[idx]) {
                        enter(FIND_ROCK);
                    } else if (enableMetalScan || enableTeletubbySweep) {
                        // No search/centre, but still run the camera scan (if
                        // enabled) BEFORE lowering the claw. TELETUBBY_SWEEP
                        // self-bypasses to NEED_METAL when the camera is off, and
                        // NEED_METAL skips the claw when metal scan is off.
                        enter(TELETUBBY_SWEEP);
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
            clusterHeading += -SWEEP_ARC / 2.0f;
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                // Sweep across the full arc while polling the ultrasonic.
                // beginScan() wipes stale readings so no pre-sweep data can
                // trigger a false edge.
                ultra.beginScan();
                foundStartEdge = false;
                foundEndEdge = false;
                minSweepDistance = Ultrasonic::MAX_VALID_DISTANCE;  // nothing closer yet
                minSweepAngle = 0.0f;
                sumOnRockAngle = 0.0f;
                onRockCount = 0;
                drive.turn(SWEEP_ARC, SWEEP_SPEED);
                clusterHeading += SWEEP_ARC;
                Serial.printf("[SWEEP] --- sweeping %.0f deg arc ---\n", SWEEP_ARC);
                subStep = 2;
            }
        } else {  // subStep == 2: sweeping
            // Readings glitch as the beam glances off a rock's edges, so we
            // don't trust the distance to "confirm" the rock. Instead we take
            // the FIRST start edge and the LAST end edge (collapsing any edge
            // flicker into one span) and validate by angular width below.
            Ultrasonic::EdgeEvent e = ultra.checkEdgeEvents();
            float h = sweepHeadingDeg(SWEEP_ARC);
            // Latch EVERY edge (accepted or not) for debug/inspection: which
            // edge it was and how many degrees into the sweep arc it fired.
            if (e == Ultrasonic::START_EDGE || e == Ultrasonic::END_EDGE) {
                lastEdgeEvent    = e;
                lastEdgeDeltaDeg = h;  // deg from sweep start
                edgeEventSeq++;
                // Print EVERY edge, whether the accept logic below keeps it or
                // not. A START is kept only as the first one; an END is kept only
                // once a START has been seen.
                bool accepted = (e == Ultrasonic::START_EDGE) ? !foundStartEdge
                                                              : foundStartEdge;
                Serial.printf("[SWEEP] %-5s edge @ %6.1f deg  dist=%5.1f cm  %s\n",
                              e == Ultrasonic::START_EDGE ? "START" : "END",
                              h, ultra.filteredDistanceCm,
                              accepted ? "(accepted)" : "(ignored)");
            }
            if (e == Ultrasonic::START_EDGE) {
                if (!foundStartEdge) {          // keep the first entry
                    foundStartEdge = true;
                    startEdgeAngle = h;
                }
            } else if (e == Ultrasonic::END_EDGE && foundStartEdge) {
                foundEndEdge = true;
                endEdgeAngle = h;               // keep updating to the last exit
            }

            // Track the closest valid reading + its angle (the rock's nearest
            // point), used as the aim point in AIM_MIN_DISTANCE mode.
            float d = ultra.filteredDistanceCm;
            if (d > 0 && d < minSweepDistance) {
                minSweepDistance = d;
                minSweepAngle = h;
            }

            // Accumulate the angular centroid: average the heading over every
            // sample where the edge detector says we are on the object. Uses the
            // whole silhouette, so a single surface bump can't drag the aim off
            // centre the way the nearest-point does on a rough rock.
            if (ultra.scanState == Ultrasonic::TRACKING_OBJECT) {
                sumOnRockAngle += h;
                onRockCount++;
            }

            if (driveIdle()) {
                // A real rock subtends a big enough angle between its edges;
                // a noise glitch collapses to a tiny span. Require both edges
                // AND a wide enough gap.
                float rockWidth = endEdgeAngle - startEdgeAngle;
                if (rockWidth < 0) rockWidth = -rockWidth;

                // Publish the sweep result for debug/inspection (angles as deg
                // into the sweep arc; distance = closest reading seen).
                sweepFound = (foundStartEdge && foundEndEdge && rockWidth >= MIN_ROCK_ANGLE);
                sweepStartAngle = startEdgeAngle;
                sweepEndAngle   = endEdgeAngle;
                sweepDistanceCm = minSweepDistance;
                sweepResultSeq++;

                Serial.printf("[SWEEP] DONE  start=%.1f end=%.1f width=%.1f (min=%.1f)  "
                              "startEdge=%s endEdge=%s  dist=%.1f cm  -> %s\n",
                              startEdgeAngle, endEdgeAngle, rockWidth, MIN_ROCK_ANGLE,
                              foundStartEdge ? "yes" : "no",
                              foundEndEdge ? "yes" : "no",
                              minSweepDistance, sweepFound ? "ROCK" : "no rock");

                // Bench test: don't travel anywhere. If we found a rock, stop and
                // hold indefinitely so the result stays on screen/Serial. If not,
                // recenter and sweep again.
                if (sweepTestMode) {
                    if (sweepFound) {
                        drive.stop();
                        Serial.println("[SWEEP] rock found -- holding.");
                        enter(HOLD);
                    } else {
                        enter(SWEEP_TEST_RECENTER);
                    }
                    break;
                }

                if (foundStartEdge && foundEndEdge && rockWidth >= MIN_ROCK_ANGLE) {
                    float edgeMidpoint = (startEdgeAngle + endEdgeAngle) / 2.0f;
                    switch (aimMode) {
                        case AIM_CENTROID:
                            // Fall back to the edge midpoint if we somehow have
                            // no on-rock samples.
                            rockBearing = (onRockCount > 0)
                                ? (sumOnRockAngle / (float)onRockCount)
                                : edgeMidpoint;
                            break;
                        case AIM_MIN_DISTANCE:
                            rockBearing = minSweepAngle;
                            break;
                        case AIM_EDGE_MIDPOINT:
                        default:
                            rockBearing = edgeMidpoint;
                            break;
                    }
                    enter(TRAVEL_TO_ROCK);
                } else {
                    // No clean low-high-low blip wide enough to be a rock.
                    enter(ADVANCE_CLUSTER);
                }
            }
        }
        break;

    // ---- Bench test: recenter after a sweep, then sweep again --------------
    case SWEEP_TEST_RECENTER:
        // The sweep ends at +SWEEP_ARC/2 relative to the neutral heading; turn
        // back by that much so the next sweep starts from the same place and the
        // robot never walks away from where it is sitting.
        if (subStep == 0) {
            drive.turn(-SWEEP_ARC / 2.0f, SWEEP_SPEED);
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                clusterHeading = 0.0f;   // back at the neutral heading
                enter(FIND_ROCK);        // sweep again
            }
        }
        break;

    // ---- Ramp (after rock 4): rotate to acquire the tape at the ramp foot --
    // Same acquire logic as the panel FIND_LINE, but on success it climbs the
    // ramp (RAMP_FOLLOW_LINE) rather than heading for the panel beacon. The LF
    // channels are sampled by the IR DMA scan, so start the scan here.
    case RAMP_FIND_LINE:
        if (subStep == 0) {
            drive.stop();        // park the drivetrain loop; drive motors directly
            line.start();
            if (robotConfig::IR_ADC_PIN >= 0) ir.startSearch();  // powers the LF DMA scan
            subStep = 1;
        } else {
            line.update(ir.lfLeftRaw(), ir.lfMidRaw(), ir.lfRightRaw());
            if (line.seesLine()) {
                drive.leftMotor.drive(0, robotConfig::STOPPED);
                drive.rightMotor.drive(0, robotConfig::STOPPED);
                enter(RAMP_FOLLOW_LINE);
            } else {
                // Negative / CCW in-place rotation, right motor scaled up to match.
                drive.leftMotor.drive(LINE_SEEK_PWM, robotConfig::REVERSE);
                drive.rightMotor.drive((int)(LINE_SEEK_PWM * LINE_RIGHT_SCALE), robotConfig::FORWARD);
            }
        }
        break;

    // ---- Ramp (after rock 4): follow the tape up the ramp to the crest -----
    // Steer directly from the LF correction. Crest = the tilt sensor latched onto
    // the incline and returned to flat; with the IMU unpinned (not present) that
    // can't fire, so a forward-distance cap (RAMP_CLIMB_MAX_MM) is the fallback.
    case RAMP_FOLLOW_LINE: {
        if (subStep == 0) {
            rampClimbOriginMM = drive.forwardOdometryMM();  // baseline the climb distance
            rampWasTilted = false;
            subStep = 1;
        }

        line.update(ir.lfLeftRaw(), ir.lfMidRaw(), ir.lfRightRaw());
        double corr = line.getCorrection();
        int leftPWM  = constrain((int)(RAMP_BASE_PWM + corr), 0, robotConfig::MAX_DUTY);
        int rightPWM = constrain((int)(RAMP_BASE_PWM - corr), 0, robotConfig::MAX_DUTY);
        drive.leftMotor.drive(leftPWM,  robotConfig::FORWARD);
        drive.rightMotor.drive(rightPWM, robotConfig::FORWARD);

        
        if (tilt.isOnRamp()) {
            if (!rampWasTilted) {
                // This is the FIRST moment we detected the ramp
                rampTime = millis();
                rampWasTilted = true;
            }
        }

        bool crestedByTilt = tilt.isPresent() && rampWasTilted && !tilt.isOnRamp() && (millis() - rampTime > RAMP_MIN_TIME);
        float climbed = drive.forwardOdometryMM() - rampClimbOriginMM;
        bool crestedByDist = (climbed >= RAMP_CLIMB_MAX_MM);

        if (crestedByTilt || crestedByDist) {
            // Cut power but DON'T hard-stop: let the climb momentum coast us forward.
            // The hop-5 first drive absorbs that coast and trims to the leg distance
            // (like the panel's first move), instead of braking then driving cold.
            drive.leftMotor.drive(0, robotConfig::STOPPED);
            drive.rightMotor.drive(0, robotConfig::STOPPED);
            line.stop();
            if (robotConfig::IR_ADC_PIN >= 0) ir.stop();
            level = UPPER;              // on the upper deck now
            crestForwardOriginMM = drive.forwardOdometryMM();  // measure the coast from here
            absorbCrestMomentum = true;
            if (pauseAtCrest) {
                enter(HOLD);            // TEMP: stop at the crest for testing
            } else {
                enter(HOP_TO_CLUSTER);  // hop to rock 5 (HOP_LEGS[4]); first drive rides the momentum
            }
        }
        break;
    }

    // ---- n3: Travel to Rock -----------------------------------------------
    case TRAVEL_TO_ROCK:
        if (subStep == 0) {
            // Face the rock: from +SWEEP_ARC/2 to rockBearing.
            drive.turn(rockBearing - SWEEP_ARC / 2.0f, SWEEP_SPEED);
            clusterHeading += rockBearing - SWEEP_ARC / 2.0f;
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
                // Reached give-up distance without arriving: rock lost. The
                // forward odometer already captured this leg, so if the attempt
                // is later abandoned ADVANCE_CLUSTER reverses it and the robot
                // returns to the arrival pose. Just count the try and re-sweep.
                approachAttempts++;
                if (approachAttempts >= APPROACH_MAX_TRIES) {
                    // Can't close on this rock (out of range, bad aim, or the
                    // sensor never reads <= GRAB_DISTANCE_CM). Abandon cleanly via
                    // ADVANCE_CLUSTER -> realign -> (stopAfterRock) HOLD, instead
                    // of re-sweeping forever and spiralling in circles.
                    enter(ADVANCE_CLUSTER);
                } else {
                    enter(FIND_ROCK);  // re-sweep and try again
                }
            }
        }
        break;

    // ---- n4: Centre on Rock (drive to the target distance) ----------------
    // TRAVEL overshoots because the averaged reading lags the motion, so here
    // we close the loop: let the reading settle, then nudge forward/back to sit
    // at GRAB_DISTANCE_CM. Per-move relative odometry: facing the rock IS the
    // fresh origin, nothing global to reset.
    case CENTRE_ROCK:
        if (subStep == 0) {
            // Let the ultrasonic filter settle now the robot is stopped.
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            if (millis() - stateTimer < CENTRE_SETTLE_MS) break;

            float d = ultra.filteredDistanceCm;
            bool valid = (d > 0 && d < Ultrasonic::MAX_VALID_DISTANCE);
            if (!valid) {
                enter(ADVANCE_CLUSTER);  // lost the rock while aligning
                break;
            }

            float errorCm = d - GRAB_DISTANCE_CM;  // + = too far, - = too close
            if (fabs(errorCm) <= CENTRE_MARGIN_CM || centreAttempts >= CENTRE_MAX_TRIES) {
                // Centred. Run another sweep+centre pass to refine from closer in,
                // up to SWEEP_PASSES total, then move on to the scan.
                sweepPass++;
                if (sweepPass < SWEEP_PASSES) {
                    enter(FIND_ROCK);
                } else {
                    enter(TELETUBBY_SWEEP);
                }
            } else {
                // Drive the distance error: forward if too far, back if too close.
                centreAttempts++;
                drive.driveStraight(errorCm * 10.0f, CENTRE_SPEED);
                subStep = 2;
            }
        } else {  // subStep == 2: wait for the correction move, then re-measure
            if (driveIdle()) {
                // No manual banking: the forward odometer already accounts for
                // this correction move (and every one before it).
                subStep = 0;
            }
        }
        break;

    // ---- n5: Teletubby scan (stationary; the camera does the scanning) ----
    case TELETUBBY_SWEEP:
        if (subStep == 0) {
            // Bypass the scan if it's disabled, or if we already have both
            // teletubbies -- no point holding for the camera on the rest of the
            // rocks once the count is full.
            if (!enableTeletubbySweep || teletubbies >= 2) {
                enter(NEED_METAL);
                break;
            }
            // Rock is aligned: settle briefly before triggering the camera.
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            if (millis() - stateTimer >= CAMERA_PRESCAN_DELAY_MS) {
                // Trigger the camera; checkForTeletubby() blocks until it replies
                // (or times out), so there's no need to hold afterward -- move
                // straight on to the metal check once we have the answer.
                if (teletubbies < 2 && camera.checkForTeletubby()) {
                    teletubbies += 1;
                }
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
            claw.lowerToHover();          // close -> hover -> open, stop at hover
            subStep = 1;
        } else if (subStep == 1) {
            // Let the frequency filter settle at hover before baselining.
            if (!claw.actionBusy()) {
                stateTimer = millis();
                subStep = 2;
            }
        } else if (subStep == 2) {
            if (millis() - stateTimer < HOVER_SETTLE_MS) break;
            // Baseline the detector HERE, at hover: the coil is extended out
            // front, clear of the rock and the metal at the back of the robot.
            // tare() zeroes the baseline to the current (settled) reading. Only
            // touch the hardware when using the real detector.
            if (testMetalOnRock == 0) {
                metal.tare();
            }
            claw.lowerToRock();           // hover -> down onto the rock
            subStep = 3;
        } else {
            if (!claw.actionBusy()) enter(SCAN_METAL);
        }
        break;

    // ---- n7: Scan for metal -----------------------------------------------
    case SCAN_METAL:
        if (subStep == 0) {
            // Claw just reached the rock: let the lowering spike flush out of
            // the frequency filter before we trust the shift.
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            if (millis() - stateTimer < SCAN_SETTLE_MS) break;
            metalSampleCount = 0;
            metalAllAbove = true;
            stateTimer = millis();
            subStep = 2;
        } else {  // subStep == 2: take several spaced confirmation samples
            if (millis() - stateTimer < METAL_SAMPLE_SPACING_MS) break;
            stateTimer = millis();

            bool aboveThreshold;
            if (testMetalOnRock != 0) {
                // TEST override: metal only on rock N (>0). Any other non-zero
                // value (e.g. -1) forces every rock to read as a decoy.
                aboveThreshold = (testMetalOnRock > 0) && (rocks_visited + 1 == testMetalOnRock);
            } else {
                aboveThreshold = metal.isMetalDetected();
            }

            if (!aboveThreshold) metalAllAbove = false;  // one miss = not metal
            metalSampleCount++;

            if (metalSampleCount >= METAL_SAMPLE_COUNT) {
                // Grab only if EVERY sample cleared the threshold. A transient
                // spike decays before all samples land, so it can't misfire.
                if (metalAllAbove) {
                    enter(ENGAGE_CLAW);   // sustained metal
                } else {
                    enter(RAISE_CLAW);    // decoy / spike only
                }
            }
        }
        break;

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
        } else if (claw.armLiftedOffRock()) {
            // Arm is up off the rock: start realigning/hopping now and let the claw
            // finish storing (release + jitter) in the background as we move.
            rock = 1;   // we now hold the metal rock; later clusters skip the claw
            enter(ADVANCE_CLUSTER);
        }
        break;

    // ---- n16: Raise Claw --------------------------------------------------
    case RAISE_CLAW:
        if (subStep == 0) {
            claw.raiseToRest();
            subStep = 1;
        } else if (claw.armLiftedOffRock()) {
            // Arm is back at hover: start realigning/hopping now and let the claw
            // finish raising to rest in the background as we move.
            enter(ADVANCE_CLUSTER);
        }
        break;

    // ---- n12: Advance to next cluster -------------------------------------
    case ADVANCE_CLUSTER:
        // subStep 0-1: TEST return. Reverse the net forward distance driven this
        //   excursion. All forward motion was along rockBearing and the robot
        //   still faces that way (the claw is up here), so a straight reverse
        //   lands it back at the arrival position.
        // subStep 2-3: undo the net rotation the sweep/approach added, so the
        //   next hop is measured from the heading we ARRIVED with.
        if (subStep == 0) {
            // Latch the panel line-acquire spin direction for the rock we just
            // finished (the robot faces differently at each). Used if we divert to
            // the panel now (early exit) or at rock 6's normal completion.
            crestSpinCW = crestSpinCWByRock[rocks_visited <= 5 ? rocks_visited : 5];

            // EARLY EXIT: both objectives met (metal rock + both teletubbies) ->
            // abandon the remaining rocks and head straight to the panel via the
            // one continuous line. Rock 6 (rocks_visited == 5) always takes the
            // normal completion path below instead (so its crest backup still runs).
            if (rock == 1 && teletubbies >= 2 && rocks_visited < 5) {
                phase = PANEL;
                enter(CREST);   // panel line-find (spins crestSpinCW) -> follow -> removal
                break;
            }

            // Skip the post-rock realignment for rocks 4, 5 and 6 (rocks_visited
            // 3, 4, 5 here, before it is incremented). Rocks 4 and 6 are each
            // followed immediately by a crest that re-acquires position from the
            // line, so the realign is wasted; rock 5 is skipped too by request --
            // its hop to rock 6 (HOP_LEGS[5]) is then measured from the post-collection
            // pose rather than the arrival pose.
            if (rocks_visited == 3 || rocks_visited == 4 || rocks_visited == 5) {
                if (stopAfterRock) {
                    enter(HOLD);
                } else {
                    rocks_visited += 1;
                    enter(ALL_DONE);
                }
                break;
            }
            // Net forward distance since arriving at this rock, read straight from
            // both encoders: the sweep turns cancel in the average, leaving only
            // the travel + centring translation. Reverse it to land back at the
            // arrival pose.
            excursionForward = drive.forwardOdometryMM() - excursionOriginMM;
            if (returnAfterCentre && fabs(excursionForward) > 1.0f) {
                drive.driveStraight(-excursionForward, TRAVEL_SPEED);
                subStep = 1;
            } else {
                subStep = 2;   // nothing to return
            }
        } else if (subStep == 1) {
            if (driveIdle()) subStep = 2;
        } else if (subStep == 2) {
            if (fabs(clusterHeading) > 0.5f) {
                drive.turn(-clusterHeading, SWEEP_SPEED);
                clusterHeading = 0.0f;
                subStep = 3;
            } else {
                // Realigned to the arrival pose. In the at-rock test, hold here;
                // otherwise count the rock and move on to the next cluster.
                if (stopAfterRock) {
                    enter(HOLD);
                } else {
                    rocks_visited += 1;
                    enter(ALL_DONE);
                }
            }
        } else {  // subStep == 3
            if (driveIdle()) {
                // Realigned to the arrival pose. In the at-rock test, hold here;
                // otherwise count the rock and move on to the next cluster.
                if (stopAfterRock) {
                    enter(HOLD);
                } else {
                    rocks_visited += 1;
                    enter(ALL_DONE);
                }
            }
        }
        break;

    // ---- n23: All rocks done? ---------------------------------------------
    case ALL_DONE:
        if (rocks_visited >= 6) {
            phase = PANEL;  // n11: collection complete, head for the panel
        }
        enter(ROUTER);
        break;

    // ---- n27: Approach ramp (old dead-reckoned path; unused) --------------
    case RAMP_APPROACH:
        if (subStep == 0) {
            drive.driveStraight(RAMP_APPROACH_MM, RAMP_SPEED);
            stateTimer = millis();
            subStep = 1;
        } else {
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

    // ---- n26: Crest: on the upper deck, head for the panel ----------------
    case CREST:
        level = UPPER;
        // Kick off the IR beacon search now (latches 1kHz/10kHz from the hardware
        // select pin and starts background DMA sampling). Guarded: begin()/start
        // abort on an unset pin, so only touch it once IR_ADC_PIN is wired.
        if (robotConfig::IR_ADC_PIN >= 0) ir.startSearch();
        enter(FIND_LINE);   // panel phase: find the line, then follow it to the panel
        break;

    // ---- Panel: rotate to acquire the tape --------------------------------
    // Spin in place until any LF sensor sees the tape, then follow it. The spin
    // direction (crestSpinCW: CW or CCW) is chosen per rock we came from, since the
    // robot faces the panel line differently at each. Motors are driven directly,
    // so the drivetrain state machine is parked Idle (drive.stop) to keep it away.
    case FIND_LINE:
        if (subStep == 0) {
            drive.stop();       // park the drivetrain loop; we drive motors directly
            line.start();       // enable the LF sensor reads
            subStep = 1;
        } else {
            line.update(ir.lfLeftRaw(), ir.lfMidRaw(), ir.lfRightRaw());  // LF from the IR scan
            if (line.seesLine()) {
                drive.leftMotor.drive(0, robotConfig::STOPPED);
                drive.rightMotor.drive(0, robotConfig::STOPPED);
                enter(FOLLOW_LINE);
            } else if (crestSpinCW) {
                // CW in-place rotation: left wheel fwd, right wheel back.
                drive.leftMotor.drive(LINE_SEEK_PWM, robotConfig::FORWARD);
                drive.rightMotor.drive((int)(LINE_SEEK_PWM * LINE_RIGHT_SCALE), robotConfig::REVERSE);
            } else {
                // CCW in-place rotation: left wheel back, right wheel fwd.
                // Right motor is weaker, so scale its PWM up to keep the spin even.
                drive.leftMotor.drive(LINE_SEEK_PWM, robotConfig::REVERSE);
                drive.rightMotor.drive((int)(LINE_SEEK_PWM * LINE_RIGHT_SCALE), robotConfig::FORWARD);
            }
        }
        break;

    // ---- Panel: follow the tape until the IR beacon reads high ------------
    // Steer by driving the motors directly from the LF correction. Stop once the
    // IR_Sensor's Goertzel amplitude for the selected tone crosses the threshold.
    case FOLLOW_LINE: {
        if (subStep == 0) {
            irWasAbove = false;      // fresh trigger latch for this follow
            irTriggerValid = false;
            subStep = 1;
        }

        line.update(ir.lfLeftRaw(), ir.lfMidRaw(), ir.lfRightRaw());  // LF from the IR scan
        double corr = line.getCorrection();
        // On an early exit the continuous panel line can run up the ramp: use the
        // stronger ramp PWM while the tilt sensor says we're on the incline, then
        // drop back to the slower panel speed on the flat at the top. Harmless for
        // the normal (already-upper-deck) case -- it just stays on LINE_BASE_PWM.
        int base = (tilt.isPresent() && tilt.isOnRamp()) ? RAMP_BASE_PWM : LINE_BASE_PWM;
        int leftPWM  = constrain((int)(base + corr), 0, robotConfig::MAX_DUTY);
        int rightPWM = constrain((int)(base - corr), 0, robotConfig::MAX_DUTY);
        drive.leftMotor.drive(leftPWM,  robotConfig::FORWARD);
        drive.rightMotor.drive(rightPWM, robotConfig::FORWARD);

        if (robotConfig::IR_ADC_PIN >= 0) {
            // Latch the odometer position where the beacon amplitude FIRST crosses
            // the threshold (the start of an above-threshold streak). detected()
            // only confirms a few windows later, and the robot coasts past. The
            // removal's first move (PANEL_REMOVE) realigns to this point + pre-drive.
            bool above = (ir.magnitude() >= ir.threshold());
            if (above && !irWasAbove) {
                irTriggerMM = drive.forwardOdometryMM();
                irTriggerValid = true;
            }
            irWasAbove = above;

            // Stop once the IR sensor CONFIRMS the hardware-selected tone. detected()
            // uses the per-tone threshold (THRESHOLD_1K / THRESHOLD_10K, chosen from
            // the select pin) plus a multi-window confirm, so 1kHz and 10kHz are
            // judged independently.
            if (ir.detected()) {
                // Cut power but coast to a STRAIGHT stop (smart coast keeps the
                // wheels matched during the roll-out) so we don't curve off the
                // beacon before PANEL_REMOVE realigns to the trigger point.
                drive.coast();
                line.stop();
                ir.stop();
                enter(PANEL_REMOVE);   // removal's first move realigns to the trigger
            }
        }
        break;
    }

    // ---- Panel: removal by drive-into-stall + sweep -----------------------
    // Runs once the beacon trips. The line->panel distance is inconsistent, so we
    // FIND the panel by driving into it: turn toward panel -> drive forward until
    // the wheels stall against it -> back up a set distance -> lower the claw ->
    // sweep it off with a turn. Claw is raised (from collection) through the
    // approach so the body finds the panel, then lowered only for the sweep.
    case PANEL_REMOVE:
        if (subStep == 0) {
            // Hand CLOSED the whole time; lower the arm to the panel (hover) angle.
            claw.setAngle(claw.hpin, robotConfig::HAND_CLOSE_ANGLE);
            claw.setAngle(claw.apin, panel().armAngle);
            Serial.printf("[PANEL] surface %d: lower to panel angle (hand closed)\n", panelSurface + 1);
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            // Wait for the arm to settle AND the forward coast to fully stop before
            // measuring the odometer (a mid-coast read under-corrects). Then the
            // realign IS the first movement: one straight move to trigger+pre-drive.
            if (millis() - stateTimer < panel().armSettleMs) break;
            float settleThresh = 0.03f;
            if (fabs(drive.leftMotor.speed())  > settleThresh ||
                fabs(drive.rightMotor.speed()) > settleThresh) {
                break;   // still coasting; keep waiting
            }
            // Target = beacon trigger + pre-drive. move = target - current position;
            // positive drives forward, negative trims the overshoot back.
            float move;
            if (irTriggerValid) {
                move = (irTriggerMM + panel().predriveMM) - drive.forwardOdometryMM();
            } else {
                move = panel().predriveMM;  // no trigger latched: plain pre-drive
            }
            if (fabs(move) > IR_ALIGN_DEADBAND_MM) {
                drive.driveStraight(move, panel().driveSpeed);
            }
            Serial.printf("[PANEL] realign + pre-drive: %.1f mm\n", move);
            subStep = 2;
        } else if (subStep == 2) {
            // Pre-drive done: first turn.
            if (driveIdle()) {
                if (panel().turn1Deg != 0.0f) {
                    drive.turn(panel().turn1Deg, panel().turnSpeed);
                }
                Serial.println("[PANEL] turn 1");
                subStep = 3;
            }
        } else if (subStep == 3) {
            // First turn done: drive straight (hand stays closed throughout).
            if (driveIdle()) {
                if (panel().driveMM != 0.0f) {
                    drive.driveStraight(panel().driveMM, panel().driveSpeed);
                }
                Serial.println("[PANEL] drive straight (hand closed)");
                subStep = 4;
            }
        } else if (subStep == 4) {
            // Final turn, claw still out and closed.
            if (driveIdle()) {
                if (panel().turn2Deg != 0.0f) {
                    drive.turn(panel().turn2Deg, panel().turnSpeed);
                }
                Serial.println("[PANEL] turn 2 (final)");
                subStep = 5;
            }
        } else {  // subStep == 5
            if (driveIdle()) {
                Serial.println("[PANEL] removal done -- holding");
                enter(HOLD);   // removal done; hold
            }
        }
        break;

    // ---- terminal: PANEL / DONE out of scope this pass --------------------
    case HOLD:
        drive.stop();
        break;
    }
}
