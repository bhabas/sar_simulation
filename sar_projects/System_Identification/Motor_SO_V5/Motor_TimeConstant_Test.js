/* //////////////// Description ////////////////

Experimental. This script is a simple method to obtain an approximate system's reaction time and acceleration.

The script will record two additional columns in the output CSV file: the acceleration and the reaction time.

The reaction time is calculated as how long it takes for your system to reach 90% of its final RPM value, after a step input.

The acceleration is an estimate of the acceleration slope at the start of the step. It is calculated as RPM/s. So at the start of a step, the RPM will increase by the value in RPM for each second.

If you want to obtain details to better analyze and understand the results, set the "saveAll" variable to true. This will record ALL datapoints instead of just one result per step.

Important note: this is experimental and results may be incorrect. Please give us feedback on the script to scripts@rcbenchmark.com

///////////// User defined script parameters //////////// */

// Throttle range
var protocol = "dshot600"; // ESC protocol
var minVal = 0;     // ESC initialization value
var initDur = 4;    // ESC initialization time (s)
var testMin = 200;  // 0% throttle value
var testMax = 500;  // 100% throttle value

var settlePercent = 0.9; // will return the settling time to 90% of the final RPM

// List of steps, in terms of throttle between 0 and 100%. Add steps or modify as needed.
steps = [0, 25, 50, 100];

// CSV File
var filePrefix = "90PERCENT";
var saveAll = true; // if true, all data points will be recorded. If false, only final point at end of step will be saved.
var extraTime = 1;  // adds extra time (seconds) before saving the result and going to the next step after the RPM is stable. This gives a changc for the load cells to also stabilize

///////////////// Beginning of the script code //////////////////

var settling = false;   // if currently in the settling phase
var stepUp = false;     // direction of step (up or down)
var initRPM = 0;
var stepIndex = 0;

// start new log file
var add = [
    settlePercent * 100 + "% settling time (s)",
    "Max acceleration (RPM/s)",
];
rcb.files.newLogFile({ prefix: filePrefix, additionalHeaders: add });

// hide console debug info
rcb.console.setVerbose(false);

rcb.console.warning(
    "It is highly recommended to use the optical probe to obtain accurate acceleration data."
);
rcb.console.warning(
    "Plots slow down app which reduces sampling rate. For best accuracy, uncheck all plots."
);

// ESC initialization
rcb.console.print("Initializing ESC...");
rcb.output.set("esc", minVal, protocol);

// Start continuous read loop
rcb.console.print("Starting script loop...");
read();

// start a continuous read cycle
function read() {
    rcb.sensors.read(readDone, 1);
}

// Main loop
var RPMfilter = [];
var RPMhistory = [];
function readDone(result) {
    var add = [];
    if (settling) {
        // wait until the rpm settles (ie stable)

        // save rpm to array
        RPMhistory.push({
            rpm: getRPM(result),
            time: window.performance.now(), //ms
        });

        // Rotating array
        var samples = 30;
        RPMfilter.push(getRPM(result));
        if (RPMfilter.length > samples) {
            RPMfilter.shift();
        }
        if (RPMfilter.length === samples) {
            if (isStable(RPMfilter)) {
                rcb.console.append("stable");
                settling = false;

                // extract important results
                add = extractMotorResults(RPMhistory);
            }
        }
    } else {
        // calculate the ESC throttle signal for this step

        if (stepIndex < steps.length) {
            rcb.console.print(
                "Step " + (stepIndex + 1) + " of " + steps.length + "..."
            );
            var stepPercent = steps[stepIndex];
            if (stepPercent > 100) {
                rcb.console.error("Step % values must be between 0 and 100");
            }
            var throttle = testMin + (testMax - testMin) * stepPercent * 0.01;

            // set the output
            rcb.output.set("esc", Math.round(throttle));

            stepIndex++;
        } else {
            // stop the motor
            rcb.output.set("esc", minVal);
            rcb.wait(rcb.endScript, 0.5);
        }

        // reset filters
        RPMfilter = [getRPM(result)];
        RPMhistory = [
            {
                rpm: getRPM(result),
                time: window.performance.now(), //ms
            },
        ];

        settling = true;
    }

    // continue loop
    if (saveAll || add.length > 0) {
        rcb.files.newLogEntry(result, read, add);
    } else {
        read();
    }
}

// extracts acceleration and settling time from the supplied data
function extractMotorResults(data) {
    var initRPM = data[0].rpm;
    var initTime;
    var lastIndex = data.length - 1;
    var finalRPM = data[lastIndex].rpm;
    var maxAccel = 0;
    var settlingTime;

    // Go through the array
    var lastItem;
    var accelFilter = [];
    data.forEach(function check(item) {
        if (lastItem) {
            if (initTime) {
                var target;

                // Extract current acceleration
                var dt = 0.001 * (item.time - lastItem.time); //seconds
                var dRPM = item.rpm - lastItem.rpm; //rpm
                var accel = Math.round(dRPM / dt); //rpm/s

                // rotating filter
                var filtSize = 3;
                accelFilter.push(accel);
                if (accelFilter.length > filtSize) {
                    accelFilter.shift();
                }
                if (accelFilter.length === filtSize) {
                    // filter glitches from acceleration value
                    if (initRPM < finalRPM) {
                        accelFiltered = Math.min.apply(Math, accelFilter);
                        if (accelFiltered > maxAccel) {
                            maxAccel = accelFiltered;
                        }
                    } else {
                        accelFiltered = Math.max.apply(Math, accelFilter);
                        if (accelFiltered < maxAccel) {
                            maxAccel = accelFiltered;
                        }
                    }
                }

                // Extract settling time
                if (!settlingTime) {
                    target = initRPM + settlePercent * (finalRPM - initRPM);
                    if (initRPM < finalRPM) {
                        if (item.rpm >= target) {
                            settlingTime = 0.001 * (item.time - initTime); // seconds
                        }
                    } else {
                        if (item.rpm <= target) {
                            settlingTime = 0.001 * (item.time - initTime); //seconds
                        }
                    }
                }
            } else {
                // determine the exact time the motor started to react (delay with ESC signal)
                if (Math.abs(item.rpm - initRPM) > 100) {
                    initTime = lastItem.time;
                }
            }
        }
        lastItem = item;
    });

    return [settlingTime, maxAccel];
}

// checks if the readings are stable
var waitUntil = false;
function isStable(readings) {
    if (waitUntil) {
        if (waitUntil < window.performance.now()) {
            waitUntil = false;
            return true;
        }
    } 
    else {
        var sum = 0;
        var up = 0;
        var down = 0;
        var lastVal;
        readings.forEach(function check(val) {
            if (lastVal) {
                if (val > lastVal) {
                    up++;
                } else {
                    down++;
                }
            }
            lastVal = val;
            sum = sum + val;
        });

        //stable if value oscillates around final value
        //(ie, at least 10% of readings are going up, and at least 10% are going down)
        //and values are within 75 rpm and at least 50 rpm
        var criteria = 0.1 * readings.length;
        if (up >= criteria && down >= criteria) {
            var min = Math.min.apply(Math, readings);
            var max = Math.max.apply(Math, readings);
            if (max - min < 75 && min > 50) {
                waitUntil = 1000 * extraTime + window.performance.now(); //ms
            }
        } else {
            if (sum === 0) {
                waitUntil = 1000 * extraTime + window.performance.now(); //ms
            }
        }
    }

    return false;
}

// extracts the RPM from the result variable
function getRPM(result) {
    var electrical = result.motorElectricalSpeed.workingValue;
    var optical = result.motorOpticalSpeed.workingValue;
    if (electrical === 0) {
        return optical;
    } else {
        return electrical;
    }
}
