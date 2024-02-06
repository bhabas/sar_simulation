/* //////////////// Discrete steps V2 ////////////////
 The script will sweep between the input values "minVal" and "maxVal". The sweep will be made in discrete "stepsQty" steps. Each step will consist of a settling time "settlingTime" after which a new log entry will be recorded. To reduce noise, "samplesAvg" will be averaged and recorded. This script uses the improved steps2 function, that can introduce a cooling time, as well as a slew-rate limiter for smooth step transitions.
 
The '.' represents a sample is recorded. 5 steps will record 6 data rows (one for zero).

 ^ Motor Input
 |                             __.  maxVal                
 |                         __./   \                 
 |                     __./        \                      
 |                 __./             \           
 |      minVal __./                  \  
 | escInit___./                       \ 
 |_______________________________________> Time
 
///////////// User defined variables //////////// */

var protocol = "dshot600";  // ESC protocol
var escStart = 0;           // ESC start value [NULL THROTTLE DETECTION REQUIRED]
var minVal = 100;           // Min. input value [150, 2047] 
var maxVal = 1800;          // Max. input value [150, 2047] 
var samplesAvg = 100;       // Number of samples to average at each step
var repeat = 10;            // How many times to repeat the same sequence
var filePrefix = "StepsTestV2";

// step parameters
var params = {
    steps_qty: 18,               // Number of steps
    settlingTime_s: 2,          // Settling time before measurement
    cooldownTime_s: 0,          // If the motor needs to cool down between steps. Zero disables cooldown.
    cooldownThrottle_us: 1175,  // Cool down faster when slowly spinning
    cooldownMinThrottle: 1500,  // Only activates the cooldown time for high throttle
    max_slew_rate_us_per_s: 75  // Limits torque from throttle changes
};
   


///////////////// Beginning of the script //////////////////

//Start new file
rcb.files.newLogFile({prefix: filePrefix});

//Tare the load cells
rcb.sensors.tareLoadCells(initESC);

//Arms the ESC
function initESC(){
    //ESC initialization
    rcb.console.print("Initializing ESC...");
    rcb.output.set("esc", escStart,protocol);
    rcb.wait(startSteps, 4);
}

//Start steps
function startSteps(){
    takeSample(ramp);
}

// Records a sample to CSV file
function takeSample(callback){
    rcb.sensors.read(function (result){
        // Write the results and proceed to next step
        rcb.files.newLogEntry(result, callback); 
    }, samplesAvg);
}

// Start the ramp up function
function ramp(){
    rcb.output.steps2("esc", minVal, maxVal, stepFct, finish, params);
}

// The following function will be executed at each step.
function stepFct(nextStepFct){
    takeSample(nextStepFct);  
}

// Ramp back down then finish script
function finish(){
    // Calculate the ramp down time
    var rate = params.max_slew_rate_us_per_s;
    var time = 0;
    if(rate>0){
        time = (maxVal-escStart) / rate;
    }
    rcb.output.ramp("esc", maxVal, escStart, time, endScript);
}

//Ends or loops the script
function endScript() {
    if(--repeat > 0){
      if(repeat === 0){
        rcb.console.print("Repeating one last time...");
      }else{
        rcb.console.print("Repeating " + repeat + " more times...");
      }
      startSteps();
    }else{
      rcb.endScript();
    }
}