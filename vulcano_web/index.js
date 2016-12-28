var pos_x = 0.0;
var pos_y = 0.0;
var pos_zoom = 0.0;
var is_recording = 0;
var yaw_angle = 0.0;
var yaw_angle_left = 0.0;
var yaw_angle_right = 0.0;
var default_max_angle = 5.0;
var max_angle = default_max_angle;
var max_angle_message;
var default_trim_angle = 0.0;
var trim_angle = default_trim_angle;
var trim_angle_message;
var max_angle_defined = false;
var trim_angle_defined = false;
var battery_level = 0.0;
var battery_level_corrected;
var maximum_battery_level = 27.2;
var minimum_battery_level = 22.5;
var battery_status = false;
var min_radius = 0.0;
var reverse_direction = false;
var deactivate_imu_msg;
var imu_state = false;
var limit_temperature = 60.0;
var x_position;
var y_position;
var z_position;
var x_orientation;
var y_orientation;
var z_orientation;
var emergency_button_status = false;
var flw_data = ["", "", "", ""];
var blw_data = ["", "", "", ""];
var frw_data = ["", "", "", ""];
var brw_data = ["", "", "", ""];
var flw_direction_data = ["", "", "", ""];
var blw_direction_data = ["", "", "", ""];
var frw_direction_data = ["", "", "", ""];
var brw_direction_data = ["", "", "", ""];
var torso_elevation_data = ["", "", "", ""];
var torso_rotation_data = ["", "", "", ""];
var status_word_string = "";
var status_word_sub_string = "";

// wheel drivers flag history
var flw_data_history = ["", "", "", ""];
var blw_data_history = ["", "", "", ""];
var frw_data_history = ["", "", "", ""];
var brw_data_history = ["", "", "", ""];
var flw_direction_data_history = ["", "", "", ""];
var blw_direction_data_history = ["", "", "", ""];
var frw_direction_data_history = ["", "", "", ""];
var brw_direction_data_history = ["", "", "", ""];
var torso_elevation_data_history = ["", "", "", ""];
var torso_rotation_data_history = ["", "", "", ""];

// digital inputs/outputs
var digital_inputs = [];
var digital_outputs = [];

//controller options
var gearbox_reduction = 12.52;
var diameter_wheel = 0.207;
var motors_encoders_factor = 4000;
var take_over = true;
var kinematic_mode = "skid";
var motion_odometry = "false";
var motors_encoder = "false";
var x_wam = false;

var status_codes = ["SW_READY_TO_SWITCH_ON", //0
    "SW_SWITCHED_ON",
    "SW_OP_ENABLED",
    "SW_FAULT",
    "SW_VOLTAGE_ENABLED",
    "SW_QUICK_STOP", //5
    "SW_SWITCH_ON_DISABLED",
    "SW_WARNING",
    "SW_DELAY",
    "FLAG NOT USED",
    "SW_TARGET_REACHED", //10
    "FLAG NOT USED",
    "SW_HOME_ATTAINED",
    "SW_HOMING_ERROR",
    "SW_CAPTURE", //14
    "FLAG NOT USED"
];

var drive_status_codes = [
    "DS_BRIDGE_ENABLED", //0
    "DS_DYNAMIC_BRAKE_ENABLED",
    "DS_SHUNT_ENABLED",
    "DS_POSITIVE_STOP_ENABLED",
    "DS_NEGATIVE_STOP_ENABLED",
    "DS_POSITIVE_TORQUE_INHIBIT_ACTIVE", //5
    "DS_NEGATIVE_TORQUE_INHIBIT_ACTIVE",
    "DS_EXTERNAL_BRAKE_ACTIVE",
    "DS_DRIVE_RESET",
    "DS_DRIVE_INTERNAL_ERROR",
    "DS_SHORT_CIRCUIT", //10
    "DS_CURRENT_OVERSHOOT",
    "DS_UNDER_VOLTAGE",
    "DS_OVER_VOLTAGE",
    "DS_DRIVER_OVER_TEMPERATURE",
    "DS_PARAMETER_RESTORE_ERROR", //15
    "DS_PARAMETER_STORE_ERROR",
    "DS_INVALID_HALL_STATE",
    "DS_PHASE_SYNC_ERROR",
    "DS_MOTOR_OVER_TEMPERATURE",
    "DS_PHASE_DETECTION_FAULT", //20
    "DS_FEEDBACK_SENSOR_ERROR",
    "DS_MOTOR_OVER_SPEED",
    "DS_MAX_MEASURED_POSITION",
    "DS_MIN_MEASURED_POSITION",
    "DS_COMMUNICATION_ERROR", //25
    "DS_LOG_ENTRY_MISSED",
    "DS_COMMANDED_DISABLE",
    "DS_USER_DISABLE",
    "DS_POSITIVE_INHIBIT",
    "DS_NEGATIVE_INHIBIT", //30
    "DS_CURRENT_LIMITING",
    "DS_CONTINUOUS_CURRENT",
    "DS_CURRENT_LOOP_SATURED",
    "DS_USER_UNDER_VOLTAGE",
    "DS_USER_OVER_VOLTAGE", //35
    "DS_NONSINUSOIDAL_COMMUTATION",
    "DS_PHASE_DETECTION",
    "DS_USER_AUXILIARY_DISABLE",
    "DS_SHUNT_REGULATOR",
    "DS_PHASE_DETECTION_COMPLETE", //40
    "DS_ZERO_VELOCITY",
    "DS_AT_COMMAND",
    "DS_VELOCITY_FOLLOWING_ERROR",
    "DS_POSITIVE_TARGET_VELOCITY_LIMIT",
    "DS_NEGATIVE_TARGET_VELOCITY_LIMIT", //45
    "DS_COMMAND_LIMITER_ACTIVE",
    "DS_IN_HOME_POSITION",
    "DS_POSITON_FOLLOWING_ERROR",
    "DS_MAX_TARGET_POSITION_LIMIT",
    "DS_MIN_TARGET_POSITION_LIMIT", //50
    "DS_LOAD_MEASURED_POSITION",
    "DS_LOAD_TARGET",
    "DS_HOMING_ACTIVE",
    "DS_HOMING_COMPLETE",
    "DS_PVT_BUFFER_FULL", //55
    "DS_PVT_BUFFER_EMPTY",
    "DS_PVT_BUFFER_THRESHOLD",
    "DS_PVT_BUFFER_FAILURE",
    "DS_PVT_BUFFER_EMPTY_STOP",
    "DS_PVT_BUFFER_SEQUENCE_ERROR", //60
    "DS_COMMANDED_STOP",
    "DS_USER_QUICKSTOP",
    "DS_COMMANDED_POSITIVE_LIMIT",
    "DS_COMMANDED_NEGATIVE_LIMIT",
    "DS_ABSOLUTE_POSITION_VALID", //65
    "DS_POSITIVE_STOP_ACTIVE",
    "DS_NEGATIVE_STOP_ACTIVE",
    "FLAG NOT USED"
]; //68

var torso_status_codes = ["SBM (SYSTEM READY)", //0
    "QUE (Power Supply ON)",
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved", //5
    "Reserved",
    "Reserved",
    "QRF (Controller Enabled)", //8
    "Reserved",
    "Reserved", //10
    "Reserved",
    "Reserved",
    "Reserved",
    "Reserved", //14
    "Reserved"
];

var ros = new ROSLIB.Ros({
    url : 'ws://vulcano-base:9090'
    //url: 'ws://localhost:9090'
});


//battery topic
var battery_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'battery/battery_state',
    messageType: 'sensor_msgs/BatteryState'
});

battery_listener.subscribe(function(message) {
    battery_level = message.percentage * 100;
});

//imu state topic
var imuStateListener = new ROSLIB.Topic({
    ros: ros,
    name: '/imu/data_ok',
    messageType: 'std_msgs/Bool'
});

// IMU State
imuStateListener.subscribe(function(message) {


    //console.log(message.data);
    if (message.data == true && !imu_state) {
        //document.querySelector('#imu_status span').innerHTML = "OK";
        document.querySelector('#imu_status span').innerHTML = "<img width=30  height=30 src=images/light-green-flash.jpg border=\"0\">";
        imu_state = true;

    }
    if (message.data == false && imu_state) {
        //document.querySelector('#imu_status span').innerHTML = "FAILURE";
        document.querySelector('#imu_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif border=\"0\">";
        imu_state = false;
    }
});

// Odometry topic
var odometry_listener = new ROSLIB.Topic({
    ros: ros,
    name: 'base/odom',
    messageType: 'nav_msgs/Odometry'
});

// Odometry Listener
odometry_listener.subscribe(function(message) {
    x_position = message.pose.pose.position.x;
    y_position = message.pose.pose.position.y;
    z_position = message.pose.pose.position.z;

	var qx = parseFloat(message.pose.pose.orientation.x);
	var qy = parseFloat(message.pose.pose.orientation.y);
	var qz = parseFloat(message.pose.pose.orientation.z);
	var qw = parseFloat(message.pose.pose.orientation.w);
	var qw2 = qw*qw;
	var qx2 = qx*qx;
	var qy2 = qy*qy;
	var qz2 = qz*qz;
	var test= qx*qy + qz*qw;
	if (test > 0.499) {
		y_orientation = 2.0*Math.atan2(qx,qw);
		z_orientation = Math.PI/2.0;
		x_orientation = 0;
		return;  
	}
	if (test < -0.499) {
		y_orientation = -2.0*Math.atan2(qx,qw);
		z_orientation = -Math.PI/2.0;
		x_orientation = 0;
		return;  
	}
	
	y_orientation = Math.atan2(2*qy*qw-2*qx*qz,1-2*qy2-2*qz2);
	z_orientation = Math.asin(2*qx*qy+2*qz*qw);
	x_orientation = Math.atan2(2*qx*qw-2*qy*qz,1-2*qx2-2*qz2);

});

// Motor Status topic
var motor_status_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/base/vulcano_base_hw/status',
    messageType: 'vulcano_base_hw/VulcanoMotorsStatus'
});

// Motor Status Listener
motor_status_listener.subscribe(function(message) {
    flw_data[0] = message.motor_status[4].state;
    flw_data[1] = message.motor_status[4].status;
    flw_data[2] = message.motor_status[4].statusword;
    flw_data[3] = message.motor_status[4].driveflags;

    blw_data[0] = message.motor_status[5].state;
    blw_data[1] = message.motor_status[5].status;
    blw_data[2] = message.motor_status[5].statusword;
    blw_data[3] = message.motor_status[5].driveflags;

    frw_data[0] = message.motor_status[6].state;
    frw_data[1] = message.motor_status[6].status;
    frw_data[2] = message.motor_status[6].statusword;
    frw_data[3] = message.motor_status[6].driveflags;

    brw_data[0] = message.motor_status[7].state;
    brw_data[1] = message.motor_status[7].status;
    brw_data[2] = message.motor_status[7].statusword;
    brw_data[3] = message.motor_status[7].driveflags;

    flw_direction_data[0] = message.motor_status[0].state;
    flw_direction_data[1] = message.motor_status[0].status;
    flw_direction_data[2] = message.motor_status[0].statusword;
    flw_direction_data[3] = message.motor_status[0].driveflags;

    blw_direction_data[0] = message.motor_status[1].state;
    blw_direction_data[1] = message.motor_status[1].status;
    blw_direction_data[2] = message.motor_status[1].statusword;
    blw_direction_data[3] = message.motor_status[1].driveflags;

    frw_direction_data[0] = message.motor_status[2].state;
    frw_direction_data[1] = message.motor_status[2].status;
    frw_direction_data[2] = message.motor_status[2].statusword;
    frw_direction_data[3] = message.motor_status[2].driveflags;

    brw_direction_data[0] = message.motor_status[3].state;
    brw_direction_data[1] = message.motor_status[3].status;
    brw_direction_data[2] = message.motor_status[3].statusword;
    brw_direction_data[3] = message.motor_status[3].driveflags;


    // space is required for the comparison
    if (flw_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#front_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#front_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (blw_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#back_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#back_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (frw_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#front_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#front_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (brw_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#back_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#back_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    // space is required for the comparison
    if (flw_direction_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#front_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#front_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (blw_direction_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#back_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#back_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (frw_direction_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#front_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#front_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (brw_direction_data[1] == "OPERATION_ENABLED ") {
        document.querySelector('#back_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#back_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

});

// Motor Status topic
var torso_status_listener = new ROSLIB.Topic({
    ros: ros,
    name: '/torso_guidance/vulcano_torso_hw/status',
    messageType: 'vulcano_base_hw/VulcanoMotorsStatus'
});

torso_status_listener.subscribe(function(message) {
	torso_rotation_data[0] = message.motor_status[0].state;
    torso_rotation_data[1] = message.motor_status[0].status;
    torso_rotation_data[2] = message.motor_status[0].statusword;
    torso_rotation_data[3] = message.motor_status[0].driveflags;
	
	torso_elevation_data[0] = message.motor_status[1].state;
    torso_elevation_data[1] = message.motor_status[1].status;
    torso_elevation_data[2] = message.motor_status[1].statusword;
    torso_elevation_data[3] = message.motor_status[1].driveflags;
	
    // space is required for the comparison //TODO check OPERATION_ENABLED?
    if (torso_elevation_data[1] == "DS_OPERATION_ENABLED ") {
        document.querySelector('#torso_elevation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#torso_elevation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }

    if (torso_rotation_data[1] == "DS_OPERATION_ENABLED ") {
        document.querySelector('#torso_rotation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
    } else {
        document.querySelector('#torso_rotation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    }
});

var io_listener = new ROSLIB.Topic({
	ros: ros,
	name: '/io/input_output', //TODO: check topic name
	messageType: 'robotnik_msgs/inputs_outputs'
});

io_listener.subscribe(function(message) {
	for (var i = 0; i < message.digital_inputs.length; i++) {
		var text = "<b>1</b>";
		if (message.digital_inputs[i] == false)
			text = "0";
		document.getElementById("digital_input_status_" + (i+1)).innerHTML = text;
	}
	for (var i = 0; i < message.digital_outputs.length; i++) {
		var text = "<b>1</b>";
		if (message.digital_outputs[i] == false)
			text = "0";
		document.getElementById("digital_output_status_" + (i+1)).innerHTML = text;
	}

});

function deactivateIMU() {
    deactivate_imu_msg.data = true;
    document.querySelector('#imu_connected span').innerHTML = "NO";
}

function activateIMU() {
    deactivate_imu_msg.data = false;
    document.querySelector('#imu_connected span').innerHTML = "SI";
}

function reset_odometry() {

    var reset_odometry_service = new ROSLIB.Service({
        ros: ros,
        name: '/set_odometry',
        messageType: 'robotnik_msgs/set_odometry'
    });

    var data = new ROSLIB.ServiceRequest({
        x: 0.0,
        y: 0.0,
        z: 0.0,
        orientation: 0.0
    });

    reset_odometry_service.callService(data, function(res) {

    });

}

function resetDriverHistory() {

    for (var i = 0; i < status_codes.length; i++) {
        flw_data_history[2] = "0000000000000000";
        frw_data_history[2] = "0000000000000000";
        blw_data_history[2] = "0000000000000000";
        brw_data_history[2] = "0000000000000000";

        flw_direction_data_history[2] = "0000000000000000";
        frw_direction_data_history[2] = "0000000000000000";
        blw_direction_data_history[2] = "0000000000000000";
        brw_direction_data_history[2] = "0000000000000000";

        // TODO check all this 0
        torso_elevation_data_history[2] = "0000000000000000";
        torso_rotation_data_history[2] = "0000000000000000";
    }
    for (var i = 0; i < drive_status_codes.length; i++) {
        flw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
        frw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
        blw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
        brw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";

        flw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
        frw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
        blw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
        brw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";


        // TODO check all this 0
        torso_elevation_data_history[3] = "0000000000000000";
        torso_rotation_data_history[3] =  "0000000000000000";
    }
};

function mainLoop() {

	// TODO: check battery
    $("#progressbar_battery").progressbar({
        value: battery_level
    });

	


    // update battery
    // -------------
    battery_level = Math.round(battery_level);
    document.querySelector('#battery_status span').innerHTML = battery_level;

    if (battery_level < 23.0 && battery_status) {
        document.querySelector('#battery_ok span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
        battery_status = false;
        //console.log("Bateria not ok");
    } else if (battery_level >= 23.0 && !battery_status) {
        document.querySelector('#battery_ok span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
        //console.log("Bateria ok");
        battery_status = true;
    }

    // update odometry
    // -------------

    x_position = Math.round(x_position * 10000) / 10000;
    y_position = Math.round(y_position * 10000) / 10000;
    z_position = Math.round(z_position * 10000) / 10000;

    x_orientation = Math.round(x_orientation * 10000) / 10000;
    y_orientation = Math.round(y_orientation * 10000) / 10000;
    z_orientation = Math.round(z_orientation * 10000) / 10000;

    if (x_position == 0) x_position = x_position.toFixed(4);
    if (y_position == 0) y_position = y_position.toFixed(4);
    if (z_position == 0) z_position = z_position.toFixed(4);
    if (x_orientation == 0) x_orientation = x_orientation.toFixed(4);
    if (y_orientation == 0) y_orientation = y_orientation.toFixed(4);
    if (z_orientation == 0) z_orientation = z_orientation.toFixed(4);


    document.querySelector('#odometry_position_x span').innerHTML = x_position;
    document.querySelector('#odometry_position_y span').innerHTML = y_position;
    document.querySelector('#odometry_position_z span').innerHTML = z_position;

    document.querySelector('#odometry_orientation_x span').innerHTML = x_orientation;
    document.querySelector('#odometry_orientation_y span').innerHTML = y_orientation;
    document.querySelector('#odometry_orientation_z span').innerHTML = z_orientation;

    //update motor values

    // Front Left Motor
    // ----------------
    document.querySelector('#flw_state span').innerHTML = flw_data[0];
    document.querySelector('#flw_status span').innerHTML = flw_data[1];

    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < flw_data[2].length; i++) {
        if (flw_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            flw_data_history[2] = flw_data_history[2].substr(0, i) + '1' + flw_data_history[2].substr(i + 1);
        } else {
            if (flw_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#flw_status_words span').innerHTML = status_word_string;

    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < flw_data[3].length; i++) {
        if (flw_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            flw_data_history[3] = flw_data_history[3].substr(0, i) + '1' + flw_data_history[3].substr(i + 1);
        } else {
            if (flw_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#flw_driver_status_words span').innerHTML = status_word_string;


    // Front Right Motor
    // ----------------

    document.querySelector('#frw_state span').innerHTML = frw_data[0];
    document.querySelector('#frw_status span').innerHTML = frw_data[1];

    // status codes
    // -------------
    status_word_string = "";
    status_word_sub_string = "";

    for (var i = 0; i < frw_data[2].length; i++) {
        if (frw_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            frw_data_history[2] = frw_data_history[2].substr(0, i) + '1' + frw_data_history[2].substr(i + 1);
        } else {
            if (frw_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#frw_status_words span').innerHTML = status_word_string;
    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < frw_data[3].length; i++) {
        if (frw_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            frw_data_history[3] = frw_data_history[3].substr(0, i) + '1' + frw_data_history[3].substr(i + 1);
        } else {
            if (frw_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#frw_driver_status_words span').innerHTML = status_word_string;


    // Back Left Motor
    // ----------------
    document.querySelector('#blw_state span').innerHTML = blw_data[0];
    document.querySelector('#blw_status span').innerHTML = blw_data[1];
    status_word_string = "";
    status_word_sub_string = "";

    for (var i = 0; i < blw_data[2].length; i++) {
        if (blw_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            blw_data_history[2] = blw_data_history[2].substr(0, i) + '1' + blw_data_history[2].substr(i + 1);
        } else {
            if (blw_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#blw_status_words span').innerHTML = status_word_string;
    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < blw_data[3].length; i++) {
        if (blw_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            blw_data_history[3] = blw_data_history[3].substr(0, i) + '1' + blw_data_history[3].substr(i + 1);
        } else {
            if (blw_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#blw_driver_status_words span').innerHTML = status_word_string;


    // Back Right Motor
    // ----------------

    document.querySelector('#brw_state span').innerHTML = brw_data[0];
    document.querySelector('#brw_status span').innerHTML = brw_data[1];
    status_word_string = "";
    status_word_sub_string = "";

    for (var i = 0; i < brw_data[2].length; i++) {
        if (brw_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            brw_data_history[2] = brw_data_history[2].substr(0, i) + '1' + brw_data_history[2].substr(i + 1);
        } else {
            if (brw_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#brw_status_words span').innerHTML = status_word_string;
    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < brw_data[3].length; i++) {
        if (brw_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            brw_data_history[3] = brw_data_history[3].substr(0, i) + '1' + brw_data_history[3].substr(i + 1);
        } else {
            if (brw_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#brw_driver_status_words span').innerHTML = status_word_string;


    // Front Left Motor Direction 
    // ----------------
    document.querySelector('#flw_direction_state span').innerHTML = flw_direction_data[0];
    document.querySelector('#flw_direction_status span').innerHTML = flw_direction_data[1];

    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < flw_direction_data[2].length; i++) {
        if (flw_direction_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            flw_direction_data_history[2] = flw_direction_data_history[2].substr(0, i) + '1' + flw_direction_data_history[2].substr(i + 1);
        } else {
            if (flw_direction_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#flw_direction_status_words span').innerHTML = status_word_string;

    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < flw_direction_data[3].length; i++) {
        if (flw_direction_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            flw_direction_data_history[3] = flw_direction_data_history[3].substr(0, i) + '1' + flw_direction_data_history[3].substr(i + 1);
        } else {
            if (flw_direction_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#flw_direction_driver_status_words span').innerHTML = status_word_string;


    // Front Right Motor Direction
    // ----------------

    document.querySelector('#frw_direction_state span').innerHTML = frw_direction_data[0];
    document.querySelector('#frw_direction_status span').innerHTML = frw_direction_data[1];

    // status codes
    // -------------
    status_word_string = "";
    status_word_sub_string = "";

    for (var i = 0; i < frw_direction_data[2].length; i++) {
        if (frw_direction_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            frw_direction_data_history[2] = frw_direction_data_history[2].substr(0, i) + '1' + frw_direction_data_history[2].substr(i + 1);
        } else {
            if (frw_direction_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#frw_direction_status_words span').innerHTML = status_word_string;
    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < frw_direction_data[3].length; i++) {
        if (frw_direction_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            frw_direction_data_history[3] = frw_direction_data_history[3].substr(0, i) + '1' + frw_direction_data_history[3].substr(i + 1);
        } else {
            if (frw_direction_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#frw_direction_driver_status_words span').innerHTML = status_word_string;


    // Back Left Motor Direction
    // ----------------
    document.querySelector('#blw_direction_state span').innerHTML = blw_direction_data[0];
    document.querySelector('#blw_direction_status span').innerHTML = blw_direction_data[1];
    status_word_string = "";
    status_word_sub_string = "";

    for (var i = 0; i < blw_direction_data[2].length; i++) {
        if (blw_direction_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            blw_direction_data_history[2] = blw_direction_data_history[2].substr(0, i) + '1' + blw_direction_data_history[2].substr(i + 1);
        } else {
            if (blw_direction_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#blw_direction_status_words span').innerHTML = status_word_string;
    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < blw_direction_data[3].length; i++) {
        if (blw_direction_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            blw_direction_data_history[3] = blw_direction_data_history[3].substr(0, i) + '1' + blw_direction_data_history[3].substr(i + 1);
        } else {
            if (blw_direction_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#blw_direction_driver_status_words span').innerHTML = status_word_string;


    // Back Right Motor Direction
    // ----------------

    document.querySelector('#brw_direction_state span').innerHTML = brw_direction_data[0];
    document.querySelector('#brw_direction_status span').innerHTML = brw_direction_data[1];
    status_word_string = "";
    status_word_sub_string = "";

    for (var i = 0; i < brw_direction_data[2].length; i++) {
        if (brw_direction_data[2][i] == 1) {
            status_word_sub_string = status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            brw_direction_data_history[2] = brw_direction_data_history[2].substr(0, i) + '1' + brw_direction_data_history[2].substr(i + 1);
        } else {
            if (brw_direction_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#brw_direction_status_words span').innerHTML = status_word_string;
    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < brw_direction_data[3].length; i++) {
        if (brw_direction_data[3][i] == 1) {
            status_word_sub_string = drive_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            brw_direction_data_history[3] = brw_direction_data_history[3].substr(0, i) + '1' + brw_direction_data_history[3].substr(i + 1);
        } else {
            if (brw_direction_data_history[3][i] == 1) // flag used in the past
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = drive_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#brw_direction_driver_status_words span').innerHTML = status_word_string;


    //Torso //TODO check

    // Torso Elevation Motor
    // ----------------
    document.querySelector('#torso_elevation_state span').innerHTML = torso_elevation_data[0];
    document.querySelector('#torso_elevation_status span').innerHTML = torso_elevation_data[1];

    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < torso_elevation_data[2].length; i++) {
        if (torso_elevation_data[2][i] == 1) {
            status_word_sub_string = torso_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            torso_elevation_data_history[2] = torso_elevation_data_history[2].substr(0, i) + '1' + torso_elevation_data_history[2].substr(i + 1);
        } else {
            if (torso_elevation_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = torso_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = torso_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#torso_elevation_status_words span').innerHTML = status_word_string;

    document.querySelector('#torso_elevation_driver_status_words span').innerHTML = torso_elevation_data[3]; //status_word_string;

    // Torso Rotation Motor
    // ----------------
    document.querySelector('#torso_rotation_state span').innerHTML = torso_rotation_data[0];
    document.querySelector('#torso_rotation_status span').innerHTML = torso_rotation_data[1];

    status_word_string = "";
    status_word_sub_string = "";
    for (var i = 0; i < torso_rotation_data[2].length; i++) {
        if (torso_rotation_data[2][i] == 1) {
            status_word_sub_string = torso_status_codes[i];
            status_word_string = status_word_string.concat("<br><b>", status_word_sub_string, "</b>");

            // updating history
            torso_rotation_data_history[2] = torso_rotation_data_history[2].substr(0, i) + '1' + torso_rotation_data_history[2].substr(i + 1);
        } else {
            if (torso_rotation_data_history[2][i] == 1) // flag used in the past
            {
                status_word_sub_string = torso_status_codes[i];
                status_word_string = status_word_string.concat("<br><ins>", status_word_sub_string, "</ins>");
                //console.log("flag used in the past");
            } else // flag off
            {
                status_word_sub_string = torso_status_codes[i];
                status_word_string = status_word_string.concat("<br>", status_word_sub_string);
            }
        }
    };
    document.querySelector('#torso_rotation_status_words span').innerHTML = status_word_string;

    document.querySelector('#torso_rotation_driver_status_words span').innerHTML = torso_rotation_data[3]; //status_word_string;


}

//jquery init
$(document).ready(function() {

    //set tab
    $("#tabs").tabs();

    //Progress bars
    $("#progressbar_battery").progressbar({
        value: battery_level
    });
    $("#progressbar_battery").progressbar("option", "max", 100.0);

    // init alarms
    document.querySelector('#imu_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg border=\"0\">";
    document.querySelector('#battery_ok span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#front_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#front_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#back_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#back_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#front_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#front_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#back_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#back_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#torso_elevation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
    document.querySelector('#torso_rotation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";


	// init io table
	var number_of_digital_inputs = 16;
	for(var i=1; i<= number_of_digital_inputs; i++) { // check! is <= not, <!!
		$('#digital_inputs_table').append("<tr> <td> " + i + "</td> <td> <div id=\"digital_input_name_"+i+"\"> INPUT_" + i + " </div> </td> <td> <div id=\"digital_input_status_"+i+"\"> NA </div> </td> </tr>");
	}
	
	var number_of_digital_outputs = 16;
	for(var i=1; i <= number_of_digital_outputs; i++) {
		$('#digital_outputs_table').append("<tr> <td> " + i + "</td> <td> <div id=\"digital_output_name_"+i+"\"> OUTPUT_" + i + " </div> </td> <td> <div id=\"digital_output_status_"+i+"\"> NA </div> </td> </tr>");
	}
	
	// it would be nicer to read the io names from the param server, but 
	// roslibjs reads params with a callback that receives the value of the
	// param, without the name of the param, so we hardcode it hear
	document.getElementById("digital_input_name_1").innerHTML = "E_STOP";
	document.getElementById("digital_input_name_2").innerHTML = "LASER_STOP";
	document.getElementById("digital_input_name_3").innerHTML = "BACK_SWITCH_LEFT";
	document.getElementById("digital_input_name_4").innerHTML = "BACK_SWITCH_RIGHT";
	document.getElementById("digital_input_name_5").innerHTML = "TORSO_INPUT_1";
	document.getElementById("digital_input_name_6").innerHTML = "TORSO_INPUT_2";
	document.getElementById("digital_input_name_7").innerHTML = "FRONT_CONNECTOR_1";
	document.getElementById("digital_input_name_8").innerHTML = "FRONT_CONNECTOR_2";
	document.getElementById("digital_input_name_9").innerHTML = "PRESSURE_OK";
	
	document.getElementById("digital_output_name_1").innerHTML = "ELECTROVALVE_1";
	document.getElementById("digital_output_name_2").innerHTML = "ELECTROVALVE_2";
	document.getElementById("digital_output_name_3").innerHTML = "ELECTROVALVE_3";
	document.getElementById("digital_output_name_4").innerHTML = "ELECTROVALVE_4";
	document.getElementById("digital_output_name_5").innerHTML = "ELECTROVALVE_5";
	document.getElementById("digital_output_name_6").innerHTML = "ELECTROVALVE_6";
	document.getElementById("digital_output_name_7").innerHTML = "FLASHING_LIGHT_1";
	document.getElementById("digital_output_name_8").innerHTML = "FLASHING_LIGHT_2";
	document.getElementById("digital_output_name_9").innerHTML = "FRONT_CONNECTOR_1";
	document.getElementById("digital_output_name_9").innerHTML = "RESERVED";
	document.getElementById("digital_output_name_11").innerHTML = "PILOT_LIGHT";
	document.getElementById("digital_output_name_12").innerHTML = "BUZZER";
	document.getElementById("digital_output_name_13").innerHTML = "AIR_PUMP";
	document.getElementById("digital_output_name_14").innerHTML = "LOWER_FAN";
	document.getElementById("digital_output_name_15").innerHTML = "UPPER_FAN";
	document.getElementById("digital_output_name_16").innerHTML = "SAFETY_RELAY";

    //init messages
    max_angle_message = new ROSLIB.Message({
        data: max_angle
    });

    trim_angle_message = new ROSLIB.Message({
        data: trim_angle
    });

    deactivate_imu_msg = new ROSLIB.Message({
        data: false
    });

    // Motor Dialogs
    // --------------

    // Front Left Direction Wheel
    // ----------------

    $("#dialog_flw_direction").dialog({
        autoOpen: false
    });

    $("#button_flw_direction").click(function() {
        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_flw_direction").dialog("open");
    });

    // Front Right Direction Wheel
    // ----------------

    $("#dialog_frw_direction").dialog({
        autoOpen: false
    });

    $("#button_frw_direction").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_frw_direction").dialog("open");
    });

    // Back Left Direction Wheel
    // ----------------

    $("#dialog_blw_direction").dialog({
        autoOpen: false
    });

    $("#button_blw_direction").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_blw_direction").dialog("open");
    });

    // Back Right Direction Wheel
    // ----------------

    $("#dialog_brw_direction").dialog({
        autoOpen: false
    });

    $("#button_brw_direction").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_brw_direction").dialog("open");
    });

    // Front Left Traction Wheel
    // ----------------

    $("#dialog_flw").dialog({
        autoOpen: false
    });

    $("#button_flw").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_flw").dialog("open");
    });

    // Front Right Traction Wheel
    // ----------------

    $("#dialog_frw").dialog({
        autoOpen: false
    });

    $("#button_frw").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_frw").dialog("open");
    });

    // Back Left Traction Wheel
    // ----------------

    $("#dialog_blw").dialog({
        autoOpen: false
    });

    $("#button_blw").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_blw").dialog("open");
    });

    // Back Right Traction Wheel
    // ----------------

    $("#dialog_brw").dialog({
        autoOpen: false
    });

    $("#button_brw").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_brw").dialog("open");
    });


    // Torso Elevation Motor
    // ----------------

    $("#dialog_torso_elevation").dialog({
        autoOpen: false
    });

    $("#button_torso_elevation").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_torso_elevation").dialog("open");
    });


    // Torso Elevation Motor
    // ----------------

    $("#dialog_torso_rotation").dialog({
        autoOpen: false
    });

    $("#button_torso_rotation").click(function() {

        //close other dialogs
        $('[id^="dialog_"]').dialog("close");
        // open selected dialog
        $("#dialog_torso_rotation").dialog("open");
    });


    // reset drivers data history
    resetDriverHistory();


    //call to mainLoop each x ms
    setInterval(mainLoop, 1000);
});
