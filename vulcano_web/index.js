var pos_x= 0.0;
var pos_y= 0.0;	
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
var imu_temperature = 0.0;
var imu_temperature_status = 0.0;
var limit_temperature = 60.0;
var x_position;
var y_position;
var z_position;
var x_orientation;
var y_orientation;
var z_orientation;
var emergency_button_status = false;
var flw_data = ["","","",""];
var blw_data = ["","","",""];
var frw_data = ["","","",""];
var brw_data = ["","","",""];
var flw_direction_data = ["","","",""];
var blw_direction_data = ["","","",""];
var frw_direction_data = ["","","",""];
var brw_direction_data = ["","","",""];
var torso_elevation_data = ["","","",""];
var torso_rotation_data = ["","","",""];
var status_word_string = "";
var status_word_sub_string = "";

// wheel drivers flag history
var flw_data_history = ["","","",""];
var blw_data_history = ["","","",""];
var frw_data_history = ["","","",""];
var brw_data_history = ["","","",""];
var flw_direction_data_history = ["","","",""];
var blw_direction_data_history = ["","","",""];
var frw_direction_data_history = ["","","",""];
var brw_direction_data_history = ["","","",""];
var torso_elevation_data_history = ["","","",""];
var torso_rotation_data_history = ["","","",""];

// digital inputs/outputs
var digital_inputs = [];
var digital_outputs = [];

//controller options
var gearbox_reduction = 12.52;
var diameter_wheel = 0.207;
var motors_encoders_factor = 4000;
var take_over= true;
var kinematic_mode = "skid";
var motion_odometry = "false";
var motors_encoder = "false";
var x_wam= false;

var status_codes = 
["SW_READY_TO_SWITCH_ON", //0
"SW_SWITCHED_ON",
"SW_OP_ENABLED",
"SW_FAULT",
"SW_VOLTAGE_ENABLED",
"SW_QUICK_STOP",		//5
"SW_SWITCH_ON_DISABLED",
"SW_WARNING",
"SW_DELAY",
"FLAG NOT USED",
"SW_TARGET_REACHED",	//10
"FLAG NOT USED",
"SW_HOME_ATTAINED",
"SW_HOMING_ERROR",
"SW_CAPTURE",		//14
"FLAG NOT USED"];

var drive_status_codes = [
"DS_BRIDGE_ENABLED",  //0
"DS_DYNAMIC_BRAKE_ENABLED",
"DS_SHUNT_ENABLED",
"DS_POSITIVE_STOP_ENABLED",
"DS_NEGATIVE_STOP_ENABLED",
"DS_POSITIVE_TORQUE_INHIBIT_ACTIVE",	//5
"DS_NEGATIVE_TORQUE_INHIBIT_ACTIVE",
"DS_EXTERNAL_BRAKE_ACTIVE",
"DS_DRIVE_RESET",
"DS_DRIVE_INTERNAL_ERROR",
"DS_SHORT_CIRCUIT",//10
"DS_CURRENT_OVERSHOOT",
"DS_UNDER_VOLTAGE",
"DS_OVER_VOLTAGE",
"DS_DRIVER_OVER_TEMPERATURE",
"DS_PARAMETER_RESTORE_ERROR",//15
"DS_PARAMETER_STORE_ERROR",
"DS_INVALID_HALL_STATE",
"DS_PHASE_SYNC_ERROR",
"DS_MOTOR_OVER_TEMPERATURE",
"DS_PHASE_DETECTION_FAULT",//20
"DS_FEEDBACK_SENSOR_ERROR",
"DS_MOTOR_OVER_SPEED",
"DS_MAX_MEASURED_POSITION",
"DS_MIN_MEASURED_POSITION",
"DS_COMMUNICATION_ERROR",//25
"DS_LOG_ENTRY_MISSED",
"DS_COMMANDED_DISABLE",
"DS_USER_DISABLE",
"DS_POSITIVE_INHIBIT",
"DS_NEGATIVE_INHIBIT",//30
"DS_CURRENT_LIMITING",
"DS_CONTINUOUS_CURRENT",
"DS_CURRENT_LOOP_SATURED",
"DS_USER_UNDER_VOLTAGE",
"DS_USER_OVER_VOLTAGE",//35
"DS_NONSINUSOIDAL_COMMUTATION",
"DS_PHASE_DETECTION",
"DS_USER_AUXILIARY_DISABLE",
"DS_SHUNT_REGULATOR",
"DS_PHASE_DETECTION_COMPLETE",//40
"DS_ZERO_VELOCITY",
"DS_AT_COMMAND",
"DS_VELOCITY_FOLLOWING_ERROR",
"DS_POSITIVE_TARGET_VELOCITY_LIMIT",
"DS_NEGATIVE_TARGET_VELOCITY_LIMIT",//45
"DS_COMMAND_LIMITER_ACTIVE",
"DS_IN_HOME_POSITION",
"DS_POSITON_FOLLOWING_ERROR",
"DS_MAX_TARGET_POSITION_LIMIT",
"DS_MIN_TARGET_POSITION_LIMIT",//50
"DS_LOAD_MEASURED_POSITION",
"DS_LOAD_TARGET",
"DS_HOMING_ACTIVE",
"DS_HOMING_COMPLETE",
"DS_PVT_BUFFER_FULL",//55
"DS_PVT_BUFFER_EMPTY",
"DS_PVT_BUFFER_THRESHOLD",
"DS_PVT_BUFFER_FAILURE",
"DS_PVT_BUFFER_EMPTY_STOP",
"DS_PVT_BUFFER_SEQUENCE_ERROR",//60
"DS_COMMANDED_STOP",
"DS_USER_QUICKSTOP",
"DS_COMMANDED_POSITIVE_LIMIT",
"DS_COMMANDED_NEGATIVE_LIMIT",
"DS_ABSOLUTE_POSITION_VALID", //65
"DS_POSITIVE_STOP_ACTIVE",
"DS_NEGATIVE_STOP_ACTIVE",
"FLAG NOT USED"]; //68

/*
var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'
});*/


var ros = new ROSLIB.Ros({
	//url : 'ws://192.168.0.200:9090'
	url : 'ws://localhost:9090'
});


// Publishers
// -----------
var aux = new ROSLIB.Topic({  
	ros : ros,
	name : '/axis_camera/ptz_command',
	messageType : 'robotnik_msgs/ptz'
});


// Subscribing to Topics
	// ----------------------

	
	//battery topic
	var listener = new ROSLIB.Topic({
		ros : ros,
		name : '/summit_xl_controller/battery',
		messageType : 'std_msgs/Float32'
	});

//imu yaw angle topic
/*
	var yawListener = new ROSLIB.Topic({
		ros : ros,
		name : '/imu/yaw_acc',
		messageType : 'std_msgs/Float32',
	});
	*/
	
	//imu state topic
	var imuStateListener = new ROSLIB.Topic({
		ros : ros,
		name : '/imu/data_ok',
		messageType : 'std_msgs/Bool'
	});
	
	
	// IMU temperature topic (Arduimu)
	//var imu_temperature_listener = new ROSLIB.Topic({
		//ros : ros,
		//name : '/imu/temperature',
		//messageType : 'std_msgs/Float32'
	//});
	
	// IMU temperature topic (Arduimu)
	var imu_temperature_listener = new ROSLIB.Topic({
		ros : ros,
		name : '/mavros/imu/temperature',
		messageType : 'sensor_msgs/Temperature'
	});
	
	//emergency stop topic
	var emergency_stop_listener = new ROSLIB.Topic({
		ros : ros,
		name : '/summit_xl_controller/emergency_stop',
		messageType : 'std_msgs/Bool'
	});
	
	// Odometry topic
	var odometry_listener = new ROSLIB.Topic({
		ros : ros,
		name : '/odom',
		messageType : 'nav_msgs/Odometry'
	});
	
	// Motor Status topic
	var motor_status_listener = new ROSLIB.Topic({
		ros : ros,
		name : '/vulcano_base_hw/status',
		messageType : '/vulcano_base_hw/VulcanoMotorsStatus'
	});
	
	 //gps topic
	var gps_listener = new ROSLIB.Topic({
		ros : ros,
		name : '/extended_fix',
		messageType : 'gps_common/GPSFix'
	});



// Message Handlers
// ----------------------

// Battery
	listener.subscribe(function(message) {
	battery_level = message.data;
	});
	
	// Yaw Angle
	/*
	yawListener.subscribe(function(message) {
	if ( message.data >= 90.0 )
	{
		yaw_angle_right = message.data - 90.0;
		yaw_angle_left = 0.0;
	}
	if ( message.data < 90.0 ) 
	{
		yaw_angle_right = 0.0;
		yaw_angle_left = 90.0 - message.data;
	}
	yaw_angle = message.data;
	
	document.querySelector('#yaw_angle span').innerHTML = Math.round(yaw_angle * 10000) / 10000;
	//document.querySelector('#yaw_angle_left span').innerHTML = Math.round(yaw_angle_left * 10000) / 10000;
	//document.querySelector('#yaw_angle_right span').innerHTML = Math.round(yaw_angle_right * 10000) / 10000;

});
*/

// IMU State
imuStateListener.subscribe(function(message) {
	
	
	//console.log(message.data);
	if ( message.data == true && !imu_state ){ 
		//document.querySelector('#imu_status span').innerHTML = "OK";
		document.querySelector('#imu_status span').innerHTML = "<img width=30  height=30 src=images/light-green-flash.jpg border=\"0\">";
		imu_state = true;
		
	}
	if ( message.data == false && imu_state){
		//document.querySelector('#imu_status span').innerHTML = "FAILURE";
		document.querySelector('#imu_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif border=\"0\">";
		imu_state = false;
	}
	});
	
	// IMU temperature listener (Arduimu)
	//imu_temperature_listener.subscribe(function(message){
		//imu_temperature = message.data;
	//});
	
	// IMU temperature listener (Arduimu)
	imu_temperature_listener.subscribe(function(message){
		imu_temperature = message.temperature;
	});
 
	
	
	// Emergency Stop Listener
	emergency_stop_listener.subscribe(function(message){
	
	if ( message.data) {
		document.querySelector('#emergency_stop span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
		//emergency_button_status = true;
	}
	else if ( !message.data) {
		document.querySelector('#emergency_stop span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
		//emergency_button_status = false;
	}
});

// Odometry Listener
	odometry_listener.subscribe(function(message){
	
	
	x_position = message.pose.pose.position.x;
	y_position = message.pose.pose.position.y;
	z_position = message.pose.pose.position.z;
	
	x_orientation = message.pose.pose.orientation.x;
	y_orientation = message.pose.pose.orientation.y;
	z_orientation = message.pose.pose.orientation.z;
	
	
});

// Motor Status Listener
	motor_status_listener.subscribe(function(message){
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
	// TODO fill torso data
		
	// space is required for the comparison
	if(flw_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#front_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#front_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(blw_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#back_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#back_left_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(frw_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#front_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#front_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(brw_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#back_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#back_right_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	// space is required for the comparison
	if(flw_direction_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#front_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#front_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(blw_direction_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#back_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#back_left_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(frw_direction_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#front_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#front_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(brw_direction_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#back_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#back_right_direction_wheel_status span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	// space is required for the comparison //TODO check OPERATION_ENABLED?
	if(torso_elevation_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#torso_elevation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#torso_elevation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	if(torso_rotation_data[1] == "OPERATION_ENABLED "){
		document.querySelector('#torso_rotation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
	}else{
		document.querySelector('#torso_rotation_status_flash span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";	
	}
	
	
	
});

	gps_listener.subscribe(function(message) {
	
	document.querySelector('#gps_satellites_visible span').innerHTML =  message.status.satellites_visible;
	document.querySelector('#gps_satellites_used span').innerHTML =  message.status.satellites_used;
	document.querySelector('#gps_latitude span').innerHTML =  message.latitude;
	document.querySelector('#gps_longitude span').innerHTML =  message.longitude;
			
	});
	
	
	
/*function startTime()
{
aux.subscribe(function(mes){console.log('Received message on ' + mes.pan + ': ' + mes.tilt + ':' + mes.zoom + ':' + mes.relative);
			var divelement = document.getElementById("rosmessage");
			var content =mes.pan + ";" + mes.tilt + ";" + mes.zoom + ";" + mes.relative; 
			divelement.innerHTML = content;    					 
			} 
);
var ros2 = new ROSLIB.Ros({
	url:'ws://localhost:9090'
});
var sensor = new ROSLIB.Topic({
	ros : ros2,
	name : '/tilt_scan',
	massageTyper : 'sensor_msgs/LaserScan'
});
sensor.subscribe(function(message){console.log('Recived message on' + message.angle_min );
		 var divelement = document.getElementById("sensortopic");
		 var content =message.angle_min; 
		 divelement.innerHTML = content; )	

}
function startTime2(){

	var ros = new ROSLIB.Ros({
	url : 'ws://localhost:9090'   
});

var sensor = new ROSLIB.Topic({
	ros : ros,
	name : '/tilt_scan',
	messageType : 'sensor_msgs/LaserScan'
});

sensor.subscribe(function(mes){
		console.log('Recived message on' + mes.angle_min + ":");				
		var divelement = document.getElementById("sensortopic");
		var content = mes.angle_min; 
		divelement.innerHTML = content;
	
 });

}
*/



		function stopJo(){
	var ros = new ROSLIB.Ros({
			url : 'ws://summit:9090'    
		});

	var aux2 = new ROSLIB.Service({  
		ros : ros,
		name : '/summit_xl_pad/enable_disable_pad',
		messageType : 'summit_xl_pad/enable_disable_pad'
	});
	
	var data = new ROSLIB.ServiceRequest({
		value : true
	});
	
	aux2.callService(data,function(res){
		console.log("disable joystick");
	});	

		}
			function startJo(){
	var ros = new ROSLIB.Ros({
			url : 'ws://summit:9090'    
		});

	var aux2 = new ROSLIB.Service({  
		ros : ros,
		name : '/summit_xl_pad/enable_disable_pad',
		messageType : 'summit_xl_pad/enable_disable_pad'
	});
	
	var data = new ROSLIB.ServiceRequest({
		value : false
	});
	
	aux2.callService(data,function(res){
		console.log("enable joystick");
	});	

		}

		function initial(){	
	  
	pos_x= 0.0;
	pos_y= 0.0;
	pos_zoom = 0.0;	
	$( "#slider_zoom" ).slider( "value", 0 );
	var pos = new ROSLIB.Message({
		pan: 0.0,
		tilt: 0.0,
											zoom: 0.0,
											relative: false
		
	});
	aux.publish(pos);
		}
		function move_cam_right(){
	
	if(pos_zoom>4000){
	pos_x = pos_x + 2.5;
	}else{pos_x=pos_x + 10.0;
	}
	move();
	
		}
		function move_cam_left(){
        if(pos_zoom>4000){
	pos_x = pos_x - 2.5;
	}else{pos_x= pos_x - 10.0;
	}
	move();
		}
		function move_cam_top(){
	  
	  
	//limit to 90º?
	if(pos_y < 180 ){
		if(pos_zoom > 4000){
			pos_y = pos_y + 2.5;
		}
		else{ 
			pos_y = pos_y + 10.0;
		}		
	}
	move();
		}
		
		//function flip180(){}
		function move_cam_dowm(){
	  
	if ( pos_y > 0 ){  
		if(pos_zoom > 4000){
			pos_y = pos_y - 2.5;
		}
		else{ 
			pos_y = pos_y - 10.0;
		}
    }
	move();
		}
		function zoomx0(){

	pos_zoom = 0.0;
	move();
		}
		function zoomx15(){

	pos_zoom = 600.0;
	move();
		}
		function zoomx2(){

	pos_zoom = 1550.0;
	move();
		}
		function zoomx3(){

	pos_zoom = 2550.0;
	move();
		}
		function zoomx4(){

	pos_zoom = 5000.0;
	move();
		}
		function zoomx5(){

	pos_zoom = 8000.0;
	move();
		}
		function zoomx6(){

	pos_zoom = 10000.0;
	move();
		}
		function zoomx7(){

	pos_zoom = 13000.0;
	move();
		}
		function zoomx8(){

	pos_zoom = 17200.0;
	move();
		}
		
		function move(){
	var pos = new ROSLIB.Message({
		pan: pos_x,
		tilt: pos_y,
					zoom: pos_zoom,
					relative: false
		
	});
	aux.publish(pos);
	     
}
		function pos1(){
	pos_x= 180.0;
	pos_y= 30.0;
	pos_zoom = 0.0;
	var pos = new ROSLIB.Message({
		pan: 180.0,
		tilt: 30.0,
											zoom: 0.0,
											relative: false
		
	});
	aux.publish(pos);
		}
		function pos2(){
	pos_x= 100.0;
	pos_y= 30.0;
	pos_zoom = 0.0;
	var pos = new ROSLIB.Message({
		pan: 100.0,
		tilt: 30.0,
											zoom: 0.0,
											relative: false
		
	});
	aux.publish(pos);
		}
		function pos3(){
	pos_x= 50.0;
	pos_y= 30.0;
	pos_zoom = 0.0;
	var pos = new ROSLIB.Message({
		pan: 50.0,
		tilt: 30.0,
											zoom: 0.0,
											relative: false
		
	});
	aux.publish(pos);
		}
		function pos4(){
	pos_x= 0.0;
	pos_y= 80.0;
	pos_zoom = 0.0;
	var pos = new ROSLIB.Message({
		pan: 0.0,
		tilt: 80.0,
											zoom: 0.0,
											relative: false
		
	});
	aux.publish(pos);
		}

	 function Foward(){
	

	

	var aux2 = new ROSLIB.Topic({  
		ros:ros,
		name : '/summit_xl_controller/command',
		messageType : 'geometry_msgs/Twist'
	});


	var twist = new ROSLIB.Message({
	linear : {
		x : 0.2,
		y : 0.0,
		z : 0.0
		},
	angular : {
		x : 0.0,
		y : 0.0,
		z : 0.0
	}
	});

	aux2.publish(twist);  
	
		 }
 function Back(){





	var aux2 = new ROSLIB.Topic({  
		ros:ros,
		name : '/summit_xl_controller/command',
		messageType : 'geometry_msgs/Twist'
	});


	var twist = new ROSLIB.Message({
	linear : {
		x : -0.2,
		y : 0.0,
		z : 0.0
		},
	angular : {
		x : 0.0,
		y : 0.0,
		z : 0.0
	}
	});

	aux2.publish(twist); 

		 }
	 function Right(){


	var aux2 = new ROSLIB.Topic({  
		ros:ros,
		name : '/summit_xl_controller/command',
		messageType : 'geometry_msgs/Twist'
	});


	var twist = new ROSLIB.Message({
	linear : {
		x : 0.0,
		y : 0.0,
		z : 0.0
		},
	angular : {
		x : 0.0,
		y : 0.0,
		z : -0.2
	}
	});

	aux2.publish(twist); 

		 }
	 function Left(){



	var aux2 = new ROSLIB.Topic({  
		ros:ros,
		name : '/summit_xl_controller/command',
		messageType : 'geometry_msgs/Twist'
	});


	var twist = new ROSLIB.Message({
	linear : {
		x : 0.0,
		y : 0.0,
		z : 0.0
		},
	angular : {
		x : 0.0,
		y : 0.0,
		z : 0.2
	}
	});

	aux2.publish(twist); 

		 }
		 
		 function startRecording(){
	   
	   	if(is_recording == 0)
			is_recording = 1;

	   var aux2 = new ROSLIB.Service({  
		ros : ros,
		name : '/start_recording',
		messageType : 'summit_xl_web/RecordVideo'
		});
	
		var data = new ROSLIB.ServiceRequest({
			startRecording : 1
		});
		
		aux2.callService(data,function(res){
			console.log("start recording");
			filename = res.filename;
			
			
		});
		updateStrings(filename = "");
	}
	
	function stopRecording(){
	   
	   if(is_recording == 1)
			is_recording = 0;
	   
	   var aux2 = new ROSLIB.Service({  
		ros : ros,
		name : '/start_recording',
		messageType : 'summit_xl_web/RecordVideo'
		});
	
		var data = new ROSLIB.ServiceRequest({
			startRecording : 0
		});
		
		aux2.callService(data,function(res){
			console.log("stop recording");
			filename = res.filename;
			console.log('Saved file on ' + res.filename);
			updateStrings(filename);
		});
		
		
	}
	
	function saveImage(){
		
		//console.log("Saving Image");
		
		var aux2 = new ROSLIB.Service({  
			ros : ros,
			name : '/start_recording',
			messageType : 'summit_xl_web/RecordVideo'
		});
		
		var data = new ROSLIB.ServiceRequest({
			startRecording : 2
		});
		
		aux2.callService(data,function(res){
			//console.log("Image saved");

			document.querySelector('#imageStatus').innerHTML = "Image saved at " + res.filename;
		});
		
	}
	
	/*
	function callkeydownhandler(evnt, event2) {
	   var ev = (evnt) ? evnt : event;
	   var ev2 = (evnt) ? evnt : event2;
	   var code=(ev.which) ? ev.which : event.keyCode;
	   var code2=(ev.which) ? ev.which : event.keyCode;
		
	   if(code == 37)
		Left();
	   else if(code2 == 38)
		Foward();
	   else if(code == 39)
		Right();
	   else if(code2 == 40 )
		Back();
	}
	

	if (window.document.addEventListener) {
	   window.document.addEventListener("keydown", callkeydownhandler, false);
	} else {
	   window.document.attachEvent("onkeydown", callkeydownhandler);
	}
	*/
	
	function updateStrings(filename)
	{
		if(is_recording == 0)
			document.querySelector('#videoStatus').innerHTML = filename + ' recorded';
		else if(is_recording == 1)			
			document.querySelector('#videoStatus').innerHTML = 'Recording video';
	}
	
	
	function setMaximumAngle()
	{
		console.log("Set maximum angle");
		maxAngle = $("#max_angle_spinner").spinner("value");
		maxAngle = parseFloat(maxAngle);
		console.log( maxAngle );
					
		max_angle_message = new ROSLIB.Message({
		 data : maxAngle
		});
		
		//set parameter
		//max_angle_param.set(maxAngle);
	}
	
	function setTrimAngle()
	{
		console.log("Set trim angle");
		trim_angle = $("#trim_spinner").spinner("value");
		trim_angle = parseFloat(trim_angle);
								
		trim_angle_message = new ROSLIB.Message({
		 data : trim_angle
		});
		
		//set parameter
		//trim_angle_param.set(trim_angle);
	}
	
	function saveSettings()
	{
		maxAngle = $("#max_angle_spinner").spinner("value");
		maxAngle = parseFloat(maxAngle);
		trim_angle = $("#trim_spinner").spinner("value");
		trim_angle = parseFloat(trim_angle);
		min_radius = $("#min_radius_spinner").spinner("value");
		min_radius = parseFloat(min_radius);
		reverse_direction = document.getElementById("reverse_direction_checkbox").checked;
		//console.log(maxAngle + " " + trim_angle + " " + min_radius + " " + reverse_direction);
		
		
		
	}
	
	function deactivateIMU(){
		deactivate_imu_msg.data = true;
		document.querySelector('#imu_connected span').innerHTML = "NO";
	}
	
	function activateIMU(){
		deactivate_imu_msg.data = false;
		document.querySelector('#imu_connected span').innerHTML = "SI";
	}
	
	function reset_odometry(){
				
		var aux2 = new ROSLIB.Service({  
			ros : ros,
			name : '/set_odometry',
			messageType : 'robotnik_msgs/set_odometry'
		});
		
		var data = new ROSLIB.ServiceRequest({
			x : 0.0,
			y : 0.0,
			z : 0.0,
			orientation : 0.0
		});
		
		aux2.callService(data,function(res){
						
		});
		
	}
	
	function saveControllerOptions(){
		
		//get options from ui
		
		take_over = document.getElementById("take_over_checkbox").checked;
		
		x_wam = document.getElementById("robot_type").value;
		
		kinematic_mode = document.getElementById("kinematic_mode").value;	
		
		gearbox_reduction = $("#gearbox_reduction_spinner").spinner("value");
		gearbox_reduction = parseFloat(gearbox_reduction);
		
		diameter_wheel = $("#diameter_wheel_spinner").spinner("value");
		diameter_wheel = parseFloat(diameter_wheel);
		
		motion_odometry = document.getElementById("motion_odometry_checkbox").checked;
		if (motion_odometry) 
			motion_odometry = "true";
		else
			motion_odometry = "false";
			
		motors_encoders = document.getElementById("motors_encoders_checkbox").checked;
		if (motors_encoders) 
			motors_encoders = "true";
		else
			motors_encoders = "false";
			
		motors_encoders_factor = $("#motors_encoders_factor_spinner").spinner("value");
		motors_encoders_factor = parseFloat(motors_encoders_factor);
		
		
		
		var aux2 = new ROSLIB.Service({  
			ros : ros,
			name : '/set_controller_options',
			messageType : 'summit_xl_web/set_controller_options'
		});
		
		var data = new ROSLIB.ServiceRequest({
			takeOver : take_over,
			kinematicMode: kinematic_mode,
			gearboxReduction : gearbox_reduction,
			diameterWheel : diameter_wheel,
			motionOdometry : motion_odometry,
			motorsEncoder : motors_encoders,
			motorsEncoderFactor : motors_encoders_factor,
			xWam : x_wam
		});
		
		aux2.callService(data,function(res){});
		
	}
	
	function update_controller_options(){
		var aux2 = new ROSLIB.Service({  
			ros : ros,
			name : '/get_controller_options',
			messageType : 'summit_xl_web/get_controller_options'
		});
		var data = new ROSLIB.ServiceRequest({});
		
		aux2.callService(data,function(res){
			console.log(res.takeOver);
			console.log(res.kinematicMode);
			console.log(res.gearboxReduction);
			console.log(res.diameterWheel);
			console.log(res.motionOdometry);
			console.log(res.motorsEncoder);
			console.log(res.motorsEncoderFactor);
			console.log(res.xWam);
			
			
			if(res.motionOdometry == "true")
				motion_odometry = true;
			else
				motion_odometry = false;
				
			if(res.motorsEncoder == "true")
				motors_encoder = true;
			else
				motors_encoder = false;
			
			document.getElementById("take_over_checkbox").checked = res.takeOver;
			
			
			document.getElementById("motion_odometry_checkbox").checked = motion_odometry;
			
			
			document.getElementById("motors_encoders_checkbox").checked = motors_encoder;
			
			$("#gearbox_reduction_spinner").spinner("value",res.gearboxReduction);
			$("#diameter_wheel_spinner").spinner("value",res.diameterWheel);
			$("#motors_encoders_factor_spinner").spinner("value",res.motorsEncoderFactor);
			document.getElementById("robot_type").value = res.xWam;
			document.getElementById("kinematic_mode").value = res.kinematicMode;
			});
		
	}
	
	function resetDriverHistory()
	{
		
		for(var i=0; i<status_codes.length; i++)
		{
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
		for(var i=0; i<drive_status_codes.length; i++)
		{
			flw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			frw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			blw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			brw_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			
			flw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			frw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			blw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			brw_direction_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			
			
			// TODO check all this 0
			torso_elevation_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
			torso_rotation_data_history[3] = "00000000000000000000000000000000000000000000000000000000000000000000";
		}
	};
	
	function mainLoop(){
		
	    
	    //min valuye = 23, max value = 27.2
	    battery_level_corrected = battery_level - minimum_battery_level;
	    $("#progressbar_battery").progressbar({value: battery_level_corrected});
	    

	    
	    // update battery
	    // -------------
	    battery_level = Math.round(battery_level * 100) / 100;
		document.querySelector('#battery_status span').innerHTML = battery_level;
	
		if(battery_level < 23.0 && battery_status ) {
			document.querySelector('#battery_ok span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
			battery_status = false;
			//console.log("Bateria not ok");
		}
		else if(battery_level >= 23.0 && !battery_status ) {
			document.querySelector('#battery_ok span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
			//console.log("Bateria ok");
			battery_status = true;
		}
	    
	    
	    // update imu temperature
	    // -------------
		imu_temperature = Math.round(imu_temperature * 100) / 100;
		document.querySelector('#imu_temperature span').innerHTML = imu_temperature;
		
		if ( imu_temperature < limit_temperature && !imu_temperature_status) {
			document.querySelector('#imu_temperature_alarm span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg>";
			imu_temperature_status = true;
		}
		else if (imu_temperature >= limit_temperature && imu_temperature_status)
			document.querySelector('#imu_temperature_alarm span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
			imu_temperature_status = false;
	    
	    		    
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
		for( var i = 0; i < flw_data[2].length ; i++)
		{
				if(flw_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					flw_data_history[2] = flw_data_history[2].substr(0, i) + '1' + flw_data_history[2].substr(i + 1);
				}
				else 
				{
					if(flw_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#flw_status_words span').innerHTML = status_word_string;

		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < flw_data[3].length ; i++)
		{
				if(flw_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					flw_data_history[3] = flw_data_history[3].substr(0, i) + '1' + flw_data_history[3].substr(i + 1);
				}
				else 
				{
					if(flw_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
						
		for( var i = 0; i < frw_data[2].length ; i++)
		{
				if(frw_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					frw_data_history[2] = frw_data_history[2].substr(0, i) + '1' + frw_data_history[2].substr(i + 1);
				}
				else 
				{
					if(frw_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#frw_status_words span').innerHTML = status_word_string;
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < frw_data[3].length ; i++)
		{
				if(frw_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					frw_data_history[3] = frw_data_history[3].substr(0, i) + '1' + frw_data_history[3].substr(i + 1);
				}
				else 
				{
					if(frw_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
						
		for( var i = 0; i < blw_data[2].length ; i++)
		{
				if(blw_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					blw_data_history[2] = blw_data_history[2].substr(0, i) + '1' + blw_data_history[2].substr(i + 1);
				}
				else 
				{
					if(blw_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#blw_status_words span').innerHTML = status_word_string;
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < blw_data[3].length ; i++)
		{
				if(blw_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					blw_data_history[3] = blw_data_history[3].substr(0, i) + '1' + blw_data_history[3].substr(i + 1);
				}
				else 
				{
					if(blw_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
						
		for( var i = 0; i < brw_data[2].length ; i++)
		{
				if(brw_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					brw_data_history[2] = brw_data_history[2].substr(0, i) + '1' + brw_data_history[2].substr(i + 1);
				}
				else 
				{
					if(brw_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#brw_status_words span').innerHTML = status_word_string;
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < brw_data[3].length ; i++)
		{
				if(brw_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					brw_data_history[3] = brw_data_history[3].substr(0, i) + '1' + brw_data_history[3].substr(i + 1);
				}
				else 
				{
					if(brw_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
		for( var i = 0; i < flw_direction_data[2].length ; i++)
		{
				if(flw_direction_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					flw_direction_data_history[2] = flw_direction_data_history[2].substr(0, i) + '1' + flw_direction_data_history[2].substr(i + 1);
				}
				else 
				{
					if(flw_direction_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#flw_direction_status_words span').innerHTML = status_word_string;

		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < flw_direction_data[3].length ; i++)
		{
				if(flw_direction_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					flw_direction_data_history[3] = flw_direction_data_history[3].substr(0, i) + '1' + flw_direction_data_history[3].substr(i + 1);
				}
				else 
				{
					if(flw_direction_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
						
		for( var i = 0; i < frw_direction_data[2].length ; i++)
		{
				if(frw_direction_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					frw_direction_data_history[2] = frw_direction_data_history[2].substr(0, i) + '1' + frw_direction_data_history[2].substr(i + 1);
				}
				else 
				{
					if(frw_direction_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#frw_direction_status_words span').innerHTML = status_word_string;
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < frw_direction_data[3].length ; i++)
		{
				if(frw_direction_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					frw_direction_data_history[3] = frw_direction_data_history[3].substr(0, i) + '1' + frw_direction_data_history[3].substr(i + 1);
				}
				else 
				{
					if(frw_direction_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
						
		for( var i = 0; i < blw_direction_data[2].length ; i++)
		{
				if(blw_direction_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					blw_direction_data_history[2] = blw_direction_data_history[2].substr(0, i) + '1' + blw_direction_data_history[2].substr(i + 1);
				}
				else 
				{
					if(blw_direction_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#blw_direction_status_words span').innerHTML = status_word_string;
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < blw_direction_data[3].length ; i++)
		{
				if(blw_direction_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					blw_direction_data_history[3] = blw_direction_data_history[3].substr(0, i) + '1' + blw_direction_data_history[3].substr(i + 1);
				}
				else 
				{
					if(blw_direction_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
						
		for( var i = 0; i < brw_direction_data[2].length ; i++)
		{
				if(brw_direction_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					brw_direction_data_history[2] = brw_direction_data_history[2].substr(0, i) + '1' + brw_direction_data_history[2].substr(i + 1);
				}
				else 
				{
					if(brw_direction_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#brw_direction_status_words span').innerHTML = status_word_string;
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < brw_direction_data[3].length ; i++)
		{
				if(brw_direction_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					brw_direction_data_history[3] = brw_direction_data_history[3].substr(0, i) + '1' + brw_direction_data_history[3].substr(i + 1);
				}
				else 
				{
					if(brw_direction_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
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
		for( var i = 0; i < torso_elevation_data[2].length ; i++)
		{
				if(torso_elevation_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					torso_elevation_data_history[2] = torso_elevation_data_history[2].substr(0, i) + '1' + torso_elevation_data_history[2].substr(i + 1);
				}
				else 
				{
					if(torso_elevation_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#torso_elevation_status_words span').innerHTML = status_word_string;

		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < torso_elevation_data[3].length ; i++)
		{
				if(torso_elevation_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					torso_elevation_data_history[3] = torso_elevation_data_history[3].substr(0, i) + '1' + torso_elevation_data_history[3].substr(i + 1);
				}
				else 
				{
					if(torso_elevation_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#torso_elevation_driver_status_words span').innerHTML = status_word_string;
			    
		// Torso Rotation Motor
		// ----------------
	    document.querySelector('#torso_rotation_state span').innerHTML = torso_rotation_data[0];
		document.querySelector('#torso_rotation_status span').innerHTML = torso_rotation_data[1];
		
		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < torso_rotation_data[2].length ; i++)
		{
				if(torso_rotation_data[2][i] == 1)
				{
					status_word_sub_string = status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
					
					// updating history
					torso_rotation_data_history[2] = torso_rotation_data_history[2].substr(0, i) + '1' + torso_rotation_data_history[2].substr(i + 1);
				}
				else 
				{
					if(torso_rotation_data_history[2][i] == 1) // flag used in the past
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#torso_rotation_status_words span').innerHTML = status_word_string;

		status_word_string = "";
		status_word_sub_string = "";
		for( var i = 0; i < torso_rotation_data[3].length ; i++)
		{
				if(torso_rotation_data[3][i] == 1)
				{
					status_word_sub_string = drive_status_codes[i];
					status_word_string = status_word_string.concat("<br><b>",status_word_sub_string,"</b>");
				
					// updating history
					torso_rotation_data_history[3] = torso_rotation_data_history[3].substr(0, i) + '1' + torso_rotation_data_history[3].substr(i + 1);
				}
				else 
				{
					if(torso_rotation_data_history[3][i] == 1) // flag used in the past
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br><ins>",status_word_sub_string,"</ins>");
						//console.log("flag used in the past");
					}
					else // flag off
					{
						status_word_sub_string = drive_status_codes[i];
						status_word_string = status_word_string.concat("<br>",status_word_sub_string);
					}
				}
		};
		document.querySelector('#torso_rotation_driver_status_words span').innerHTML = status_word_string;
			    
	    
	} 
	 
	//jquery init
	$(document).ready(function() {
		
		//set tab
		$( "#tabs" ).tabs();
		
		$( "#tabs" ).on( "tabsactivate", function( event, ui ) {
			//console.log("before activate");
			console.log(event);
			//console.log(ui.newPanel.selector);
			
			// update displays (controller)
			update_controller_options();
			
			if(ui.newPanel.selector == "#tabs-2" )
			{
				var pass1 = prompt('Introduzca contraseña','');
				if(pass1 == "robotnik")
				{
					console.log("correct password");
					
					//update displays (elliot)
					/*
					$("#max_angle_spinner").spinner("value",max_angle);
					$("#trim_spinner").spinner("value",trim_angle);
					$("#min_radius_spinner").spinner("value",min_radius);
					document.getElementById("reverse_direction_checkbox").checked = reverse_direction
					*/

				}
				else {
					console.log("incorrect password");
					$( "#tabs" ).tabs( "option", "active", 0 );
					//$( "#tabs" ).tabs( "load", 0 );
				}
			}
			
			var active = $( "#tabs" ).tabs( "option", "active" );
			console.log("panel active: " + active);
		});
		
		var yaw_angle_left_100 = 0.0;
		var yaw_angle_right_100 = 0.0;
		
		//Progress bars
		$("#progressbar_battery").progressbar({value: battery_level});
		$("#progressbar_battery" ).progressbar( "option", "max", maximum_battery_level - minimum_battery_level );
		
		$("#progressbar_left").progressbar({value: yaw_angle_left});
		$("#progressbar_right").progressbar({value: yaw_angle_right});

												
		//Spinners (elliot options
		/*
		$("#max_angle_spinner").spinner({step: 0.1, min: 0.0, max: 90.0});
		$("#max_angle_spinner").spinner("value",max_angle);
		$("#trim_spinner").spinner({step: 0.1, min: -1.0, max: 1.0});
		$("#trim_spinner").spinner("value",trim_angle);
		$("#min_radius_spinner").spinner({step: 0.1, min: 0.0, max: 10.0});
		$("#min_radius_spinner").spinner("value",1.0);
		*/
		
		//Spinners (controller options)
		$("#gearbox_reduction_spinner").spinner({step: 0.01, min: 0.0, max: 100.0});
		$("#gearbox_reduction_spinner").spinner("value",gearbox_reduction);
		$("#diameter_wheel_spinner").spinner({step: 0.001, min: 0.0, max: 1.0});
		$("#diameter_wheel_spinner").spinner("value",diameter_wheel);
		$("#motors_encoders_factor_spinner").spinner({step: 1, min: 0, max: 100000});
		$("#motors_encoders_factor_spinner").spinner("value",motors_encoders_factor);
	
	
		//document.querySelector('#imu_connected span').innerHTML = "SI";
		
		
		// Sliders
		$( "#slider_zoom" ).slider();
		$( "#slider_focus" ).slider();
		$( "#slider_zoom" ).on( "slidechange", function( event, ui ) {
			
			
			var slider_value = $( "#slider_zoom" ).slider( "value" );
			pos_zoom = slider_value * 172;
			//console.log(pos_zoom);
			move();
			} );
			
		// init alarms
		document.querySelector('#imu_status span').innerHTML = "<img width=30 height=30 src=images/light-green-flash.jpg border=\"0\">";
		document.querySelector('#battery_ok span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
		document.querySelector('#imu_temperature_alarm span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
		document.querySelector('#emergency_stop span').innerHTML = "<img width=30 height=30 src=images/light-red-flash.gif>";
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
		
		//init messages
		max_angle_message = new ROSLIB.Message({
		 data : max_angle
		});
		
		trim_angle_message = new ROSLIB.Message({
		 data : trim_angle
		});
		
		deactivate_imu_msg = new ROSLIB.Message({
		 data : false
		});
		
		// Motor Dialogs
		// --------------
		
		// Front Left Direction Wheel
		// ----------------
		
		$("#dialog_flw_direction").dialog({
			autoOpen: false
		});
		
		$("#button_flw_direction").click(function(){	
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
		
		$("#button_frw_direction").click(function(){
			
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
		
		$("#button_blw_direction").click(function(){
			
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
		
		$("#button_brw_direction").click(function(){
			
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
		
		$("#button_flw").click(function(){
			
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
		
		$("#button_frw").click(function(){
			
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
		
		$("#button_blw").click(function(){
			
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
		
		$("#button_brw").click(function(){
			
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
		
		$("#button_torso_elevation").click(function(){
			
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
		
		$("#button_torso_rotation").click(function(){
			
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
