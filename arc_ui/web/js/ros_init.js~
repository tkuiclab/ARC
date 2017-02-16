//Navigation Menu Slider
$('#nav-expander').on('click',function(e){
	e.preventDefault();
	$('body').toggleClass('nav-expanded');
});
$('#nav-close').on('click',function(e){
	e.preventDefault();
	$('body').removeClass('nav-expanded');
});
// Connecting to ROS
// -----------------
var ipAddr = '127.0.0.1';
$('#rosConnect').bind("click touchstart",function(e){
	ipAddr = prompt("Please enter the master's IP address.\nYou are connect to  '"+ipAddr+"'  right now.");
	var ros = new ROSLIB.Ros({
		url : 'ws://' + ipAddr + ':9090'
	});
	document.getElementById('ipNum').title = ipAddr;
	ros.on('connection', function() {console.log('Connected to websocket server.');	});
	ros.on('error', function(error) {console.log('Error connecting to websocket server: ', error);});
	ros.on('close', function() {console.log('Connection to websocket server closed.');});
});

var ros = new ROSLIB.Ros({
	url : 'ws://' + ipAddr + ':9090'
});

document.getElementById('ipNum').title = ipAddr;

ros.on('connection', function() {
	console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
	var request = new ROSLIB.ServiceRequest({
	    cmd : "Teach:EEF_Pose",
	});		
	ui_client.callService(request, function(res) {
		var l = res.pose.linear;
		var a = res.pose.angular;
		
		console.log( 'Result : '   + res.result);
		console.log( 'Pose : '   + l.x + "," + l.y + "," + l.z + "," + a.x + "," + a.y + "," + a.z );
	  	
	  	
	  	var refer = $('#'+m_cmd_id).children("td.SubCmd");
	  	refer.children("input:nth-child(1)").val(l.x.toFixed(2));
	  	refer.children("input:nth-child(2)").val(l.y.toFixed(2));
	  	refer.children("input:nth-child(3)").val(l.z.toFixed(2));
	  	refer.children("input:nth-child(4)").val(a.x.toFixed(2));
	  	refer.children("input:nth-child(5)").val(a.y.toFixed(2));
	  	refer.children("input:nth-child(6)").val(a.z.toFixed(2));
	  	
	  	
	});
	console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
	console.log('Connection to websocket server closed.');
});
// The ActionClient
// ----------------
// Teach_mode
var teachModeClient = new ROSLIB.ActionClient({
	ros : ros,
	serverName : '/teach_mode_server',
	actionName : 'mbot_control/TeachCommandListAction'
});
// Remote_control
var teachModeClient = new ROSLIB.ActionClient({
	ros : ros,
	serverName : '/teach_mode_server',
	actionName : 'mbot_control/TeachCommandListAction'
});
// Json_decoder
var jsonClient = new ROSLIB.ActionClient({
	ros : ros,
	serverName : '/strategy_ui_info',
	actionName : 'mbot_control/UI_InfoAction'
});
// Publishing to a Topic
// ----------------------
// Remote_control
var direct_Pub = new ROSLIB.Topic({
	ros : ros,
	name:'speed_vel',
	messageType : 'geometry_msgs/Twist'
});
var base_Pub = new ROSLIB.Topic({
	ros : ros,
	name:'base_vel',
	messageType : 'geometry_msgs/Twist'
});
// Teach_mode
var joint_sub = new ROSLIB.Topic({
	ros:ros,
	name: '/joint_states',
	messageType : 'sensor_msgs/JointState'
});
// Subscribing to a Topic
// ----------------------
// Remote_control
var joint_Sub = new ROSLIB.Topic({
	ros:ros,
	name: '/joint_states',
	messageType : 'sensor_msgs/JointState'
});
var eef_Sub = new ROSLIB.Topic({
	ros:ros,
	name: '/eef_states',
	messageType : 'geometry_msgs/Twist'
});
// Calling a service
// ----------------------
// Teach_mode
var ui_client = new ROSLIB.Service({
    ros : ros,
    name : '/ui_server',
    serviceType : 'mbot_control/UI_Server'
  });
