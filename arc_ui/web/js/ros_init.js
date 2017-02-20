
// Connecting to ROS
// -----------------
// Web Storage get the storage "rosURL", "rosHOST" :
var tmpURL,tmpHOST;
if(typeof(Storage) !== "undefined") {
	tmpURL = localStorage.getItem("rosURL");
	tmpHOST = localStorage.getItem("rosHOST");

	//If storage is null, Setting the local to storage
	if(!tmpURL){
		localStorage.setItem("rosURL",'127.0.0.1');
		tmpURL = localStorage.getItem("rosURL");
	}else if(!tmpHOST){
		localStorage.setItem("rosHOST",'9090');
		tmpHOST = localStorage.getItem("rosHOST");
	}
} else {
	console.log("Sorry, your browser does not support Web Storage...");
	tmpURL = '127.0.0.1';
	tmpHOST = '9090';
}

// if($('#ipNum')document.getElementById('ipNum')!=null){
// 	document.getElementById('ipNum').title = tmpURL+':'+tmpHOST;
// }




// Dialog to connect ROS master
// index.html
$('#rosConnect').bind("click touchstart",function(e){
	tmp = prompt("Please enter the ROS master's IP address.");
	if(tmp != "" && tmp !=null){
		RosConnect(tmp,9090);
	}
});


// Connect ros to the assigned url:host
function RosConnect(URL,HOST)
{
	if(typeof(Storage) !== "undefined") {// Check if the browser support Web Storage
		localStorage.setItem("rosURL", URL);
		localStorage.setItem("rosHOST", HOST);
	} else {
		console.log("Sorry, your browser does not support Web Storage...");
		URL = '127.0.0.1';
		HOST = '9090'
	}
	ros = new ROSLIB.Ros({
		url : 'ws://' + URL + ':'+HOST
	});
	if($('#ipNum')!=undefined){
		$('#ipNum').title = tmpURL+':'+tmpHOST;

	}
	ros.on('connection', function() {
		console.log('Connected to websocket server.');
		$('#wifiBanned').removeClass('fa fa-ban fa-stack-2x text-danger');
	});
	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
		$('#wifiBanned').addClass('fa fa-ban fa-stack-2x text-danger');
	});
	ros.on('close', function() {console.log('Connection to websocket server closed.');});
}


var ros;
RosConnect(tmpURL,tmpHOST);
/*
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
	name: '/robotis/present_joint_states',
	messageType : 'sensor_msgs/JointState'
});
// Subscribing to a Topic
// ----------------------
// Remote_control
var joint_Sub = new ROSLIB.Topic({
	ros:ros,
	name: '/robotis/present_joint_states',
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

*/
