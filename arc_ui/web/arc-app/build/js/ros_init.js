
// Connecting to ROS
// -----------------
// Web Storage get the storage "rosURL", "rosHOST" :


// Connect ros to the assigned url:host
function RosConnect(URL,HOST)
{
	
	ros = new ROSLIB.Ros({
		url : 'ws://' + URL + ':'+HOST
	});
	
	ros.on('connection', function() {
		console.log('Connected to websocket server.');
		//$('#wifiBanned').removeClass('fa fa-ban fa-stack-2x text-danger');
	});
	ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
		//$('#wifiBanned').addClass('fa fa-ban fa-stack-2x text-danger');
	});
	ros.on('close', function() {console.log('Connection to websocket server closed.');});
}

var tmpURL = '192.168.1.13';
var tmpHOST = '9090';

var ros;
RosConnect(tmpURL,tmpHOST);