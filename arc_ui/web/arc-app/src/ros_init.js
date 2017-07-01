
// Connecting to ROS
// -----------------
// Web Storage get the storage "rosURL", "rosHOST" :
import ROSLIB from 'roslib';

// Connect ros to the assigned url:host
export function RosConnect(URL,HOST)
{
	var ros;
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

	return ros;
}


export function use_default_ros_connet(){
	var default_URL = '127.0.0.1';
	var default_HOST = '9090';

	var url, host;

	if(typeof(Storage) !== "undefined") {
		// localStorage.setItem("rosURL",default_URL);
		// localStorage.setItem("rosHOST",default_HOST);


		url = localStorage.getItem("rosURL");
		host = localStorage.getItem("rosHOST");

		//If storage is null, Setting the local to storage
		if(!url){
			localStorage.setItem("rosURL",default_URL);
			url = localStorage.getItem("rosURL");
		}
		if(!host){
			localStorage.setItem("rosHOST",default_HOST);
			host = localStorage.getItem("rosHOST");
		}
		
	} 
	console.log('ros_init say connet to ' + url + ':' + host)
	return RosConnect(url,host);
}
