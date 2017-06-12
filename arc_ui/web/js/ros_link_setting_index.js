
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
	}
	if(!tmpHOST){
		localStorage.setItem("rosHOST",'9090');
		tmpHOST = localStorage.getItem("rosHOST");
	}
} else {
	console.log("Sorry, your browser does not support Web Storage...");
	tmpURL = '127.0.0.1';
	tmpHOST = '9090';
}
// Dialog to connect ROS master
// index.html
$('#rosConnect').bind("click touchstart",function(e){
	tmp = prompt("Please enter the ROS master's IP address.");
	if(tmp != "" && tmp !=null){
		localStorage.setItem("rosURL", URL);
		location.reload();
	}
});

console.log('tmpURL='+tmpURL);
console.log('tmpHOST='+tmpHOST);
