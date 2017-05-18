//Base Control Button
//cause there has a lot of button need to send the value each other,so move this js code stand out the index.js
var speedArr;
var default_speed = 0.5;
$('#direction_base_speed_send').on('click',function(e){
	if(document.getElementById('direction_base_speed_value').value >=0 && document.getElementById('direction_base_speed_value').value<=100){
		default_speed = document.getElementById('direction_base_speed_value').value / 100;
		document.getElementById('direction_base_speed_value').value = '';
		document.getElementById('direction_base_speed_value').placeholder = default_speed * 100;
	}else{
		alert("Speed only allow 0~100%");
	}
});
function direct(dirID,spArr){
	$(dirID).bind('mousedown touchstart',function(e){
		if(dirID == '#direction_base_enable' || dirID == '#direction_base_disable'){
			var twist = new ROSLIB.Message({
				linear : {x : spArr[0],y : spArr[1],z : spArr[2]},
				angular : {x : spArr[3],y : spArr[4],z : spArr[5]}
			});
		}else{
			var twist = new ROSLIB.Message({
				linear : {
				x : spArr[0]*default_speed,
				y : spArr[1]*default_speed,
				z : spArr[2]*default_speed
				},
				angular : {
				x : spArr[3]*default_speed,
				y : spArr[4]*default_speed,
				z : spArr[5]*default_speed
				}
			});
		}
		base_Pub.publish(twist);
		$("#infoContent").html('Topic was send twist'
					+"<BR>linear:<BR>"+ twist.linear.x +"<BR>"+ twist.linear.y +"<BR>"+ twist.linear.z
					+"<BR>angular<BR>"+ twist.angular.x +"<BR>"+ twist.angular.y +"<BR>"+ twist.angular.z
		);
	});
	//when mouseUp stop the motion
	$(dirID).bind('mouseup touchend',function(e){
		var twist = new ROSLIB.Message({
			linear : {x : 0,y : 0,z : 0},
			angular : {x : 0,y : 0,z : 0}
		});
		base_Pub.publish(twist);
		$("#infoContent").html('Topic was send twist'
					+"<BR>linear:<BR>"+ twist.linear.x +"<BR>"+ twist.linear.y +"<BR>"+ twist.linear.z
					+"<BR>angular<BR>"+ twist.angular.x +"<BR>"+ twist.angular.y +"<BR>"+ twist.angular.z
		);
	});
	//Speacial case for dialog setting
	if(dirID == 'Base'){
		var twist = new ROSLIB.Message({
			linear : {x : spArr[0],y : spArr[1],z : spArr[2]},
			angular : {x : spArr[3],y : spArr[4],z : spArr[5]}
		});
		base_Pub.publish(twist);
		$("#infoContent").html('Topic was send twist'
					+"<BR>linear:<BR>"+ twist.linear.x +"<BR>"+ twist.linear.y +"<BR>"+ twist.linear.z
					+"<BR>angular<BR>"+ twist.angular.x +"<BR>"+ twist.angular.y +"<BR>"+ twist.angular.z
		);
	}
};
//enable the controller
//angular.x -> 0:speed mode, 1:enable, 2:stop
speedArr = [ 0, 0, 0, 1, 0, 0];
direct("#direction_base_enable",speedArr);
//disable the controller
speedArr = [ 0, 0, 0, 2, 0, 0];
direct("#direction_base_disable",speedArr);
//Speed Mode
speedArr = [ 0, 1, 0, 0, 0, 0];
direct("#direction_base_front",speedArr);
speedArr = [ 0,-1, 0, 0, 0, 0];
direct("#direction_base_back",speedArr);
speedArr = [-1, 0, 0, 0, 0, 0];
direct("#direction_base_left",speedArr);
speedArr = [ 1, 0, 0, 0, 0, 0];
direct("#direction_base_right",speedArr);
//Base Dialog Setting
$("#control_send_base_d").on('click',function(e){
	var lx = $("#input_base_lx_d").val()/100;
	var ly = $("#input_base_ly_d").val()/100;
	var az = $("#input_base_az_d").val()/100;
	if(lx >= 0 && lx <=1 && ly >= 0 && ly <=1 && az >= 0 && az <=1){
		speedArr = [ lx, ly, 0, 0, 0, az];
		direct("Base",speedArr);
	}else{
		alert("Value needs to be 0~100%");
	}
});
