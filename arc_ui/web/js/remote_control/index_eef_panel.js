	//EEF_plus or minus
	//cause there has 12 button need to send the value each other,so move this js code stand out the index.js
	function actionPTP(valArr){
		var mlist=[];
		var twist = new ROSLIB.Message({
			linear : {
			x : valArr[0],
			y : valArr[1],
			z : valArr[2]
			},
			angular : {
			x : valArr[3],
			y : valArr[4],
			z : valArr[5]
			}
		});
		var send = new ROSLIB.Message({
			cmd : 'PTP',
			pose : twist
		});
		mlist.push(send);
		var goal = new ROSLIB.Goal({
			actionClient : teachModeClient,
			goalMessage : {
				cmd_list : mlist
			}
		});
		$("#infoContent").html('Goal was send cmdlist'
					+'<BR>Cmd : ' + send.cmd
					+"<BR>Pose : "
					+"<BR>linear:<BR>"+ send.pose.linear.x +"<BR>"+ send.pose.linear.y +"<BR>"+ send.pose.linear.z
					+"<BR>angular<BR>"+ send.pose.angular.x +"<BR>"+ send.pose.angular.y +"<BR>"+ send.pose.angular.z
		);
		goal.send();
		mlist = [];
	};
	//Show the dialog table which one is needed.
	function showDialog(showTagID){
		$('#dialog_table_eef').hide();
		$('#dialog_table_joint').hide();
		$('#dialog_table_plus').hide();
		$('#dialog_table_base').hide();
		$(showTagID).show();
	};
	//Minus or Plus the value to EEF Position right now.
	function MinusPlusDialog(tagID,operator){
		$('.'+tagID).html(eef_Sub_lx + " "+operator+" ");
		$('#control_send_plus_d').on('click',function(e){
			var float64 = new Float64Array(6);
			if(tagID != '' && operator != ''){
				float64[0] = eef_Sub_lx;
				float64[1] = eef_Sub_ly;
				float64[2] = eef_Sub_lz;
				float64[3] = eef_Sub_ax;
				float64[4] = eef_Sub_ay;
				float64[5] = eef_Sub_az;

				if(tagID == 'plus_lx' || tagID == 'minus_lx'){
					if(operator == '+') float64[0] = float64[0] + $('#input_plus_d').val();
					else if(operator == '-') float64[0] = float64[0] - $('#input_plus_d').val();
				}else if(tagID == 'plus_ly' || tagID == 'minus_ly'){
					if(operator == '+') float64[1] = float64[1] + $('#input_plus_d').val();
					else if(operator == '-') float64[1] = float64[1] - $('#input_plus_d').val();
				}else if(tagID == 'plus_lz' || tagID == 'minus_lz'){
					if(operator == '+') float64[2] = float64[2] + $('#input_plus_d').val();
					else if(operator == '-') float64[2] = float64[2] - $('#input_plus_d').val();
				}else if(tagID == 'plus_ax' || tagID == 'minus_ax'){
					if(operator == '+') float64[3] = float64[3] + $('#input_plus_d').val();
					else if(operator == '-') float64[3] = float64[3] - $('#input_plus_d').val();
				}else if(tagID == 'plus_ay' || tagID == 'minus_ay'){
					if(operator == '+') float64[4] = float64[4] + $('#input_plus_d').val();
					else if(operator == '-') float64[4] = float64[4] - $('#input_plus_d').val();
				}else if(tagID == 'plus_az' || tagID == 'minus_az'){
					if(operator == '+') float64[5] = float64[5] + $('#input_plus_d').val();
					else if(operator == '-') float64[5] = float64[5] - $('#input_plus_d').val();
				}
				actionPTP(float64);
				tagID="";operator = "";
			}
		});
	};
	$('#plus_lx').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("plus_lx","+");
	});
	$('#minus_lx').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("minus_lx","-");
	});
	$('#plus_ly').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("plus_ly","+");
	});
	$('#minus_ly').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("minus_ly","-");
	});
	$('#plus_lz').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("plus_lz","+");
	});
	$('#minus_lz').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("minus_lz","-");
	});
	//angular
	$('#plus_ax').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("plus_ax","+");
	});
	$('#minus_ax').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("minus_ax","-");
	});
	$('#plus_ay').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("plus_ay","+");
	});
	$('#minus_ay').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("minus_ay","-");
	});
	$('#plus_az').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("plus_az","+");
	});
	$('#minus_az').on('click',function(e){
		showDialog('#dialog_table_plus');
		MinusPlusDialog("minus_az","-");
	});
