var l = console.log;
var run_cmd_ind = 0;
var CmdType = {
	Joint: "Joint",
	PTP: "PTP",
	Line: "Line",
	Shift_X: "Shift_X",
	Shift_Y: "Shift_Y",
	Shift_Z: "Shift_Z",
	Rotate_X: "Rotate_X",
	Rotate_Y: "Rotate_Y",
	Rotate_Z: "Rotate_Z",
	Rot_Fai: "Rot_Fai",
	Vaccum: "Vaccum",
};
var yn_cnt = false;
var test_cnt=0;
var Control_Type = {REL:"REL", ABS:"ABS"};

function command_selected(cmd){
	//console.log('cmd='+cmd);

	var command_select = '<select class="options_cmd_none" disabled="disabled" onchange="Cmd_change(this)">'+
			'<option value="Joint"'+((cmd==CmdType.Joint)?"selected":"")+'>Joint</option>'+
			'<option value="PTP"'+((cmd==CmdType.PTP)?"selected":"")+'>PTP</option>'+
			'<option value="Line"'+((cmd==CmdType.Line)?"selected":"")+'>Line</option>'+
			'<option value="Shift_X"'+((cmd==CmdType.Shift_X)?"selected":"")+'>Shift_X</option>'+
			'<option value="Shift_Y"'+((cmd==CmdType.Shift_Y)?"selected":"")+'>Shift_Y</option>'+
			'<option value="Shift_Z"'+((cmd==CmdType.Shift_Z)?"selected":"")+'>Shift_Z</option>'+
			'<option value="Rotate_X"'+((cmd==CmdType.Rotate_X)?"selected":"")+'>Rotate_X</option>'+
			'<option value="Rotate_Y"'+((cmd==CmdType.Rotate_Y)?"selected":"")+'>Rotate_Y</option>'+
			'<option value="Rotate_Z"'+((cmd==CmdType.Rotate_Z)?"selected":"")+'>Rotate_Z</option>'+
			'<option value="Rot_Fai"'+((cmd==CmdType.Rot_Fai)?"selected":"")+'>Rot_Fai</option>'+
			'<option value="Vaccum"'+((cmd==CmdType.Vaccum)?"selected":"")+'>Vaccum</option>'+
			'</select>';

	return command_select;
}

var addbtn = document.getElementById("addbtn");
var edit_img   = '<img name="edit_btn"   src="img/edit.png"   onclick="edit_Cmd(this)"/>';
var true_img   = '<img name="true_btn"   src="img/true.png"   class="img_show" onclick="save_Cmd(this)"/>';
var false_img  = '<img name="false_btn"  src="img/false.png"  class="img_show" onclick="break_Cmd(this)"/>';
var delete_img = '<img name="delete_opt" src="img/delete.png" style="width:20px; height:auto; text-align:right;" onclick="delete_Cmd(this)"/>';
var teach_a = '<button name="teach_btn" class="btn btn-info btn-block" style="width:80px; display:none;" onclick="teach_click(this)"><span class="glyphicon glyphicon-pushpin"></span>Teach</button>';
var cmd_id;

function hide_all(){
	$("#block").hide();
	$("#vaccum_block").hide();
	$("#shift_block").hide();
	$("#base_vel_block").hide();
}

function order_list()
{
	var num = 1;
	$('#teach_table tr').each(function(){
		cmd_id = 'cmd_' + num;
		$(this).attr('id',cmd_id);
		if($(this).children('td.order').length == 0){
			$(this).children("td:first").before('<td class="order" onclick="copy(this)">'+$('#teach_table').children().length+'</td>');
		}else{
			$(this).children("td:first").remove();
			$(this).children("td:first").before('<td class="order" onclick="copy(this)">'+num+'</td>');
		}
		num++;
	});
	mov_it();
}

function copy(tr){
	var tar_id = "cmd_"+$(tr)[0].outerText;

	var mod = $('tr#'+tar_id).find('[name=cmd_mod]').children('select').val();

	$("#cmd_select").val(mod);

	cmd_select_change(tar_id,mod);
}

function mov_it(){
	var mov = $("#move_cmd").prop("checked");
	$('#teach_table tr').each(function(){
		if(mov == true){
			$(this).children("td:first").attr('draggable',true);
			$(this).children("td:first").attr('ondragstart','drag(event)');
			$(this).children("td:first").attr('ondragenter','dragEnter(event)');
			$(this).children("td:first").attr('ondragleave','dragLeave(event)');
		}else{
			$(this).children("td:first").removeAttr('draggable',true);
			$(this).children("td:first").removeAttr('ondragstart','drag(event)');
			$(this).children("td:first").removeAttr('ondragenter','dragEnter(event)');
			$(this).children("td:first").removeAttr('ondragleave','dragLeave(event)');
		}
	});
}

$("#move_cmd").change(function(){
	console.log("Order Changed");
	mov_it();
});

$("#cmd_select").change(function() {
	cmd_select_change();
});

function cmd_select_change(ev,mod){
	var cmd = $("#cmd_select").val();

	hide_all();
	if(cmd==CmdType.Joint || cmd==CmdType.PTP || cmd==CmdType.Line){
		$("#block").show();
		$("#block").css("display","inline");
	}else if(cmd==CmdType.Vaccum){
		$("#vaccum_block").show();
		$("#vaccum_block").css("display","inline");
	}else if(cmd==CmdType.Shift_X  || cmd==CmdType.Shift_Y  || cmd==CmdType.Shift_Z ||
			 cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z|| cmd==CmdType.Rot_Fai){
		$("#shift_block").show();
		$("#shift_block").css("display","inline");
	}else if(cmd==CmdType.Base_Vel){
		$("#base_vel_block").show();
		$("#base_vel_block").css("display","inline");
	}

	if(ev != undefined){
		if(mod==CmdType.Joint || mod==CmdType.PTP || mod==CmdType.Line){
			var i = 1;
			$('#'+ev).children("td.SubCmd").children("input").each(function(){
				$("#block_"+i++).val($(this).val());
			});
		}else if(mod==CmdType.Vaccum){
			$("#vaccum_select").val($('#'+ev).find('[name=vaccum_select]').val());
		}else if(mod==CmdType.Shift_X || mod==CmdType.Shift_Y || mod==CmdType.Shift_Z ||
				 mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z||mod==CmdType.Rot_Fai){
			$("#shift_val").val($('#'+ev).children("td.SubCmd").children("input").val());
		}
	}
}

function get_block_tr(option_cmd,val){
	var sub_cmd = '';

	if(option_cmd==CmdType.Joint || option_cmd==CmdType.PTP || option_cmd==CmdType.Line){
		if(val==undefined){
			for(var i=1;i<=7;i++){
				var t_id = '#block_'+i;
				sub_cmd += '<input class="block_sty_none" type="number" value="'+$(t_id).val()+'"readonly>';
			}
		}else{
			for(var i=0;i<7;i++){
				sub_cmd += '<input class="block_sty_none" type="number" value="'+val[i]+'"readonly>\n';
			}
		}
	}else if(option_cmd==CmdType.Vaccum){
		if(val==undefined){
			sub_cmd =
			'<select class="options_cmd_none" name="vaccum_select" disabled="disabled">' +
				'<option value="On"'+(($('#vaccum_select').val() == "On")?"selected":"")+'>On</option>'+
				'<option value="Off"'+(($('#vaccum_select').val() == "Off")?"selected":"")+'>Off</option>' +
			'</select>';
		}else{
			sub_cmd =
			'<select class="options_cmd_none" name="vaccum_select" disabled="disabled">' +
				'<option value="On"'+((val==true)?"selected":"")+'>On</option>'+
				'<option value="Off"'+((val==false)?"selected":"")+'>Off</option>' +
			'</select>';
			//console.log("in vaccum",val);
			if(val == 'true') console.log("vaccum == true");
		}

	}else if(option_cmd==CmdType.Shift_X || option_cmd==CmdType.Shift_Y || option_cmd==CmdType.Shift_Z||
		     option_cmd==CmdType.Rotate_X || option_cmd==CmdType.Rotate_Y || option_cmd==CmdType.Rotate_Z|| option_cmd==CmdType.Rot_Fai){
		if(val==undefined){
			sub_cmd = '<input class="block_sty_none" type="number" value="'+$('#shift_val').val()+'"readonly>';
		}else{
			sub_cmd = '<input class="block_sty_none" type="number" value="'+val+'"readonly>';
		}
	}else if(option_cmd==CmdType.Base_Vel ){

		if(val==undefined){
			for(var i=1;i<=3;i++){
				var t_id = '#base_vel_block_'+i;
				sub_cmd += '<input class="block_sty_none" type="number" value="'+$(t_id).val()+'"readonly>';
			}
		}else{

			for(var i=0;i<3;i++){
				sub_cmd += '<input class="block_sty_none" type="number" value="'+val[i]+'"readonly>\n';
			}
		}
	}

	var add_tr =
	'<tr class="font_black" id="" >' +
		'<td name="cmd_mod">'+command_selected(option_cmd)+'</td>'+
		'<td class="SubCmd">'+sub_cmd+'</td>'+
		'<td>'+edit_img + true_img + false_img + teach_a +'</td>'+
		'<td style="text-align:center;">'+delete_img+'</td>'+
	'</tr>';
	return add_tr;
}

$("#addbtn").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var cmd = $("#cmd_select").val();
	var tr_html = '';

	if(cmd != 'Choose') tr_html = get_block_tr(cmd);

	console.log('in_add_btn');

	$('#teach_table').append(tr_html);
	$('#teach_table').scrollTop($('#teach_table')[0].scrollHeight);

	order_list();

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

var cmd_edit = new Array;
var vac_cmd;

function Cmd_change(edit,val_7){
	var cmd = $(edit).val();
	var sub_cmd = '';

	if(cmd==CmdType.Joint || cmd==CmdType.PTP || cmd==CmdType.Line){
		if(val_7==undefined){
			for(var i=1;i<=7;i++){
				var t_id = '#block_'+i;
				sub_cmd += '<input class="block_sty" type="number" value="'+$(t_id).val()+'">';
			}
		}else{
			for(var i=0;i<7;i++){
				sub_cmd += '<input class="block_sty" type="number" value="'+val_7[i]+'">\n';
			}
		}
	}else if(cmd==CmdType.Vaccum){
		sub_cmd =
			'<select class="options_cmd" name="vaccum_select">' +
				'<option value="On"'+(($('#vaccum_select').val() == "On")?"selected":"")+'>On</option>'+
				'<option value="Off"'+(($('#vaccum_select').val() == "Off")?"selected":"")+'>Off</option>' +
			'</select>';
	}else if(cmd==CmdType.Shift_X || cmd==CmdType.Shift_Y || cmd==CmdType.Shift_Z||
		     cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z|| cmd==CmdType.Rot_Fai){
		sub_cmd = '<input class="block_sty" type="number" value="'+$('#shift_val').val()+'">';
	}
	// else if(cmd==CmdType.Joint)
	// {
	// 	$("#block_lab_1").val()="dd";
	// }

	$(edit).parents("tr").children('.SubCmd').children().remove();
	$(edit).parents("tr").children('.SubCmd').append(sub_cmd);
}

function edit_Cmd(edit){
	var m_cmd_id = $(edit).parents('tr').attr('id');

	console.log("m_cmd_id="+m_cmd_id);

	$('#'+m_cmd_id).find('[name=edit_btn]').hide();
	$('#'+m_cmd_id).find('[name=true_btn]').show();
	$('#'+m_cmd_id).find('[name=false_btn]').show();
	$('#'+m_cmd_id).find('[name=teach_btn]').show();
	$('#'+m_cmd_id).find('[name=teach_btn]').css('display','inline');

	//var mod = $('#'+m_cmd_id).children("[name=cmd_mod]").text();

	var mod = $('tr#'+m_cmd_id).find('[name=cmd_mod]').children('select').val();

	$('#'+m_cmd_id).find('[name=cmd_mod]').children('select').attr("class","options_cmd");
	$('#'+m_cmd_id).find('[name=cmd_mod]').children('select').removeAttr("disabled");

	var i = 0;

	if(mod=='Base_Vel' || mod==CmdType.Joint || mod==CmdType.PTP || mod==CmdType.Line ||
			mod==CmdType.Shift_X || mod==CmdType.Shift_Y || mod==CmdType.Shift_Z ||
			mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z ||
			mod==CmdType.Base_Vel || mod==CmdType.Rot_Fai){

		$('#'+m_cmd_id).children("td.SubCmd").children("input").each(function(){
			$(this).removeAttr("readonly");
			//$(this).css("border-style","inset");
			$(this).removeClass("block_sty_none");
			$(this).addClass("block_sty");
			cmd_edit[i++] = $(this).val();
		});
	}else if(mod==CmdType.Vaccum){
		$('#'+m_cmd_id).find('[name=vaccum_select]').removeAttr("disabled");
		$('#'+m_cmd_id).find('[name=vaccum_select]').attr("class","options_cmd");
		vac_cmd = $('#'+m_cmd_id).find('[name=vaccum_select]').val();
	}
}

function save_Cmd(edit){
	var m_cmd_id = $(edit).parents('tr').attr('id');
	console.log("m_cmd_id="+m_cmd_id);

	$('#'+m_cmd_id).find('[name=edit_btn]').show();
	$('#'+m_cmd_id).find('[name=true_btn]').hide();
	$('#'+m_cmd_id).find('[name=false_btn]').hide();
	$('#'+m_cmd_id).find('[name=teach_btn]').hide();

	var mod = $('tr#'+m_cmd_id).find('[name=cmd_mod]').children('select').val();

	$('#'+m_cmd_id).find('[name=cmd_mod]').children('select').attr("class","options_cmd_none");
	$('#'+m_cmd_id).find('[name=cmd_mod]').children('select').attr("disabled","disabled");

	if(mod==CmdType.Joint || mod==CmdType.PTP || mod==CmdType.Line ||
			mod==CmdType.Shift_X || mod==CmdType.Shift_Y || mod==CmdType.Shift_Z ||
			mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z ||
			mod==CmdType.Rot_Fai || mod==CmdType.Base_Vel){
		$('#'+m_cmd_id).children("td.SubCmd").children("input").each(function(){
			//console.log("in save");
			$(this).attr("readonly","readonly");
			$(this).addClass("block_sty_none");
			$(this).removeClass("block_sty");
			$(this).attr("value",$(this).val());
		});
	}else if(mod==CmdType.Vaccum){
		$('#'+m_cmd_id).find('[name=vaccum_select]').attr("disabled","disabled");
		$('#'+m_cmd_id).find('[name=vaccum_select]').attr("class","options_cmd_none");
	}
}

function break_Cmd(edit){
	var m_cmd_id = $(edit).parents('tr').attr('id');
	console.log("m_cmd_id="+m_cmd_id);

	$('#'+m_cmd_id).find('[name=edit_btn]').show();
	$('#'+m_cmd_id).find('[name=true_btn]').hide();
	$('#'+m_cmd_id).find('[name=false_btn]').hide();
	$('#'+m_cmd_id).find('[name=teach_btn]').hide();

	var mod = $('tr#'+m_cmd_id).find('[name=cmd_mod]').children('select').val();

	$('#'+m_cmd_id).find('[name=cmd_mod]').children('select').attr("class","options_cmd_none");
	$('#'+m_cmd_id).find('[name=cmd_mod]').children('select').attr("disabled","disabled");
	var i = 0;
	if(mod==CmdType.Joint || mod==CmdType.PTP || mod==CmdType.Line ||
			mod==CmdType.Shift_X || mod==CmdType.Shift_Y || mod==CmdType.Shift_Z ||
			mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z || 
			mod==CmdType.Rot_Fai||mod==CmdType.Base_Vel){
		$('#'+m_cmd_id).children("td.SubCmd").children("input").each(function(){
			$(this).val(cmd_edit[i++]);
			$(this).attr("readonly","readonly");
			$(this).addClass("block_sty_none");
			$(this).removeClass("block_sty");
		});
	}else if(mod==CmdType.Vaccum){
		$('#'+m_cmd_id).find('[name=vaccum_select]').val(vac_cmd);
		$('#'+m_cmd_id).find('[name=vaccum_select]').attr("disabled","disabled");
		$('#'+m_cmd_id).find('[name=vaccum_select]').attr("class","options_cmd_none");
	}
}

function delete_Cmd(edit){
	var m_cmd_id = $(edit).parents('tr').attr('id');
	console.log("m_cmd_id="+m_cmd_id);

	$('#'+m_cmd_id).remove();
	order_list();
}

function allowDrop(ev) {
  ev.preventDefault();
}

var first_ev;

function drag(ev) {
	console.log("drag",ev);
	first_ev = ev.target.outerText;
}

function drop(ev) {
  ev.preventDefault();
	order_list();
}

function dragEnter(ev) {
  var ty_id = ev.target.outerText;

	if(ev.offsetY < 10)	$('#cmd_'+first_ev).insertAfter('#cmd_'+ty_id);
	else	$('#cmd_'+first_ev).insertBefore('#cmd_'+ty_id);
}

function dragLeave(ev) {
}

$("#show_btn").click(function(){
  $("#test_json_data").slideToggle("slow");
});

//===========================================================================================
//===========================================================================================
//===========================================================================================
//===========================================================================================


function get_twist(){
	var twist = new ROSLIB.Message({
		linear : {
			x : 0.0,
			y : 0.0,
			z : 0.0,
		},
		angular : {
			x : 0.0,
			y : 0.0,
			z : 0.0,
		}
	});

	return twist;
}

$("#set_mode_btn").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var str_msg = new ROSLIB.Message({
		data : "set_mode"
	});
	console.log('in set_mode_bnt');
	set_mode_pub.publish(str_msg);

	$(this).addClass('active');
	$(this).removeClass('disabled');

	FB_Timer();
});

function FB_Timer()
{
	setInterval
	(
		function()
		{
			Get_Robot_FB();
		},100
	);
}

function Get_Robot_FB() 
{
	var request_TCP = new ROSLIB.ServiceRequest({
		group_name: "arm",
	});

	pose_client.callService(request_TCP, function (res) 
	{
		// Display TCP FeedBack
		var p = res.group_pose.position;						//position
		var o = res.group_pose.orientation;					//orientation
		var e = euler_fomr_quaternion([o.x, o.y, o.z, o.w]);	//Convert quaternion to RPY
		var f = res.group_redundancy;

		var fb_TCP=[];
		var fb_TCP_name=["#output_lx", "#output_ly", "#output_lz", 
						 "#output_ay", "#output_ax", "#output_az", "#output_f"];
		
		fb_TCP[0] = _Math._Roundn(parseFloat(p.x),2);
		fb_TCP[1] = _Math._Roundn(parseFloat(p.y),2);	
		fb_TCP[2] = _Math._Roundn(parseFloat(p.z),2);
		fb_TCP[6] = _Math._Roundn(f*_Math.RAD2DEG, 2);

		for(var i=0;i<3;i++)
			fb_TCP[i+3] = _Math._Roundn(e[i]*_Math.RAD2DEG, 2);
		for(var i=0;i<7;i++)
			$(fb_TCP_name[i]).html(fb_TCP[i]);
	});

	var request_Joint = new ROSLIB.ServiceRequest
	({
		joint_name :['joint1', 'joint2', 'joint3','joint4','joint5','joint6','joint7'],
	});
	joint_client.callService(request_Joint, function (res) 
	{
		// Display Joint FeedBack
		// $("#MF_block_x").html("gh");
		var fb_ang=[];
		var fb_ang_name=["#output_j1p", "#output_j2p", "#output_j3p", "#output_j4p", 
						 "#output_j5p", "#output_j6p", "#output_j7p"];
		for(var i=0;i<7;i++)
		{
			fb_ang[i] = _Math._Roundn(res.joint_value[i]*_Math.RAD2DEG, 2);
			$(fb_ang_name[i]).html(fb_ang[i]);
		}
		
	});
}
$("#Test_btn").click(function(){ //tb
	$(this).removeClass('active');
	$(this).addClass('disabled');
	var request = new ROSLIB.ServiceRequest
	({
		joint_name :['joint1', 'joint2', 'joint3','joint4','joint5','joint6','joint7'],
	});

	joint_client.callService(request, function (res) {
		var fb_ang = res.joint_value[0];
		$("#output_lx").html(fb_ang);
	});
	
	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#plus_lx").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Shift_Pose").val();
	
	Move_TCP_Rel(CmdType.Shift_X, data);
	console.log('in plus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_lx").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Shift_Pose").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Shift_X, data);
	console.log('in minus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#plus_ly").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Shift_Pose").val();
	
	Move_TCP_Rel(CmdType.Shift_Y, data);
	console.log('in plus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_ly").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Shift_Pose").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Shift_Y, data);
	console.log('in minus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});


$("#plus_lz").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Shift_Pose").val();
	
	Move_TCP_Rel(CmdType.Shift_Z, data);
	console.log('in plus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_lz").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Shift_Pose").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Shift_Z, data);
	console.log('in minus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#plus_ax").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	
	Move_TCP_Rel(CmdType.Rotate_X, data);
	console.log('in plus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_ax").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Rotate_X, data);
	console.log('in minus_x'+data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#plus_ay").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	Move_TCP_Rel(CmdType.Rotate_Y, data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_ay").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Rotate_Y, data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#plus_az").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	Move_TCP_Rel(CmdType.Rotate_Z, data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_az").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Rotate_Z, data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#plus_f").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	Move_TCP_Rel(CmdType.Rot_Fai, data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#minus_f").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var data = $("#txt_Rot_Orien").val();
	data*=-1;
	Move_TCP_Rel(CmdType.Rot_Fai, data);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});
// btn_Joint_ABS

$("#btn_Joint_ABS").click(function() 
{
	var Ctrl_Joint_Num;
	var sel_item = $("#Joint_Ctrl_Typ_Sel").val();
	if	   (sel_item=='Joint1')  Ctrl_Joint_Num = 0;
	else if(sel_item=='Joint2')  Ctrl_Joint_Num = 1;
	else if(sel_item=='Joint3')  Ctrl_Joint_Num = 2;
	else if(sel_item=='Joint4')  Ctrl_Joint_Num = 3;
	else if(sel_item=='Joint5')  Ctrl_Joint_Num = 4;
	else if(sel_item=='Joint6')  Ctrl_Joint_Num = 5;
	else if(sel_item=='Joint7')  Ctrl_Joint_Num = 6;
	else 						 Ctrl_Joint_Num = -1;
	
	
	var ang = parseFloat($("#txt_Joint_Rel").val());

	console.log('sel_item = '+sel_item);
	console.log('Ctrl_Joint_Num = '+Ctrl_Joint_Num);
	console.log('ang = '+ang);
	Joint_Control(Ctrl_Joint_Num, ang, "ABS");
});

// Joint Rel_Control
$("#plus_J1").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(0, ang, "REL");
});

$("#minus_J1").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(0, ang, "REL");
});

$("#plus_J2").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(1, ang, "REL");
});

$("#minus_J2").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(1, ang, "REL");
});

$("#plus_J3").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(2, ang, "REL");
});

$("#minus_J3").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(2, ang, "REL");
});

$("#plus_J4").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(3, ang, "REL");
});

$("#minus_J4").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(3, ang, "REL");
});

$("#plus_J5").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(4, ang, "REL");
});

$("#minus_J5").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(4, ang, "REL");
});

$("#plus_J6").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(5, ang, "REL");
});

$("#minus_J6").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(5, ang, "REL");
});

$("#plus_J7").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	Joint_Control(6, ang, "REL");
});

$("#minus_J7").click(function()
{
	var ang = parseFloat($("#txt_Joint_Rel").val());
	ang *= -1;
	Joint_Control(6, ang, "REL");
});

$("#Vaccum_Control").click(function() {
	yn_cnt = !yn_cnt;
	console.log('yn_cntn = '+yn_cnt);

	var req = new ROSLIB.ServiceRequest({
		cmd: yn_cnt? 'vacuumOn': 'vacuumOff',
	});

	vacuum_client.callService(req, function (res) {
		console.log('Is vacuum cmd success: ' + res.success);
	});
});

function Joint_Control(Joint_id, ang, Control_Type)
{
	run_cmd_ind = $('#teach_table tr').length;
	var Joint_Num;
	if	   (Joint_id==0)	Joint_Num = 'joint1';
	else if(Joint_id==1)	Joint_Num = 'joint2';
	else if(Joint_id==2)	Joint_Num = 'joint3';
	else if(Joint_id==3)	Joint_Num = 'joint4';
	else if(Joint_id==4)	Joint_Num = 'joint5';
	else if(Joint_id==5)	Joint_Num = 'joint6';
	else if(Joint_id==6)	Joint_Num = 'joint7';
	else 					Control_Type = 'none' ;
	if(Control_Type=="REL")
	{
		var request = new ROSLIB.ServiceRequest({
		    joint_name :['joint1', 'joint2', 'joint3','joint4','joint5','joint6','joint7'],
		});
		joint_client.callService(request, function (res) 
		{
			$(this).removeClass('active'); 
			$(this).addClass('disabled');

			var joint_name_ary = [Joint_Num] ;
			var float_ary = [];
			var orig_ang = res.joint_value[Joint_id];

			ang = ang*_Math.DEG2RAD + orig_ang;
			float_ary.push(ang);

			var cmd_msg = new ROSLIB.Message({
				name: joint_name_ary,
				value: float_ary
			});
			console.log('in REL joint control');
			console.log('Joint_Num = ' + orig_ang);

			joint_pub.publish(cmd_msg);

			$(this).addClass('active');
			$(this).removeClass('disabled');
		});
	}
	else if(Control_Type=="ABS")
	{
		$(this).removeClass('active'); 
		$(this).addClass('disabled');

		var joint_name_ary = [Joint_Num];
		var float_ary = [];

		ang = ang * _Math.DEG2RAD;
		float_ary.push(ang);

		var cmd_msg = new ROSLIB.Message({
			name: joint_name_ary,
			value: float_ary
		});
		console.log('in ABS joint control');
		console.log('ABS CMD='+ang);

		joint_pub.publish(cmd_msg);

		$(this).addClass('active');
		$(this).removeClass('disabled');
	}
}

$("#btn_PTP").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');
	run_cmd_ind = $('#teach_table tr').length;
	//var tmp_num_arr = [0, 0.3, 0.2, -90, 0, 0, 0];
	var tmp_num_arr = [];
	tmp_num_arr.push(parseFloat($("#MF_block_x").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_y").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_z").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_pitch").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_roll").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_yaw").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_fai").val()));

	var vel = parseFloat($("#MF_block_spd").val());

	var Test_msg = new ROSLIB.Message({
		data : tmp_num_arr
	});
	Test_pub.publish(Test_msg);//P2P

	var Vel_msg = new ROSLIB.Message({
		data : vel
	});
	vel_pub.publish(Vel_msg);
	

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#btn_Line").click(function(){
	$(this).removeClass('active');
	$(this).addClass('disabled');
	run_cmd_ind = $('#teach_table tr').length;
	var tmp_num_arr = [];
	tmp_num_arr.push(parseFloat($("#MF_block_x").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_y").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_z").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_pitch").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_roll").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_yaw").val()));
	tmp_num_arr.push(parseFloat($("#MF_block_fai").val()));

	// console.log('in test_bnt');

	var Test_msg = new ROSLIB.Message({
		data : tmp_num_arr
	});
		Test_pub2.publish(Test_msg);//Line

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#btn_InitPos").click(function(){
	$(this).removeClass('active'); 
	$(this).addClass('disabled');
	
	// 對於Input元件來說，要用val()顯示，不能用html()
	//var tmp_num_arr = [0.3, 0, 0.2, -90, 0, 0, 0];
	var tmp_num_arr = [0.65, 0, -0.2, 0, 0, 0, 0];
	$("#MF_block_x")	.val(tmp_num_arr[0]);
	$("#MF_block_y")	.val(tmp_num_arr[1]);
	$("#MF_block_z")	.val(tmp_num_arr[2]);
	$("#MF_block_pitch").val(tmp_num_arr[3]);
	$("#MF_block_roll")	.val(tmp_num_arr[4]);
	$("#MF_block_yaw")	.val(tmp_num_arr[5]);
	$("#MF_block_fai")	.val(tmp_num_arr[6]);

	var Test_msg = new ROSLIB.Message({
		data : tmp_num_arr
	});
		Test_pub.publish(Test_msg);//P2P

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#btn_Home").click(function(){
	$(this).removeClass('active'); 
	$(this).addClass('disabled');
	run_cmd_ind = $('#teach_table tr').length;
	var joint_name_ary = ['joint1', 'joint2', 'joint3', 'joint4',  'joint5','joint6',  'joint7' ];
	var float_ary = [0, -1.57, 0, 0, 0, 0, 0];

	var cmd_msg = new ROSLIB.Message({
		name: joint_name_ary,
		value: float_ary
	});
	console.log('in Home control');

	joint_pub.publish(cmd_msg);

	$(this).addClass('active');
	$(this).removeClass('disabled');
});

$("#run_btn").click(function() {
	$(this).removeClass('active');
	$(this).addClass('disabled');

	run_cmd_ind = 0;
	next_command();

});

function next_command(){

	//var cmd_mod =$(selector).find('[name=cmd_mod]').children('select').val();
	//l('lenth='+$('#teach_table tr').length);

	if(run_cmd_ind < $('#teach_table tr').length){
		run_cmd_ind++;

		var selector = '#teach_table tr:nth-child('+run_cmd_ind+')';

		//change to arrow img
		var arrow_img = '<img src="img/right_arrow.png" align="center" style="width: 35%;"/>';
		$(selector).children("td:first").html(arrow_img);

		//restore previous to id
		if(run_cmd_ind > 1){
			var pre_selector = '#teach_table tr:nth-child('+(run_cmd_ind-1)+')';
			$(pre_selector).children("td:first").html((run_cmd_ind-1));
		}

		run_unit_command(run_cmd_ind);

		// next_command();
	}else{
		$("#run_btn").addClass('active');
		$("#run_btn").removeClass('disabled');

		//restore final cmd to id
		var pre_selector = '#teach_table tr:nth-child('+run_cmd_ind+')';
		$(pre_selector).children("td:first").html(run_cmd_ind);
	}
}

//run one command
function run_unit_command(run_cmd_ind)
{
	var selector = '#teach_table tr:nth-child('+run_cmd_ind+')';
	var cmd_mod =$(selector).find('[name=cmd_mod]').children('select').val();

	l('cmd_mod= ' + cmd_mod);

	if(cmd_mod==CmdType.Joint)
	{
		  //var joint_name_ary = ['joint1', 'joint2', 'joint3', 'joint4',  'joint5',]
			var joint_name_ary = [] ;
			var float_ary = [];

			$(selector).children("td.SubCmd").children("input").each(function(index){
				var t_float = parseFloat( $(this).val() );

				float_ary.push(t_float*_Math.DEG2RAD);
				joint_name_ary.push('joint'+ (index+1));
			});

			var cmd_msg = new ROSLIB.Message({
				name : joint_name_ary,
				value : float_ary
			});

			joint_pub.publish(cmd_msg);
	}
	else if(cmd_mod==CmdType.PTP || cmd_mod==CmdType.Line)
	{
		//-------------CmdType.PTP,Line-------------//
		//=================chg start======================================
		var refer = $(selector).children("td.SubCmd");

		var x     = parseFloat(refer.children("input:nth-child(1)").val()) ;
		var y     = parseFloat(refer.children("input:nth-child(2)").val()) ;
		var z     = parseFloat(refer.children("input:nth-child(3)").val()) ;
		var roll  = parseFloat(refer.children("input:nth-child(4)").val()) ;
		var pitch = parseFloat(refer.children("input:nth-child(5)").val()) ;
		var yaw   = parseFloat(refer.children("input:nth-child(6)").val()) ;
		var fai   = parseFloat(refer.children("input:nth-child(7)").val()) ;

		var tmp_Cmd = [x, y, z, pitch, roll, yaw, fai];
		var Test_msg = new ROSLIB.Message({
			data : tmp_Cmd
		});
		if(cmd_mod==CmdType.PTP)
			Test_pub.publish(Test_msg);
		else
			Test_pub2.publish(Test_msg);
		console.log('in task');
		//=================chg over=======================================

	//-------------CmdType.Shift_X-------------//
	} 
	else if( cmd_mod==CmdType.Shift_X  || cmd_mod==CmdType.Shift_Y  || cmd_mod==CmdType.Shift_Z  ||
			 cmd_mod==CmdType.Rotate_X || cmd_mod==CmdType.Rotate_Y || cmd_mod==CmdType.Rotate_Z ||
			 cmd_mod==CmdType.Rot_Fai)
	{
		var data = $(selector).children("td.SubCmd").children("input").val();
		Move_TCP_Rel(cmd_mod, data);
	}
	//-------------CmdType.Vaccum-------------// 
	else if (cmd_mod == CmdType.Vaccum) 
	{
		var vaccum_yn = $(selector).find('[name=vaccum_select]').val()
			== 'On'? true: false;

		var req = new ROSLIB.ServiceRequest({
			cmd: vaccum_yn? 'vacuumOn': 'vacuumOff',
		});

		vacuum_client.callService(req, function (res) {
			console.log('Is vacuum cmd success: ' + res.success);

			// rework
			// if (res.success == false)
			// 	run_cmd_ind--;

			next_command();
		});
	}
}

function Move_TCP_Rel(cmd_mod, data)
{
	run_cmd_ind = $('#teach_table tr').length;
	var request = new ROSLIB.ServiceRequest({
		group_name: "arm",
	});
	pose_client.callService(request, function (res) {
		var p = res.group_pose.position;						//position
		var o = res.group_pose.orientation;					//orientation
		var e = euler_fomr_quaternion([o.x, o.y, o.z, o.w]);	//Convert quaternion to RPY
		var f = res.group_redundancy;
		var val = parseFloat(data);
		
		e[0]=e[0]*180/3.1415926;
		e[1]=e[1]*180/3.1415926;
		e[2]=e[2]*180/3.1415926;
		f=f*180/3.1415926;

		if (cmd_mod == CmdType.Shift_X) {
			p.x += val;
		} else if (cmd_mod == CmdType.Shift_Y) {
			p.y += val;
		} else if (cmd_mod == CmdType.Shift_Z) {
			p.z += val;
		} else if (cmd_mod == CmdType.Rotate_X) {//_Math.DEG2RAD
			e[1] += val;
		} else if (cmd_mod == CmdType.Rotate_Y) {
			e[0] += val;
		} else if (cmd_mod == CmdType.Rotate_Z) {
			e[2] += val;
		} else if (cmd_mod == CmdType.Rot_Fai) {
			f += val;
		}

		var x 	  = parseFloat(p.x);
		var y 	  = parseFloat(p.y);
		var z 	  = parseFloat(p.z);
		var roll  = parseFloat(e[0]);
		var pitch = parseFloat(e[1]);
		var yaw   = parseFloat(e[2]);
		var fai   = parseFloat(f);
		var tmp_Cmd = [x, y, z, pitch, roll, yaw, fai];
		
		console.log('roll='+roll);
		console.log('pitch='+pitch);
		console.log('yaw='+yaw);

		var pose_msg = new ROSLIB.Message({
			data: tmp_Cmd
		});
		pose_pub.publish(pose_msg);
	});
}

$("#file_save_btn").click(function() {
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var save_data = "{\n";

	var cmd_count = $('#teach_table tr').length;
	$('#teach_table tr').each(function(index,element) {
		var cmd_mod = $(this).children('[name=cmd_mod]').children('select').val();
		console.log('cmd type = ' + cmd_mod);
		//-------------CmdType.Joint-------------//
		save_data += '\t"'+$(this).children('td.order').html()+'":{\n';
		if(cmd_mod==CmdType.Joint || cmd_mod==CmdType.PTP || cmd_mod==CmdType.Line){
			save_data += '\t\t"cmd": "'+cmd_mod+'",\n';	// Joint or PTP or Line START
			save_data += '\t\t"val_7": ';	  //val_7 Start
			var float_ary = [];
			 $(this).children("td.SubCmd").children("input").each(function()
		    {
		    	var t_float = parseFloat( $(this).val() );
		      float_ary.push( t_float );
		    });
			save_data += '[' + float_ary.toString()+"]\n";  //val_7 End

		//-------------CmdType.Shift-------------//
		}else if(cmd_mod==CmdType.Shift_X  || cmd_mod==CmdType.Shift_Y  || cmd_mod==CmdType.Shift_Z ||
			     cmd_mod==CmdType.Rotate_X || cmd_mod==CmdType.Rotate_Y || cmd_mod==CmdType.Rotate_Z ||
				 cmd_mod==CmdType.Rot_Fai ){
			//save_data += '\t"'+cmd_mod+'":{\n';	// Shift_X or Shift_Y or Shift_Z START
			save_data += '\t\t"cmd": "'+cmd_mod+'",\n';	// Joint or PTP or Line START
			save_data += '\t\t"val": ';	  //Val Start

			var refer = $(this).children("td.SubCmd").children("input");
			var val = refer.val();
			save_data +=  val ;  //Val End

			//console.log('refer.prop("pose")='+refer.attr('pose'));

			if(refer.attr('pose')!=undefined){
				save_data +=  ',\n';
				save_data += '\t\t"pose": ' + '[' + refer.attr('pose') +"]\n";	  //pose
			}

			save_data +=  '\n';


		}else if(cmd_mod==CmdType.Vaccum){
			//save_data += '\t"'+cmd_mod+'":{\n';	// Vaccum START
			save_data += '\t\t"cmd": "'+cmd_mod+'",\n';	// Joint or PTP or Line START
			save_data += '\t\t"val": ';	  //Val Start
			var vaccum_yn = $(this).find('[name=vaccum_select]').val()=='On' ? true:false;
			save_data +=  vaccum_yn +"\n";  //Val End

			//save_data += '\t},\n'; // Vaccum END
		}

		if(index==cmd_count-1){
			save_data += '\t}\n';
		}else{
			save_data += '\t},\n';
		}

	});

	save_data += "}";
	var request = new ROSLIB.ServiceRequest({
	    cmd : "Teach:SaveFile",
	    req_s : save_data
	});

	ui_client.callService(request, function(res) {
		console.log( 'Result : '   + res.result);
	  	$("#file_save_btn").removeClass('disabled');
			$("#file_save_btn").addClass('active');
	});

	$("#test_json_data").val(save_data);

});

$("#file_read_btn").click(function() {
	$(this).removeClass('active');
	$(this).addClass('disabled');

	var request = new ROSLIB.ServiceRequest({
	    cmd : "Teach:ReadFile"
	});

	ui_client.callService(request, function(res) {
		console.log('Result : '   + res.result);
		cmd_id = 0;

		$('#teach_table').html('');


		$("#test_json_data").val(res.res_s);

		var json = JSON.parse(res.res_s);


		for (var index in json) {
			var cmd = json[index].cmd;
			//console.log('cmd : ' + cmd);


			var tr_html = '';
			if(cmd==CmdType.PTP || cmd==CmdType.Line|| cmd==CmdType.Joint){
				tr_html = get_block_tr(cmd,json[index].val_7);

			}else if(cmd==CmdType.Vaccum){
				tr_html = get_block_tr(cmd,json[index].val);
			}else if(cmd==CmdType.Shift_X  || cmd==CmdType.Shift_Y  || cmd==CmdType.Shift_Z||
			     	 cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z||
					 cmd==CmdType.Rot_Fai){
				tr_html = get_block_tr(cmd,json[index].val,json[index].pose);
			}


			$('#teach_table').append(tr_html);
			order_list();
		}

	  $("#file_read_btn").removeClass('disabled');
		$("#file_read_btn").addClass('active');
	});

});


function parse_json_2_cmd_list(cmd,val){
	var tr_html = '';

	console.log('cmd='+cmd+',val='+val)

	if(cmd==CmdType.PTP || cmd==CmdType.Line){
		tr_html = get_block_tr(cmd,val.val_7);

	}else if(cmd==CmdType.Vaccum){
		tr_html = get_block_tr(val.Val);
	}else if(cmd==CmdType.Shift_X  || cmd==CmdType.Shift_Y  || cmd==CmdType.Shift_Z ||
			 cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z || cmd==CmdType.Rot_Fai)
	{
		tr_html = get_block_tr(cmd,val.Val);
	}
	else
	{
		return;
	}
	$('#teach_table').append(tr_html);
}

function teach_click(t){

	var m_cmd_id = $(t).parents('tr').attr('id');
	//sconsole.log("m_cmd_id="+m_cmd_id);

	var mod;

	if(m_cmd_id == undefined){
		mod = $("#cmd_select").val();
	}else{
		mod = $('#'+m_cmd_id).children('[name=cmd_mod]').children('select').val();
	}

	console.log("mod="+mod);

	var i = 0;
	if(mod==CmdType.Joint){
		var i = 0;
		console.log('in CmdType.Joint');

		var request = new ROSLIB.ServiceRequest({
		    joint_name :['joint1', 'joint2', 'joint3','joint4','joint5','joint6','joint7'],
		});

		joint_client.callService(request, function(res) {
				var refer = (m_cmd_id == undefined) ? $('#block'):$('#'+m_cmd_id).children("td.SubCmd");

				refer.children("input").each(function(){
						var v = res.joint_value[i++]*_Math.RAD2DEG;
						$(this).val(v.toFixed(2));			//ex:  0.123456789  ->  0.1234
				});
	  });

	}else if(mod==CmdType.PTP || mod==CmdType.Line){
		var request = new ROSLIB.ServiceRequest({
		    group_name : "arm",
		});

		pose_client.callService(request, function(res) {
			var p = res.group_pose.position;		//position
			var o = res.group_pose.orientation;			//orientation
			var fai = res.group_redundancy;

			var e = euler_fomr_quaternion([o.x, o.y, o.z, o.w]);

			var refer = (m_cmd_id == undefined) ? $('#block'):$('#'+m_cmd_id).children("td.SubCmd");

			var val_7 = [p.x, p.y, p.z, e[0]*_Math.RAD2DEG, e[1]*_Math.RAD2DEG, e[2]*_Math.RAD2DEG, fai*_Math.RAD2DEG];
			refer.children("input").each(function(){
					var v = val_7[i++];
					$(this).val(v.toFixed(3));			//ex:  0.123456789  ->  0.1234

			});
		});

	}else if(mod==CmdType.Shift_X  || mod==CmdType.Shift_Y  || mod==CmdType.Shift_Z||
				 mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z || mod==CmdType.Rot_Fai || mod==CmdType.Vaccum){

		alert('Cannot teach With Shift or Vaccum');
	}
}

//----------------------------------------ROS----------------------------------------//

// ------------------------------------//
// ROS for this UI
// -----------------------------------//

var ui_client = new ROSLIB.Service({
    ros : ros,
    name : '/ui_server',
    serviceType : 'mbot_control/UI_Server'
  });

var joint_client = new ROSLIB.Service({
    ros : ros,
    name : '/robotis/base/get_joint_pose',
    serviceType : 'manipulator_h_base_module_msgs/GetJointPose'
});

var pose_client = new ROSLIB.Service({
    ros : ros,
    name : '/robotis/base/get_kinematics_pose',
    serviceType : 'manipulator_h_base_module_msgs/GetKinematicsPose'
});

var set_mode_pub = new ROSLIB.Topic({
	ros : ros,
	name:'/robotis/base/set_mode_msg',
	messageType : 'std_msgs/String'
});

var Test_pub = new ROSLIB.Topic({
	ros : ros,
	name:'/robotis/base/JointP2P_msg',
	//messageType : 'std_msgs/Float64'
	messageType : 'manipulator_h_base_module_msgs/IK_Cmd'
});

var Test_pub2 = new ROSLIB.Topic({
	ros : ros,
	name:'/robotis/base/TaskP2P_msg',
	messageType : 'manipulator_h_base_module_msgs/IK_Cmd'
});

var joint_pub = new ROSLIB.Topic({
	ros : ros,
	name:'/robotis/base/Joint_Control',
	messageType : 'manipulator_h_base_module_msgs/JointPose'
});

var pose_pub = new ROSLIB.Topic({
	ros : ros,
	name:'/robotis/base/TaskP2P_msg',
	messageType : 'manipulator_h_base_module_msgs/IK_Cmd'
});

var vel_pub = new ROSLIB.Topic({
	ros : ros,
	name:'/robotis/base/set_velocity',
	messageType : 'std_msgs/Float64'
});

var vacuum_client = new ROSLIB.Service({
    ros : ros,
    name : '/robot_cmd',
    serviceType : 'vacuum_cmd_msg/VacuumCmd'
});

var status_sub = new ROSLIB.Topic({
	ros:ros,
	name: '/robotis/status',
	messageType : 'robotis_controller_msgs/StatusMsg'
});

status_sub.subscribe(function(msg){
	if(msg.status_msg=="End Trajectory"){
		l('in End Trajectory');
		next_command();
	}
});

//=====================
//Navigation Menu Slider
$('#nav-expander').on('click',function(e){
	e.preventDefault();
	$('body').toggleClass('nav-expanded');
});
$('#nav-close').on('click',function(e){
	e.preventDefault();
	$('body').removeClass('nav-expanded');
});
//=====================
$(document).ready(function(){
	$('#control_edit_joint').on('click',function(e){
		$('#dialog_table_joint').show();
		$('#dialog_table_eef').hide();
		$('#dialog_table_plus').hide();
		$('#dialog_table_base').hide();
	});
	$('#control_edit_eef').on('click',function(e){
		$('#dialog_table_eef').show();
		$('#dialog_table_joint').hide();
		$('#dialog_table_plus').hide();
		$('#dialog_table_base').hide();
	});
});
