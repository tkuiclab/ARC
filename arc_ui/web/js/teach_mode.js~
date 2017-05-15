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
	Vaccum: "Vaccum",
	Base_Init: "Base_Init",
	Base_Stop: "Base_Stop",
	Base_Vel : "Base_Vel",
	Base_Pos_Index_1: "Base_Pos_Index_1",
	Base_Pos_Index_2: "Base_Pos_Index_2",
	Base_Pos_Index_3: "Base_Pos_Index_3"
};



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
			'<option value="Vaccum"'+((cmd==CmdType.Vaccum)?"selected":"")+'>Vaccum</option>'+
			'<option value="-----------------------">----------------------</option>'+
			'<option value="Base_Init"'+((cmd==CmdType.Base_Init)?"selected":"")+'>Base_Init</option>'+
			'<option value="Base_Stop"'+((cmd==CmdType.Base_Stop)?"selected":"")+'>Base_Stop</option>'+
			'<option value="Base_Vel"'+((cmd==CmdType.Base_Vel)?"selected":"")+'>Base_Vel</option>'+
			'<option value="Base_Pos_Index_1"'+((cmd==CmdType.Base_Pos_Index_1)?"selected":"")+'>Base_Pos_Index_1</option>'+
			'<option value="Base_Pos_Index_2"'+((cmd==CmdType.Base_Pos_Index_2)?"selected":"")+'>Base_Pos_Index_2</option>'+
			'<option value="Base_Pos_Index_3"'+((cmd==CmdType.Base_Pos_Index_3)?"selected":"")+'>Base_Pos_Index_3</option>'+
			'</select>';
			
     	
	return command_select;
}

var addbtn = document.getElementById("addbtn");
var edit_img = '<img name="edit_btn" src="img/edit.png" onclick="edit_Cmd(this)"/>';
var true_img = '<img name="true_btn" src="img/true.png" class="img_show" onclick="save_Cmd(this)"/>';
var false_img = '<img name="false_btn" src="img/false.png" class="img_show" onclick="break_Cmd(this)"/>';
var delete_img = '<img name="delete_opt" src="img/delete.png" style="width:20px; height:auto; text-align:right;" onclick="delete_Cmd(this)"/>';
var teach_a = '<button name="teach_btn" class="btn btn-info btn-block" style="width:80px; display:none;" onclick="teach_click(this)"><span class="glyphicon glyphicon-pushpin"></span>Teach</button>';
var cmd_id;
	
function hide_all(){
	$("#block").hide();
	$("#vaccum_block").hide();
	$("#shift_block").hide();
	$("#base_vel_block").hide();
}

function order_list(){
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
	}else if(cmd==CmdType.Shift_X || cmd==CmdType.Shift_Y || cmd==CmdType.Shift_Z || 
				 cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z){
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
					 mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z){
			$("#shift_val").val($('#'+ev).children("td.SubCmd").children("input").val());
		}
	}
}

function get_block_tr(option_cmd,val){
	var sub_cmd = '';
	
	if(option_cmd==CmdType.Joint || option_cmd==CmdType.PTP || option_cmd==CmdType.Line){
		if(val==undefined){
			for(var i=1;i<=6;i++){
				var t_id = '#block_'+i;
				sub_cmd += '<input class="block_sty_none" type="number" value="'+$(t_id).val()+'"readonly>';
			}
		}else{
			for(var i=0;i<6;i++){
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
		     option_cmd==CmdType.Rotate_X || option_cmd==CmdType.Rotate_Y || option_cmd==CmdType.Rotate_Z){
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
		'<td>'+edit_img+
		true_img +
		false_img +
		teach_a +
		'</td>'+
		'<td style="text-align:center;">'+delete_img+'</td>'+
	'</tr>';
	
	return add_tr;
}

addbtn.onclick = function(){
	
	var cmd = $("#cmd_select").val();
	var tr_html = '';
	
	if(cmd != 'Choose') tr_html = get_block_tr(cmd);
	
	//console.log(tr_html);
	
	$('#teach_table').append(tr_html);

	$('#teach_table').scrollTop($('#teach_table')[0].scrollHeight);
	
	order_list();
}

var cmd_edit = new Array;
var vac_cmd;

function Cmd_change(edit,val_6){
	var cmd = $(edit).val();
	var sub_cmd = '';
	
	if(cmd==CmdType.Joint || cmd==CmdType.PTP || cmd==CmdType.Line){
		if(val_6==undefined){
			for(var i=1;i<=6;i++){
				var t_id = '#block_'+i;
				sub_cmd += '<input class="block_sty" type="number" value="'+$(t_id).val()+'">';
			}
		}else{
			for(var i=0;i<6;i++){
				sub_cmd += '<input class="block_sty" type="number" value="'+val_6[i]+'">\n';
			}
		}
	}else if(cmd==CmdType.Vaccum){
		sub_cmd =
			'<select class="options_cmd" name="vaccum_select">' +
				'<option value="On"'+(($('#vaccum_select').val() == "On")?"selected":"")+'>On</option>'+
				'<option value="Off"'+(($('#vaccum_select').val() == "Off")?"selected":"")+'>Off</option>' +
			'</select>';	
	}else if(cmd==CmdType.Shift_X || cmd==CmdType.Shift_Y || cmd==CmdType.Shift_Z|| 
		     cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z){
		sub_cmd = '<input class="block_sty" type="number" value="'+$('#shift_val').val()+'">';
	}
	
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
			mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z || mod==CmdType.Base_Vel
			){
				
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
			mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z ||  mod==CmdType.Base_Vel){
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
			mod==CmdType.Rotate_X || mod==CmdType.Rotate_Y || mod==CmdType.Rotate_Z || mod==CmdType.Base_Vel){
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

$("#run_btn").click(function() {
	$(this).removeClass('active');
	$(this).addClass('disabled');
	
	var mlist = [];
	
	//get each command
	$('#teach_table tr').each(function(){
		var cmd_mod = $(this).find('[name=cmd_mod]').children('select').val();
		console.log(cmd_mod);
		//-------------CmdType.Joint-------------//
		if(cmd_mod==CmdType.Joint){
			var float_ary = [];
			
			$(this).children("td.SubCmd").children("input").each(function(){
				var t_float = parseFloat( $(this).val() );
				//console.log('$(this).val()='+t_float);
				float_ary.push(t_float);
			});
			
			var cmd_msg = new ROSLIB.Message({
				cmd : CmdType.Joint,
				joint_position : float_ary
			});
			
			mlist.push(cmd_msg);
		//-------------CmdType.PTP-------------//
		}else if(cmd_mod==CmdType.PTP || cmd_mod==CmdType.Line){
		    
			var refer = $(this).children("td.SubCmd");
			var twist = new ROSLIB.Message({
				linear : {
					x : parseFloat(refer.children("input:nth-child(1)").val()),
					y : parseFloat(refer.children("input:nth-child(2)").val()),
					z : parseFloat(refer.children("input:nth-child(3)").val()),
				},
				angular : {
					x : parseFloat(refer.children("input:nth-child(4)").val()),
					y : parseFloat(refer.children("input:nth-child(5)").val()),
					z : parseFloat(refer.children("input:nth-child(6)").val()),
				}
			});
			
			console.log('twist.linear.x ='+ twist.linear.x  +',y=' + twist.linear.y + ',z=' + twist.linear.z);
		
			var cmd_msg = new ROSLIB.Message({
				cmd : cmd_mod,
				pose : twist
			});
			mlist.push(cmd_msg);
		//-------------CmdType.Shift_X-------------//
		}else if(cmd_mod==CmdType.Shift_X  || cmd_mod==CmdType.Shift_Y  || cmd_mod==CmdType.Shift_Z||
			     cmd_mod==CmdType.Rotate_X || cmd_mod==CmdType.Rotate_Y || cmd_mod==CmdType.Rotate_Z ){
		    
			var data = $(this).children("td.SubCmd").children("input").val();
			var twist = get_twist();
			
			if(cmd_mod==CmdType.Shift_X){
				twist.linear.x = parseFloat(data);
			}else if(cmd_mod==CmdType.Shift_Y){
				twist.linear.y = parseFloat(data);
			}else if(cmd_mod==CmdType.Shift_Z){
				twist.linear.z = parseFloat(data);
			}else if(cmd_mod==CmdType.Rotate_X){
				twist.angular.x = parseFloat(data);
			}else if(cmd_mod==CmdType.Rotate_Y){
				twist.angular.y = parseFloat(data);
			}else if(cmd_mod==CmdType.Rotate_Z){
				twist.angular.z = parseFloat(data);
			}
			//console.log('twist.linear.x ='+ twist.linear.x  +',y=' + twist.linear.y + ',z=' + twist.linear.z);
			console.log(cmd_mod,parseFloat(data));
			var cmd_msg = new ROSLIB.Message({
				cmd : cmd_mod,
				pose : twist
			});
			mlist.push(cmd_msg);
		//-------------CmdType.Vaccum-------------//
		}else if(cmd_mod==CmdType.Vaccum){
			var vaccum_yn = $(this).find('[name=vaccum_select]').val()=='On' ? true:false;
			
			
			var cmd_msg = new ROSLIB.Message({
				cmd : CmdType.Vaccum,
				vaccum : vaccum_yn
			});
			
			mlist.push(cmd_msg);
		}else if(cmd_mod==CmdType.Base_Vel ){
		    
			var refer = $(this).children("td.SubCmd");
			var twist = new ROSLIB.Message({
				linear : {
					x : parseFloat(refer.children("input:nth-child(1)").val()),
					y : parseFloat(refer.children("input:nth-child(2)").val()),
					z : 0,
				},
				angular : {
					x : 0,
					y : 0,
					z : parseFloat(refer.children("input:nth-child(3)").val()),
				}
			});
			
			//console.log('twist.linear.x ='+ twist.linear.x  +',y=' + twist.linear.y + ',z=' + twist.linear.z);
		
			var cmd_msg = new ROSLIB.Message({
				cmd : cmd_mod,
				pose : twist
			});
			mlist.push(cmd_msg);
		//-------------CmdType.Shift_X-------------//
		}else if(cmd_mod==CmdType.Base_Init || cmd_mod==CmdType.Base_Stop || 
				 cmd_mod==CmdType.Base_Pos_Index_1 || cmd_mod==CmdType.Base_Pos_Index_2 || cmd_mod==CmdType.Base_Pos_Index_3){
			var cmd_msg = new ROSLIB.Message({
				cmd : cmd_mod
			});
			
			mlist.push(cmd_msg);
		}
	});

	//console.log('teachModeClient='+teachModeClient);
	
	var goal = new ROSLIB.Goal({
		actionClient : teachModeClient,
		goalMessage : {
			cmd_list : mlist
		}
	});
	goal.on('feedback', teach_feedback);
	goal.on('result', teach_result);
	
	
	teach_result_trigger = false;
	now_exe_id = 0;
	
	goal.send();
});

$("#file_save_btn").click(function() {
	$(this).removeClass('active');
	$(this).addClass('disabled');
	
	var save_data = "{\n";
	
	var cmd_count = $('#teach_table tr').length;
	$('#teach_table tr').each(function(index,element) {
		var cmd_mod = $(this).children('[name=cmd_mod]').children('select').val();
		
		//-------------CmdType.Joint-------------//
		save_data += '\t"'+$(this).children('td.order').html()+'":{\n';	
		if(cmd_mod==CmdType.Joint || cmd_mod==CmdType.PTP || cmd_mod==CmdType.Line){
			save_data += '\t\t"cmd": "'+cmd_mod+'",\n';	// Joint or PTP or Line START
			save_data += '\t\t"val_6": ';	  //val_6 Start
			var float_ary = [];
			 $(this).children("td.SubCmd").children("input").each(function()
		    {
		    	var t_float = parseFloat( $(this).val() );
		      float_ary.push( t_float );
		    });
			save_data += '[' + float_ary.toString()+"]\n";  //val_6 End
			
		//-------------CmdType.Shift-------------//
		}else if(cmd_mod==CmdType.Shift_X  || cmd_mod==CmdType.Shift_Y  || cmd_mod==CmdType.Shift_Z ||
			     cmd_mod==CmdType.Rotate_X || cmd_mod==CmdType.Rotate_Y || cmd_mod==CmdType.Rotate_Z ){
			//save_data += '\t"'+cmd_mod+'":{\n';	// Shift_X or Shift_Y or Shift_Z START
			save_data += '\t\t"cmd": "'+cmd_mod+'",\n';	// Joint or PTP or Line START
			save_data += '\t\t"val": ';	  //Val Start
			
			var refer = $(this).children("td.SubCmd").children("input");
			var val = refer.val();
			save_data +=  val ;  //Val End
			
			console.log('refer.prop("pose")='+refer.attr('pose'));
			
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
		}else if(cmd_mod==CmdType.Base_Vel){
			save_data += '\t\t"cmd": "'+cmd_mod+'",\n';	//Base_Vel
			save_data += '\t\t"val_3": ';	  //val_3 Start
			var float_ary = [];
			 $(this).children("td.SubCmd").children("input").each(function()
		    {
		    	var t_float = parseFloat( $(this).val() );
		      float_ary.push( t_float );
		    });
			save_data += '[' + float_ary.toString()+"]\n";  //val_3 End
		}else if(cmd_mod==CmdType.Base_Init || cmd_mod==CmdType.Base_Stop || 
				 cmd_mod==CmdType.Base_Pos_Index_1 || cmd_mod==CmdType.Base_Pos_Index_2 || cmd_mod==CmdType.Base_Pos_Index_3){
			save_data += '\t\t"cmd": "'+cmd_mod+'"\n';	//Base_Vel
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
				tr_html = get_block_tr(cmd,json[index].val_6);
				
			}else if(cmd==CmdType.Vaccum){
				tr_html = get_block_tr(cmd,json[index].val);
			}else if(cmd==CmdType.Shift_X  || cmd==CmdType.Shift_Y  || cmd==CmdType.Shift_Z||
			     	 cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z 
					){
				tr_html = get_block_tr(cmd,json[index].val,json[index].pose);
			}else if(cmd==CmdType.Base_Vel){
				
				tr_html = get_block_tr(cmd,json[index].val_3);
				
			}else if(cmd==CmdType.Base_Init || cmd==CmdType.Base_Stop || 
				 cmd==CmdType.Base_Pos_Index_1 || cmd==CmdType.Base_Pos_Index_2 || cmd==CmdType.Base_Pos_Index_3
				 ){
				tr_html = get_block_tr(cmd);
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
		tr_html = get_block_tr(cmd,val.val_6);
		
	}else if(cmd==CmdType.Vaccum){
		tr_html = get_block_tr(val.Val);
	}else if(cmd==CmdType.Shift_X  || cmd==CmdType.Shift_Y  || cmd==CmdType.Shift_Z ||
			 cmd==CmdType.Rotate_X || cmd==CmdType.Rotate_Y || cmd==CmdType.Rotate_Z ){
		tr_html = get_block_tr(cmd,val.Val);
	}else{
		return;
	}
	$('#teach_table').append(tr_html);
}

function teach_click(t){
	
	var m_cmd_id = $(t).parents('tr').attr('id');
	console.log("m_cmd_id="+m_cmd_id);

	var mod;

	if(m_cmd_id == undefined){
		mod = $("#cmd_select").val();
	}else{
		mod = $('#'+m_cmd_id).children('[name=cmd_mod]').children('select').val();
	}

	//console.log("mod="+mod);

	var i = 0;
	if(mod==CmdType.Joint){
		var i = 0;
		$('#'+m_cmd_id).children("td.SubCmd").children("input").each(function(){
			//$(this).val(joint_ary[i++].toFixed(5));			//toFixed ex:  0.123456789  ->  0.1234
			console.log(joint_ary[i]);
			$(this).val(joint_ary[i++]);			//ex:  0.123456789  ->  0.1234
			
		});
	}else if(mod==CmdType.PTP || mod==CmdType.Line){
		var request = new ROSLIB.ServiceRequest({
		    cmd : "Teach:EEF_Pose",
		});
		
		ui_client.callService(request, function(res) {
			var l = res.pose.linear;
			var a = res.pose.angular;
			
			console.log( 'Result : '   + res.result);
			console.log( 'Pose : '   + l.x + "," + l.y + "," + l.z + "," + a.x + "," + a.y + "," + a.z );

		  	if(m_cmd_id == undefined){
		  		var val_6 = [l.x,l.y,l.z,a.x,a.y,a.z];
		  		if(mod != 'Choose') tr_html = get_block_tr(mod,val_6);
				$('#teach_table').append(tr_html);
			}else{
				var refer = $('#'+m_cmd_id).children("td.SubCmd");
				refer.children("input:nth-child(1)").val(l.x.toFixed(2));
				refer.children("input:nth-child(2)").val(l.y.toFixed(2));
				refer.children("input:nth-child(3)").val(l.z.toFixed(2));
				refer.children("input:nth-child(4)").val(a.x.toFixed(2));
				refer.children("input:nth-child(5)").val(a.y.toFixed(2));
				refer.children("input:nth-child(6)").val(a.z.toFixed(2));
			}
		});
	
	}else if(mod==CmdType.Shift_X || mod==CmdType.Shift_Y || mod==CmdType.Shift_Z){
		
		var find = false;
		
		var now_cmd_id = m_cmd_id;
		var request;
		var pre_pose = [];
		
		console.log(m_cmd_id);
		do{
			var pre_id = $('#'+now_cmd_id).prev("tr").prop("id");
			if(pre_id==undefined){
				console.error('Teach Click -> Cannot find previous position');
				return;
			}
			
			//get previous tr's command
			var pre_mod = $('#'+pre_id).children('[name=cmd_mod]').children('select').val();
			
			if(pre_mod==CmdType.PTP || pre_mod==CmdType.Line){
				var refer = $('#'+pre_id).children("td.SubCmd");
				

				refer.children('input').each(function()
			    {
			    	var t_float = parseFloat( $(this).val() );
			    	
			        pre_pose.push( t_float );
			    });
				find = true;
			}else if(pre_mod==CmdType.Shift_X || pre_mod==CmdType.Shift_Y || pre_mod==CmdType.Shift_Z){
				
				var pre_pose_str =  $('#'+pre_id).children("td.SubCmd").children("input").attr('pose');
				console.log('pre_pose_str='+pre_pose_str);
				
				if(pre_pose_str!=undefined){
				
					var pre_pose_ary = pre_pose_str.split(",");
					for(var ind in pre_pose_ary){
						console.log('str='+pre_pose_ary[ind]);
						pre_pose.push( parseFloat( pre_pose_ary[ind] ) );
					}
					find = true;
				}
			}
			now_cmd_id = pre_id;
		}while(!find);
		
		if(!find)	return;
		
		var twist = new ROSLIB.Message({
		    linear : {
		      x : pre_pose[0],
		      y : pre_pose[1],
		      z : pre_pose[2]
		    },
		    angular : {
		      x : pre_pose[3],
		      y : pre_pose[4],
		      z : pre_pose[5]
		    }
		});
		//console.log('twist.linear.x ='+ twist.linear.x  +',y=' + twist.linear.y + ',z=' + twist.linear.z);
		
		request = new ROSLIB.ServiceRequest({
		    cmd : "Teach:" + mod,
		    pose : twist
		});
		
		//client call service
		ui_client.callService(request, function(res) {
	  		var shift = res.f.toFixed(3);
	  		$('#'+m_cmd_id).children("td.SubCmd").children("input").val(shift);;
	  		
	  		var now_pose = pre_pose;
				
	  		if     (mod==CmdType.Shift_X){  	now_pose[0] += parseFloat(shift);		now_pose[0] = now_pose[0].toFixed(3);	}
	  		else if(mod==CmdType.Shift_Y){  	now_pose[1] += parseFloat(shift);		now_pose[1] = now_pose[1].toFixed(3);	}
	  		else if(mod==CmdType.Shift_Z){  	now_pose[2] += parseFloat(shift);		now_pose[2] = now_pose[2].toFixed(3);	}
			
			//console.log('new_now_pose='+now_pose.toString() );
			$('#'+m_cmd_id).children("td.SubCmd").children("input").attr('pose',now_pose.toString());
		});
	}	
}

//----------------------------------------ROS----------------------------------------//
// Connecting to ROS
var ros = new ROSLIB.Ros({
	url : 'ws://192.168.5.80:9090'
});

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {var request = new ROSLIB.ServiceRequest({
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
	console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
	console.log('ROS Connection made!');
});

ros.on('close', function() {
    console.log('ROS Connection closed.');
});

//-----------ActionClient-------------//
var teachModeClient = new ROSLIB.ActionClient({
	ros : ros,
	serverName : '/mbot_control',
	actionName : 'mbot_control/TeachCommandListAction'
});


var now_exe_id = 0;
var teach_result_trigger = false;

function teach_feedback(feedback){
	if(teach_result_trigger)	return;
	console.log('Feedback: ' + feedback.status);
	
	var arrow_img = '<img src="img/right_arrow.png" align="center" style="width: 35%;"/>';
	
	//get cmd_id
	var fb_str = feedback.status;
	var str_index = fb_str.indexOf('->');
	var exe_id = fb_str.substring(str_index+2,fb_str.length);
	var m_cmd_id = 'cmd_'+exe_id;
	console.log('m_cmd_id=' + m_cmd_id);
	//change to arrow_img
	$('#'+m_cmd_id).children("td:first").html(arrow_img);
	
	
	if(now_exe_id!=0){
		m_cmd_id = 'cmd_'+now_exe_id;
		//change to number
		$('#'+m_cmd_id).children("td:first").html(now_exe_id.pad(3));	
	}
	
	now_exe_id = parseInt(exe_id);
}

function teach_result(result){
	
	console.log('Final Result: ' + result.notify);
	$("#run_btn").removeClass('disabled');
	$("#run_btn").addClass('active');
	
	
	/*
	$('#teach_table tr').each(function() {
		$(this).children("td:first").html(now_exe_id.pad(3));	
	});*/
	
	
	teach_result_trigger = true;
	
	
	if(now_exe_id!=0){
		m_cmd_id = 'cmd_'+now_exe_id;
		//change to number
		$('#'+m_cmd_id).children("td:first").html(now_exe_id.pad(3));	
	}
	
	
}


// ------------------------------------//
// Subscribing to a "joint_states" Topic
// -----------------------------------//

var joint_ary = new Float32Array(6);;

var joint_sub = new ROSLIB.Topic({
	ros:ros,
	name: '/joint_states',
	messageType : 'sensor_msgs/JointState'
});


joint_sub.subscribe(function(msg){
	for(var i =0 ;i < 6;i++){
		joint_ary[i] = msg.position[i];
	}
});	



// ------------------------------------//
// Client for a "ui_server" Service
// -----------------------------------//


var ui_client = new ROSLIB.Service({
    ros : ros,
    name : '/ui_server',
    serviceType : 'mbot_control/UI_Server'
  });


/*  Teach:SaveFile test
var request = new ROSLIB.ServiceRequest({
    cmd : "Teach:SaveFile",
    req_s : "hello\niam from web\nhello\n"
});

ui_client.callService(request, function(res) {
	console.log( 'Result : '   + res.result);
  		
});
*/

/* Teach:ReadFile test
var request = new ROSLIB.ServiceRequest({
    cmd : "Teach:ReadFile"
});

ui_client.callService(request, function(res) {
	console.log( 'res_s : '   + res.res_s);
  	
	console.log( 'Result : '   + res.result);
  		
});
*/

