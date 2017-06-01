import React from 'react';
import ROSLIB from 'roslib';
import ReactDOM from 'react-dom';
import Pick from './arc.jsx';
import './index.css';
import {use_default_ros_connet} from './ros_init.js';


//var item_location = require('json!item_location_file.json');
import item_location_json from './empty_item_location_file.json';
import order_json from './empty_order.json';

//var fs2 = require('fs');
var l = console.log;


var ros = use_default_ros_connet();
var status_sub = new ROSLIB.Topic({
      ros:ros,
      name: '/stratege/info',
      messageType : 'std_msgs/String'
    });


var task_client = new ROSLIB.Service({
    ros : ros,
    name : '/task',
    serviceType : 'stratege/Task'
});

//var rm_items_ary=['mesh_cup','hand_weight'];

class ROS extends React.Component {
  //_pick from render
  
  constructor(props) {
    super(props);
    this.box_add_item_btn_click = this.box_add_item_btn_click.bind(this);
    this.shelf_remove_item_btn_click = this.shelf_remove_item_btn_click.bind(this);
    this.change_focus_item_btn_click = this.change_focus_item_btn_click.bind(this);
    this.set_status_btn_click = this.set_status_btn_click.bind(this);
    this.json_cb = this.json_cb.bind(this);

    
  }

  componentDidMount(){
    
    
    status_sub.subscribe(
      function(msg){
          // var info  = msg.data;
          
          var json = JSON.parse(msg.data);
          this._pick.set_status(json.info);

          var info = json.info;

          if(info.indexOf("(GoBin)")!==-1){
            this._pick.change_focus_item(json.item);
          }else if(info.indexOf("(GoBox)")!==-1 && 
                  info.indexOf("[Success]")!==-1
            ){
            this._pick.box_add_item(json.box,json.item);
            this._pick.shelf_remove_item(json.item);
            this._pick.change_focus_item('');
          }

       }.bind(this)
    );

  }

  json_cb(task_cmd, json){
    l('json_cb say task_cmd = '+ task_cmd);

    var request = new ROSLIB.ServiceRequest({
        task_name : task_cmd,
        task_json : JSON.stringify(json)
    });
    task_client.callService(request);

  }

  pick_run_cb(){
    l('pick_run_cb()');

    var request = new ROSLIB.ServiceRequest({
        task_name : 'pick_run',
        task_json : ''
    });
    task_client.callService(request);

  }

  box_add_item_btn_click(){
    
    this._pick.box_add_item('1A5','tissue_box');
    
  }

  shelf_remove_item_btn_click(){
    
    this._pick.shelf_remove_item('tissue_box');
  
  }

  change_focus_item_btn_click(){
    this._pick.change_focus_item('band_aid_tape');
  }

  set_status_btn_click(){
    this._pick.set_status('HI');
    if(ROSLIB!==undefined){
      var Test_pub2 = new ROSLIB.Topic({
        ros : ros,
        name:'/test',
        messageType : 'std_msgs/String'
      });
        var msg = new ROSLIB.Message({
          data : "react"
        });
        Test_pub2.publish(msg);
    }
  }



  render() {

    return (
      <div>
          <Pick 
            item_location={item_location_json} 
            order={order_json} 
            remove_items={[]}
            //boxes={boxes}
            //focus_item="irish_spring_soap"
             ref={(child) => { 
                 this._pick = child; 
            }} 
            run_cb={this.pick_run_cb}
            json_cb={this.json_cb}
         />

         <button onClick={this.box_add_item_btn_click}>box_add_item_btn</button>
         <button onClick={this.shelf_remove_item_btn_click}>shelf_remove_item_btn</button>
         <button onClick={this.change_focus_item_btn_click}>change_focus_item_btn</button>
         <button onClick={this.set_status_btn_click}>set_status_btn</button>
      </div>
    );
  }

}


var a = ReactDOM.render(
  <ROS />, document.getElementById("root"));
