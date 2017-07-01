import React from 'react';
import ROSLIB from 'roslib';
import ReactDOM from 'react-dom';
import {Pick,Stow} from './arc.jsx';
//import Stow from './stow.jsx';
import './index.css';
import {use_default_ros_connet} from './ros_init.js';


//var item_location = require('json!item_location_file.json');
import item_location_json from './empty_item_location_file.json';
//import order_json from './empty_order.json';

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

var TaskType = {
  Pick : "Pick", 
  Stow: "Stow" 
};

class ROS extends React.Component {
  //_pick from render
  
  constructor(props) {
    super(props);
    this.json_cb = this.json_cb.bind(this);
    
    this.pick_btn_click = this.pick_btn_click.bind(this);
    this.stow_btn_click = this.stow_btn_click.bind(this);
    
    this.state = {
      task_type: props.task_type,
      pre_info : ''
    }

    this.test_bind();
  }

  componentDidMount(){
    
    status_sub.subscribe(
      function(msg){
          // var info  = msg.data;
          
          var json = JSON.parse(msg.data);
          var info = json.info;

          if(this.state.pre_info === info) return;

          this.state.pre_info = info;
          l('json='+JSON.stringify(json));

          this._storage.set_status(json.info);                 

          if(info.indexOf("(GoBin)")!==-1){
            this._storage.change_focus_item(json.item);
          }else if(info.indexOf("(GoBox)")!==-1 && 
                  info.indexOf("[Success]")!==-1
            ){
            this._storage.box_add_item(json.box,json.item);
            this._storage.shelf_remove_item(json.item);
            this._storage.change_focus_item('');
          }else if(info.indexOf("(GoTote)")!==-1){
            this._storage.tote_focus_item(json.item,json.bin);
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
  

  pick_btn_click(){
    
    this.setState({
        task_type: TaskType.Pick
    });
  }

  stow_btn_click(){
    this.setState({
        task_type: TaskType.Stow
    });
  }

  stow_run_cb(){
    l('stow_run_cb()');

    var request = new ROSLIB.ServiceRequest({
        task_name : 'stow_run',
        task_json : ''
    });
    task_client.callService(request);

  }
  

  render() {
    l('ROS Render');
     
    let pick_btn_css = (this.state.task_type===TaskType.Pick) ?  `btn brown_btn_use`: `btn brown_btn`;
    let stow_btn_css = (this.state.task_type===TaskType.Stow) ?  `btn red_btn_use`: `btn red_btn`;
  
    //pick_btn_css = `btn brown_btn`;
    //stow_btn_css = `btn red_btn`;

    // l('pick_btn_css = '+ pick_btn_css);
    // l('stow_btn_css= ' + stow_btn_css);

    return (
      
      <div>
       
        <div className="top_area">
          <button onClick={this.pick_btn_click} className={pick_btn_css}>Pick</button>
          <button onClick={this.stow_btn_click} className={stow_btn_css}>Stow</button>
        </div>

        <div>
             {this.state.task_type === TaskType.Pick ? (
               //------Pick------//
                    <Pick 
                        item_location={item_location_json} 
                        //order={order_json} 
                        remove_items={[]}
                        //focus_item="irish_spring_soap"
                        ref={(child) => { 
                            //this._pick = child; 
                            this._storage = child; 
                        }} 
                        run_cb={this.pick_run_cb}
                        json_cb={this.json_cb}
                    />
             
              ) :this.state.task_type === TaskType.Stow ? (
               //------Stow------//
                    <Stow 
                        item_location={item_location_json} 
                        remove_items={[]}
                        //focus_item="irish_spring_soap"
                        ref={(child) => { 
                            this._storage = child; 
                        }} 
                        run_cb={this.stow_run_cb}
                        json_cb={this.json_cb}
                    />
                ) : null}
            
          <button onClick={this.box_add_item_btn_click}>box_add_item_btn</button>
          <button onClick={this.shelf_remove_item_btn_click}>shelf_remove_item_btn</button>
          <button onClick={this.change_focus_item_btn_click}>change_focus_item_btn</button>
          <button onClick={this.set_status_btn_click}>set_status_btn</button>

          <button onClick={this.tote_test}>tote_test</button>
        </div>
      </div>
    );
  }

  //-------Test------------//
  test_bind(){
    this.box_add_item_btn_click = this.box_add_item_btn_click.bind(this);
    this.shelf_remove_item_btn_click = this.shelf_remove_item_btn_click.bind(this);
    this.change_focus_item_btn_click = this.change_focus_item_btn_click.bind(this);
    this.set_status_btn_click = this.set_status_btn_click.bind(this);
    this.tote_test = this.tote_test.bind(this);
    

  }

  tote_test(){
    //this._storage.tote_rm_item('balloons');
    this._storage.tote_focus_item('balloons');
  }

  box_add_item_btn_click(){
    
    this._storage.box_add_item('1A5','tissue_box');
    
  }

  shelf_remove_item_btn_click(){
    
    this._storage.shelf_remove_item('tissue_box');
  
  }

  change_focus_item_btn_click(){
    this._storage.change_focus_item('band_aid_tape');
  }

  set_status_btn_click(){
    this._storage.set_status('HI');
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



}


ReactDOM.render(
  <ROS task_type={TaskType.Pick} />, 
  document.getElementById("root"));
