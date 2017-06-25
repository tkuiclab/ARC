import React from 'react';
import ReactDOM from 'react-dom';
import Pick from './arc.jsx';
import './index.css';
// import 'roslib/eventemitter2.min.js';
import ROSLIB from 'roslib';
import './ros_init.js';
//import EventEmitter2 from 'eventemitter2';
//import $ from 'jquery';
//import ros from './ros_init.js';

//var item_location = require('json!item_location_file.json');
import item_location_json from './example_pick_task/item_location_file.json';
import order_json from './example_pick_task/order_file.json';

//var fs2 = require('fs');
var l = console.log;

//l("item_location_file.bins = " + item_location_file.bins );
//l("item_location_file.bins[0].contents = " + item_location_file.bins[0].contents );

var rm_items_ary=['mesh_cup','hand_weight'];
var box1 = {
      "size_id": "A1",
      "contents": [
          "mouse_traps",
          "irish_spring_soap",
          "laugh_out_loud_jokes"
      ]
    };

var box2 = {
    "size_id": "1A5",
    "contents": [
        "tennis_ball_container",
        "mesh_cup",
        "band_aid_tape"
    ]
};

var box3 = {
  "size_id": "A1",
  "contents": [
      "mouse_traps",
      "irish_spring_soap",
      "laugh_out_loud_jokes"
  ]
};

var boxes = {
    box1 : box1,
    box2 : box2,
    box3 : box3
}



class Test extends React.Component {
  //_pick from render
  
  constructor(props) {
    super(props);
    this.box_add_item_btn_click = this.box_add_item_btn_click.bind(this);
    this.shelf_remove_item_btn_click = this.shelf_remove_item_btn_click.bind(this);
    this.change_focus_item_btn_click = this.change_focus_item_btn_click.bind(this);
    this.set_status_btn_click = this.set_status_btn_click.bind(this);

     this.state = {
       win: this.props.win
     }
    
    //this.emitter = new EventEmitter2();
   
    
    //this.emitter.on("RRR", this.set_status_btn_click);
  }

  componentDidMount() {
    l('componentDidMount');
    this.state.win.test();
      // var that = this, p = this.props, ee = p.eventEmitter;
      // //ee.on(p.startEvent, function(){
      // ee.on("RRR", function(){
      //   l('inininininin');
      //   //that.setState({style: {display:'block'}});
      // }.bind(this));
      // // ee.on(p.endEvent, function(){
      // //   that.setState({style: {display:'none'}});
      // // });


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
    if(ROSLIB!=undefined){
      // var Test_pub2 = new ROSLIB.Topic({
      //   ros : ros,
      //   name:'/test',
      //   messageType : 'std_msgs/String'
      // });
      //   var msg = new ROSLIB.Message({
      //     data : "hahaha"
      //   });
      //   Test_pub2.publish(msg);
    }
  }



  render() {

    return (
      <div>
          <Pick 
            item_location={item_location_json} 
            order={order_json} 
            remove_items={rm_items_ary}
            boxes={boxes}
            focus_item="irish_spring_soap"
             ref={(child) => { 
                 this._pick = child; 

            }} 
         />

         <button onClick={this.box_add_item_btn_click}>box_add_item_btn</button>
         <button onClick={this.shelf_remove_item_btn_click}>shelf_remove_item_btn</button>
         <button onClick={this.change_focus_item_btn_click}>change_focus_item_btn</button>
         <button onClick={this.set_status_btn_click}>set_status_btn</button>
      </div>
    );
  }

}

// var HelloElement = React.createElement(Hello, {
//     name: "World"
// });

// var HelloRendered = React.render(HelloElement, document.getElementById('container'));
// ReactDOM.render(
//   <Pick 
//     item_location={item_location_json} 
//     order={order_json} 
//     remove_items={rm_items_ary}
//     boxes={boxes}
//     />, document.getElementById("root"));


var a = ReactDOM.render(
  <Test  eventEmitter={ kartik_cool} win={window}/>, document.getElementById("root"));
