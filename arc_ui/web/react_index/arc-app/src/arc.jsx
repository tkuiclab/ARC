import React from 'react';
//import ReactDOM from 'react-dom';
import './index.css';

var l = console.log;


function JsonUpload(props) {
  // textInput must be declared here so the ref callback can refer to it
  let data_cb = props.data_cb;


  function openFile(event) {
        var input = event.target;

  
        var fReader = new FileReader();           
        fReader.onload = function (evt) {
            var fileString = evt.target.result;
      
            //console.log('JsonUpload get file.name  = ' + file.name);
            //console.log('JsonUpload get evt.target.fileName = ' + evt.target.fileName);

            //console.log('JsonUpload get evt.target.result = ' + fileString);
            
           
            data_cb( JSON.parse(fileString) );
        };
        fReader.readAsText(input.files[0]);
    
  }

  return (
      <input type="file" name="pick_fname" onChange={openFile}  accept=".json" />
  );
}




function Box(props) {
  var items = '';
  
  if(props.contents != null){
     items = props.contents.map((val, index) => {
        return (
          <li key={index}>
            {val}
          </li>
        );
    });
  }

  return (
    <div className="box">
      <div className="box_name">{props.size_id}</div>
      <div >
        <ol>{items}</ol>
      </div>
    </div>
  );
}
  


function Bin(props) {
  var bin = '';
  var check_order_fun = props.check_order;
  var check_remove_fun = props.check_remove;
  var check_focus_fun = props.check_focus;
  
  
  if(props.value != null){
     bin = props.value.map((val, index) => {

    
        //rm_item class
        var rm_item = check_remove_fun(val) ? 'rm_item' : '';
        var check_focus_css = check_focus_fun(val) ? 'focus_item' : '';

        //mark class
        var mark = '';
        var check_result = check_order_fun(val);
        if(check_result.in_order){
          mark = 'order_mark';

          val = val + ' -> ' + check_result.box;
        }

        //combine rm_item class & mark class
        let cssClasses = `${mark} ${rm_item} ${check_focus_css}`;



        return (
          <li key={index} className={cssClasses}>
            {val}
          </li>
        );
    });
  }

  return (
    <div className="bin">
       <ol>{bin}</ol>
    </div>
  );
}

function Shelf(props) {
  let item_location = props.item_location;
  let order_file = props.order;
  let remove_items = props.remove_items;
  let focus_item = props.focus_item;

  //l('shelf () props.focus_item= '+props.focus_item);
  //l('shelf () focus_item= '+focus_item);
  // function shouldComponentUpdate(nextProps, nextState) {
  //   l('in shouldComponentUpdate');
  //   if (this.props.color !== nextProps.color) {
  //     return true;
  //   }
  //   if (this.state.count !== nextState.count) {
  //     return true;
  //   }
  //   return false;
  // }

  function check_in_order(item){

    for(var i = 0; i < order_file.orders.length; i++){
      var order = order_file.orders[i];
      for( var ind in order.contents){
          if(order.contents[ind] === item){
              return {
                  in_order: true,
                  box: order.size_id
              };
          }
      }
    }

    return {
        in_order: false,
        box: null
    };

  }

  function check_remove(item){
    if(remove_items===undefined){
      return false;
    }
    return remove_items.includes(item);

  }

  function check_focus(item){
    //l('focus_item='+focus_item+', i_item='+item);

    return (focus_item===item);
  }


  function renderBin(i) {
    if( item_location.bins === undefined ||
      item_location.bins.length <= i){
      return (
        <Bin  />
      );
    }else{
      return (
      <Bin
          value={item_location.bins[i].contents}
          check_order={check_in_order}
          check_remove={check_remove}
          check_focus={check_focus}
      />
      );
    }

  }

 


  //l('shelf render all');

  return (
    <div>
      <div className="shelf-row">
        {renderBin(0)}
        {renderBin(1)}
        {renderBin(2)}
      </div>
      <div className="shelf-row">
        {renderBin(3)}
        {renderBin(4)}
        {renderBin(5)}
      </div>
      <div className="shelf-row">
        {renderBin(6)}
        {renderBin(7)}
        {renderBin(8)}
      </div>
      <div className="shelf-row">
        {renderBin(9)}
        {renderBin(10)}
        {renderBin(11)}
      </div>
    </div>
  );
  
}



class Pick extends React.Component {
  
  constructor(props) {
    super(props);

    //l('Pick::constructor() this.props.focus_item='+this.props.focus_item);

    var m_boxes = (this.props.order!= undefined) ? this.get_box_from_order(this.props.order) : '';
    this.state = {
       item_location: props.item_location,
       order: props.order,
       rm_items: props.remove_items,
       //boxes: this.props.boxes,
       boxes: m_boxes,
       focus_item: props.focus_item,
       status: props.status,
       json_cb: props.json_cb,
       run_cb: props.run_cb,
      //  task_client: props.ros_task_client
    };

    //bind function
    this.item_location_change = this.item_location_change.bind(this);
    this.order_change = this.order_change.bind(this);
    //this.boxes_change = this.boxes_change.bind(this);
  }

  // run_cb(){
  //   l('in pick run_cb()');
  //   this.state.run_cb();
  // }

  item_location_change(json){
    //l('in Pick item_location_change data = ' + json);
    this.setState({
        item_location: json,
        boxes: this.get_box_from_item_location_file(json)
    });

    this.state.json_cb('pick_json_item_location',json);
  }


  get_box_from_order(json){
      
      if(json.orders.length >= 3){
          var parse_boxes = {
              box1 : json.orders[0],
              box2 : json.orders[1],
              box3 : json.orders[2]
          }

          var clone_json = JSON.parse(JSON.stringify(json)); //close json object here


          var parse_boxes = {
              box1 : clone_json.orders[0],
              box2 : clone_json.orders[1],
              box3 : clone_json.orders[2]
          }

          parse_boxes.box1.contents = [];
          parse_boxes.box2.contents = [];
          parse_boxes.box3.contents = [];

          return parse_boxes;
      }else{
        l('Pick::get_box_from_order() say json.orders.length < 3')

      }

  }


  get_box_from_item_location_file(json){
      
      if(json.boxes.length >= 3){
          var clone_json = JSON.parse(JSON.stringify(json)); //close json object here

          var parse_boxes = {
              box1 : clone_json.boxes[0],
              box2 : clone_json.boxes[1],
              box3 : clone_json.boxes[2]
          }

          return parse_boxes;
      }else{
        l('Pick::get_box_from_order() say json.orders.length < 3')

      }

  }

  order_change(json){
    this.setState({
        order: json,
    });

    this.state.json_cb('pick_json_order',json);
  }


  shelf_remove_item(item){
    this.state.rm_items.push(item);

    this.setState({
        rm_items: this.state.rm_items
    });
  }


  box_add_item(box_id, item){
     for(var key in this.state.boxes){
        var box = this.state.boxes[key];

        if(box.size_id === box_id){
            if(box.contents.indexOf(item) == -1){
              box.contents.push(item);
            }
        }
    }
    

    this.setState({
        boxes: this.state.boxes,
    });

   
  }

  change_focus_item(item){
    
    this.setState({
        focus_item: item,
    });
    
  }


  set_status(s){
    //l('in set_status s='+s)
    this.setState({
        status: s,
    });
    
  }


  render() {
    //l('Pick render() this.state.item_location = ' + JSON.stringify(this.state.item_location)  );
    

    return (
      <div>
          <div className="upload_and_status">
            
            <div className="upload_area">
                <div className="upload"> Item Location File: <JsonUpload data_cb={this.item_location_change} /> </div>
                <div className="upload"> Order File: <JsonUpload data_cb={this.order_change} /> </div>
          
                <button onClick={this.state.run_cb} >Run</button>
            </div>
            <div className="task_status">
              {this.state.status}
            </div>
            
          </div>
          <div className="shelf_and_box">
            <div>
              <Shelf  item_location={this.state.item_location} 
                      order={this.state.order} 
                      remove_items={this.state.rm_items}  
                      focus_item={this.state.focus_item}
              />
            </div>
            
            <div className="box_area">
              <Box size_id={this.state.boxes.box1.size_id} contents={this.state.boxes.box1.contents} />
              <Box size_id={this.state.boxes.box2.size_id} contents={this.state.boxes.box2.contents} />
              <Box size_id={this.state.boxes.box3.size_id} contents={this.state.boxes.box3.contents} />
            </div>
          </div>

           
      </div>
    );
  }
}

export default Pick;
