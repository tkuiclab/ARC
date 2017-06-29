import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';
import bin_spec from './bin_spec.json';
import arc_items from './arc_items.json';

var l=console.log;

var ARC_Items = "arc_items/";

function mm2cm(v){
  return parseFloat(v*100).toPrecision(2);
}
function Item(props) {
  
  var fit_bins=[];
  var bins = bin_spec.bins;
  for(var ind in bins){
      var bin = bins[ind];
      var bin_dims = bin.dimensions.slice();  //slice for clone
      var item_dims = props.dims.slice();  //slice for clone
      
      bin_dims.sort();
      item_dims.sort();
      //l(bin.id + '->' +  bin_dims);
      //l(props.name + '->' +  props.dims);
      

      if(item_dims[0] < bin_dims[0] &&
        item_dims[1] < bin_dims[1] && 
       item_dims[2] < bin_dims[2]  ){
        fit_bins.push(bin.id);

       } 
  }
  

  var dims_str =  mm2cm(props.dims[0]) + " x " + mm2cm(props.dims[1]) + " x " + mm2cm(props.dims[2]);
  return (
    <div className="item">
        <div className="item_pic"> <img  src={props.img_src} /></div>
        <div className="item_name"> {props.name}</div>
         <div className="item_type"> {props.type}</div>
        <div className="item_dims"> {dims_str}</div>
        <div className="item_fix_box">{fit_bins} </div>
    </div>
    
  );
}

function Bin(props) {
  var len = props.dims[0] * 100;
  var w = props.dims[1] * 100;
  var h = props.dims[2] * 100;
  var vol = (len) * (w) * (h);
  
  function to_parent(e){
      
    var t = e.target;

    if(t.name === "L")        len = t.value;
    else if(t.name === "W")   w = t.value;
    else if(t.name === "H")   h = t.value;

    var dims=[len,w,h];
    props.on_chage(props.name,dims);
  }

  return (
    <div className="bin">
        <div className="bin_name"> {props.name}</div>
        <div className="bin_length"><input name="L" type="text" value={len} onChange={to_parent}/></div>
        <div className="bin_width"><input  name="W" type="text" value={w} onChange={to_parent}/> </div>
        <div className="bin_height"><input name="H" type="text" value={h} onChange={to_parent}/> </div>
        <div className="bin_vol">{vol}</div>
    </div>
    
  );
  
}

class BinSpec extends React.Component {
  
  constructor(props) {
    super(props);

    this.state = {
        bin_spec: props.bin_spec
    };
    //bind function
    this.input_chage = this.input_chage.bind(this);
  }

  input_chage(bin_name,dims){
    l("name:" + bin_name + " dims=" + dims);
    var bins = this.state.bin_spec.bins;
    for(var ind in bins){
         var bin = bins[ind];
         if(bin_name===bin.id){
            bin.dimensions = [dims[0] / 100 , dims[1] / 100 ,dims[2] / 100 ];
            break;
         }
    }
    
    this.setState({
        bin_spec: this.state.bin_spec
    });
  }

  cal_sum_volume(){
    var sum = 0;
    var bins = this.state.bin_spec.bins;
    for(var ind in bins){
        var bin = bins[ind];
        var vol = bin.dimensions[0] * bin.dimensions[1] * bin.dimensions[2];
        sum += vol;
    }
    return sum;
  }

  

  render() {
    //var vol_sum = parseInt(this.cal_sum_volume());
    var vol_sum = this.cal_sum_volume();
    var show_vol_sum = parseInt(vol_sum* 1000000);
    var bin_rows = [];
    var item_rows = [];
    var bins = this.state.bin_spec.bins;
    var items = arc_items.items;
    //this.all_item_2_item_list();

    //get all bins
    for(var ind in bins){
        var bin = bins[ind];
        bin_rows.push( <Bin key={ind} name={bin.id} dims={bin.dimensions} on_chage={this.input_chage}/>);
    }

    for(var ind in items){
        var it = items[ind];
        item_rows.push(<Item name={it.name}  dims={it.dimensions} img_src={it.img} type={it.type} /> );
    }

    //get all items


    var t_path = ARC_Items + "Avery_Binder/Avery_Binder_Bottom_01.png"
    l('t_path = ' + t_path);

    var t_dims = [22,33,44];
    //var t_img_src = require(t_path);

    return (
      
    <div className="main_row">
      <div className="bin_area">
          Unit: cm
        <div className="bin">
            <div className="bin_name">ID</div>
            <div className="bin_length">L </div>
            <div className="bin_width">W </div>
            <div className="bin_height">H </div>
            <div className="bin_vol">V </div>
        </div>
        {bin_rows}

        <div className="bin_sum_vol">{show_vol_sum}</div>

      </div>

      <div className="item_area">
        {item_rows}
      </div>
    </div>
    );
   }

}
//<Item name="expoEraser"  dims={t_dims} img_src={"Avery_Binder_Bottom_01.png"} />
// <Item name={items[0].name}  dims={items[0].dimensions} img_src={items[0].img} />
         
ReactDOM.render(
  <BinSpec bin_spec={bin_spec} />, 
  document.getElementById("root"));