import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';

//var item_location = require('json!item_location_file.json');
import item_location_file from './example_pick_task/item_location_file.json';
import order_file from './example_pick_task/order_file.json';

//var fs2 = require('fs');
var l = console.log;

l("item_location_file.bins = " + item_location_file.bins );
l("item_location_file.bins[0].contents = " + item_location_file.bins[0].contents );


function check_in_order(item){
  //l('input is '+ item);

  //for(var order in order_file.orders) {
  for(var i = 0; i < order_file.orders.length; i++){
    var order = order_file.orders[i];
    //l('order is '+ order);
      for( var ind in order.contents){
       // l('content is '+ order.contents[ind]);
          if(order.contents[ind] == item){
              return true;
          }
      }
  }

  return false;

}

function Bin(props) {
  var bin = '';
  
  if(props.value != null){
    var bin = props.value.map((val, index) => {
        var mark = check_in_order(val) ? 'order_mark': '';

        l('mark = ' + mark);

        return (
          <li key={index} className={mark}>
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

class Shelf extends React.Component {
  constructor(props) {
    super(props);
    // this.state = {
    //   bins: Array(12),
    // };
    this.state = {
       item_location: this.props.value,
    };

    // this.state.bins[0] = [];
    // this.state.bins[0].push("Oreo");

    // this.state.bins[1] = [];
    // this.state.bins[1].push("Windex");
    // this.state.bins[1].push("Sponge");

    //l('this.state.bins='+ JSON.stringify(this.state.bins) ) ;
  }


  renderBin(i) {
    if( this.state.item_location.bins == undefined ||
      this.state.item_location.bins.length <= i){
      return (
        <Bin  />

      );
    }else{
      return (
      <Bin
          value={this.state.item_location.bins[i].contents}
      />
      );


    }

    return (
      <Bin
          value={this.state.item_location.bins[i].contents}
      />
      );

    
  }

  render() {
    
    let status;


    console.log('Shelf render all');
    return (
      <div>
        <div className="status">{status}</div>
        <div className="board-row">
          {this.renderBin(0)}
          {this.renderBin(1)}
          {this.renderBin(2)}
        </div>
        <div className="board-row">
          {this.renderBin(3)}
          {this.renderBin(4)}
          {this.renderBin(5)}
        </div>
        <div className="board-row">
          {this.renderBin(6)}
          {this.renderBin(7)}
          {this.renderBin(8)}
        </div>
        <div className="board-row">
          {this.renderBin(9)}

        </div>
      </div>
    );
  }
}

class Game extends React.Component {
  constructor(props) {
    super(props);

    this.state = {
       item_location: this.props.item_location,
    };

  }


  render() {
    return (
      <div className="game">
        <div className="game-shelf">
          <Shelf  value={this.state.item_location} />
        </div>
        <div className="game-info">
          <div></div>
          <ol></ol>
        </div>
      </div>
    );
  }
}

// ========================================



//var shelf = ReactDOM.render(<Shelf value={item_location_file} />, document.getElementById("Shelf"));

var shelf = ReactDOM.render(<Game item_location={item_location_file} />, document.getElementById("root"));

//var shelf = ReactDOM.render(<Shelf value='' />, document.getElementById("Shelf"));

//shelf.setState({ item_location: item_location_file });
/*
class MyComponent extends React.Component {
  handleClick() {
    // 使用原始的DOM API来聚焦输入框。
    if (this.myTextInput !== null) {
      this.myTextInput.focus();
    }
  }
  render() {
    // 这里ref属性是一个回调函数，
    // 它在组件加载后保存组件的引用到this.myTextInput
    return (
      <div>
        <input type="text" ref={(ii) => this.myTextInput = ii} />
        <input
          type="button"
          value="Focus the text input"
          onClick={()=>this.handleClick()}
        />
      </div>
    );
  }
}

ReactDOM.render(
  <MyComponent />,
  document.getElementById('example')
);*/