import React from 'react';
import ReactDOM from 'react-dom';
import './index.css';


function Bin(props) {
  return (
    <button className="bin" onClick={props.onClick}>
      {props.value}
    </button>
  );
}



class Shelf extends React.Component {
  constructor() {
    super();
    this.state = {
      bins: Array(12).fill(null),
      xIsNext: true,
    };
  }

  handleClick(i) {
    const bins = this.state.bins.slice();
  

    bins[i] = this.state.xIsNext ? 'X' : 'O' ;
    
    this.setState({
        bins: bins,
        xIsNext: !this.state.xIsNext,
    });
  }

  renderBin(i) {
    return (
      <Bin
        value={this.state.bins[i]}
        onClick={() => this.handleClick(i)}
      />
    );
  }

  render() {
    
    let status;


    console.log(' render all');
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
          {this.renderBin(6)}
          {this.renderBin(7)}
          {this.renderBin(8)}
        </div>
      </div>
    );
  }
}

class Game extends React.Component {
  constructor() {
    super();
    this.state = {
      history: [{
        bins: Array(9).fill(null),
      }],
      xIsNext: true,
    };
  }

  handleClick(i) {
    const history = this.state.history;
    const current = history[history.length - 1];
    const bins = current.bins.slice();

    bins[i] = this.state.xIsNext ? 'X' : 'O';
    this.setState({
      history: history.concat([{
        bins: bins
      }]),
      xIsNext: !this.state.xIsNext,
    });
  }
 
 render() {
    const history = this.state.history;
    const current = history[history.length - 1];


    const moves = history.map((step, move) => {
      const desc = move ?
        'Move #' + move :
        'Game start';
      return (
        <li>
          <a href="#" onClick={() => this.jumpTo(move)}>{desc}</a>
        </li>
      );
    });

    let status;

    return (
      <div className="game">
        <div className="game-board">
          <Shelf
            bins={current.bins}
            onClick={(i) => this.handleClick(i)}
          />
        </div>
        <div className="game-info">
          <div>{status}</div>
          <ol>{moves}</ol>
        </div>
      </div>
    );
  }
}


// ========================================

ReactDOM.render(
  <Game />,
  document.getElementById('root')
);

