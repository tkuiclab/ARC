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
  renderBin(i) {
    return (
      <Bin
        value={this.props.bins[i]}
        onClick={() => this.props.onClick(i)}
      />
    );
  }

  render() {
    console.log('in Shelf::render()')
    return (
      <div>
        <div className="shelf-row">
          {this.renderBin(0)}
          {this.renderBin(1)}
          {this.renderBin(2)}
        </div>
        <div className="shelf-row">
          {this.renderBin(3)}
          {this.renderBin(4)}
          {this.renderBin(5)}
        </div>
        <div className="shelf-row">
          {this.renderBin(6)}
          {this.renderBin(7)}
          {this.renderBin(8)}
        </div>
        <div className="shelf-row">
          {this.renderBin(9)}
          {this.renderBin(10)}
          {this.renderBin(11)}
        </div>
      </div>
    );
  }
}

class Game extends React.Component {
  constructor() {
    super();
    this.state = {
      history: [
        {
          bins: Array(12).fill(null)
        }
      ],
      stepNumber: 0,
      xIsNext: true
    };
  }

  handleClick(i) {
    console.log('in Game::handleClick(), i = ' + i);
    const history = this.state.history.slice(0, this.state.stepNumber + 1);
    const current = history[history.length - 1];
    const bins = current.bins.slice();
    
    bins[i] = this.state.xIsNext ? "X" : "O";
    this.setState({
      history: history.concat([
        {
          bins: bins
        }
      ]),
      stepNumber: history.length,
      xIsNext: !this.state.xIsNext
    });
  }

  jumpTo(step) {
    this.setState({
      stepNumber: step,
      xIsNext: step % 2 ? false : true
    });
  }

  render() {
    const history = this.state.history;
    const current = history[this.state.stepNumber];

    //squa -> value
    //move -> key
    const moves = history.map((squa, move) => {

      console.log('squa='+ JSON.stringify(squa) ) ;
      console.log('move='+ move);
      
      

      const desc = move ? "Move #" + move : "Game start";
      return (
        <li key={move}>
          <a href="#" onClick={() => this.jumpTo(move)}>{desc}</a>
        </li>
      );
    });

    let status;
    

    console.log('in Game::render()')

    return (
      <div className="game">
        <div className="game-shelf">
          <Shelf
            bins={current.bins}
            onClick={i => this.handleClick(i)}
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

ReactDOM.render(<Game />, document.getElementById("root"));
