import React from 'react';
import './index.css';

import './arc.jsx';


class Stow extends Pick{
  render() {
    //l('Pick render() this.state.item_location = ' + JSON.stringify(this.state.item_location)  );
  
    return (
      <div>
          <div className="upload_and_status">
            
            <div className="upload_area">
                <div className="upload"> Item Location File: <JsonUpload data_cb={this.item_location_change} /> </div>
                
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

export default Stow;