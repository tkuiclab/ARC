var fs = require('fs');

var l = console.log;

var ARC_Items_DIR = __dirname + "/../public/arc_items/";
var Public_DIR = "arc_items/"; //for web public 


var abs_path = require('path').resolve(ARC_Items_DIR) + "/";

var arc_items_json = {
   "dir": abs_path,
   "items": []
};


fs.readdir(ARC_Items_DIR, (err, files) => {
      files.forEach(dir => {
        console.log('-----Parse ' + dir + '--------');
        parse_dir(dir);
      });

      l('==========finish foreach==============');

      write_json();
});


//note save data to arc_items_json
function parse_dir(dir){
  //var json_name = find_json(ARC_Items_DIR + dir+"/");
  //var json_name = find_json(abs_path + dir+"/");

  //l('json_name='+ json_name );
  //ex: Avery_Binder/avery_binder.json
  var j = require(ARC_Items_DIR + dir+'/' + dir.toLowerCase() );
  //var j = require(ARC_Items_DIR + dir+'/' + json_name );
  
  //var png_name = find_png(ARC_Items_DIR + dir + "/" + );
  //var png_name = find_png(abs_path + dir);

  // l('png_name='+ png_name );

  

  var item = {
    "name": j.name,
    //public/arc_items/Avery_Binder/Avery_Binder_Top_01.png
    "img": Public_DIR + dir + "/" + dir + "_Top_01.png",
    //"img": Public_DIR + dir + "/" +  png_name,
    "dimensions": j.dimensions,
    "weight": j.weight,
    "type": j.type,
    "description": j.description
  }
  arc_items_json.items.push(item);
  //l(JSON.stringify(item_json));
}

function find_json(dir){
l(dir);
  fs.readdir(dir, (err, files) => {
       
      files.forEach(file => {
       
        if(file.includes(".json")){
          return file;
        }
      })

      l(err);
  });

  console.warn("In " + dir + ", NO JSON");
}

function find_png(dir){
   fs.readdir(dir, (err, files) => {
      files.forEach(file => {
        if(file.includes(".png")){
          return file;
        }
      })
   });
   console.warn("In " + dir + ", NO PNG");
}
 
//note: need arc_items_json
function write_json(){
  fs.writeFile(__dirname + "/arc_items.json", JSON.stringify(arc_items_json,null,4), function(err) {
    if(err) {
        return console.log(err);
    }

    console.log("The file was saved!");
}); 
}