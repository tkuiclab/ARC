var express = require('express');
var app = express();
var fs = require('fs');
var path = require('path');
var url = require('url');
var handlers = require("./requestHandlers");

var imageDir = process.env.HOME+'/ROS/ARC/Image/';
var xmlDir = process.env.HOME+'/web/xml/';
 
app.get('/', function (req, res) {
  // Load __dirname/web/js directory -> /js
  app.use('/js', express.static(__dirname + '/web/js'));

  var filter = req.query.name;
  var start = req.query.start;
  var end = req.query.end;
  //use the url to parse the requested url and get the image name
  var query = url.parse(req.url,true).query;
    pic = query.image;

  if (typeof pic === 'undefined') {
    console.log("test");
    GetImages(imageDir, function (err, files) {
      var imageShow = '<output id="list">';
      for (var i=0; i<files.length; i++) {
        if (files[i].substring(0, files[i].indexOf('-')) === filter) {
          // console.log(files[i].substring(files[i].indexOf('-')+1, files[i].indexOf('.')));
          if (start!='' || end!=''){
            if (Number(files[i].substring(files[i].indexOf('-')+1, files[i].indexOf('.'))) >= start &&
                Number(files[i].substring(files[i].indexOf('-')+1, files[i].indexOf('.'))) <= end) {
              imageShow += '<span><img class="thumb" src="/?image=' + files[i] + '" title="'+files[i]+'" onclick="PicFunc(this)">'+'</img></span>';
            }
          }else {
            imageShow += '<span><img class="thumb" src="/?image=' + files[i] + '" title="'+files[i]+'" onclick="PicFunc(this)">'+'</img></span>';
          }
        }
      }
      imageShow += '</output>';
      res.writeHead(200, {'Content-type':'text/html'});
      res.write('<head>'+imageShow+'</head>');
    });
    fs.readFile('./web/pic.html', function (err, html)
    {
      if (err) {
        console.log("Read File Error: "+err);
      }
      res.write(html);
      res.end();
    });
  } else {
    //read the image using fs and send the image content back in the response
    fs.readFile(imageDir + pic, function (err, content) {
      if (err) {
        res.writeHead(400, {'Content-type':'text/html'})
        console.log("Read File Error: "+err);
        res.end("No such image");    
      } else {
        // specify the content type in the response will be an image
        res.writeHead(200,{'Content-type':'image/jpg'});
        res.end(content);
      }
    });
  }
});

app.post('/upload', function (req, res) {
  console.log("***********************");
  console.log("Handler '/upload' is started.");
  // POST
  var postData = "";
  //var pathname = url.parse(request.url).pathname;
  //console.log("Request for " + pathname + " received.");

  req.setEncoding("utf8");

  req.addListener("data", function(postDataChunk)
  {
    postData += postDataChunk;
    console.log("Received POST data chunk '"+
    postDataChunk + "'.");
  });

  req.addListener("end", function()
  {
    handlers.upload(res, postData);
    console.log("***********************");
  });
});

app.get('/done.xml', function (req, res) {
  // //res.setHeader('Content-Type', 'text/xml');
  // //res.end(fs.readFileSync('./done.xml', {encoding: 'utf-8'}));
  fs.readdir(xmlDir, function(err, files){
    if (!err) {
      res.end(files.toString());
    }
  });
});
 
app.listen(8000);
console.log('Server跑起來了，現在時間是:' + new Date());

//get the list of jpg files in the image dir
function GetImages(imageDir, callback) {
  var fileType = '.jpg',
  files = [], i;
  fs.readdir(imageDir, function (err, list) {
    for(i=0; i<list.length; i++) {
      if(path.extname(list[i]) === '.jpg' || path.extname(list[i]) === '.png') {
        files.push(list[i]); //store the file name into the array files
      }
    }
    callback(err, files);
  });
}
