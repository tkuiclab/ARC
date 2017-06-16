//include http, fs and url module
var http = require('http');
var express = require('express');
var app = express();
var fs = require('fs');
var path = require('path');
var url = require('url');
var handlers = require("./requestHandlers");

var log4js = require('log4js');
//log4js.configure({
//  appenders: { node: { type: 'dateFile', filename: 'log/access.log', pattern: '-yyyy-MM-dd.log', alwaysIncludePattern: true, category: 'access' } }
//});
//const logger = log4js.getLogger('node');

//var logger = log4js.getLogger();
//logger.level = 'all';

log4js.configure('log/log4js.json');
var logger = log4js.getLogger('Server');

var imageDir = process.env.HOME+'/NODEJS/web/Image/';
var xmlDir = process.env.HOME+'/NODEJS/web/xml/';

app.get('/', function (req, res) {
  // Load __dirname/web/js directory -> /js
  app.use('/js', express.static(__dirname + '/web/js'));
  app.use('/css', express.static(__dirname + '/web/css'));
  app.use('/label', express.static(__dirname + '/web/TrainingItemsImg'));

  var filter = req.query.name;
  var start = req.query.start;
  var end = req.query.end;

  //use the url to parse the requested url and get the image name
  var query = url.parse(req.url,true).query;
      pic = query.image;
  if (typeof pic === 'undefined') {
    fs.readFile(__dirname + '/web/pic.html', function (err, html)
    {
      if (err) {
        console.log("Read File Error: "+err);
      }
      getImages(imageDir, filter, start, end, function (err, files, show) {
        res.write('<head>'+show+'</head>');
        res.write(html);
        res.end();
      });
    });
  } else {
    //read the image using fs and send the image content back in the response
    fs.readFile(imageDir + pic, function (err, content) {
      if (err) {
        res.writeHead(400, {'Content-type':'text/html'})
        console.log(err);
        res.end("No such image");    
      } else {
        //specify the content type in the response will be an image
        res.writeHead(200,{'Content-type':'image/jpg'});
        res.end(content);
      }
    });
  }
});

app.post('/upload', function (req, res) {
  console.log("***********************");
  console.log("Handler '/upload' is started.");
  logger.info("***********************\nHandler '/upload' is started.");
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
    logger.info("Received POST data chunk ' "+postDataChunk + "'.");
  });

  req.addListener("end", function()
  {
    handlers.upload(res, postData);
    console.log("***********************");
    logger.info("***********************");
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

app.get('/connect.png', function (req, res) {
  fs.readFile('./connect.png', function (err, content) {
    if (err) {
      res.writeHead(400, {'Content-type':'text/html'})
      console.log(err);
      res.end("No such image");    
    } else {
      //specify the content type in the response will be an image
      res.writeHead(200,{'Content-type':'image/jpg'});
      res.end(content);
    }
  });
});

app.listen(8000);
console.log("Server running at http://localhost:8000/");
logger.info("Server running at http://localhost:8000/");

 
//get the list of jpg files in the image dir
function getImages(imageDir, filter, start, end, callback) {
  var fileType = '.jpg',
      files = [], i;
  var show = '<output id="list">';
  
  fs.readdir(imageDir, function (err, list) {
    var index = 0;
    for(i=0; i<list.length; i++) {
      if(path.extname(list[i]).toLowerCase() === '.jpg' || path.extname(list[i]).toLowerCase() === '.png') {
        if (list[i].substring(0, list[i].indexOf('_')) === filter) {
          if (start!='' || end!=''){
            if (Number(list[i].substring(list[i].indexOf('_')+1, list[i].indexOf('.'))) >= start &&
                Number(list[i].substring(list[i].indexOf('_')+1, list[i].indexOf('.'))) <= end) {
              files.push(list[i]); //store the file name into the array files
              show += '<span><img class="thumb" src="/?image=' + list[i] + '" title="'+list[i]+'" onclick="PicFunc(this, '+index+++')">'+'</img></span>';
            }
          }else {
            files.push(list[i]); //store the file name into the array files
            show += '<span><img class="thumb" src="/?image=' + list[i] + '" title="'+list[i]+'" onclick="PicFunc(this, '+index+++')">'+'</img></span>';
          }
        }
      }
    }
    show+= '</output>'
    callback(err, files, show);
  });
}
