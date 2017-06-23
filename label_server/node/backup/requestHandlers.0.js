var fs = require('fs');
var querystring = require("querystring");
var formidable = require("formidable");
var builder = require('xmlbuilder');

function start(response, postData)
{
  console.log("Handler 'start' is started.");
/*
  var body = '<html>'+
    '<head>'+
    '<meta http-equiv="Content-Type" content="text/html; '+
    'charset=UTF-8" />'+
    '</head>'+
    '<body>'+
    '<form action="/upload" method="post">'+
    '<input name="objName" type="text" />'+
    '<textarea name="objXml" rows="20" cols="60"></textarea>'+
    '<input type="submit" value="Submit text" />'+
    '</form>'+
    '</body>'+
    '</html>';

    response.writeHead(200, {"Content-Type": "text/html"});
    response.write(body);
    response.end();
*/

  fs.readFile('../pic.html', function (err, html)
  {
    if (err) {
      throw err;
    }
    response.writeHeader(200, {"Content-Type": "text/html"});
    response.write(html);
    response.end();
  });
}

function upload(response, postData)
{
  var data = querystring.parse(postData);
  console.log("Request handler 'upload' was called.");
/*
  response.writeHead(200, {"Content-Type": "text/plain"});
  response.write("You've sent the info fileName: "+
  querystring.parse(postData).fileName)+
  response.write(", fileWidth: "+
  querystring.parse(postData).fileWidth)+
  response.write(", fileHeight: "+
  querystring.parse(postData).fileHeight)+
  response.write(", objName: "+
  querystring.parse(postData).objName)+
  response.write(", objPosition: ("+
  querystring.parse(postData).objXmin)+
  response.write(", "+
  querystring.parse(postData).objYmin)+
  response.write(", "+
  querystring.parse(postData).objXmax)+
  response.write(", "+
  querystring.parse(postData).objYmax);
  response.end();
*/
  // XMLBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax)
  var xml = XMLBuilder(data.fileName, data.fileWidth, data.fileHeight, data.objName, data.objXmin, data.objYmin, data.objXmax, data.objYmax);

  // Get the name of file with out format(.jpg .png ...)
  var formatIndex = data.fileName.indexOf(".");
  var saveName = data.fileName.substring(0,formatIndex);

  fs.writeFile("./"+saveName+".xml", xml, function(err)
  {
    if(err) {
        return console.log(err);
    }
    console.log("The file was saved!");
  });
/*
  var form = new formidable.IncomingForm();
  console.log("about to parse");
  form.parse(request, function(error, fields, files)
  {
    console.log("parsing done");
    fs.renameSync(files.upload.path, "/tmp/test.png");
    response.writeHead(200, {"Content-Type": "text/html"});
    response.write("received image:<br/>");
    response.write("<img src='/show' />");
    response.end();
  });
*/
}

function show(response, postData)
{
  console.log("Request handler 'show' was called.");
  fs.readFile("/tmp/test.png", "binary", function(error, file)
  {
    if(error) {
      response.writeHead(500, {"Content-Type": "text/plain"});
      response.write(error + "\n");
      response.end();
    } else {
      response.writeHead(200, {"Content-Type": "image/png"});
      response.write(file, "binary");
      response.end();
    }
  });
}

function XMLBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax)
{
  var xml = builder.create('annotation')
  .ele('folder').txt('VOC2007').up()
  .ele('filename').txt(filename).up()    // change
  .ele('source')
    .ele('database').txt('The VOC2007 Database').up()
		.ele('annotation').txt('PASCAL VOC2007').up()
		.ele('image').txt('flickr').up()
    .ele('flickrid').txt('341012865').up()
  .up()
  .ele('owner')
    .ele('flickrid').txt('Fried Camels').up()
    .ele('name').txt('Jinky the Fruit Bat').up()
  .up()
	.ele('size')
		.ele('width').txt(fileWidth).up()        // change
		.ele('height').txt(fileHeight).up()       // change
		.ele('depth').txt('3').up()
  .up()
  .ele('segmented').txt('0').up()
  .ele('object')
		.ele('name').txt(objName).up()         // change
		.ele('pose').txt('Frontal').up()
		.ele('truncated').txt('1').up()
		.ele('difficult').txt('0').up()
		.ele('bndbox')
			.ele('xmin').txt(xMin).up()       //change
			.ele('ymin').txt(yMin).up()      //change
			.ele('xmax').txt(xMax).up()      //change
			.ele('ymax').txt(yMax).up()      //change
  .end({ pretty: true});

//  console.log(xml);

  return xml;
}

exports.start = start;
exports.upload = upload;
exports.show = show;
