var fs = require('fs');
var querystring = require("querystring");
var formidable = require("formidable");
var builder = require('xmlbuilder');
var DOMParser = require('xmldom').DOMParser;

function xml(response, postData)
{
  response.setHeader('Content-Type', 'text/xml');
  response.end(fs.readFileSync('./done.xml', {encoding: 'utf-8'}));
}

function start(response, postData)
{
  console.log("Handler 'start' is started...");

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

  // XMLBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax)
  var xml = XMLBuilder(data.fileName, data.fileWidth, data.fileHeight, data.objName, data.objXmin, data.objYmin, data.objXmax, data.objYmax);

  // Get the name of file with out format(.jpg .png ...)
  var reqData = data;
  var formatIndex = data.fileName.indexOf(".");
  var saveName = data.fileName.substring(0,formatIndex);

  if (saveName != '') {
    // Write Object information xml file
    fs.writeFile("../xml/"+saveName+".xml", xml, function(err)
    {
      if (err) {
          return console.log(err);
      }
      console.log("The file was saved!");
    });
    // Write the xml file of Object that already done
    fs.readFile("done.xml", 'utf8', function(err, data) {
      if (err) throw err;
      var doc = new DOMParser().parseFromString(data);
      var newEle = doc.createElement("file");
      var newText = doc.createTextNode(reqData.fileName);
      newEle.appendChild(newText);
      doc.getElementsByTagName('done')[0].appendChild(newEle);
      fs.writeFile("done.xml", doc, function(err)
      {
        if (err) {
            return console.log(err);
        }
        console.log("The 'done' file was saved!");
      });
    });
/*
    //content to be inserted
    var content = '<file>new file</file>';

    var fileName = 'done.xml',
      buffer = new Buffer('\n'+content+'\n'+'</done>'), 
      fileSize = fs.statSync(fileName)['size'];

    fs.open(fileName, 'r+', function(err, fd) {
      fs.write(fd, buffer, 0, buffer.length, fileSize-7, function(err) {
        if (err) throw err
        console.log('done') 
      })  
    });
*/
  }
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
exports.xml = xml;
