var fs = require('fs');
var querystring = require("querystring");
var builder = require('xmlbuilder');
var DOMParser = require('xmldom').DOMParser;
var XMLSerializer = require('xmldom').XMLSerializer;

var log4js = require('log4js');
//var logger = log4js.getLogger();
//logger.level = 'all';
log4js.configure('log/log4js.json');
var logger = log4js.getLogger('File');

//var xmlDir = process.env.HOME+'/NODEJS/web/xml/';
var xmlDir = __dirname+'/../xml/';

function upload(response, postData)
{
  var data = querystring.parse(postData);
  console.log("Request handler 'upload' was called.");
  logger.info("Request handler 'upload' was called.");

  // XMLBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax)
  //var xml = XMLBuilder(data.fileName, data.fileWidth, data.fileHeight, data.objName, data.objXmin, data.objYmin, data.objXmax, data.objYmax);
  if (data.objXmin == -1 && data.objYmin == -1 && data.objXmax == -1 && data.objYmax == -1) {
    var objName = data.objName;
    var saveName = data.fileName.substring(0, data.fileName.indexOf("."));
    if (fs.existsSync(xmlDir+saveName+".xml")) {
      fs.readFile(xmlDir+saveName+".xml", 'utf8', function(err, data) {
        if (err) throw err;
        var doc = new DOMParser().parseFromString(data);
        for (i = 0; i < doc.getElementsByTagName('object').length; i++) {
          if (doc.getElementsByTagName('object')[i].childNodes[1].childNodes[0].nodeValue == objName) {
            doc.getElementsByTagName('object')[i].parentNode.removeChild(doc.getElementsByTagName('object')[i]);
            fs.writeFile(xmlDir+saveName+".xml", doc, function(err)
            {
              if (err) {
                logger.error("Error : "+err);
                return console.log("Error : "+err);
              }
              console.log("Remove Object : "+objName);
              logger.error("Remove Object : "+objName);
            });
          }
        }
      });
    }
    response.end("Remove : "+objName);
  }else if (data.objXmin == -9 && data.objYmin == -9 && data.objXmax == -9 && data.objYmax == -9) {
    console.log("Remove All Objects");
    logger.info("Remove All Objects");
    var saveName = data.fileName.substring(0, data.fileName.indexOf("."));
    if (fs.existsSync(xmlDir+saveName+".xml")) {
      fs.readFile(xmlDir+saveName+".xml", 'utf8', function(err, data) {
        if (err) throw err;
        var doc = new DOMParser().parseFromString(data);
        var element = doc.getElementsByTagName("object"), index;
        for (index = element.length - 1; index >= 0; index--) {
          element[index].parentNode.removeChild(element[index]);
          fs.writeFile(xmlDir+saveName+".xml", doc, function(err)
          {
            if (err) {
              logger.error("Error : "+err);
              return console.log("Error : "+err);
            }
            console.log("Remove All Objects");
            logger.info("Remove All Objects");
          });
        }
      });
    }
    response.end("Remove All Objects");
  }else {
    XMLBuilder(data.fileName, data.fileWidth, data.fileHeight, data.objName, data.objXmin, data.objYmin, data.objXmax, data.objYmax);
    var saveName = data.fileName.substring(0, data.fileName.indexOf("."));
    if (saveName != '')
      response.end("File Saved: "+data.fileName+" ("+data.objXmin+", "+data.objYmin+", "+data.objXmax+", "+data.objYmax+")");
    else
      response.end("Error");
  }

  // // Get the name of file with out format(.jpg .png ...)
  // var reqData = data;
  // var formatIndex = data.fileName.indexOf(".");
  // var saveName = data.fileName.substring(0,formatIndex);

  // if (saveName != '') {
  //   // Write Object information xml file
  //   fs.writeFile(xmlDir+saveName+".xml", xml, function(err)
  //   {
  //     if (err) {
  //         response.end("Error");
  //         return console.log(err);
  //     }
  //     DoneFileWriter(reqData.fileName);
  //     console.log("The file was saved!");
  //     response.end("File Saved!!!");
  //   });
  // }else {
  //   console.log("Save Name is empty!!!");
  //   response.end("Save Name is empty");
  // }
}

// function DoneFileWriter(filename)
// {
//   // Write the xml file of Object that already done
//   fs.readFile("done.xml", 'utf8', function(err, data) {
//     var check = 0;
//     if (err) throw err;
//     var doc = new DOMParser().parseFromString(data);
//     for (i = 0; i < doc.getElementsByTagName('file').length; i++) {
//       if (doc.getElementsByTagName("file")[i].childNodes[0].nodeValue == filename) {
//         console.log("The file name is already exist");
//         check++;
//         break;
//       }
//     }
//     if (check == 0) {
//       var newEle = doc.createElement("file");
//       var newText = doc.createTextNode(filename);
//       newEle.appendChild(newText);
//       doc.getElementsByTagName('done')[0].appendChild(newEle);
//       fs.writeFile("done.xml", doc, function(err)
//       {
//         if (err) {
//             return console.log(err);
//         }
//         console.log("The 'done' file was saved!");
//       });
//     }
//   });
// }

function AnnotationBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax)
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

    return xml;
}

function XMLBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax)
{
  // Get the name of file with out format(.jpg .png ...)
  var saveName = filename.substring(0, filename.indexOf("."));
  if (fs.existsSync(xmlDir+saveName+".xml")) {
    console.log("FILE EXISTS");
    logger.info("FILE EXISTS");
    fs.readFile(xmlDir+saveName+".xml", 'utf8', function(err, data) {
      if (err) throw err;
      var doc = new DOMParser().parseFromString(data);
      var check = 0;
      var index;
/*
      for (i = 0; i < doc.getElementsByTagName('name').length; i++) {
        //console.log("Test : " + doc.getElementsByTagName('name')[i].childNodes[0].nodeValue);
        if (doc.getElementsByTagName('name')[i].childNodes[0].nodeValue == objName) {
          // The object is already exist
          var obj = doc.getElementsByTagName('name')[i].parentNode;
          console.log(obj.nodeName);
          var bnd = obj.parentNode;
          console.log(bnd.nodeValue);
        }
      }
*/
      for (i = 0; i < doc.getElementsByTagName('object').length; i++) {
        if (doc.getElementsByTagName('object')[i].childNodes[1].childNodes[0].nodeValue == objName) {
          check++;
          index = i;
        }
      }
      if (check != 0) {
          console.log("The object is already exist");
          console.log(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[1].childNodes[0].nodeValue +" -> "+xMin);
          console.log(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[3].childNodes[0].nodeValue +" -> "+yMin);
          console.log(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[5].childNodes[0].nodeValue +" -> "+xMax);
          console.log(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[7].childNodes[0].nodeValue +" -> "+yMax);
          logger.info("The object is already exist");
          logger.info(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[1].childNodes[0].nodeValue +" -> "+xMin);
          logger.info(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[3].childNodes[0].nodeValue +" -> "+yMin);
          logger.info(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[5].childNodes[0].nodeValue +" -> "+xMax);
          logger.info(doc.getElementsByTagName('object')[index].childNodes[9].childNodes[7].childNodes[0].nodeValue +" -> "+yMax);

          doc.getElementsByTagName('object')[index].childNodes[9].childNodes[1].childNodes[0].data = xMin;
          doc.getElementsByTagName('object')[index].childNodes[9].childNodes[3].childNodes[0].data = yMin;
          doc.getElementsByTagName('object')[index].childNodes[9].childNodes[5].childNodes[0].data = xMax;
          doc.getElementsByTagName('object')[index].childNodes[9].childNodes[7].childNodes[0].data = yMax;
          fs.writeFile(xmlDir+saveName+".xml", doc, function(err)
          {
            if (err) return console.log("Error : "+err);
            // DoneFileWriter(filename);
            console.log("The bndbox was change");
          });
      }else {
        console.log("Append New Object");
        logger.info("Append New Object");
        var text =  "  <object>\n"+
                    "    <name>"+objName+"</name>\n"+
                    "    <pose>Frontal</pose>\n"+
                    "    <truncated>1</truncated>\n"+
                    "    <difficult>0</difficult>\n"+
                    "    <bndbox>\n"+
                    "      <xmin>"+xMin+"</xmin>\n"+
                    "      <ymin>"+yMin+"</ymin>\n"+
                    "      <xmax>"+xMax+"</xmax>\n"+
                    "      <ymax>"+yMax+"</ymax>\n"+
                    "    </bndbox>\n"+
                    "  </object>";
        var newObj = new DOMParser().parseFromString(text, "text/xml");
        doc.getElementsByTagName('annotation')[0].appendChild(newObj);
        fs.writeFile(xmlDir+saveName+".xml", doc, function(err)
        {
          if (err) {
            logger.error("Error : "+err);
            return console.log("Error : "+err);
          }
          // DoneFileWriter(filename);
        });
      }
    });
  }else {
    var xml = AnnotationBuilder(filename, fileWidth, fileHeight, objName, xMin, yMin, xMax, yMax);

    if (saveName != '') {
      // Write Object information xml file
      fs.writeFile(xmlDir+saveName+".xml", xml, function(err)
      {
        if (err) {
            // response.end("Error");
            logger.error("Error : "+err);
            return console.log("Error : "+err);
        }
        // DoneFileWriter(filename);
        console.log("The file was saved!");
        logger.info("The file was saved!");
      });
    }else {
      console.log("Save Name is empty!!!");
      logger.info("Save Name is empty!!!");
    }
  }
}

exports.upload = upload;
