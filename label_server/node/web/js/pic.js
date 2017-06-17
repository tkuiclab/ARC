
var checkedValue = null; 
var inputElements = document.getElementsByClassName('messageCheckbox');

var picTmp = '';

var rect = {}, drag = false, wellDown = false;
var rectColor = ['#000',
                '#da40b3', '#da4066', '#32c07a', '#32bee2', '#f5925d',
                '#ffcf50', '#875d32', '#a0a0a0', '#d528aa', '#d52854',
                '#19b869', '#19b6de', '#f48449', '#ffc93a', '#784919',
                '#949494', '#d111a1', '#d11141', '#00b159', '#00aedb',
                '#f37735', '#ffc425', '#6a3500', '#898989', '#bc0f90',
                '#bc0f3a', '#009f50', '#009cc5', '#da6b2f', '#e5b021',
                '#5f2f00', '#7b7b7b', '#a70d80', '#a70d34', '#008d47',
                '#008baf', '#c25f2a', '#cc9c1d', '#542a00', '#6d6d6d'];
var canvasArr = [];
var ctxArr = [];
var canvas_mouse;
var ctx_mouse;

var currentIndex = 0;

l = console.log;

document.addEventListener("DOMContentLoaded", function(event) {
  var canvas_ary = document.getElementsByTagName('canvas');
  for (var i = 0; i < canvas_ary.length; i++) {
    //l('push canvas_ary[' + i + '].id= '  +  canvas_ary[i].id);
    if(canvas_ary[i].id == 'canvas_mouse_move') 
          continue;
    // canvasArr.push(document.getElementById('canvas'+i));
    // ctxArr.push(canvasArr[i].getContext('2d'));
    canvasArr.push(canvas_ary[i]);
    ctxArr.push(canvas_ary[i].getContext('2d'));
  }

  canvas_mouse = document.getElementById('canvas_mouse_move');
  ctx_mouse = canvas_mouse.getContext('2d');

  init();
});

var img = new Image();

function init()
{
  canvas_mouse.addEventListener('mousedown', MouseDown, false);
//  canvas_mouse.addEventListener('mouseup', MouseUp, false);
  canvas_mouse.addEventListener('mousemove', MouseMove, false);
  document.addEventListener('mouseup', MouseUp, false);
}

function MouseDown(e)
{
  rect.startX = (e.pageX - this.offsetLeft);
  rect.startY = (e.pageY - this.offsetTop);
  drag = true;
  wellDown = true;
}

function MouseUp()
{
  if (wellDown) {
    var xMin = (rect.startX < rect.startX + rect.w) ? rect.startX : rect.startX + rect.w;
    var xMax = (rect.startX > rect.startX + rect.w) ? rect.startX : rect.startX + rect.w;
    var yMin = (rect.startY < rect.startY + rect.h) ? rect.startY : rect.startY + rect.h;
    var yMax = (rect.startY > rect.startY + rect.h) ? rect.startY : rect.startY + rect.h;

    for(var i=0; inputElements[i]; ++i){
      if(inputElements[i].checked){
        checkedValue = inputElements[i].value;
        // console.log(checkedValue);
        break;
      }
    }
    ctxArr[checkedValue].font = "15px Arial";

    //console.log('innerText = ' + document.getElementsByClassName('messageLabel')[checkedValue-1].innerText);
    l('checkedValue = ' + checkedValue);

    ctxArr[checkedValue].fillText(document.getElementsByClassName('messageLabel')[checkedValue-1].innerText, xMin, yMin);

    console.log("Position: "+xMin+", "+yMin+", "+xMax+", "+yMax);
    document.getElementById('objXmin').value = xMin;
    document.getElementById('objYmin').value = yMin;
    document.getElementById('objXmax').value = xMax;
    document.getElementById('objYmax').value = yMax;

    drag = false;

    Send();
    document.getElementById("nextPicture").focus();
  }
  wellDown = false;
}

function MouseMove(e)
{
    ctx_mouse.clearRect(0,0,canvas_mouse.width,canvas_mouse.height);
    ctx_mouse.beginPath();
    ctx_mouse.moveTo(0, (e.pageY - this.offsetTop));
    ctx_mouse.lineTo(this.width, (e.pageY - this.offsetTop));
    ctx_mouse.moveTo((e.pageX - this.offsetLeft), 0);
    ctx_mouse.lineTo((e.pageX - this.offsetLeft), this.height);
    ctx_mouse.strokeStyle = '#CAF4D3';
    ctx_mouse.stroke();
    ctx_mouse.font = "12px Arial";
    ctx_mouse.fillStyle = '#CAF4D3';
    ctx_mouse.fillText("("+(e.pageX - this.offsetLeft)+","+(e.pageY - this.offsetTop)+")",
                        (e.pageX - this.offsetLeft),
                        (e.pageY - this.offsetTop));
  if (drag) {
    rect.w = ((e.pageX - this.offsetLeft) - rect.startX);
    rect.h = ((e.pageY - this.offsetTop) - rect.startY);
    //console.log("e.pageX: "+e.pageX+" this.offsetLeft: "+this.offsetLeft+" rect.startX: "+rect.startX);
    //console.log("e.pageY: "+e.pageY+" this.offsetRight: "+this.offsetTop+" rect.startY: "+rect.startY);
    draw();
  }
}

function draw()
{
  if (picTmp == '') {
    console.log("no Image");
    ctxArr[0].clearRect(0,0,canvasArr[0].width,canvasArr[0].height);
    ctxArr[0].fillStyle = 'black';
    ctxArr[0].globalAlpha = 0.5;
    ctxArr[0].fillRect(rect.startX, rect.startY, rect.w, rect.h);
  }else {
    for(var i=0; inputElements[i]; ++i){
      if(inputElements[i].checked){
        checkedValue = inputElements[i].value;
        // console.log(checkedValue);
        break;
      }
    }
    ctxArr[checkedValue].clearRect(0,0,canvasArr[0].width,canvasArr[0].height);
    ctxArr[checkedValue].fillStyle = rectColor[checkedValue];
    ctxArr[checkedValue].globalAlpha = 0.5;
    ctxArr[checkedValue].fillRect(rect.startX, rect.startY, rect.w, rect.h);
  }
}

function PicFunc(e, index)
{
  location.href = "#responseText";
  picTmp = e;
  currentIndex = index;

  img.onload = function(){
    for (var i = 0; i < document.getElementsByTagName('canvas').length; i++) {
      var canvas = document.getElementsByTagName('canvas')[i];
      canvas.width = img.width;
      canvas.height = img.height;
    }
    ctxArr[0].drawImage(img, 0, 0, img.width, img.height);
  };
  img.src = e.src;
  document.getElementById('canvasDiv').style.height = img.height + 20;
  document.getElementById('fileNameShow').innerHTML = e.title;
  document.getElementById('fileName').value = e.title;
  document.getElementById('fileWidth').value = img.width;
  document.getElementById('fileHeight').value = img.height;

  document.getElementById('responseText').innerHTML = '';
}

function Remove(type)
{
  if (type == -1) {
    for(var i=0; inputElements[i]; ++i){
      if(inputElements[i].checked){
        checkedValue = inputElements[i].value;
        // console.log(checkedValue);
        break;
      }
    }
    ctxArr[checkedValue].clearRect(0,0,canvasArr[0].width,canvasArr[0].height);

  }else if (type == -9) {
    for (var i = 1; i < ctxArr.length; i++) {
      ctxArr[i].clearRect(0,0,canvasArr[0].width,canvasArr[0].height);
    }
  }
  document.getElementById('objXmin').value = type;
  document.getElementById('objYmin').value = type;
  document.getElementById('objXmax').value = type;
  document.getElementById('objYmax').value = type;
  Send();
}

function NextPicture()
{
  currentIndex += 1;
  var x = document.querySelectorAll("img.thumb");
  console.log(x[currentIndex].src);
  img.onload = function(){
    for (var i = 0; i < document.getElementsByTagName('canvas').length; i++) {
      var canvas = document.getElementsByTagName('canvas')[i];
      canvas.width = img.width;
      canvas.height = img.height;
    }
    ctxArr[0].drawImage(img, 0, 0, img.width, img.height);
  };
  img.src = x[currentIndex].src;

  document.getElementById('fileNameShow').innerHTML = x[currentIndex].title;
  document.getElementById('fileName').value = x[currentIndex].title;
  document.getElementById('fileWidth').value = img.width;
  document.getElementById('fileHeight').value = img.height;
}
