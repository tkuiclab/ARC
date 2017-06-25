
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
var canvasImg, ctxImg;
var canvasMouse, ctxMouse;

var currentIndex = 0;

document.addEventListener("DOMContentLoaded", function(event) {
  canvasImg = document.getElementById('canvasImg');
  ctxImg = canvasImg.getContext('2d');
  canvasMouse = document.getElementById('canvasMouse');
  ctxMouse = canvasMouse.getContext('2d');
  for (var i = 0; i < document.getElementsByTagName('canvas').length-2; i++) {
    canvasArr.push(document.getElementById('canvas'+(i+1)));
    ctxArr.push(canvasArr[i].getContext('2d'));
  }
  init();
});

var img = new Image();

function init()
{
  canvasMouse.addEventListener('mousedown', MouseDown, false);
  canvasMouse.addEventListener('mousemove', MouseMove, false);
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
        checkedValue -= 1;
        //console.log(checkedValue);
        break;
      }
    }
    ctxArr[checkedValue].font = "15px Arial";
    ctxArr[checkedValue].fillText(document.getElementsByClassName('messageLabel')[checkedValue].innerText, xMin, yMin);

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
    ctxMouse.clearRect(0,0,canvasMouse.width,canvasMouse.height);
    ctxMouse.beginPath();
    ctxMouse.moveTo(0, (e.pageY - this.offsetTop));
    ctxMouse.lineTo(this.width, (e.pageY - this.offsetTop));
    ctxMouse.moveTo((e.pageX - this.offsetLeft), 0);
    ctxMouse.lineTo((e.pageX - this.offsetLeft), this.height);
    ctxMouse.strokeStyle = '#CAF4D3';
    ctxMouse.stroke();
    ctxMouse.font = "12px Arial";
    ctxMouse.fillStyle = '#CAF4D3';
    ctxMouse.fillText("("+(e.pageX - this.offsetLeft)+","+(e.pageY - this.offsetTop)+")",
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
    ctxImg.clearRect(0,0,canvasImg.width,canvasImg.height);
    ctxImg.fillStyle = 'black';
    ctxImg.globalAlpha = 0.7;
    ctxImg.fillRect(rect.startX, rect.startY, rect.w, rect.h);
  }else {
    for(var i=0; inputElements[i]; ++i){
      if(inputElements[i].checked){
        checkedValue = inputElements[i].value;
        checkedValue -= 1;
        //console.log(checkedValue);
        break;
      }
    }
    ctxArr[checkedValue].clearRect(0,0,canvasImg.width,canvasImg.height);
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
    ctxImg.drawImage(img, 0, 0, img.width, img.height);
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
        checkedValue -= 1;
        // console.log(checkedValue);
        break;
      }
    }
    ctxArr[checkedValue].clearRect(0,0,canvasImg.width,canvasImg.height);

  }else if (type == -9) {
    for (var i = 1; i < ctxArr.length; i++) {
      ctxArr[i].clearRect(0,0,canvasImg.width,canvasImg.height);
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
    ctxImg.drawImage(img, 0, 0, img.width, img.height);
  };
  img.src = x[currentIndex].src;

  document.getElementById('fileNameShow').innerHTML = x[currentIndex].title;
  document.getElementById('fileName').value = x[currentIndex].title;
  document.getElementById('fileWidth').value = img.width;
  document.getElementById('fileHeight').value = img.height;
}
