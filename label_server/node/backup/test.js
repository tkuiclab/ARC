var express = require("express"),  
app = express();

var imageDir = '/home/neet/ROS/ARC/Image/';

app.get("/", function(req, res) {
    res.sendfile(__dirname + '/web/pic.html', function(err) {
        if (err) res.send(404);
    });
});

app.get(/(.*)\.(jpg|gif|png|ico|css|js|txt)/i, function(req, res) {  
    // res.sendfile(__dirname + "/" + req.params[0] + "." + req.params[1], function(err) {
    //     if (err) res.send(404);
    // });
    res.sendfile(imageDir + "/colgateToothbrushs-00002.jpg", function(err) {
        if (err) res.send(404);
    });
});

app.listen(8000);
