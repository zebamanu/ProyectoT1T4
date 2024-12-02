const express = require('express')
const path = require('path');
const ROSLIB = require('roslib')
const app = express()
const port = 3000

var ros_server = 'ws://localhost:9090';
var ros = new ROSLIB.Ros();

ros.on('error', function(error)  {
  console.log(error);
});


ros.connect(ros_server);


// Publishers
var walkState = new ROSLIB.Topic({
    ros : ros,
    name : 'T1T4/walk',
    messageType : 'std_msgs/msg/Float32'
  });

  // Publishers
var resetState = new ROSLIB.Topic({
  ros : ros,
  name : 'T1T4/reset',
  messageType : 'std_msgs/msg/Float32'
});

var saludarState = new ROSLIB.Topic({
  ros : ros,
  name : 'T1T4/saludar',
  messageType : 'std_msgs/msg/Float32'
});

var moverState = new ROSLIB.Topic({
    ros : ros,
    name : 'T1T4/moverServo',
    messageType : 'geometry_msgs/msg/Vector3'
  });


  

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '/index.html'));
})

app.use(express.json());
app.use(express.urlencoded({ extended: true }));

app.post('/Walk', (req, res) => {

    var walkAction = new ROSLIB.Message({
        data : parseInt(req.body.data)
      });

      walkState.publish(walkAction);
      res.redirect('/');
})

app.post('/Reset', (req, res) => {

  var resetAction = new ROSLIB.Message({
      data : parseInt(req.body.data)
    });

    resetState.publish(resetAction);
    res.redirect('/');
})

app.post('/Saludar', (req, res) => {

  var saludarAction = new ROSLIB.Message({
      data : parseInt(req.body.data)
    });

    saludarState.publish(saludarAction);
    res.redirect('/');
})

app.post('/MoverServo', (req, res) => {

  var moverAction = new ROSLIB.Message({
      x : parseFloat(req.body.idServo),
      y : parseFloat(req.body.pulseWidth),
      z : 0.0
    });

    moverState.publish(moverAction);
    res.sendStatus(200);
})


app.listen(port, () => {
  console.log(`Example app listening on port ${port}`)
})

