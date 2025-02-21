<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<title>Điều khiển Servo</title>
<style>
body, html {
  margin: 0;
  padding: 0;
  overflow: hidden; /* Ngăn chặn cuộn trang */
}
.container {
  display: flex;       /* Keep flexbox for easy alignment */
  flex-direction: row; /* Arrange items in a row (default) */
  align-items: center; /* Vertically align items to the center */
  justify-content: space-around; /* Distribute sliders evenly */
}

.servo-control {
  width: 250px; /* Adjust width as needed */
  margin: 10px; /* Add some spacing around sliders */
}
</style>
</head>
<body>
<div class="container">
  <div class="container2"><h4>Servo1:</h4>
    <input type="range" id="slider" min="0" max="180" value="90" step="1" class="slider">
    <div class="container3" id="sliderValue">90</div> 
  </div>
  <div class="container2"><h4>Servo2:</h4>
    <input type="range" id="slider2" min="0" max="180" value="90" step="1" class="slider2">
    <div class="container3" id="sliderValue2">90</div>
  </div>
</div>

<script>
// Kết nối đến WebSocket server
document.addEventListener('DOMContentLoaded', function () {
  var servoValue1 = 90;
  var servoValue2 = 90;
  const socket = new WebSocket("ws://" + window.location.host + ":81");

  socket.addEventListener('open', function (event) {
    console.log('Đã kết nối đến máy chủ');
    // Gửi giá trị ban đầu
    sendServoValues();
  });

  document.getElementById('slider').addEventListener('input', function () {
    servoValue1 = this.value;
    document.getElementById('sliderValue').innerText = servoValue1;
    sendServoValues();
  });

  document.getElementById('slider2').addEventListener('input', function () {
    servoValue2 = this.value;
    document.getElementById('sliderValue2').innerText = servoValue2;
    sendServoValues();
  });

  ['mouseup', 'touchend'].forEach(function (event) {
    document.getElementById('slider').addEventListener(event, function () {
      this.value = 90;
      document.getElementById('sliderValue').innerText = 90;
      servoValue1 = 90;
      sendServoValues();
    });
    document.getElementById('slider2').addEventListener(event, function () {
      this.value = 90;
      document.getElementById('sliderValue2').innerText = 90;
      servoValue2 = 90;
      sendServoValues();
    });
  });

  function sendServoValues() {
    if (socket.readyState === WebSocket.OPEN) {
      socket.send("Servo1: " + servoValue1 + ", Servo2: " + servoValue2);
    }
  }

  socket.addEventListener('close', function (event) {
    console.log('Kết nối đã đóng');
  });

  socket.addEventListener('error', function (event) {
    console.error('Lỗi WebSocket:', event);
  });
});
</script>

</body>
</html>
