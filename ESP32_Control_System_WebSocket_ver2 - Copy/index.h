

const char *HTML_CONTENT = R"=====(


<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>WebSocket Slider Example</title>
<style>
body, html {
    margin: 0;
    padding: 0;
    overflow: hidden; /* Ngăn chặn cuộn trang */
}

canvas { background-color: #ffffff; }

.container {
    display: flex;
    justify-content: space-between;
    align-items: center;
    width: 80%;
    margin: auto;
    padding: 10px;
    background-color: #f0f0f0;
    border-radius: 10px;
    box-shadow: 0 0 10px rgba(0, 0, 0, 0.1);
}

.container2 {
    width: calc(50% - 20px);
    height: calc(50% - 20px);
    padding: 20px;
    text-align: center;
    border: 1px solid #ccc;
    border-radius: 10px;
    background-color: #fff;
}

.container2 input[type="range"] {
    width: 60%;
    margin: 10px auto;
}
.container3
{
  width: calc(10% );
  height: calc(10%);
  padding: 10px;
  border: 2px solid #000000;
  border-radius: 10px;
  background-color: #fff;
  margin: auto;
  font-weight:bold;
  font-size: 20px;
}

  .slider {
    -webkit-appearance: none;
    margin: 0 auto;
    width: 60%;
    height: 15px;
    border-radius: 10px;
    background: #FFD65C;
    outline: none;
  }
  .slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 30px;
    height: 30px;
    border-radius: 50%;
    background: #034078;
    cursor: pointer;
  }
  .slider::-moz-range-thumb {
    width: 30px;
    height: 30px;
    border-radius: 50% ;
    background: #034078;
    cursor: pointer;
  }



  .slider2 {
    -webkit-appearance: none;
    margin: 0 auto;
    width: 60%; /* Độ rộng của thanh trượt dọc */
    height: 15px;/* Chiều cao của thanh trượt dọc */
    transform: translate(-50%, -50%) rotate(90deg); /* Xoay thanh trượt 90 độ và căn giữa */
    transform-origin: center; 
    border-radius: 10px;
    background: #FFD65C;
    outline: none;
  }
  .slider2::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 30px;
    height: 30px;
    border-radius: 50%;
    background: #034078;
    cursor: pointer;
  }
  .slider2::-moz-range-thumb {
    width: 30px;
    height: 30px;
    border-radius: 50% ;
    background: #034078;
    cursor: pointer;
  }
</style>
</head>
<body>
<div class="container">
    <div class="container2">
        <h4>Servo1: </h4>
        <div><input style="text-align:center;" type="range" id="slider" min="0" max="180" value="90" step="1" class="slider"></div>
        <div class="container3" id="sliderValue"></div>
    </div>

    <div class="container2">
    
    <h4 style="margin-left: 120px;">Servo2: </h4>
    <div style="margin-left: 60px;"><input type="range" id="slider2" min="0" max="180" value="90" step="1" class="slider2"></div>
        <div class="container3" id="sliderValue2" style="margin-left:170px;"></div>
    </div>
</div>
<script>
// Kết nối đến WebSocket server
var servoValue1 = 90;
var servoValue2 = 90;
const socket = new WebSocket("ws://" + window.location.host + ":81");

// Lắng nghe sự kiện mở kết nối
socket.addEventListener('open', function (event) {
    console.log('Connected to server');
});

// Lắng nghe sự kiện khi thanh trượt thay đổi giá trị
document.getElementById('slider').addEventListener('input', function(event) {
    servoValue1 = event.target.value;
    document.getElementById('sliderValue').innerText = servoValue1;

    // Gửi giá trị của thanh trượt đến server
    socket.send("Servo1: " +servoValue1 + ", " + "Servo2: " + servoValue2);
});

document.getElementById('slider2').addEventListener('input', function(event) {
    servoValue2 = event.target.value;
    document.getElementById('sliderValue2').innerText = servoValue2;

    // Gửi giá trị của thanh trượt đến server
    socket.send("Servo1: " +servoValue1 + ", " + "Servo2: " + servoValue2);
});


document.getElementById('slider').addEventListener('mouseup', function(event) {
    // Set value back to 90 when mouse is released
    document.getElementById('slider').value = 90;
    document.getElementById('sliderValue').innerText = 90;
    servoValue1 = 90;
    socket.send("Servo1: " + servoValue1 + ", " + "Servo2: " + servoValue2);
});

document.getElementById('slider').addEventListener('touchend', function(event) {
    // Set value back to 90 when mouse is released
    document.getElementById('slider').value = 90;
    document.getElementById('sliderValue').innerText = 90;
    servoValue1 = 90;
    socket.send("Servo1: " + servoValue1 + ", " + "Servo2: " + servoValue2);
});


document.getElementById('slider2').addEventListener('mouseup', function(event) {
    // Set value back to 90 when mouse is released
    document.getElementById('slider2').value = 90;
    document.getElementById('sliderValue2').innerText = 90;
    servoValue2 = 90;
    socket.send("Servo1: " +servoValue1 + ", " + "Servo2: " + servoValue2);
});

document.getElementById('slider2').addEventListener('touchend', function(event) {
    // Set value back to 90 when mouse is released
    document.getElementById('slider2').value = 90;
    document.getElementById('sliderValue2').innerText = 90;
    servoValue2 = 90;
    socket.send("Servo1: " +servoValue1 + ", " + "Servo2: " + servoValue2);
});




// Lắng nghe sự kiện khi đóng kết nối
socket.addEventListener('close', function (event) {
    console.log('Connection closed');
});

// Lắng nghe sự kiện lỗi
socket.addEventListener('error', function (event) {
    console.error('WebSocket error:', event);
});
</script>
</body>
</html>

)=====";
