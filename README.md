# IOT_Project
Team Members
Mirko Di Criscio, Deante MiriJello

System overview and block diagrams
This project implements an end-to-end IoT system using a Raspberry Pi 4B sensor, led's, a motor, a camera and a buzzer. The system collects live telemetry, publishes it to Adafruit IO via MQTT, logs data locally with daily rotation, and uploads logs to cloud storage automatically.

Option 1
Detects motion, sound, and environmental changes, captures images, and provides remote dashboard controls for home devices such as lights and fans.

Materials
Component Model/Description Quantity Link

Raspberry Pi 4B 2GB or 4GB version 1 Raspberry Pi 4B

PIR Motion Sensor HC-SR501 1

Temperature/Humidity Sensor DHT22 1

Ultrasonic Sensor HC-SR04 1

Camera Module Pi Camera v2 1

Actuator LED strip / Fan / Motor driver —

Buzzer Murata 1

Servo Motor

Video

https://www.youtube.com/shorts/p_izFyDbBKE 

Setup Instructions
Setup the pi sudo apt update && sudo apt upgrade -y sudo raspi-config # Enable Camera/I2C/SPI as needed

Clone repo git clone https://github.com//.git cd

Create Python Env python3 -m venv venv source venv/bin/activate pip install -r requirements.txt

Configure env AIO_USERNAME=<your_adafruit_io_username> AIO_KEY=<your_adafruit_io_key> WIFI_SSID=<your_wifi_ssid> WIFI_PASS=<your_wifi_password>

Create config/config.json { "feeds": { "temperature": "home.temp", "humidity": "home.humidity", "motion": "home.motion", "light": "home.light", "control_light": "home.control.light", "control_fan": "home.control.fan" }, "thresholds": { "motion_sensitivity": 0.7 }, "logging": { "data_dir": "data/", "upload_time": "00:05" } }


How to Run
cd IOT_Project

python3 main.py


Data Format Specifications
[Unit] Description=IoT System Service After=network.target

[Service] ExecStart=/usr/bin/python3 /home/pi//src/main.py WorkingDirectory=/home/pi/ Restart=on-failure

[Install] WantedBy=multi-user.target

Future Work
Add OTA (over-the-air) configuration updates
Improve dashboard UI
Add data analytics or predictive monitoring
Optimize camera capture performance
