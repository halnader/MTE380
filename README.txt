MTE380 Resuce Robot

Pins for Distance Sensor
A5 (board) -> PC_0 (mcu) ADC1/10 PIN

Pins for IMU
D7 (board) -> PA_8 (mcu) I2C3_SCL
D5 (board) -> PB_4 (mcu) I2C3_SDA

Pins for L293
//motor left
D13 (board) -> PA_5 (mcu) gpio to Pin 1 L293
D12 (board) -> PA_6 (mcu) pwm to Pin 2 L293
D11 (board) -> PA_7 (mcu) pwm to Pin 7 L293
//motor right
D10 (board) -> PB_6 (mcu) gpio to Pin 9 L293
D9 (board) -> PC_7 (mcu) pwm to Pin 15 L293
D8 (board) -> PA_9 (mcu) pwm to Pin 10 L293

Pins for AS Colour
//front left
D15 (board) -> PB_8 (mcu) I2C1_SCL
D14 (board) -> PB_9 (mcu) I2C1_SDA
//front right
D6 (board) -> PB_10 (mcu) I2C2_SCL
D3 (board) -> PB_3 (mcu) I2C2_SDA

Pins for TCS Colour
//back left
D15 (board) -> PB_8 (mcu) I2C1_SCL
D14 (board) -> PB_9 (mcu) I2C1_SDA
//back right
D6 (board) -> PB_10 (mcu) I2C2_SCL
D3 (board) -> PB_3 (mcu) I2C2_SDA
