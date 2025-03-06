# SPA_Module
Parking Assist System

## Description
Parking assist module based on STM32F103 and 2 chinese ultrasonic parking modules.

## Ultrasonic pulses protocol
This protocol is used by chinese ultrasonicl parking modules. Signal is send by one wire and has active state with high level.
It consists of header and 4 sequences of pulses that represents distance value in binary format. This sequences of pulses sends every 200 ms.
Start pulse = 190 ms, 0 = 100 us, 1 = 200 us
![pulses](https://github.com/SPA_Module/docs/img/pulse_sequence.png)
In this example after header next 4 sequences will be decoded as: Sensor1(A) = 00001001 => 9, Sensor2(D) = 11111111 => 255, Sensor3(C) = 00000111 => 7, Sensor4(B) = 11111111=> 255
To get real values in cm just multipy by 10, so actual distance will be Sensor1(A) = 9 * 10 => 90 cm

## Parking modes

### 1. Rear parking
### 2. Front parking
### 3. Left mirror parking
### 4. Right mirror parking
### 5. Front speed auto parking
