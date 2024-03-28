# Simulated-Home-Management-System-via-STM32F439-microcontroller
Using Keil uVision to develop an application that simulates a Home Management System, or an alarm system, written in C and deployed to a physical STM32F439. 

## AIMS

(i) To design, simulate, implement and test a series of external peripheral interfaces using Keil uVision and an STM32F439 microcontroller.

(ii) To develop an understanding of the design workflow when writing code (firmware) for an ARM-based microcontroller using an Integrated Development Environment (IDE) such as Keil uVision.

(iii) To use the C programming language to develop comprehensive firmware that controls integrated peripherals such as ADC, Timers, GPIO and the UART.

(iv) To use the STM32F439 datasheets to determine and interpret essential characteristics of the microcontroller.

(v) To develop a large-scale C project that will require numerous sub-modules that are required to work together to achieve a common complex task.

## REQUIREMENTS: 
The system comprises of a humidity sensor (analogue input), a fan and light switch as well as
a light intensity sensor (threshold detection). The required I/O mapping can be found below:
```
| HMS Function            | Port / Pin | I/O Type       | Function           |
|-------------------------|------------|----------------|--------------------|
| Humidity Sensor         | PF10       | Analogue Input | Potentiometer      |
| Light Intensity Sensor  | PA3        | Digital Input  | Up Switch (SW1)    |
| Fan Switch              | PA8        | Digital Input  | Down Switch (SW2)  |
| Light Switch            | PA9        | Digital Input  | Left Switch (SW3)  |
| Fan Control Output      | PA10       | Digital Output | LED 3              |
| Light Control Output    | PB0        | Digital Output | LED 4              |
| UART3 Receive           | PB11       | I/O Alternate  |                    |
| UART3 Transmit          | PB10       | I/O Alternate  |                    |
```


