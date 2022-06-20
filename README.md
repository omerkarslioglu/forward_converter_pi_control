# forward_converter_pi_control
9-36V Input Voltage, 28 Output Voltage, All Ripple: %2, 50KHz, 50 Watt DC-DC Forward Converter Design And Production

The aim of the project is to design and produce a dc-dc forward converter according to following
requirements:
Vin: 36-9 V
Vout: 28 V
Power: 50 Watt
Ripple Ratings: 2%
Fsw : 50 KHz
Control System: PI with a STM32F4 MCU
In addition to this, this paper shows formulas, calculations, experimental and simulation results of dc-dc
forward converter.

# Forward Converter
A forward converter is a switched-mode power supply (SMPS) circuit that transfers energy to
secondary side from primary side and ensures to decrease secondary side input voltage. Forward
converter circuit is similar to the fly-back converter circuit but it is more efficient than fly-back
converter circuit. Forward converter is mainly used for the application which requires higher power
output.
Secondary side of forward converter basicly derives from buck converter. But forward converter can
increase or decrease the input voltage thanks to transformer.
Forward converter is an isolated circuit as it has transformer.
![image](https://user-images.githubusercontent.com/67158049/174551599-e0be528e-241c-4d2d-8f76-e27110498eb3.png)

As per the above block diagram, when the switch is turned ON, the input is applied to the primary
winding of the transformer and a voltage is appeared at the secondary winding of transformer.
Therefore, the dot polarity of the windings of transformer is positive, due to this the diode D1 gets
forward biased. Then the output voltage of the transformer is fed to the low pass filter circuit which is
connected to the load. When switch is turned OFF, the current in the windings of transformer comes
down to zero (assuming the transformer to be ideal).
A typical forward converter consists of a:
 Transformer which is either a step-up or step-down with a single or multiple secondary
windings. The type used depends on the available input voltage and desired output voltage. It
also provides isolation of the load from the input voltage.
- Switching components
- Diodes
- Capacitors
- Inductor

# Circuit Design For Simulation On Simulink
![image](https://user-images.githubusercontent.com/67158049/174551914-3fc5c894-5bc4-4373-889d-c8520e5eda02.png)

The all calculations, values and simulations result in the report.

# Circuit Schematic
![image](https://user-images.githubusercontent.com/67158049/174552737-529c52a9-cb49-4d0f-97ff-9a0388f207cb.png)

# PCB Design
![image](https://user-images.githubusercontent.com/67158049/174553691-54f1f4f6-66c8-4c67-9614-ee799dd2a315.png)
![image](https://user-images.githubusercontent.com/67158049/174553896-04a5c4fe-a04c-4dce-ae09-071cf4b8c56d.png)

# About Software
As it is known, this designed circuit is designed as a close loop system and will be controlled by
PI. The software of this circuit works as follows: voltage is read with ADC and required ratio
calculation is made. This calculated value enters the PI algorithm and is compared with the
setpoint. If this value is not the same as the setpoint, the PI calculates a new value close to or the
same as the setpoint. This value is again proportioned and the PWM signal driving the mosfet is
updated again.
![image](https://user-images.githubusercontent.com/67158049/174554102-fff5f319-5eb6-4f05-85f1-ad6f167d6832.png)

The closed loop system of Forward Converter is as above diagram. It is PID system. But Kd equals 0
as seting PI.
The software generally consists of two or three main modules:
1. main.c : It is the file where all the configurations are found, the main algorithm and PWM are
set.
2. ADC_READ.c : It is an analog reading value module that I developed specially for the
STM32F4 discovery board.
3. PID.c : It is the module where I wrote the PID algorithm.
Other folders belong to the HAL Library used and the processor's drivers.

![image](https://user-images.githubusercontent.com/67158049/174554167-5d8d515c-5895-4001-821d-5ba8253b341d.png)

All necessary explanations are in the software modules in the software folder in the project file.

If you have any question, you can contact me.

Omer Karslioglu
omerkarsliogluu@gmail.com





