# Absolute encoder with SPI interface  (STM32G431KB)
An STM32 HAL example of communicating with an absolute encoder over the SPI interface. A single-turn 14-bit encoder from CUI Devices is taken as an example. The relevant evaluation kit is AMT222B-V.

![Single-turn absolute encoder in action](/Assets/Images/cui_devices_single_turn_spi_in_action.jpg)

> [!TIP]
> Yes, the encoder happens to be mechanically compatible with LEGO Technic/Mindstorms bricks - no need to 3D print to align the rotating part or to drill to bolt the housing.

![CUI Devices encoder LEGO compatibility](/Assets/Images/cui_devices_lego_compatibility.jpg)

The main motivation behind this submission is to encourage you to get familiar with delays inevitably accompanying any code execution. In the embedded world microcontrollers interact with physical processes through their peripherals connected to sensors and actuators. Time flows ever onward for the physical plant, energy gets converted, d/dt is relentless, and neglecting delays on the uC side can ruin your project.

Here our goal is to reproduce the following waveforms under the assumption of getting close to the required minimal time delays:

![CUI Devices AMT22 timing waveform](/Assets/Images/datasheet_timing_waveform.JPG)

Source: [AMT22 Series Datasheet](https://www.cuidevices.com/product/resource/amt22.pdf)

We will fail miserably in doing that but hopefully we will learn a lot along the way. This does not mean that we will be unable to read the position - they are the minimal delays required by the sensor, not the maximal allowable ones. The latter are not specified - lucky we :wink:

# Blocking vs. non-blocking code
The library implements both approaches. The blocking one is taken from [AMT22 Library](https://github.com/SDibla/Arduino-STM32-AMT22_Library). The non-blocking version is a small addition from my side. Both of them fail miserably in getting close to the minimal time delays accepted by the sensor. Other approaches should be explored if reaching higher position sampling frequencies is critical for your application. Turning into the LL (low layer) libraries instead of the HAL (hardware abstraction layer) ones should improve things. The DMA transfers triggered by a timer are also to be considered[^1].

[^1]: [Search: stm32 tim triggered dma](https://www.google.com/search?q=stm32+tim+triggered+dma)

# Let's play with it to understand why 2.5 us spacing between bytes is hard (impossible?) to achieve in HAL

## Logic analyzer as elapsed time measuring tool
```c
HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,	GPIO_PIN_SET);
// Code to measure
HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,	GPIO_PIN_RESET);
```
It's convenient but not very useful for sub-microsecond measurements. There is a delay between calling HAL_GPIO_WritePin() and the physical pin reaction.

## Debug Watch and Trace (DWT) module[^2]
```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
// Code not to measure
uint32_t t1 = DWT->CYCCNT;
// Code to measure
uint32_t t2 = DWT->CYCCNT;
uint32_t diff21 = t2 - t1;
```
And now you can correct for the delay introduced by HAL_GPIO_WritePin():
```c
uint32_t t1 = DWT->CYCCNT;
HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,	GPIO_PIN_SET);
uint32_t t2 = DWT->CYCCNT;
HAL_GPIO_WritePin(LOGIC_ANALYZER_GPIO_Port, LOGIC_ANALYZER_Pin,	GPIO_PIN_RESET);
uint32_t t3 = DWT->CYCCNT;
uint32_t diff21 = t2 - t1;
uint32_t diff32 = t3 - t2;
```
Obviously, accessing DWT->CYCCNT and storing its value in a variable (memory) is not delayless. Nevertheless, HAL_GPIO_WritePin() is around an order of magnitude slower.

And now compare accessing global vs. local variables:
```c
t1 = DWT->CYCCNT;
t2 = DWT->CYCCNT;
```
vs.
```c
uint32_t t1 = DWT->CYCCNT;
uint32_t t2 = DWT->CYCCNT;
```
Surprised?

Be aware (or even beware) of playing too much with DWT->CYCCNT - it may be highly addictive :sunglasses:

[^2]: [Measuring Code Execution Time on ARM Cortex-M MCUs](https://www.iar.com/knowledge/learn/programming/measuring-code-execution-time-on-arm-cortex-m-mcus/) and [STM32 - How to enable DWT Cycle counter](https://stackoverflow.com/questions/36378280/stm32-how-to-enable-dwt-cycle-counter)

## Delay between invoking HAL_SPI_TransmitReceive() or HAL_SPI_TransmitReceive_IT() and the first SPI CLK edge
Now measure the delay between invoking HAL_SPI_TransmitReceive() or HAL_SPI_TransmitReceive_IT() and the first SPI CLK edge. Eye-opening?

## Time needed to reach HAL_TIM_PeriodElapsedCallback() or HAL_SPI_TxRxCpltCallback()
And now measure the delay e.g. between the last CLK edge and reaching HAL_SPI_TxRxCpltCallback(). Eye-opening? See [/Assets/Images/](/Assets/Images/) for some of my results.

## else if vs. switch() case
Switch is generally faster.

## Debugging mode
Remember that pausing the core does not pause/freeze the peripherals by default. If you need to stop also a selected peripheral, you can use macros provided in the library. For example:
```c
#ifdef FREEZE_TIM4
__HAL_DBGMCU_FREEZE_TIM4();
#else
__HAL_DBGMCU_UNFREEZE_TIM4();
#endif
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
```

> [!CAUTION]
> Freezing the core stops the execution of a control algorithm. Do it only if you are sure what you are doing regarding your particular plant connected in the feedback loop with the uC you are debugging.

## The verdict and the hint
What is your verdict regarding the superiority of the non-blocking mode vs. the blocking one? Remember that the safest response is: It depends :wink: The verdict probably will depend on the number of bytes to be received. It may happen that the presented non-blocking mode with all the needed callbacks imposes such an overhead that the blocking part of the non-blocking code is comparable to the blocking solution if the baud rate is high enough.

# Missing files?
Don't worry :slightly_smiling_face: Just hit Alt-K to generate /Drivers/CMCIS/ and /Drivers/STM32G4xx_HAL_Driver/ based on the .ioc file. After a couple of seconds your project will be ready for building.

# Libraries
* CUI Devices encoder: [AMT22 Library](https://github.com/SDibla/Arduino-STM32-AMT22_Library) (MIT license)
* OLED: [stm32-ssd1306](https://github.com/afiskon/stm32-ssd1306) (MIT license)

# Hardware
* [Bidirectional 4-channel logic level shifter](https://sklep.msalamon.pl/produkt/konwerter-poziomow-logicznych-33v-5v-4-kanalowy/)
* [OLED display 1.3" (SH1106 or SSD1306)](https://sklep.msalamon.pl/produkt/wyswietlacz-oled-13-i2c-bialy/)
* [AMT Cable AMT-06C-1-036](https://www.mouser.pl/ProductDetail/CUI-Devices/AMT-06C-1-036?qs=fAHHVMwC%252BbhzTLN9MYRSPg%3D%3D)

# Tools
* [DSLogic Plus](https://www.dreamsourcelab.com/product/dslogic-series/) or any other logic analyzer capable of operating faster than HCLK the MCU core is clocked by. "What the eye doesn't see, the heart doesn't grieve over" does not apply here.

# Call for action
Create your own [home laboratory/workshop/garage](http://ufnalski.edu.pl/control_engineering_for_hobbyists/2024_dzien_otwarty_we/Dzien_Otwarty_WE_2024_Control_Engineering_for_Hobbyists.pdf)! Get inspired by [ControllersTech](https://www.youtube.com/@ControllersTech), [DroneBot Workshop](https://www.youtube.com/@Dronebotworkshop), [Andreas Spiess](https://www.youtube.com/@AndreasSpiess), [GreatScott!](https://www.youtube.com/@greatscottlab), [ElectroBOOM](https://www.youtube.com/@ElectroBOOM), [Phil's Lab](https://www.youtube.com/@PhilsLab), [atomic14](https://www.youtube.com/@atomic14), [That Project](https://www.youtube.com/@ThatProject), [Paul McWhorter](https://www.youtube.com/@paulmcwhorter), and many other professional hobbyists sharing their awesome projects and tutorials! Shout-out/kudos to all of them!

> [!WARNING]
> Control in power electronics and drives - do try this at home :exclamation:

190+ challenges to start from: [Control Engineering for Hobbyists at the Warsaw University of Technology](http://ufnalski.edu.pl/control_engineering_for_hobbyists/Control_Engineering_for_Hobbyists_list_of_challenges.pdf).

Stay tuned!
