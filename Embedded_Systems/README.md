## 嵌入式开发

**实验环境**

使用 EK-TM4C123GXL 开发板，Keil 开发工具。

主要综合了数字逻辑，计算机组成原理，计算机体系结构以及软件开发等知识，对嵌入式设备常见的IO，器件进行熟悉设计开发。

![img](img/tm4c123.png)



### 实验相关

**1、TM4C123GXL 开发板的熟悉**

包括实验平台搭建，安装驱动，电路连接等。实现了 LED 灯的交替闪烁。

利用有限资源对实验进行扩展，实现双控开关，流水灯等。

**2、解锁 GPIO，交通灯设计。**

了解开发板的 GPIO，对端口进行解锁。

根据端口对应的物理地址，对寄存器进行修改，需要查阅相关资料，理解不同寄存器的功能。

**3、UART 串口通信**

使用GPIO作为输入输出，使用串口使两块板子进行通信，进行相互控制。需要时钟同步，所以需要使用PLL稳定系统时钟。

**4、有限状态机实现交通灯控制**

同样使用 GPIO 作为输出连接LED 模拟交通灯，使用状态机实现交通灯状态的自动切换。可以使用Delay进行模拟延时，也可以使用SysTick精确计时。

**5、中断+DAC完成声音输出**

利用系统中断，通过中断服务程序执行发声功能，声音是通过DAC将数字信号转为模拟信号产生的，可以将音符对应频率转换为中断输出频率，加上适当的时间延迟可以实现音频播放。

**6、综合实验**

综合所学内容，完成小游戏开发，实验综合了GPIO，中断，ADC，DAC，计时，以及LCD的画面显示。



### 相关概念

GPIO 引脚可配置为数字 I/O，模拟输入，定时器 I/O 或串行 I/O 一个允许软件读取外部数字信号的的输入端口只能读，输出端口可以像正常存储器一样 参与读取和写入周期 。

```c
void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}
```



UART（Universalasynchronousreceiver/transmitter，通用异步收发传输器）： 是一种异步收发传输器，将要传输的资料在串行通信与并行通信之间加以转换，异步的 并允许双向通信。

```c
void UART_Init(void){
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
  UART0_IBRD_R = 43;                    // IBRD = int(80,000,000 / (16 * 115200)) = int(43.402778)
  UART0_FBRD_R = 26;                    // FBRD = round(0.402778 * 64) = 26
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0_CTL_R |= UART_CTL_UARTEN;       // enable UART
  GPIO_PORTA_AFSEL_R |= 0x03;           // enable alt funct on PA1-0
  GPIO_PORTA_DEN_R |= 0x03;             // enable digital I/O on PA1-0
                                        // configure PA1-0 as UART
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)+0x00000011;
  GPIO_PORTA_AMSEL_R &= ~0x03;          // disable analog functionality on PA
}
```



SysTick ：SysTick 是一个简单的计数器，我们可以用它来创建时间延迟并产生周期性中断 。

```c
// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;                   // disable SysTick during setup
  NVIC_ST_RELOAD_R = NVIC_ST_RELOAD_M;  // maximum reload value
  NVIC_ST_CURRENT_R = 0;                // any write to current clears it
                                        // enable SysTick with core clock
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
}
// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 20 nsec for 50 MHz clock)
void SysTick_Wait(unsigned long delay){
  volatile unsigned long elapsedTime;
  unsigned long startTime = NVIC_ST_CURRENT_R;
  do{
    elapsedTime = (startTime-NVIC_ST_CURRENT_R)&0x00FFFFFF;
  }
  while(elapsedTime <= delay);
}
// Time delay using busy wait.
// This assumes 50 MHz system clock.
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(500000);  // wait 10ms (assumes 50 MHz clock)
  }
}
```



PLL： 锁相环（PLL:Phase-lockedloops）是一种利用反馈（Feedback）控制原理实现的频率及 相位的同步技术，其作用是将电路输出的时钟与其外部的参考时钟保持同步。当参考时钟的 频率或相位发生改变时，锁相环会检测到这种 变化，并且通过其内部的反馈系统来调节输 出频率，直到两者重新同步，这种同步又称为“锁相”（Phase-locked）。

```c
// configure the system to get its clock from the PLL
void PLL_Init(void){
  // 0) configure the system to use RCC2 for advanced features
  //    such as 400 MHz PLL and non-integer System Clock Divisor
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;// configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;// clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;// configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  // 4) set the desired system divider and the system divider least significant bit
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) // clear system clock divider field
                  + (SYSDIV2<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}
```

ADC（Analogtodigitalconverter,measureanalogsignals）：用来测量模拟信号的振幅，在数据采集系统中起重要作用 AnalogComparator（Comparetwoanalogsignals）： 模拟比较器用来对比两个模拟输入，并根据更大的模拟输入来产生一个数字化输出 。

PWM（Pulsewidthmodulation，脉冲宽度调制）： 利用微处理器的数字输出来对模拟电路进行控制的一种非常有效的技术，PWM 输出用 于将可变功率应用于电机接口，在典型电机控制中，输入捕捉来测量转速 。

SSI（Synchronousserialinterface，串行外设接口）： 是各类 DSP 处理器中的常见接口，用于中等速度的 I/O 设备 。

I2C（Inter-integratedcircuit，集成电路总线）： 这种总线类型是一种简单、双向、二线制、同步串行总线，主要是用来连接整体电路， 用于低速外围设备。

Timer（Periodicinterrupts,inputcapture,andoutputcompare）： inputcapture和outputcompare 用来创建周期性中断并且测量周期、脉冲宽度、相位和频率 。

QEI（Quadratureencoderinterface，正交编码器接口）： 用来作为与无刷直流电动机的接口 USB（Universalserialbus，通用串行总线）： 是一种高速串行通信通道 。

CAN（Controllerareanetwork，一种串行通信协议）： 用来创建一个在微控制器和机器之间的高速通信通道，常用于分布式控制系统的应用。



### 总结

嵌入式的开发是多变的，但基本思想是相通的，理解基础理论知识，了解相应硬件资源，以不变应万变，才能适应各种变换的设计和开发。

