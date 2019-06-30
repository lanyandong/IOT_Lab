### RFID 自动读卡实验

掌握 RFID 模块自动识别 IC 卡工作原理。

使用 IAR SWSTM8 1.30  软件打开对应实验工程，对实验核心代码进行理解分析，确保实验逻辑与功能的正确性。

验证无误后进行烧录，配置串口，连接电脑主机与实验模块。使用软件“串口猎人”对程序进行检验，使用 IC 卡验证是否可以读出数据。



**实验基本流程**

通过RFID模块自动识别IC卡，对读取到的数据进行相关处理（通过串口输出到PC），PC也可以通过串口调试工具向模块发送指令进行相关操作。

可以通过简单的修改代码，实现一些相应的功能，需要理解核心代码，以及数据发送的数据格式，只有了解了基本流程修改起来才不费劲。对于程序的编译，烧录，以及串口调试，和其他常规实验差不多。

可以贴一段简单的修改的代码：

```C
//按键一次实现加减自己的学号
INTERRUPT_HANDLER(EXTI_PORTC_IRQHandler, 5)
{
  /* In order to detect unexpected events during development,
     it is recommended to set a breakpoint on the following instruction.
  */
	disableInterrupts();
    //判断是否PC1引脚触发
    if ((GPIO_ReadInputData(GPIOC) & GPIO_PIN_1) == 0x00)
    {
        
      if(!Uart_RecvFlag){
        rx_buf[4] = 2;
        rx_buf[8] = 0x08;
        rx_buf[7] = 0x45;
        rx_buf[6] = 0x16;
        rx_buf[5] = 0x20;
        Uart_RecvFlag = 1;
      }
    }
}


//串口发送数据完成学号的扣款与充值：

INTERRUPT_HANDLER(UART2_RX_IRQHandler, 21)
{
    u8 data;
    if(UART2_GetITStatus(UART2_IT_RXNE )!= RESET)  
    {
        disableInterrupts();
        data = UART2_ReceiveData8();
        if (!Uart_RecvFlag)
        {
            rx_buf[rx_counter+4] = data;
            switch (rx_counter)
            {
                case 0:
			        if (data == 0x01||data == 0x02)     rx_counter = 1;
        	        break;
                case 1:
                    rx_counter = 2;
        	        break;
                case 2:
                    rx_counter = 3;
        	        break;
                case 3: 
					rx_counter = 4;
        	        break;
                case 4: 
					rx_counter = 0;
					Uart_RecvFlag = 1;
        	        break;
                default:
                    rx_counter = 0;
                    break;
            }
  	    }
        else
            rx_counter = 0;
		
        enableInterrupts();
    }
}
```



**总结**

实验主要是理解RFID模块的功能原理，以及数据传输和模块控制大致流程。其他的开发环境可能就不太适用了，重点理解其思想。修改代码时对代码的理解要求比较高，很多时候修改的也都是面向用户相对简单的功能代码，对于底层复杂的功能基本上都由协议栈写好了。





 

