/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: Mar 20, 2024
 *      Author: ImadF
 */

#include"stm32f407xx.h"

//Helper Prototypes
static void I2CGenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2CExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddress);
static void I2CExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddress); //WARNING HERE
static void I2CClearAddrFlag(I2C_Handle_t *pI2Chandle);
static void I2CGenerateStartCondition(I2C_RegDef_t* pI2Cx);

uint32_t RCC_GetPCLK1Value(void);


//----------------------Helper Definitions--------------------------------
static void I2CExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress &= ~1;
	pI2Cx->DR = SlaveAddress; // Slave Addr + R/nW bit = 0
}

static void I2CExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddress) //ERROR HERE
{
	SlaveAddress = SlaveAddress << 1;
	SlaveAddress |= 1;
	pI2Cx->DR = SlaveAddress; // Slave Addr + R/nW bit = 0
}

static void I2CClearAddrFlag(I2C_Handle_t *pI2Chandle )
{
	if(pI2Chandle->pI2Cx->SR2 & 1 << I2C_SR2_MSL)
	{
		if(pI2Chandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2Chandle->RxSize == 1)
			{
				//disable acking
				I2C_ManageAcking(pI2Chandle->pI2Cx, I2C_ACK_DISABLE);

				//clear the addr flag
				uint32_t dummy = pI2Chandle->pI2Cx->SR1;
				 dummy = pI2Chandle->pI2Cx->SR2;
				(void) dummy;
			}
		}

		else
		{
			uint32_t dummy = pI2Chandle->pI2Cx->SR1;
			 dummy = pI2Chandle->pI2Cx->SR2;
			(void) dummy;
		}
	}

	else //In Slave mode
	{
		uint32_t dummy = pI2Chandle->pI2Cx->SR1;
		 dummy = pI2Chandle->pI2Cx->SR2;
		(void) dummy;
	}
}

void I2CGenerateStopCondition(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2CGenerateStartCondition(I2C_RegDef_t* pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}





/*IRQ COnfiguration and ISR Handling*/
void I2C_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi) //WARNING HERE
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//touch ISER0
			*NVIC_ISER0 |= (1 << IRQNumber);
		}

		else if (IRQNumber >= 32 && IRQNumber < 64)

		{
			//touch ISER1
			*NVIC_ISER1 |= (1 << IRQNumber%32);
		}

		else if (IRQNumber >= 64 && IRQNumber <96)
		{
			//touch ISER2
			*NVIC_ISER2 |= (1 << IRQNumber%64);
		}
}

	else
	{
		if(IRQNumber <= 31)
			{
				//touch ICER0
			*NVIC_ICER0 |= (1 << IRQNumber);
			}

		else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			//touch ICER1
			*NVIC_ICER1 |= (1 << IRQNumber%32);
		}

		else if (IRQNumber >= 64 && IRQNumber <96)
		{
			//touch ICER2
			*NVIC_ICER2 |= (1 << IRQNumber%64);
		}
	}
}
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority ) //WARNING HERE
{
	uint8_t offset = IRQNumber/4;  //reg number 0-59
	uint8_t temp2 = IRQNumber % 4; // reg section

	//(*NVIC_PR_BASE_ADDR + offset*4) &= ~(0xFF << (temp2*8));
	*(NVIC_PR_BASE_ADDR + offset) |= IRQPriority << ((temp2*8)+4);
}

// some general functions
void I2C_PeripheralControl(I2C_RegDef_t* pI2CX, uint8_t EnorDi) //WARNING HERE
{
	if(EnorDi == ENABLE)
		{
			pI2CX->CR1 |= (1 << I2C_CR1_PE);
		}
		else
		{
			pI2CX->CR1 &= ~(1 << I2C_CR1_PE);
		}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName) // WARNING HERE
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

//Manage Acking/Nacking

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

void I2C_PeriCLockCOntrol(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}

		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}

		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}

/*Init and Deinit*/

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//initiate peri clock
	I2C_PeriCLockCOntrol(pI2CHandle->pI2Cx, ENABLE);  //DONE

	//configure the ACK  bit in CR1 register
	uint32_t temp = 0;
	temp |= ( (pI2CHandle->I2C_Config.I2C_ACKControl) << 10);
	pI2CHandle->pI2Cx->CR1 = temp;


	//configure the CR2 register for FREQ bits
	temp = RCC_GetPCLK1Value() / 1000000U ;
	 pI2CHandle->pI2Cx->CR2 |= temp;


	 //configure the device address if it is in slave mode
	 temp = pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
	 pI2CHandle->pI2Cx->OAR1 |= temp;

	 pI2CHandle->pI2Cx->OAR1 |= 1 << 14; //maintain bit14 as 1 (manual says that)


	 //configure ccr with ccr bits
	 uint32_t ccr = 0;
	 uint32_t duty =  pI2CHandle->I2C_Config.I2C_FMDutyCycle;


	 if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	 {
		 pI2CHandle->pI2Cx->CCR &= ~(1 << 15); // configure standard mode
		 ccr  = (RCC_GetPCLK1Value()) / (2*pI2CHandle->I2C_Config.I2C_SCL_Speed );
	 }
	 else
	 {
		 pI2CHandle->pI2Cx->CCR |= (1 << 15); // configure fast mode

		 if(duty == I2C_FM_DUTY2)
		 {
			 pI2CHandle->pI2Cx->CCR &= ~(1 << 14); // configure duty
			 ccr  = (RCC_GetPCLK1Value()) / (3*pI2CHandle->I2C_Config.I2C_SCL_Speed );
		 }

		 else
		 {
			 pI2CHandle->pI2Cx->CCR |= (1 << 14); // configure duty
			 ccr  = (RCC_GetPCLK1Value()) / (25*pI2CHandle->I2C_Config.I2C_SCL_Speed );
		 }
	 }

	 pI2CHandle->pI2Cx->CCR |= ccr;

	 //Configuration of Trise
 	 uint32_t trise = 0;
	 if(pI2CHandle->I2C_Config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	 	 {
		 	 //Standard Mode
		 	 trise = ( RCC_GetPCLK1Value() / 1000000U ) +1;
		 	 pI2CHandle->pI2Cx->TRISE |= trise;
	 	 }
	 else
	 {
		 //mode is fast mode
		 trise = ( (300* RCC_GetPCLK1Value()) / 1000000000U ) + 1;
		 pI2CHandle->pI2Cx->TRISE |= trise;
	 }
}

void I2C_Deinit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REGS_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REGS_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REGS_RESET();
	}
}

/*Data Send */

void I2C_MasterSendData(I2C_Handle_t* pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t SlaveAddress, uint8_t Sr )
{
	//1. Generate the START  Condition
	I2CGenerateStartCondition(pI2CHandle->pI2Cx);

	//2. Confirm that START generation is completed checking the SB  flag in SR1
		//NOTE: Until SB is cleared, SCLK will be stretched (pulled to LOW).
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_SB ))); //SR1 read

	//3. Send the address of Slave (8 bit) along with R/nW bit with the latter set to W(0) (8 bit). This should clear SB
	I2CExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddress);

	//4. Confirm that address has been received by reading the ADDR flag in SR1 (1 on success)
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx ,I2C_FLAG_ADDR)));

	//5. Clear the ADDR  flag by reading SR1 and SR2
	  // NOTE: Until ADDR is cleared SCLK will be stretched (pulled to LOW).
	I2CClearAddrFlag(pI2CHandle);


	//6. Send the Data until len becomes 0.
	while(len > 0)
	{
		while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));

		pI2CHandle->pI2Cx->DR = *(pTxBuffer);
		pTxBuffer++;
		len--;
	}

	//7. When len becomes 0 WAIT for TXE = 1 and BTF = 1 before generating the STOP condition
	  // NOTE: TXE = 1 & BTF = 1 means both Shift Reg and DR are empty and next transmission should begin
	  // When BTF = 1, SCL will be stretched (pulled to LOW)
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF_)));

	//8. Generate STOP condition and master does not need to wait for the the completion of STOP condition.
	  // NOTE: Generating STOP automatically clears the BTF.
	if(Sr == I2C_DISABLE_SR)
	{
		I2CGenerateStopCondition(pI2CHandle->pI2Cx);
	}
	//I2CGenerateStopCondition(pI2CHandle->pI2Cx);

}

//============================== DATA RECEIVE=============================================

void I2C_MasterReceiveData(I2C_Handle_t* pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t SlaveAddress, uint8_t Sr)
{
	//1. Generate START Condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);

	//2. Confirm that START Generation is completed by reading the SB bit of SR1
	  // NOTE: Until SB is cleared by reading the SR, SCL will be stretched i.e pulled low.
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) );

	//3. Send the address of the slave with the R/nW bit set to 1
	I2CExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddress);

	//4. Check the ADDR  Flag to see if Address is sent successfully.
	while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)) );

	 //Resolve according to length of data to be sent.

	 //procedure to read one byte
	if(len == 1)
	 {
		//Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);


		 //Clear ADDR  flag by reading SR1 & SR2
		I2CClearAddrFlag(pI2CHandle);

		 // wait until RXNE is set
		 while( !(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)) );

		 //Set the STOP condition bit
		 if(Sr == I2C_DISABLE_SR)
		 {
			I2CGenerateStopCondition(pI2CHandle->pI2Cx);
		 }

		 //Read DR into a buffer variable
		 *pRxBuffer = pI2CHandle->pI2Cx->DR;
	 }

	 if(len > 1)
	 {
		 //Clear ADDR  flag by reading SR1 & SR2
		I2CClearAddrFlag(pI2CHandle);

		 //read until len  becomes zero
		 for(int i = len; i>0; i--)
		 {
			 // wait until RXNE is set
			 while( ! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

			 if(i == 2)
			 {
				 //Disable Acking
				 I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
				 //Set the STOP condition bit
				 if(Sr == I2C_DISABLE_SR)
				 {
					I2CGenerateStopCondition(pI2CHandle->pI2Cx);
				 }

			 //Read DR into a buffer variable
			 *pRxBuffer = pI2CHandle->pI2Cx->DR;

			 //increment the buffer variable pointer
			 pRxBuffer++;

			 }
		 }
	  }

	//re enable acking
	 if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	 {
		 I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	 }
}

// --------------------------------INTERRUPT BASED SEND/RECEIVE APIs--------------------------------------------

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint8_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX ;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2CGenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITEVFEN;

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITBUFEN;

	}

	return busystate;

}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer ;
		pI2CHandle->RxLen = Len; 										//CONFLICTING Data types in header file
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITERREN;

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= I2C_CR2_ITEVFEN;

		//Implement code to Generate START Condition
		I2CGenerateStartCondition(pI2CHandle->pI2Cx);

	}

	return busystate;
}

void I2CMasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle)
{

	if(pI2CHandle->RxSize == 1)
	{
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//disable acking
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//1. Generate stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			I2CGenerateStopCondition(pI2CHandle->pI2Cx);
		}

		//2. Close I2C Rx communication
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_RX_CMPLT );
	}
}

void I2C_EV_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	// =====Interrupt handling for both Master & Slave mode of a device=====

	//1. Handle for interrupt generated by SB Event
	  // NOTE: SB Flag is only applicable for MASTER state

	uint32_t temp1, temp2, temp3;
	temp1 = pI2CHandle->pI2Cx->CR1 & 1 << I2C_CR2_ITEVFEN;
	temp2 = pI2CHandle->pI2Cx->CR1 & 1 << I2C_CR2_ITBUFEN;

	temp3 = pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_SB;
	if(temp1 && temp3)
	{
		//SB is set
		uint32_t dummy =  pI2CHandle->pI2Cx->SR1;
		(void) dummy;
		//execute address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX )
		{
			I2CExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

		else
		{
			I2CExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	//2.Handle for event generated by ADDR event
	  //NOTE: When in MASTER mode: Address is sent
	  //      When in SLAVE mode: Address matched with own address

	temp3 = pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_ADDR;
	if(temp1 && temp3)
	{
		//ADDR is set
		I2CClearAddrFlag(pI2CHandle);

	}

	//3. Handle for interrupt generated for BTF Event

	temp3 = pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_BTF;

	if(temp1 && temp3)
	{
		//BTF is set

		if( (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) && (pI2CHandle->pI2Cx->SR1 |=  I2C_FLAG_TXE) )
		{
			// BTF, TXE = 1
			//1. Generate STOP Condition
			if(pI2CHandle->TxLen == 0)
			{

				if(pI2CHandle->Sr == I2C_DISABLE_SR)
				{
					I2CGenerateStopCondition(pI2CHandle->pI2Cx);
				}

				//2. Reset all member elements of the handle structure
				I2C_CloseSendData(pI2CHandle);

				//3. Notify the application about transmission complete
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_TX_CMPLT );
			}
		}

		else if( (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)  && (pI2CHandle->pI2Cx->SR1 |=  I2C_FLAG_RXNE))
		{
			// BTF, RXNE = 1
			//nothing to do
		}
	}


	//4.Handle for interrupt generated for STOPF event
	  //NOTE: STOP detection flag is applicable only in SLAVE mode.
	  //	  For Master, this flag will never be set

	temp3 = pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_STOPF;
	if(temp1 && temp3)
	{
		//STOPF is set in SLAVE
		//Clear it by first reading SR1 then writing blank to CR1
		//read is done for SR1
		//write to CR1
		pI2CHandle->pI2Cx->CR1 |= 0x0000;
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_STOP );

	}

	//5. Handle for interrput generated by TXE event
	temp3 = pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_TXE;
	if(temp1 && temp2 && temp3)

		//TXE is set
	{
		if(pI2CHandle->pI2Cx->SR2 & 1 << I2C_SR2_MSL)
		{
			if( (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) && (pI2CHandle->TxLen>0) )
			{
				//1. Load data in DR
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

				//2. Decrement len
				(pI2CHandle->TxLen)--;

				//3. Increment BufferAddress
				(pI2CHandle->pTxBuffer)++;
			}
		}
		else
		{
			// SLAVE MODE

			//make sure device slave in tx mode
			if(pI2CHandle->pI2Cx->SR2 & 1 << I2C_SR2_TRA )
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}


	//6. Handle for interrput generated by RXNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & 1 << I2C_SR1_RXNE;
	if(temp1 && temp2 && temp3)
	{
		if(pI2CHandle->pI2Cx->SR2 & 1 << I2C_SR2_MSL)
		{
			//RXNE is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2CMasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// SLAVE MODE

			//make sure device slave in Rx mode
			if( !(pI2CHandle->pI2Cx->SR2 & 1 << I2C_SR2_TRA) )
			{
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

 void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle)
{
	//Implement the code to diable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to diable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxSize = 0;
	pI2CHandle->RxLen = 0;
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

 void I2C_CloseSendData(I2C_Handle_t* pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer= NULL;
	pI2CHandle->TxLen = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}

}


void I2C_ER_IRQHandling(I2C_Handle_t* pI2CHandle)
{
	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

		//Implement the code to notify the application about the error
		 I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_ARLO);

	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_AF);

	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

		//Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_OVR);

	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error

		//Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallBack(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data )
{
	pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t) (pI2Cx->DR);
}


//Application Callbacks (weak)
__weak void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{

}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 |= I2C_CR2_ITEVFEN;

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 |= I2C_CR2_ITBUFEN;
	}
	else
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	}
}


