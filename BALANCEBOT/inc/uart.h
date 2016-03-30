#define BUFFER_SIZE  10
#define TX_PIN GPIO_Pin_9;
#define RX_PIN GPIO_Pin_10;
#define BAUDRATE 9600;

volatile char received_char;
volatile char buffer[BUFFER_SIZE];

void send_char(char c) {
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, c);
}

void send_string(const char* s) {
	while (*s)
		send_char(*s++);
}

void receive_char(void) {
	while (!USART_GetFlagStatus(USART1, USART_FLAG_RXNE))
		;
	received_char = USART_ReceiveData(USART1);

}
void receive_data(void) {
	int k = 0;
	while (k < BUFFER_SIZE
			&& USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET) {
		*buffer = USART_ReceiveData(USART2);
		k++;
	}
}

void send_data( char *buffer) {
	int i = 0;
	while (buffer[i] != '\0' && i < 10) {
		send_char(buffer[i]);
		i++;
	}
}

int __io_putchar(int a) {
	send_char(a);
	return a;
}

void uart_init() {

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC
					| RCC_APB2Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;

	//uart1 TX line
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = TX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpio);
	//uart1 RX line
	gpio.GPIO_Pin = RX_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpio);
	//uart config
	USART_StructInit(&uart);
	uart.USART_BaudRate = BAUDRATE;
	USART_Init(USART1, &uart);
	USART_Cmd(USART1, ENABLE);
}

