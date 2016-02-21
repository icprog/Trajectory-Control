typedef struct {
	BYTE low;				// Intel processors use little-endian order
	BYTE mid_low;
	BYTE mid_high;
	BYTE high;
	} LONG_BYTES;			// 4 consecutive bytes, for use in breaking up long's

typedef union { 
	unsigned long whole;	// this allows programmer access to entire long
	LONG_BYTES piece;		// or each byte
	} FRACTURED_ULONG;		// fractured unsigned long   

typedef struct {
	BYTE low;				// Intel processors use little-endian order
	BYTE high;
	} SHORT_BYTES;			// 2 consecutive bytes, for use in breaking up short's

typedef union { 
	unsigned short whole;	// this allows programmer access to entire long
	SHORT_BYTES piece;		// or each byte
	} FRACTURED_USHORT;		// fractured unsigned short   

/*************************************************************************
|    Definitions                                                         |
*************************************************************************/


// misc constant definitions
#define MAX_ERROR_MSG_LENGTH	512
#define MAX_INC_COUNTS			0x1000000
#define FAILURE					FALSE		// must be zero
#define SUCCESS					TRUE
#define INVALID_DEVICE			-1
#define INVALID_MODADDRESS		-1
#define DEVICE_NAME_BASE	"\\\\.\\USDUSBB0"
#define MAX_DEVICES			16		// max devices on USB bus
#define MAX_ADDRESSES		256		// address can be any valid single byte value
#define IOCTLCODE			0x222084
#define IOCTLCODE_SEI		0x222094
#define IOCTLCODE_INT		0x222098
#define IOCTLCODE_BUF		0x22209C
#define IOCTLCODE_FLUSH		0x2220A0		/* flush buffer command to driver */
#define MAX_BUFFER_LENGTH	64
#define	MAX_NUM_ENCODERS	4		// max number of encoders per device on USB bus
#define MAX_CHANNEL_MASK	0x03	// mask to limit encoder number to valid bits
#define SEEPROM_USER_AREA	64		/* user area starts at location 64 */
#define SEEPROM_USER_SIZE	64      /* locations 64-127 are available for user definition */
#define SEI_FRAMING_ERROR	0x01	/* if set, then there has been a framing error on SEI port */
#define SEI_OVERRUN_ERROR	0x02	/* if set, then there has been an overrun error on the SEI port */
#define SEI_BUSY_ASSERTED	0x04	/* if set, then busy is asserted on SEI port */
#define SEI_BAUD_2400	0x14
#define SEI_BAUD_4800	0x13
#define	SEI_BAUD_9600	0x12
#define	SEI_BAUD_19200	0x11
#define	SEI_BAUD_38400	0x10
#define SEI_BAUD_57600	0x01


// USB1 command definitions
#define READ_CNTR			0x01
#define RESET_CNTR			0x03
#define RESET_ALL_CNTRS		0x04
#define SET_CNTR_POS		0x05
#define SET_CNTR_MAX		0x06
#define SET_CNTR_MODE		0x07
#define SEI_SET_BAUD		0x10
#define SEI_RESET			0x11
#define SEI_READ_STATUS		0x12
#define READ_INFO			0x20
#define READ_INDEX_ENABLE	0x21
#define SET_INDEX_ENABLE	0x22
#define SAVE_PARAMETERS		0x23
#define READ_INPUT_PORT		0x24
#define SET_OUTPUT_PORT		0x25
#define READ_MODULE_ADDRESS	0x26
#define SET_MODULE_ADDRESS	0x27
#define READ_OUTPUT_PORT	0x28
#define	READ_USER_EEPROM	0x29
#define WRITE_USER_EEPROM	0x2A
#define	READ_LAST_STATUS	0x2B
#define SET_OUTPUT_BITS		0x2C
#define	CLEAR_OUTPUT_BITS	0X2D
#define RESET_HISTORY_BUFF	0x30
#define GET_HISTORY_BUFF_STATUS	0x31
#define SET_SAMPLE_RATE		0x32
#define GET_SAMPLE_RATE		0x33
#define SET_EVENTS			0x34
#define GET_EVENTS			0x35
#define	START_ACQUISITION	0x36
#define	STOP_ACQUISITION	0x37
#define	GET_ACQUISITION_STATUS	0x38
#define SET_TIMESTAMP		0x39
#define READ_POSITION_QUAL	0x3a
#define SET_POSITION_QUAL	0x3b
#define	SET_ACQUIRE_MODE	0x3c


/*************************************************************************
|	 Exported Functions Defined:                                         |
*************************************************************************/
LONG _stdcall USB1IO(UCHAR ModAddress, ULONG ioCtl, UCHAR* BufOut, ULONG SizeOut, UCHAR* BufIn, ULONG SizeIn);
int _stdcall USB1Init(void);
BOOL _stdcall USB1ReturnModuleAddress(UCHAR DeviceNumber, PUCHAR pucAddress);
BOOL _stdcall USB1Close(void);
BOOL _stdcall USB1GetIncPosition(UCHAR ModAddress, UCHAR Encoder, LONG* ulResult);
BOOL _stdcall USB1GetAllPosition(UCHAR ModAddress, ULONG* pulPosition, ULONG *pulTimeStamp, UCHAR *pucParallelInput);
BOOL _stdcall USB1GetBufferedPosition(UCHAR ModAddress, UCHAR Encoder, LONG* ulResult, BOOL *pbDataAvailable);
BOOL _stdcall USB1GetAllBuffered(UCHAR ModAddress, ULONG* ulPosition, ULONG *ulTimeStamp, UCHAR *ucParallelInput, BOOL *pbDataAvailable);
BOOL _stdcall USB1GetIncPositionExtended(UCHAR ModAddress, UCHAR Encoder, LONG* lPos, LONG* lMaxCount, LONG* lTimeStamp, PUCHAR pucCounterMode, PUCHAR pucStatus);
BOOL _stdcall USB1ResetIncPosition(UCHAR ModAddress, UCHAR Encoder);
BOOL _stdcall USB1ResetAllIncPositions(UCHAR ModAddress);
BOOL _stdcall USB1SetIncPosition(UCHAR ModAddress, UCHAR Encoder, ULONG Data);
BOOL _stdcall USB1SetIncMaxCount(UCHAR ModAddress, UCHAR Encoder, ULONG Data);
BOOL _stdcall USB1SetIncMode(UCHAR ModAddress, UCHAR Encoder, UCHAR Mode, UCHAR Prescale);
BOOL _stdcall USB1SetSEIBaudRate(UCHAR ModAddress, ULONG ulBaudRate);
BOOL _stdcall USB1ResetSEI(UCHAR ModAddress);
BOOL _stdcall USB1GetSEIStatus(UCHAR ModAddress, UCHAR *Result);
BOOL _stdcall USB1GetFactoryInfo(UCHAR ModAddress, USHORT *pusModel, USHORT *pusVersion, USHORT *pusConfiguration,
								 ULONG *pulSN, UCHAR *pucMonth, UCHAR *pucDay, USHORT *pusYear);	// get info of device 
BOOL _stdcall USB1GetIndexRegister(UCHAR ModAddress, PUCHAR pucIndex);
BOOL _stdcall USB1SetIndexRegister(UCHAR ModAddress, UCHAR ucNewIndex);
BOOL _stdcall USB1SaveParameters(UCHAR ModAddress);
BOOL _stdcall USB1GetInputPort(UCHAR ModAddress, PUCHAR pucResult);
BOOL _stdcall USB1SetOutputPort(UCHAR ModAddress, UCHAR Data);
BOOL _stdcall USB1SetModuleAddress(UCHAR ucOldModAddress, UCHAR ucNewModAddress);
BOOL _stdcall USB1GetLastErrorAnsi(LPSTR pszResultString, ULONG ulLength);
BOOL _stdcall USB1GetLastErrorVB(LPSTR pszDestination, size_t lngMaxCount);
BOOL _stdcall USB1FlushHistoryBuffer(UCHAR ModAddress);
BOOL _stdcall USB1GetHistoryBufferStatus(UCHAR ModAddress, PUCHAR pucStatus);
BOOL _stdcall USB1SetHistoryBufferRate(UCHAR ModAddress, ULONG ulRate);
BOOL _stdcall USB1GetOutputPort(UCHAR ModAddress, PUCHAR pucOutput);
BOOL _stdcall USB1ReadUserEEPROM(UCHAR ModAddress, ULONG ulLocation, ULONG ulLength, PUCHAR pucData);
BOOL _stdcall USB1WriteUserEEPROM(UCHAR ModAddress, ULONG ulLocation, ULONG ulLength, PUCHAR pucData);
BOOL _stdcall USB1SetOutputBits(UCHAR ModAddress, UCHAR Data);
BOOL _stdcall USB1ClearOutputBits(UCHAR ModAddress, UCHAR Data);
BOOL _stdcall USB1GetHistoryBufferRate(UCHAR ModAddress, PULONG pulPeriod);
BOOL _stdcall USB1SetEvents(UCHAR ModAddress, PUCHAR pucTrigger, UCHAR ucTrigAnd, PUCHAR pucQualifier, UCHAR ucQualAnd, ULONG ulNumberOfSamples);
BOOL _stdcall USB1GetEvents(UCHAR ModAddress, PUCHAR pucTrigger, PUCHAR pucTrigAnd, PUCHAR pucQualifier, PUCHAR pucQualAnd, PULONG pulNumberOfSamples);
BOOL _stdcall USB1StartAcquisition(UCHAR ModAddress);
BOOL _stdcall USB1StopAcquisition(UCHAR ModAddress);
BOOL _stdcall USB1GetAcquisitionStatus(UCHAR ModAddress, PULONG pulSamplesToCollect, PULONG pulSamplesRemaining, PUCHAR pucStatus);
BOOL _stdcall USB1GetDllVersion(PUCHAR pucDllVerHigh, PUCHAR pucDllVerLow);
BOOL _stdcall USB1CommandResponseSEI(UCHAR ModAddress, PUCHAR pucCommand, ULONG ulCommandLength, PUCHAR pucResponse, PULONG pulResponseLength);
BOOL _stdcall USB1SetTimeStamp(UCHAR ModAddress, ULONG ulNewTS);
BOOL _stdcall USB1GetPositionQual(UCHAR ModAddress, PUCHAR pucQualByte);
BOOL _stdcall USB1SetPositionQual(UCHAR ModAddress, UCHAR ucNewQualByte);
BOOL _stdcall USB1SetAcquireMode(UCHAR ModAddress, UCHAR ucNewMode);
/*************************************************************************
|	 Internal Functions Defined:                                         |
*************************************************************************/

#ifdef _DEBUG
#define PRINT(x)	OutputDebugString(x)
#else
#define PRINT(x)
#endif