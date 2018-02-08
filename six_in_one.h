extern uint16_t crc_send; 
extern uint16_t crc_resp; 
extern uint8_t addrflag;
extern uint8_t dataReadAddr[8];
extern uint8_t dataRespAddr[7];
extern uint8_t dataReadValue[8];
extern uint8_t dataRespValue[19];    


#define CO2_H 			    dataRespValue[3]			
#define CO2_L               dataRespValue[4]
#define	TVOC_H              dataRespValue[5]
#define TVOC_L              dataRespValue[6]
#define	CH2O_H			    dataRespValue[7]
#define	CH2O_L			    dataRespValue[8]
#define	PM25_H			    dataRespValue[9]
#define	PM25_L			    dataRespValue[10]
#define	HUMIDITY_H		    dataRespValue[11]
#define	HUMIDITY_L		    dataRespValue[12]
#define	TEMPERATURE_H		dataRespValue[13]
#define	TEMPERATURE_L		dataRespValue[14]
#define	PM10_H	   			dataRespValue[15]
#define	PM10_L	    		dataRespValue[16]

extern void handle_data(uint8_t *ptr, int len);
