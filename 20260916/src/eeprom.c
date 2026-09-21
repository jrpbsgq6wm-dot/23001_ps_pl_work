
#include "fcntl.h"
#include "eeprom.h"
#include "main.h"
#include <linux/i2c.h>
#include <linux/i2c-dev.h>



/*******************************************************************************
 * 函数名：EEPROM_Write_Byte
 * 功  能：写一个字节
 * 参  数：Addr要写入的地址
 Data要写入的数据
 * 返回值：无
 * 说  明：器件地址（包含写入命令） -> 1或2个字节WORD ADDR -> 数据
 *******************************************************************************/

TCPDataTypeT ifcfgIP_DRY,ifcfgIP_WET;
int fd_eeprom;



uint8_t I2c_Init(void)
{
    fd_eeprom = open("/dev/i2c-1", O_RDWR);   //允许读写 
    if (fd_eeprom < 0)
    {
        perror("Can't open /dev/i2c-1\n"); //打开iic设备文件失败
        exit(1);
    } 
    if (ioctl(fd_eeprom, I2C_SLAVE_FORCE,EEPROM_DEV_ADDR) < 0)      //设置iic从器件地址
    {
        printf("fail to set i2c device slave address!\n");
        close(fd_eeprom);
        return -1;
    }
    printf("set slave address to 0x%x success!\n", EEPROM_DEV_ADDR);        
    return(1);
}


void WR_Device()
{
	usleep(5000);
	EEPROM_Write_Byte(DRY_DEVICE,"GB400_DRY");
	usleep(5000);
	EEPROM_Write_Byte(DRY_IP,"192.168.0.4");
	usleep(5000);
	EEPROM_Write_Byte(DRY_IP_LEN,"11");
	usleep(5000);
	EEPROM_Write_Byte(DRY_MAC,"00:0a:35:00:01:02");

	I2c_Init();
	usleep(5000);
	EEPROM_Write_Byte(WET_DEVICE,"GB400_WET");
	usleep(5000);
	EEPROM_Write_Byte(WET_IP,"192.168.0.5");
	usleep(5000);
	EEPROM_Write_Byte(WET_IP_LEN,"11");
	usleep(5000);
	EEPROM_Write_Byte(WET_MAC,"00:0a:35:00:00:00");
}

int EEPROM_Write_Byte(uint16_t Addr, uint8_t recv_data[128])
{
	uint8_t buf[150] = {EEPROM_W,(uint8_t)Addr,(uint8_t)(Addr>>8)};
	int i = 0;
	while(1)
	{
		if(recv_data[i] == '\0')
		{
			break;
		}
		buf[3+i]  = recv_data[i];
		++i;
	}
	int ret = write(fd_eeprom,buf,3+i);
	usleep(5000);
	return ret;
}


/*******************************************************************************
 * 函数名：EEPROM_Read_Byte
 * 功  能：读一个字节
 * 参  数：Addr要读取的地址
 * 返回值：Data读出的数据
 * 说  明：无
 *******************************************************************************/
uint8_t EEP_Read_Data[128];

uint8_t EEPROM_Read_Byte(uint16_t Addr)
{
	usleep(5000);
	uint8_t buf[100] = {EEPROM_W,(uint8_t)Addr,(uint8_t)(Addr>>8)};
	write(fd_eeprom,buf,3);
	usleep(5000);
	memset(EEP_Read_Data,0,128);
	read(fd_eeprom,EEP_Read_Data,128);
	return 1;
}

void Eeprom_Init(void)
{
	I2c_Init();	
	bzero(&ifcfgIP_DRY,sizeof(ifcfgIP_DRY));
	EEPROM_Read_Byte(DRY_DEVICE);
	strcpy(ifcfgIP_DRY.device,EEP_Read_Data);//strncmp(ip)
	
	if(strncmp("GB400_DRY",ifcfgIP_DRY.device,9) != 0)
	{
		DBG("the frist init eeprom!\n");
		WR_Device();
	}
	ReadAll();//sendtoupper
}

void ReadAll(void)
{
	bzero(&ifcfgIP_DRY,sizeof(ifcfgIP_DRY));
	EEPROM_Read_Byte(DRY_DEVICE);
	strcpy(ifcfgIP_DRY.device,EEP_Read_Data);
	EEPROM_Read_Byte(DRY_IP_LEN);
	strcpy(ifcfgIP_DRY.len,EEP_Read_Data);
	int i = atoi(ifcfgIP_DRY.len);
	EEPROM_Read_Byte(DRY_IP);
	memcpy(ifcfgIP_DRY.address,EEP_Read_Data,i);
	EEPROM_Read_Byte(DRY_MAC);
	memcpy(ifcfgIP_DRY.mac,EEP_Read_Data,17);

	printf("*****DEVICE_DRY*****%s\n",ifcfgIP_DRY.device);
	printf("*****IP_LEN_DRY*****%s\n",ifcfgIP_DRY.len);
	printf("******IP_DRY********%s\n",ifcfgIP_DRY.address);
	printf("******MAC_DRY*******%s\n",ifcfgIP_DRY.mac);

	bzero(&ifcfgIP_WET,sizeof(ifcfgIP_WET));
	EEPROM_Read_Byte(WET_DEVICE);
	strcpy(ifcfgIP_WET.device,EEP_Read_Data);
	EEPROM_Read_Byte(WET_IP_LEN);
	strcpy(ifcfgIP_WET.len,EEP_Read_Data);
	int j = atoi(ifcfgIP_WET.len);
	EEPROM_Read_Byte(WET_IP);
	memcpy(ifcfgIP_WET.address,EEP_Read_Data,j);
	EEPROM_Read_Byte(WET_MAC);
	memcpy(ifcfgIP_WET.mac,EEP_Read_Data,17);

	printf("*****DEVICE_WET*****%s\n",ifcfgIP_WET.device);
	printf("*****IP_LEN_WET*****%s\n",ifcfgIP_WET.len);
	printf("******IP_WET********%s\n",ifcfgIP_WET.address);
	printf("******MAC_WET*******%s\n",ifcfgIP_WET.mac);

//	WriteRcConfFile("#IFACE0_IP",ifcfgIP_DRY.address);
	system("/etc/init.d/rcS restart");	// ./app alone
}


void ReadDry(void)
{
	bzero(&ifcfgIP_DRY,sizeof(ifcfgIP_DRY));
	EEPROM_Read_Byte(DRY_DEVICE);
	strcpy(ifcfgIP_DRY.device,EEP_Read_Data);
	EEPROM_Read_Byte(DRY_IP_LEN);
	strcpy(ifcfgIP_DRY.len,EEP_Read_Data);
	int i = atoi(ifcfgIP_DRY.len);
	EEPROM_Read_Byte(DRY_IP);
	memcpy(ifcfgIP_DRY.address,EEP_Read_Data,i);
	EEPROM_Read_Byte(DRY_MAC);
	memcpy(ifcfgIP_DRY.mac,EEP_Read_Data,17);

	printf("*****DEVICE_DRY*****%s\n",ifcfgIP_DRY.device);
	printf("*****IP_LEN_DRY*****%s\n",ifcfgIP_DRY.len);
	printf("******IP_DRY********%s\n",ifcfgIP_DRY.address);
	printf("******MAC_DRY*******%s\n",ifcfgIP_DRY.mac);
}

int WriteDryIP(char *newip)
{
//	I2c_Init();
	char p[3];
	char Erasure[128] = {0xFF};
	sprintf(p,"%d",strlen(newip));
	usleep(5000);
	EEPROM_Write_Byte(DRY_IP_LEN,p);
	usleep(5000);
	EEPROM_Write_Byte(DRY_IP,Erasure);
	usleep(5000);
	int ret = EEPROM_Write_Byte(DRY_IP,newip);
	return ret;
}


int WriteWetIP(char *newip)
{
//	I2c_Init();
	char p[3];
	char Erasure[128] = {0xFF};
	sprintf(p,"%d",strlen(newip));
	usleep(5000);
	EEPROM_Write_Byte(WET_IP_LEN,p);
	usleep(5000);
	EEPROM_Write_Byte(WET_IP,Erasure);
	usleep(5000);
	EEPROM_Write_Byte(WET_IP,newip);
	int ret = EEPROM_Write_Byte(DRY_IP,newip);
	return ret;
}

int WriteDryMAC(char *newmac)
{
//	I2c_Init();
	char Erasure[128] = {0xFF};
	usleep(5000);
	EEPROM_Write_Byte(DRY_MAC,Erasure);
	usleep(5000);
	int ret = EEPROM_Write_Byte(DRY_MAC,newmac);
	return ret;
}

int WriteWetMAC(char *newmac)
{
//	I2c_Init();
	char Erasure[128] = {0xFF};
	usleep(5000);
	EEPROM_Write_Byte(DRY_MAC,Erasure);
	usleep(5000);
	int ret = EEPROM_Write_Byte(DRY_MAC,newmac);
	return ret;
}


#define NETWORK_RC_CONF "/etc/init.d/rcS"
#define RC_ARRAY_SIZE 128

void WriteRcConfFile(const char* cfg,const char* value)
{
	FILE* pf = NULL;
	int line_cnt = 0;
	int i = 0;
	char buf[RC_ARRAY_SIZE] = {0};
	char** pp_EachLineBuf = NULL;
	char* pEachLineData = NULL;

	pf = fopen(NETWORK_RC_CONF, "r");
	if (NULL == pf)
	{
		return;
	}
	fseek(pf, 0L, SEEK_SET);

	while (fgets(buf, sizeof(buf), pf) != NULL)
	{
		line_cnt++;//读取每一行
	}

	pp_EachLineBuf = (char**)malloc(line_cnt * sizeof(char**));
	memset(pp_EachLineBuf, 0x00, line_cnt * sizeof(char**));
	for (i = 0; i < line_cnt; i++)
	{

		pp_EachLineBuf[i] = (char*)malloc(RC_ARRAY_SIZE);
		memset(pp_EachLineBuf[i], 0x00, RC_ARRAY_SIZE);
	}

	fseek(pf, 0L, SEEK_SET);
	i = 0;
	pEachLineData = pp_EachLineBuf[i];
	while (fgets(pEachLineData, RC_ARRAY_SIZE, pf) != NULL)
	{
		if (0 == strncmp(pEachLineData, cfg, strlen(cfg)))
		{
			i++;
			pEachLineData = pp_EachLineBuf[i];
			i++;
			fgets(pEachLineData, RC_ARRAY_SIZE, pf);
			pEachLineData = pp_EachLineBuf[i];
			fgets(pEachLineData, RC_ARRAY_SIZE, pf);
			strcpy((pEachLineData + 14), value);
			strcat((pEachLineData + 14 + strlen(value)), "\n\0");
		}
		i++;
		pEachLineData = pp_EachLineBuf[i];
	}
	fclose(pf);
	pf = NULL;
	pf = fopen(NETWORK_RC_CONF, "w");
	fseek(pf, 0L, SEEK_SET);
	ftell(pf);
	for (i = 0; i < line_cnt; i++)
	{
		pEachLineData = pp_EachLineBuf[i];
		fwrite(pEachLineData, sizeof(char), strlen(pEachLineData), pf);
	}
	for (i = 0; i < line_cnt; i++)
	{
		free(pp_EachLineBuf[i]);
	}
	free(pp_EachLineBuf);
	fclose(pf);
	pf = NULL;
}

