// ----------------------------------------------------------------------------------------------------------------
//
//	
//
// ----------------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// I2C definitions

#define I2C_SLAVE					0x0703
#define I2C_SMBUS					0x0720	// SMBus-level access

#define I2C_SMBUS_READ				1
#define I2C_SMBUS_WRITE				0

// SMBus transaction types

#define I2C_SMBUS_QUICK		    	0
#define I2C_SMBUS_BYTE		    	1
#define I2C_SMBUS_BYTE_DATA	    	2
#define I2C_SMBUS_WORD_DATA	    	3
#define I2C_SMBUS_PROC_CALL	    	4
#define I2C_SMBUS_BLOCK_DATA	    5
#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
#define I2C_SMBUS_BLOCK_PROC_CALL   7 // SMBus 2.0
#define I2C_SMBUS_I2C_BLOCK_DATA    8

// SMBus messages

#define I2C_SMBUS_BLOCK_MAX			32	// As specified in SMBus standard
#define I2C_SMBUS_I2C_BLOCK_MAX		32	// Not specified but we use same structure

// Structures used in the ioctl() calls

union i2c_smbus_data {
	uint8_t  byte;
	uint16_t word;
	uint8_t  block[I2C_SMBUS_BLOCK_MAX + 2];	// block [0] is used for length + one more for PEC
};

struct i2c_smbus_ioctl_data {
	char read_write;
	uint8_t command;
	int size;
	union i2c_smbus_data *data;
};

// Device File Access

static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size, union i2c_smbus_data *data) {
	struct i2c_smbus_ioctl_data args;
	args.read_write = rw;
	args.command    = command;
	args.size       = size;
	args.data       = data;
	return ioctl(fd, I2C_SMBUS, &args);
}

// ----------------------------------------------------------------------------------------------------------------
//
//	
//
// ----------------------------------------------------------------------------------------------------------------

// --- DEVICE ADDRESS
#define DS3231_ADDR					0x68

#define DS3231_RD_ADDR				((DS3231_ADDR << 1) | 0x01)
#define DS3231_WR_ADDR				((DS3231_ADDR << 1) & 0xFE)

// --- ADDRESS MAP
#define TIME_SECONDS_REG			0x00
#define TIME_MINUTES_REG			0x01
#define TIME_HOURS_REG				0x02
#define TIME_DAY_REG				0x03
#define TIME_DATE_REG				0x04
#define TIME_MONTH_REG				0x05
#define TIME_YEAR_REG				0x06
#define ALARM1_SECONDS_REG			0x07
#define ALARM1_MINUTES_REG			0x08
#define ALARM1_HOURS_REG			0x09
#define ALARM1_DAY_DATE_REG			0x0A
#define ALARM2_MINUTES_REG			0x0B
#define ALARM2_HOURS_REG			0x0C
#define ALARM2_DAY_DATE_REG			0x0D
// --- SPECIAL REGISTERS
#define CONTROL_REG					0x0E
#define STATUS_REG					0x0F
#define AGE_OFFSET_REG				0x10
#define TEMP_MSB_REG				0x11
#define TEMP_LSB_REG				0x12

// --- TIME FORMAT

#define FORMAT_24H					0
#define FORMAT_12H					1
#define AM							0
#define PM							1
#define DAY							0
#define DATE						1

#define START_UP					0
#define SHUT_DOWN					1

static volatile int fd = -1;

unsigned char bcd_to_decimal(unsigned char bcd) {
	return ((bcd & 0x0F) + (((bcd & 0xF0) >> 4) * 10));
}

unsigned char decimal_to_bcd(unsigned char decimal) {
	return (((decimal / 10) << 4) & 0xF0) | ((decimal % 10) & 0x0F);
}

unsigned char DS3231_read(unsigned char address) {
	union i2c_smbus_data data;
	if (i2c_smbus_access(fd, I2C_SMBUS_READ, address, I2C_SMBUS_BYTE_DATA, &data)) return -1;
	return data.byte & 0xFF;
}

int DS3231_write(unsigned char address, unsigned char value) {
	union i2c_smbus_data data;
	data.byte = value;
	return i2c_smbus_access(fd, I2C_SMBUS_WRITE, address, I2C_SMBUS_BYTE_DATA, &data);	
}

int DS3231_init(void) {
	const char *device ;
    device = "/dev/i2c-1" ;
	if ((fd = open (device, O_RDWR)) < 0) return -1; //printf("Unable to open I2C device: %s\n", strerror(errno));
	if (ioctl (fd, I2C_SLAVE, DS3231_ADDR) < 0) return -2; //printf("Unable to select I2C device: %s\n", strerror(errno));
	return 0;
}

void DS3231_getTime(unsigned char *hours, unsigned char *minutes, unsigned char *seconds, unsigned char *am_pm_state, unsigned char hour_format) {
	*(seconds) = bcd_to_decimal(DS3231_read(TIME_SECONDS_REG));
	*(minutes) = bcd_to_decimal(DS3231_read(TIME_MINUTES_REG));
	if (hour_format == FORMAT_12H) {
		*(am_pm_state) = ((0x20 & DS3231_read(TIME_HOURS_REG))>>5);
		*(hours) = bcd_to_decimal(0x1F & DS3231_read(TIME_HOURS_REG));
	} else {
		*(hours) = bcd_to_decimal(0x3F & DS3231_read(TIME_HOURS_REG));
	}
}

void DS3231_getDate(unsigned char *day, unsigned char *date, unsigned char *month, unsigned char *year) {
	*(year) = bcd_to_decimal(DS3231_read(TIME_YEAR_REG));
	*(month) = bcd_to_decimal(0x1F & DS3231_read(TIME_MONTH_REG));
    	*(date) = bcd_to_decimal(0x3F & DS3231_read(TIME_DATE_REG));
	*(day) = bcd_to_decimal(0x07 & DS3231_read(TIME_DAY_REG));
}

void DS3231_setTime(unsigned char hours, unsigned char minutes, unsigned char seconds, unsigned char am_pm_state, unsigned char hour_format) {
	DS3231_write(TIME_SECONDS_REG, (decimal_to_bcd(seconds)));
	DS3231_write(TIME_MINUTES_REG, (decimal_to_bcd(minutes)));
	if (hour_format == FORMAT_12H)
		if (am_pm_state == PM)
			DS3231_write(TIME_HOURS_REG, ((0x60 | (0x1F & (decimal_to_bcd(hours))))));
		else
			DS3231_write(TIME_HOURS_REG, ((0x40 | (0x1F & (decimal_to_bcd(hours))))));
	else
		DS3231_write(TIME_HOURS_REG, (0x3F & (decimal_to_bcd(hours))));
}

void DS3231_setDate(unsigned char day, unsigned char date, unsigned char month, unsigned char year) {
	DS3231_write(TIME_DAY_REG, decimal_to_bcd(day));
	DS3231_write(TIME_DATE_REG, decimal_to_bcd(date));
	DS3231_write(TIME_MONTH_REG, decimal_to_bcd(month));
	DS3231_write(TIME_YEAR_REG, decimal_to_bcd(year));
}

float DS3231_getTemp(void) {
	float temp = 0.0;
	unsigned char lowByte = ((DS3231_read(TEMP_LSB_REG) >> 6) & 0x03);
	signed char highByte = DS3231_read(TEMP_MSB_REG);
	signed int value = ((int)(highByte << 2) | lowByte);
	temp = ((float)value);
	temp *= 0.25;
	return temp;
}

// START-UP ALARM (ALARM1)
void DS3231_setAlarm1DateTime(unsigned char seconds, unsigned char minutes, unsigned char hours, unsigned char day_date, unsigned char sel_day_date, unsigned char hour_format, unsigned char am_pm_state) {
	DS3231_write(ALARM1_SECONDS_REG, decimal_to_bcd(seconds));
	DS3231_write(ALARM1_MINUTES_REG, decimal_to_bcd(minutes));
	if (hour_format == FORMAT_12H)
		if (am_pm_state == PM)
			DS3231_write(ALARM1_HOURS_REG, ((0x60 | (0x1F & (decimal_to_bcd(hours))))));
		else
			DS3231_write(ALARM1_HOURS_REG, ((0x40 | (0x1F & (decimal_to_bcd(hours))))));
	else
		DS3231_write(ALARM1_HOURS_REG, (0x3F & (decimal_to_bcd(hours))));
	if (sel_day_date == DAY)
		DS3231_write(ALARM1_DAY_DATE_REG, 0x40 | decimal_to_bcd(day_date));
	else
		DS3231_write(ALARM1_DAY_DATE_REG, decimal_to_bcd(day_date));
}

void DS3231_getAlarm1DateTime(unsigned char *hours, unsigned char *minutes, unsigned char *seconds, unsigned char *day_date, unsigned char *am_pm_state, unsigned char hour_format) {
	*(seconds) = bcd_to_decimal(DS3231_read(ALARM1_SECONDS_REG));
	*(minutes) = bcd_to_decimal(DS3231_read(ALARM1_MINUTES_REG));
	if (hour_format == FORMAT_12H) {
		*(am_pm_state) = ((0x20 & DS3231_read(ALARM1_HOURS_REG))>>5);
		*(hours) = bcd_to_decimal(0x1F & DS3231_read(ALARM1_HOURS_REG));
	} else {
		*(hours) = bcd_to_decimal(0x3F & DS3231_read(ALARM1_HOURS_REG));
	}
	*(day_date) = bcd_to_decimal(0x3F & DS3231_read(ALARM1_DAY_DATE_REG));
}

// SHUT-DOWN ALARM (ALARM2)
void DS3231_setAlarm2DateTime(unsigned char minutes, unsigned char hours, unsigned char day_date, unsigned char sel_day_date, unsigned char hour_format, unsigned char am_pm_state) {
	DS3231_write(ALARM2_MINUTES_REG, decimal_to_bcd(minutes));
	if (hour_format == FORMAT_12H)
		if (am_pm_state == PM)
			DS3231_write(ALARM2_HOURS_REG, ((0x60 | (0x1F & (decimal_to_bcd(hours))))));
		else
			DS3231_write(ALARM2_HOURS_REG, ((0x40 | (0x1F & (decimal_to_bcd(hours))))));
	else
		DS3231_write(ALARM2_HOURS_REG, (0x3F & (decimal_to_bcd(hours))));
	if (sel_day_date == DAY)
		DS3231_write(ALARM2_DAY_DATE_REG, 0x40 | decimal_to_bcd(day_date));
	else
		DS3231_write(ALARM2_DAY_DATE_REG, decimal_to_bcd(day_date));
}

void DS3231_getAlarm2DateTime(unsigned char *hours, unsigned char *minutes, unsigned char *day_date, unsigned char *am_pm_state, unsigned char hour_format) {
	*(minutes) = bcd_to_decimal(DS3231_read(ALARM2_MINUTES_REG));
	if (hour_format == FORMAT_12H) {
		*(am_pm_state) = ((0x20 & DS3231_read(ALARM2_HOURS_REG))>>5);
		*(hours) = bcd_to_decimal(0x1F & DS3231_read(ALARM2_HOURS_REG));
	} else {
		*(hours) = bcd_to_decimal(0x3F & DS3231_read(ALARM2_HOURS_REG));
	}
	*(day_date) = bcd_to_decimal(0x3F & DS3231_read(ALARM2_DAY_DATE_REG));
}

void DS3231_setAlarm(unsigned char alarm) {
	if (alarm == START_UP)
		DS3231_write(CONTROL_REG, (DS3231_read(CONTROL_REG) & 0xFE));
	else
		DS3231_write(CONTROL_REG, (DS3231_read(CONTROL_REG) & 0xFD));
}

void DS3231_clearAlarm(unsigned char alarm) {
	if (alarm == START_UP)
		DS3231_write(CONTROL_REG, (DS3231_read(CONTROL_REG) | 0x05));
	else
		DS3231_write(CONTROL_REG, (DS3231_read(CONTROL_REG) | 0x06));
}

void DS3231_setAlarms(void) {
	DS3231_write(CONTROL_REG, (DS3231_read(CONTROL_REG) | 0x07));
}

void DS3231_clearAlarms(void) {
	DS3231_write(CONTROL_REG, (DS3231_read(CONTROL_REG) & 0x0FC));
}

void DS3231_clearStatus(void) {
	DS3231_write(STATUS_REG, 0x00);
}

// ----------------------------------------------------------------------------------------------------------------
//
//	
//
// ----------------------------------------------------------------------------------------------------------------

#include <stdio.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

#define RASPBERRY_PI_PERI_BASE	0x3F000000
#define GPIO_BASE				(RASPBERRY_PI_PERI_BASE + 0x200000)	// GPIO controller

#define BLOCK_SIZE				(4*1024)

static volatile uint32_t *gpio = NULL;

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x)
#define INP_GPIO(g)   			*(gpio + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)   			*(gpio + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) 		*(gpio + (((g)/10))) |= (((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET  				*(gpio + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR  				*(gpio + 10) // clears bits which are 1 ignores bits which are 0
#define GPIO_READ(g)  			*(gpio + 13) &= (1<<(g))

#define PULL_OFF				0x00
#define PULL_DOWN				0x01
#define PULL_UP					0x02

#define INPUT					0x00
#define OUTPUT					0x01

int GPIO_read(unsigned char pin) {
	if ((int32_t)gpio == -1) return -1;
	return (GPIO_READ(pin & 31)>>pin);
}

int GPIO_write(unsigned char pin, unsigned char  value) {
	if ((int32_t)gpio == -1) return -1;
	switch (value) {
		case 0x00:
			GPIO_CLR = (1 << (pin & 31));
		break;
		case 0x01:
			GPIO_SET = (1 << (pin & 31));
		break;
		default:
			return -2;
		break;
	}
	return 0;
}

int GPIO_inout(unsigned char pin, unsigned char mode) {
	if ((int32_t)gpio == -1) return -1;
	if (mode  == INPUT) {
		INP_GPIO(pin & 31);
	} else {
		INP_GPIO(pin & 31);
		OUT_GPIO(pin & 31);
	}
	return 0;
}

int GPIO_pullUpDown(unsigned char pin, unsigned char mode) {
	if ((int32_t)gpio == -1) return -1;
	*(gpio + 37) = mode;
	*(gpio + 38) = (1 << (pin & 31));
	return 0;
}

int GPIO_init(void) {
	int   fd;
	// Open /dev/mem
	if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0) return -1; // printf("Failed to open /dev/mem, try checking permissions.\n");
	gpio = (uint32_t *)mmap(0, BLOCK_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GPIO_BASE);
	if ((int32_t)gpio == -1) return -1; // printf("Failed mmap (GPIO): %s\n", strerror (errno)); 
	return 0;
}

// ----------------------------------------------------------------------------------------------------------------
//
//	
//
// ----------------------------------------------------------------------------------------------------------------

#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#include <unistd.h>
#include <linux/reboot.h>
#include <sys/reboot.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <string.h>

#define DAEMON_NAME "WittyPiDaemon"

int main(int argc, char *argv[]) {
    //Set our Logging Mask and open the Log
    setlogmask(LOG_UPTO(LOG_NOTICE));
    openlog(DAEMON_NAME, LOG_CONS | LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_USER);

    syslog(LOG_INFO, "Entering Daemon");

    pid_t pid, sid;

   	//Fork the Parent Process
    pid = fork();

    if (pid < 0) { exit(EXIT_FAILURE); }

    //We got a good pid, Close the Parent Process
    if (pid > 0) { exit(EXIT_SUCCESS); }

    //Change File Mask
    umask(0);

    //Create a new Signature Id for our child
    sid = setsid();
    if (sid < 0) { exit(EXIT_FAILURE); }

    //Change Directory
    //If we cant find the directory we exit with failure.
    if ((chdir("/")) < 0) { exit(EXIT_FAILURE); }

    //Close Standard File Descriptors
    close(STDIN_FILENO);
	close(STDOUT_FILENO);
    close(STDERR_FILENO);

	// CHECK RTC
	if (DS3231_init() == -1) return -1;
	
	DS3231_write(CONTROL_REG, 0x07);
	DS3231_write(STATUS_REG, 0x00);
	
	// Get Date Time
	unsigned char hours, minutes, seconds, day, date, month, year;
	DS3231_getTime(&hours, &minutes, &seconds, NULL, FORMAT_24H);
	DS3231_getDate(&day, &date, &month, &year);
	
	//  Update RASPBERY  Date Time
	time_t t = time(NULL);
	struct tm * ptm = localtime(&t);
	if (ptm == NULL) return -1;
	ptm->tm_mon = month - 1;
	ptm->tm_mday = date;
	ptm->tm_year = year + (2000 - 1900);
	ptm->tm_hour = hours;
	ptm->tm_min = minutes;
	ptm->tm_sec = seconds;
	struct timeval tv = { mktime(ptm), 0 };
	settimeofday(&tv, 0);

	// CHECK GPIO
	if (GPIO_init() == -1) return -1;
	
	GPIO_pullUpDown(4, PULL_UP);
	GPIO_inout(4, INPUT);
	GPIO_inout(17, OUTPUT);

	//----------------
	// RUN Task
	//----------------

	while(1) {
		GPIO_write(17, 1);
		sleep(1);

		//syslog (LOG_NOTICE, "TEMPERATURE: %f\r\n", DS3231_getTemp());
		if ((DS3231_read(STATUS_REG)&0x03) == 0x02) {
			DS3231_clearStatus();
			DS3231_write(CONTROL_REG, 0x05);
			//system("shutdown -h now");
			sync();
			reboot(LINUX_REBOOT_CMD_POWER_OFF);
		}

		GPIO_write(17, 0);
		sleep(1);
	}
	syslog(LOG_INFO, "Exiting Daemon");

	//Close the log
	closelog();
}