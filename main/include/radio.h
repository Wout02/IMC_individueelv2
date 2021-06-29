#define STREAM_1 "http://open.ls.qingting.fm/live/274/64k.m3u8?format=aac"
#define STREAM_2 "http://playerservices.streamtheworld.com/api/livestream-redirect/RADIO538AAC.aac"
#define STREAM_3 "https://icecast.omroep.nl/radio1-bb-aac"
#define STREAM_4 "http://20043.live.streamtheworld.com:80/SKYRADIOAAC.aac"
#define I2C_MASTER_NUM           I2C_NUM_0
#define I2C_MASTER_TX_BUF_LEN    0                    
#define I2C_MASTER_RX_BUF_LEN    0                     
#define I2C_MASTER_FREQ_HZ       100000
#define I2C_MASTER_SDA_IO        18
#define I2C_MASTER_SCL_IO        23
#define LCD_NUM_ROWS			 4
#define LCD_NUM_COLUMNS			 40
#define LCD_NUM_VIS_COLUMNS		 20

void start_stream(char stream);
void stop_radio();

