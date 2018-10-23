#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_spi_flash.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "audio_example_file.h"
#include "esp_adc_cal.h"

// digital amplification for small signal
#define MIC_OFFSET                  0x8000  /* MIC scale offset */
#define MIC_MASK                    0xffff  /* MIC mask */
#define MIC_SCALE                   5       /* MIC amplification scale */

// I2S microphone input
#define MIC_RIGHT_INPUT     0       /* left:0, right:1 */

static const char* TAG = "i2s_mic/da";
#define PARTITION_NAME "storage"

/*---------------------------------------------------------------
                            EXAMPLE CONFIG
---------------------------------------------------------------*/
//enable record sound and save in flash
#define RECORD_IN_FLASH_EN (1)
//enable replay recorded sound in flash
#define REPLAY_FROM_FLASH_EN (1)

//i2s number
#define EXAMPLE_I2S_NUM_TX (0)
//i2s number
#define EXAMPLE_I2S_NUM_RX (1)
//i2s sample rate
#define EXAMPLE_I2S_SAMPLE_RATE (16000)
//i2s TX data bits
#define EXAMPLE_I2S_SAMPLE_BITS_TX (16)
//i2s RX data bits
#define EXAMPLE_I2S_SAMPLE_BITS_RX (24)
//enable display buffer for debug
#define EXAMPLE_I2S_BUF_DEBUG (0)
//I2S read buffer length
#define EXAMPLE_I2S_READ_LEN (16 * 1024)
//I2S tx data format
#define EXAMPLE_I2S_FORMAT_TX (I2S_CHANNEL_FMT_RIGHT_LEFT)
//I2S tx channel number
#define EXAMPLE_I2S_CHANNEL_NUM_TX ((EXAMPLE_I2S_FORMAT_TX < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))
//I2S rx data format
#define EXAMPLE_I2S_FORMAT_RX (I2S_CHANNEL_FMT_RIGHT_LEFT)
//I2S rx channel number
#define EXAMPLE_I2S_CHANNEL_NUM_RX ((EXAMPLE_I2S_FORMAT_RX < I2S_CHANNEL_FMT_ONLY_RIGHT) ? (2) : (1))

//flash record size, for recording 5 seconds' data
#define FLASH_RECORD_SIZE (EXAMPLE_I2S_CHANNEL_NUM_TX * EXAMPLE_I2S_SAMPLE_RATE * EXAMPLE_I2S_SAMPLE_BITS_RX / 8 * 5)
#define FLASH_ERASE_SIZE (FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE == 0) ? FLASH_RECORD_SIZE : FLASH_RECORD_SIZE + (FLASH_SECTOR_SIZE - FLASH_RECORD_SIZE % FLASH_SECTOR_SIZE)
//sector size of flash
#define FLASH_SECTOR_SIZE (0x1000)
//flash read / write address
#define FLASH_ADDR (0x200000)

void example_i2s_mic_init()
{
    /* RX: I2S_NUM_1 */
    i2s_config_t i2s_config_rx = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX, // Only RX
        .sample_rate = EXAMPLE_I2S_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS_RX, // ADMP441(24bit)
        .channel_format = EXAMPLE_I2S_FORMAT_RX,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .dma_buf_count = 16,                     // number of buffers, 128 max.
        .dma_buf_len = 1024,                   // size of each buffer
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1 // Interrupt level 1
    };

    i2s_pin_config_t pin_config_rx = {
        .bck_io_num = GPIO_NUM_17,
        .ws_io_num = GPIO_NUM_18,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = GPIO_NUM_5};

    i2s_driver_install(EXAMPLE_I2S_NUM_RX, &i2s_config_rx, 0, NULL);
    i2s_set_pin(EXAMPLE_I2S_NUM_RX, &pin_config_rx);
}

/**
 * @brief I2S Mic/DAC mode init.
 */

void example_i2s_dac_init()
{
    i2s_config_t i2s_config_tx = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = EXAMPLE_I2S_SAMPLE_RATE,
        .bits_per_sample = EXAMPLE_I2S_SAMPLE_BITS_TX,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = EXAMPLE_I2S_FORMAT_TX,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = 1024};
    //install and start i2s driver
    i2s_driver_install(EXAMPLE_I2S_NUM_TX, &i2s_config_tx, 0, NULL);
    //init DAC pad
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
}

void example_i2s_init()
{
    example_i2s_mic_init();
    example_i2s_dac_init();
}
/*
 * @brief erase flash for recording
 */
void example_erase_flash()
{
#if RECORD_IN_FLASH_EN
    printf("Erasing flash \n");
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                              ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    printf("Erase size: %d Bytes\n", FLASH_ERASE_SIZE);
    ESP_ERROR_CHECK(esp_partition_erase_range(data_partition, 0, FLASH_ERASE_SIZE));
#else
    printf("Skip flash erasing...\n");
#endif
}

/**
 * @brief debug buffer data
 */
void example_disp_buf(uint8_t *buf, int length)
{
#if EXAMPLE_I2S_BUF_DEBUG
    printf("======\n");
    for (int i = 0; i < length; i++)
    {
        printf("%02x ", buf[i]);
        if ((i + 1) % 8 == 0)
        {
            printf("\n");
        }
    }
    printf("======\n");
#endif
}

/**
 * @brief Reset i2s clock and mode
 */
void example_reset_play_mode()
{
    i2s_set_clk(EXAMPLE_I2S_NUM_TX, EXAMPLE_I2S_SAMPLE_RATE, EXAMPLE_I2S_SAMPLE_BITS_TX, EXAMPLE_I2S_CHANNEL_NUM_TX);
}

/**
 * @brief Set i2s clock for example audio file
 */
void example_set_file_play_mode()
{
    i2s_set_clk(EXAMPLE_I2S_NUM_TX, 16000, EXAMPLE_I2S_SAMPLE_BITS_TX, 1);
}

/**
 * @brief Scale data to 16bit/32bit for I2S DMA output.
 *        DAC can only output 8bit data value.
 *        I2S DMA will still send 16 bit or 32bit data, the highest 8bit contains DAC data.
 */
int example_i2s_dac_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
    uint32_t j = 0;
#if (EXAMPLE_I2S_SAMPLE_BITS_TX == 16)
    for (int i = 0; i < len; i++)
    {
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i];
    }
    return (len * 2);
#else
    for (int i = 0; i < len; i++)
    {
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = s_buff[i];
    }
    return (len * 4);
#endif
}

/**
 * @brief Scale data to 8bit for data from Mic.
 *        Data from Mic are 24bit width by default.
 *        DAC can only output 8 bit data.
 *        Scale each 24bit Mic data to 8bit DAC data.
 */
int_least64_t example_i2s_mic_data_scale(uint8_t *d_buff, uint8_t *s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t mic24_value = 0;
    uint16_t msb16_value = 0;

#if (EXAMPLE_I2S_SAMPLE_BITS_TX == 16)
    for (int i = 0; i < len; i +=  (EXAMPLE_I2S_CHANNEL_NUM_RX*(2*((EXAMPLE_I2S_SAMPLE_BITS_RX/8)+1)/2)) )
    {
        /*
        **   mic data format
        **  MSB                                                                                      LSB
        **  [3]                    |[2]                    |[1]                    |[0]  
        **  31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
        **   x  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  o  x  x  x  x  x  x  x
        */
//    printf("[3]=0x%x, [2]=0x%x, [1]=0x%x, [0]=0x%x\n", s_buff[i + 3], s_buff[i + 2], s_buff[i + 1], s_buff[i + 0]);
//        mic24_value = ((((uint32_t)s_buff[i + 3] & 0xff) << 17) | (((uint32_t)s_buff[i + 2] & 0xff) << 9) | (((uint32_t)s_buff[i + 1] & 0xff) << 1) | (((uint32_t)s_buff[i + 0] & 0xff) >> 7)) & 0x00ffffff;  // 24bit
        mic24_value = (((s_buff[i + 3] & 0xff) << 17) | ((s_buff[i + 2] & 0xff) << 9) | ((s_buff[i + 1] & 0xff) << 1) | ((s_buff[i + 0] & 0xff) >> 7)) & 0x00ffffff;  // 24bit
        mic24_value <<= MIC_SCALE;  // amplitude
        msb16_value = (mic24_value >> 8) & MIC_MASK; // MSB 16bit
//    printf("mic24_value=0x%x, msb16_value=0x%x\n", mic24_value, msb16_value);
        msb16_value = (msb16_value + MIC_OFFSET) & MIC_MASK;    // signed -> unsigned

        d_buff[j++] = msb16_value & 0xff;           // LSB 8bit
        d_buff[j++] = (msb16_value >> 8) & 0xff;    // MSB 8bit

        // another channel
        d_buff[j++] = msb16_value & 0xff;           // LSB 8bit
        d_buff[j++] = (msb16_value >> 8) & 0xff;    // MSB 8bit
    }
    return j;
#else
    for (int i = 0; i < len; i += 4)
    {
        mic_value = ((((uint16_t)(s_buff[i + 3] & 0xf) << 8) | ((s_buff[i + 2]))));
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = 0;
        d_buff[j++] = mic_value * 256 / 4096;
    }
    return j;
#endif
}

/**
 * @brief I2S Mic/DAC example
 *        1. Erase flash
 *        2. Record audio from mic and save in flash
 *        3. Read flash and replay the sound via DAC
 *        4. Play an example audio file(file format: 8bit/8khz/single channel)
 *        5. Loop back to step 3
 */
void example_i2s_mic_dac(void *arg)
{
    const esp_partition_t *data_partition = NULL;
    data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                              ESP_PARTITION_SUBTYPE_DATA_FAT, PARTITION_NAME);
    if (data_partition != NULL)
    {
        printf("partiton addr: 0x%08x; size: %d; label: %s\n", data_partition->address, data_partition->size, data_partition->label);
    }
    else
    {
        ESP_LOGE(TAG, "Partition error: can't find partition name: %s\n", PARTITION_NAME);
        vTaskDelete(NULL);
    }
    //1. Erase flash
    example_erase_flash();
    int i2s_read_len = EXAMPLE_I2S_READ_LEN;
    int flash_wr_size = 0;
    size_t bytes_read, bytes_written;

    //2. Record audio from ADC and save in flash
#if RECORD_IN_FLASH_EN
    char *i2s_read_buff = (char *)calloc(i2s_read_len, sizeof(char));
    uint8_t *flash_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
    //    i2s_adc_enable(EXAMPLE_I2S_NUM_TX);
    while (flash_wr_size < FLASH_RECORD_SIZE)
    {
        //read data from I2S bus, in this case, from Mic.
        i2s_read(EXAMPLE_I2S_NUM_RX, (void *)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        //save original data from I2S(Mic) into flash.
        esp_partition_write(data_partition, flash_wr_size, i2s_read_buff, i2s_read_len);
        flash_wr_size += i2s_read_len;
//        example_disp_buf((uint8_t *)i2s_read_buff, i2s_read_len);
        example_disp_buf((uint8_t *)i2s_read_buff, 32);
        ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    }
    //    i2s_adc_disable(EXAMPLE_I2S_NUM_TX);
    free(i2s_read_buff);
    i2s_read_buff = NULL;
    free(flash_write_buff);
    flash_write_buff = NULL;
#endif

    uint8_t *flash_read_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
    uint8_t *i2s_write_buff = (uint8_t *)calloc(i2s_read_len, sizeof(char));
    while (1)
    {

        //3. Read flash and replay the sound via DAC
#if REPLAY_FROM_FLASH_EN
        for (int rd_offset = 0; rd_offset < flash_wr_size; rd_offset += FLASH_SECTOR_SIZE)
        {
            //read I2S(ADC) original data from flash
            esp_partition_read(data_partition, rd_offset, flash_read_buff, FLASH_SECTOR_SIZE);
            //process data and scale to 8bit for I2S DAC.
            example_disp_buf((uint8_t *)flash_read_buff, 64);
            int i2s_wr_len = example_i2s_mic_data_scale(i2s_write_buff, flash_read_buff, FLASH_SECTOR_SIZE);
            //send data
            i2s_write(EXAMPLE_I2S_NUM_TX, i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
            example_disp_buf((uint8_t *)i2s_write_buff, 32);
            printf("playing: %d %%\n", rd_offset * 100 / flash_wr_size);
        }
#endif

        //4. Play an example audio file(file format: 8bit/16khz/single channel)
        printf("Playing file example: \n");
        int offset = 0;
        int tot_size = sizeof(audio_table);
        example_set_file_play_mode();
        while (offset < tot_size)
        {
            int play_len = ((tot_size - offset) > (4 * 1024)) ? (4 * 1024) : (tot_size - offset);
            int i2s_wr_len = example_i2s_dac_data_scale(i2s_write_buff, (uint8_t *)(audio_table + offset), play_len);
            i2s_write(EXAMPLE_I2S_NUM_TX, i2s_write_buff, i2s_wr_len, &bytes_written, portMAX_DELAY);
            offset += play_len;
            example_disp_buf((uint8_t *)i2s_write_buff, 32);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        example_reset_play_mode();
    }
    free(flash_read_buff);
    free(i2s_write_buff);
    vTaskDelete(NULL);
}

/*
void adc_read_task(void *arg)
{
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_11db);
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, &characteristics);
    while (1)
    {
        uint32_t voltage;
        esp_adc_cal_get_voltage(ADC1_TEST_CHANNEL, &characteristics, &voltage);
        ESP_LOGI(TAG, "%d mV", voltage);
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}
*/

esp_err_t app_main()
{
    example_i2s_init();
    esp_log_level_set("I2S", ESP_LOG_INFO);
    xTaskCreate(example_i2s_mic_dac, "example_i2s_mic_dac", 1024 * 2, NULL, 5, NULL);
    //    xTaskCreate(adc_read_task, "ADC read task", 2048, NULL, 5, NULL);
    return ESP_OK;
}
