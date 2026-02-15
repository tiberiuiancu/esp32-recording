#include "esp_task_wdt.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>

#define I2S_DIN 33
#define I2S_BCK 14
#define I2S_WS 27
#define I2S_MCK 26

#define SD_CS 5
#define SD_MOSI 18
#define SD_SCK 19
#define SD_MISO 21

#define SAMPLE_RATE 48000
#define CHANNELS 2
#define BITS_PER_SAMPLE 32

#define N_CHUNKS 64
#define CHUNK_SIZE 512

// 1GB
#define FILE_SIZE_LIMIT 1000000000

const size_t bufferSize = N_CHUNKS * CHUNK_SIZE;
uint8_t buffer[bufferSize];
volatile int lastRead = N_CHUNKS - 1;
int lastWritten = N_CHUNKS - 1;

File file;
uint32_t dataSize = 0;
int fileCounter = 0;
volatile bool recording = true;

const int BUTTON_PIN = 0;

void writeWavHeader(uint32_t dataSize) {
    uint32_t fileSize = dataSize + 36;
    uint32_t byteRate = SAMPLE_RATE * CHANNELS * (BITS_PER_SAMPLE / 8);
    uint16_t blockAlign = CHANNELS * (BITS_PER_SAMPLE / 8);

    file.seek(0);

    const uint8_t riff[4] = {'R', 'I', 'F', 'F'};
    const uint8_t wave[4] = {'W', 'A', 'V', 'E'};
    const uint8_t fmt[4] = {'f', 'm', 't', ' '};
    const uint8_t data[4] = {'d', 'a', 't', 'a'};

    file.write(riff, 4);
    file.write((uint8_t *)&fileSize, 4);
    file.write(wave, 4);

    file.write(fmt, 4);
    uint32_t subChunk1Size = 16;
    uint16_t audioFormat = 1;
    uint16_t channels = CHANNELS;
    uint32_t sampleRate = SAMPLE_RATE;
    uint16_t bitsPerSample = BITS_PER_SAMPLE;

    file.write((uint8_t *)&subChunk1Size, 4);
    file.write((uint8_t *)&audioFormat, 2);
    file.write((uint8_t *)&channels, 2);
    file.write((uint8_t *)&sampleRate, 4);
    file.write((uint8_t *)&byteRate, 4);
    file.write((uint8_t *)&blockAlign, 2);
    file.write((uint8_t *)&bitsPerSample, 2);

    file.write(data, 4);
    file.write((uint8_t *)&dataSize, 4);
}

void initFileCounter() {
    char name[12];
    for (int i = 1; i <= 9999; i++) {
        sprintf(name, "/%04d.wav", i);
        if (!SD.exists(name)) {
            fileCounter = i;
            return;
        }
    }
}

bool openWavFile() {
    char name[12];
    sprintf(name, "/%04d.wav", fileCounter++);
    Serial.printf("Opening %s\n", name);
    file = SD.open(name, FILE_WRITE);
    if (!file) return false;
    writeWavHeader(0xFFFFFFFF);
    return true;
}

void closeWavFile() {
    file.close();
    Serial.printf("Closed (%u bytes)\n", dataSize);
    dataSize = 0;
}

void writeTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    if (!openWavFile()) {
        Serial.println("Could not open file");
        while (1) {}
    }
    Serial.println("Starting recording");
    digitalWrite(2, HIGH);

    while (1) {
        if (lastRead == lastWritten) {
            if (!recording) {
                closeWavFile();
                Serial.println("done");
                while (1) { esp_task_wdt_reset(); }
            } else if (dataSize >= FILE_SIZE_LIMIT) {
                // only move to new file if we've caught up to writing all the buffers
                closeWavFile();
                if (!openWavFile()) {
                    Serial.println("Could not open file");
                    while (1) {}
                }
            }

            vTaskDelay(1 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        }

        int startChunk = (lastWritten + 1) % N_CHUNKS;
        int endChunk = (startChunk <= lastRead) ? lastRead : N_CHUNKS - 1;

        int writeStart = startChunk * CHUNK_SIZE;
        int writeSize = (endChunk - startChunk + 1) * CHUNK_SIZE;

        size_t wrote = file.write(buffer + writeStart, writeSize);
        if (wrote == 0) {
            Serial.println("SD write failed, retrying...");
            int retries = 3;
            while (wrote == 0 && retries-- > 0) {
                vTaskDelay(10 / portTICK_PERIOD_MS);
                wrote = file.write(buffer + writeStart, writeSize);
            }
            if (wrote == 0) {
                Serial.println("SD write failed, closing file");
                closeWavFile();
                if (!openWavFile()) {
                    Serial.println("Could not open new file");
                    while (1) {}
                }
            }
        }
        dataSize += wrote;
        lastWritten = endChunk;

        esp_task_wdt_reset();
    }
}

void readTask(void *pvParameters) {
    int start = millis();
    while (1) {
        size_t readStart = ((lastRead + 1) * CHUNK_SIZE) % bufferSize;
        size_t bytesRead;
        i2s_read(I2S_NUM_0, buffer + readStart, CHUNK_SIZE, &bytesRead, portMAX_DELAY);

        if (bytesRead > 0) {
            lastRead = (lastRead + 1) % N_CHUNKS;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(2, OUTPUT);
    // pinMode(0, INPUT_PULLUP);

    SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
    SPI.setFrequency(40 * 1000 * 1000);
    if (!SD.begin(SD_CS, SPI)) {
        Serial.println("SD failed");
        while (1)
            ;
    }

    initFileCounter();

    i2s_config_t i2s_config = {.mode =
                                   (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
                               .sample_rate = SAMPLE_RATE,
                               .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
                               .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
                               .communication_format = I2S_COMM_FORMAT_I2S,
                               .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
                               .dma_buf_count = 32,
                               .dma_buf_len = 512,
                               .use_apll = true,
                               .tx_desc_auto_clear = false,
                               .fixed_mclk = 0};

    i2s_pin_config_t pin_config = {.bck_io_num = I2S_BCK,
                                   .ws_io_num = I2S_WS,
                                   .data_out_num = I2S_PIN_NO_CHANGE,
                                   .data_in_num = I2S_DIN};

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);

    TaskHandle_t Task1;
    xTaskCreatePinnedToCore(writeTask,
                            "write",
                            8192,
                            NULL,
                            1,
                            &Task1,
                            0
    );

    TaskHandle_t Task2;
    xTaskCreatePinnedToCore(readTask,
                            "read",
                            4096,
                            NULL,
                            1,
                            &Task2,
                            1
    );

    Serial.println("Setup done");
}

void loop() {}
