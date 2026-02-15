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

#define FILE_SIZE_LIMIT 2000000

const size_t bufferSize = N_CHUNKS * CHUNK_SIZE;
uint8_t buffer[bufferSize];
volatile int lastRead = -1;
volatile int lastWritten = -1;

File files[2];
int idx;

volatile int curr = -1;
volatile int prev = -1;
volatile int next = -1;

volatile uint32_t dataSize = 0;
volatile uint32_t prevFileSize = 0;

const int BUTTON_PIN = 0;

void writeWavHeader(File file, uint32_t dataSize) {
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
    uint16_t audioFormat = 1; // PCM
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

String getNextFileName() {
    char name[12];
    for (int i = 1; i <= 9999; i++) {
        sprintf(name, "/%04d.wav", i);
        if (!SD.exists(name)) {
            return String(name);
        }
    }
    return String();
}

int openWavFile(const char* name) {
    String autoName;
    if (name == NULL) {
        autoName = getNextFileName();
        name = autoName.c_str();
    }
    files[idx] = SD.open(name, FILE_WRITE);
    if (!files[idx]) {
        return -1;
    }
    for (int i = 0; i < 44; i++)
        files[idx].write((uint8_t)0);
    idx = !idx;
    return !idx;
}

void writeTask(void *pvParameters) {
    esp_task_wdt_add(NULL);

    while (1) {
        if (lastRead == lastWritten) {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            esp_task_wdt_reset();
            continue;
        }
        int startChunk = lastWritten + 1;
        int endChunk = lastRead <= lastWritten ? N_CHUNKS - 1 : lastRead;

        int writeStart = startChunk * CHUNK_SIZE;
        int writeEnd = (endChunk + 1) * CHUNK_SIZE;
        int writeSize = writeEnd - writeStart;

        size_t wrote = files[curr].write(buffer + writeStart, writeSize);
        dataSize += writeSize;
        lastWritten = endChunk == N_CHUNKS - 1 ? -1 : endChunk;

        if (next != -1) {
            prevFileSize = dataSize;
            dataSize = 0;
            prev = curr;
            curr = next;
            next = -1;
            __sync_synchronize();
        }

        esp_task_wdt_reset();
    }
}

void readTask(void *pvParameters) {
    curr = openWavFile(NULL);
    if (curr == -1) {
        Serial.println("Could not open file");
        while (1) {}
    }
    dataSize = 0;
    Serial.println("Starting recording");

    int start = millis();
    while (millis() - start <= 20000) {
        if (dataSize >= FILE_SIZE_LIMIT && next == -1) {
            Serial.println("Writing to new file");
            next = openWavFile(NULL);
            Serial.printf("Writing at index %d\n", next);
            if (next == -1) {
                Serial.println("Could not open file");
                while (1) {}
            }
        }

        if (prevFileSize && prev != -1) {
            writeWavHeader(files[prev], prevFileSize);
            files[prev].close();
            prev = -1;
            prevFileSize = 0;
        }

        size_t readStart = ((lastRead + 1) * CHUNK_SIZE) % bufferSize;
        size_t bytesRead;
        i2s_read(I2S_NUM_0, buffer + readStart, CHUNK_SIZE, &bytesRead, portMAX_DELAY);

        if (bytesRead > 0) {
            lastRead = (lastRead + 1) % N_CHUNKS;
        }
    }

    // wait for writing to finish
    while (lastRead != lastWritten) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    writeWavHeader(files[curr], dataSize);
    files[curr].close();
    Serial.println("done");
    while(1);
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

    // Create tasks
    TaskHandle_t Task1;
    xTaskCreatePinnedToCore(writeTask, // Task function
                            "write",   // Name
                            8192,      // Stack size (words)
                            NULL,      // Parameters
                            1,         // Priority
                            &Task1,    // Task handle
                            0          // Core (0 or 1)
    );

    TaskHandle_t Task2;
    xTaskCreatePinnedToCore(readTask, // Task function
                            "read",   // Name
                            4096,     // Stack size (words)
                            NULL,     // Parameters
                            1,        // Priority
                            &Task2,   // Task handle
                            1         // Core (0 or 1)
    );

    Serial.println("Setup done");
}

void loop() {}
