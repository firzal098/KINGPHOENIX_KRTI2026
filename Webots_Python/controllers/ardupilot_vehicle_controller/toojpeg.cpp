// //////////////////////////////////////////////////////////
// toojpeg.cpp
// written by Stephan Brumme, 2018-2019
// see https://create.stephan-brumme.com/toojpeg/
//
// Fast, lightweight JPEG encoder in C++
// //////////////////////////////////////////////////////////

#include "toojpeg.hpp"
#include <algorithm>
#include <cmath>
#include <functional>

namespace TooJpeg {

namespace {

const unsigned char ZigZagInv[] = {
     0,  1,  8, 16,  9,  2,  3, 10,
    17, 24, 32, 25, 18, 11,  4,  5,
    12, 19, 26, 33, 40, 48, 41, 34,
    27, 20, 13,  6,  7, 14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36,
    29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46,
    53, 60, 61, 54, 47, 55, 62, 63
};

const unsigned char DefaultQuantLuminance[64] = {
    16, 11, 10, 16, 24, 40, 51, 61,
    12, 12, 14, 19, 26, 58, 60, 55,
    14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62,
    18, 22, 37, 56, 68,109,103, 77,
    24, 35, 55, 64, 81,104,113, 92,
    49, 64, 78, 87,103,121,120,101,
    72, 92, 95, 98,112,100,103, 99
};

const unsigned char DefaultQuantChrominance[64] = {
    17, 18, 24, 47, 99, 99, 99, 99,
    18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99
};

struct BitCode {
    BitCode() = default;
    BitCode(uint16_t code_, uint8_t numBits_) : code(code_), numBits(numBits_) {}
    uint16_t code = 0;
    uint8_t  numBits = 0;
};

struct HuffmanTable {
    BitCode huffman[256];
};

void generateHuffmanTable(const uint8_t* numCodes, const uint8_t* values, HuffmanTable& table) {
    uint16_t code = 0;
    int k = 0;
    for (uint8_t numBits = 1; numBits <= 16; ++numBits) {
        for (uint8_t i = 0; i < numCodes[numBits - 1]; ++i) {
            table.huffman[values[k++]] = BitCode(code++, numBits);
        }
        code <<= 1;
    }
}

// Standard Huffman Tables (ITU-T T.81, K.3)
const uint8_t DcLuminanceNumCodes[16] = { 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t DcLuminanceValues[12]   = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

const uint8_t DcPixChrominanceNumCodes[16] = { 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
const uint8_t DcPixChrominanceValues[12]   = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

const uint8_t AcLuminanceNumCodes[16] = { 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
const uint8_t AcLuminanceValues[162]  = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};

const uint8_t AcChrominanceNumCodes[16] = { 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
const uint8_t AcChrominanceValues[162]  = {
    0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};

struct BitWriter {
    explicit BitWriter(WriteByte writeByte_) : writeByte(writeByte_) {}
    WriteByte writeByte;
    uint32_t bitBuffer = 0;
    int32_t  numBits = 0;

    void write(uint16_t code, uint8_t bits) {
        bitBuffer |= static_cast<uint32_t>(code) << (32 - numBits - bits);
        numBits += bits;
        while (numBits >= 8) {
            uint8_t oneByte = static_cast<uint8_t>(bitBuffer >> 24);
            writeByte(oneByte);
            if (oneByte == 0xFF) {
                writeByte(0);
            }
            bitBuffer <<= 8;
            numBits -= 8;
        }
    }

    void write(const BitCode& data) {
        write(data.code, data.numBits);
    }

    void flush() {
        if (numBits > 0) {
            uint8_t oneByte = static_cast<uint8_t>(bitBuffer >> 24);
            writeByte(oneByte);
            if (oneByte == 0xFF) {
                writeByte(0);
            }
            bitBuffer = 0;
            numBits = 0;
        }
    }
};

void DCT(const float* block, float* result) {
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 8; ++x) {
            float sum = 0;
            for (int u = 0; u < 8; ++u) {
                for (int v = 0; v < 8; ++v) {
                    sum += block[v * 8 + u] *
                           std::cos((2 * u + 1) * x * 3.14159265f / 16.0f) *
                           std::cos((2 * v + 1) * y * 3.14159265f / 16.0f);
                }
            }
            float cu = (x == 0) ? 0.70710678f : 1.0f;
            float cv = (y == 0) ? 0.70710678f : 1.0f;
            result[y * 8 + x] = 0.25f * cu * cv * sum;
        }
    }
}

BitCode convertValue(int16_t value) {
    if (value == 0) {
        return BitCode(0, 0);
    }
    uint8_t numBits = 0;
    uint16_t temp = static_cast<uint16_t>(value < 0 ? -value : value);
    while (temp > 0) {
        numBits++;
        temp >>= 1;
    }
    uint16_t mask = static_cast<uint16_t>((1 << numBits) - 1);
    uint16_t code = static_cast<uint16_t>(value < 0 ? (value - 1) & mask : value);
    return BitCode(code, numBits);
}

void encodeBlock(BitWriter& bitWriter, const float* block, int16_t& lastDC,
                 const HuffmanTable& dcTable, const HuffmanTable& acTable,
                 const uint8_t* quantTable) {
    float dct[64];
    DCT(block, dct);

    int16_t quantized[64];
    for (int i = 0; i < 64; ++i) {
        quantized[i] = static_cast<int16_t>(std::round(dct[ZigZagInv[i]] / quantTable[i]));
    }

    int16_t dcDiff = quantized[0] - lastDC;
    lastDC = quantized[0];
    BitCode dcCode = convertValue(dcDiff);
    bitWriter.write(dcTable.huffman[dcCode.numBits]);
    bitWriter.write(dcCode);

    int zeroRun = 0;
    for (int i = 1; i < 64; ++i) {
        if (quantized[i] == 0) {
            zeroRun++;
        } else {
            while (zeroRun >= 16) {
                bitWriter.write(acTable.huffman[0xF0]); // ZRL
                zeroRun -= 16;
            }
            BitCode acCode = convertValue(quantized[i]);
            uint8_t symbol = static_cast<uint8_t>((zeroRun << 4) | acCode.numBits);
            bitWriter.write(acTable.huffman[symbol]);
            bitWriter.write(acCode);
            zeroRun = 0;
        }
    }
    if (zeroRun > 0) {
        bitWriter.write(acTable.huffman[0x00]); // EOB
    }
}

} // namespace

bool writeJpeg(WriteByte output, const void* pixels, unsigned short width, unsigned short height,
               bool isRGB, unsigned char quality, bool downsample, const char* comment) {
    (void)downsample;
    (void)comment;
    if (!output || !pixels || width == 0 || height == 0) {
        return false;
    }

    quality = std::clamp<unsigned char>(quality, 1, 100);
    uint32_t scale = (quality < 50) ? (5000 / quality) : (200 - quality * 2);

    uint8_t quantLuminance[64];
    uint8_t quantChrominance[64];
    for (int i = 0; i < 64; ++i) {
        uint32_t valL = (DefaultQuantLuminance[i] * scale + 50) / 100;
        quantLuminance[i] = static_cast<uint8_t>(std::clamp<uint32_t>(valL, 1, 255));
        uint32_t valC = (DefaultQuantChrominance[i] * scale + 50) / 100;
        quantChrominance[i] = static_cast<uint8_t>(std::clamp<uint32_t>(valC, 1, 255));
    }

    HuffmanTable dcLuminanceTable, acLuminanceTable;
    HuffmanTable dcChrominanceTable, acChrominanceTable;
    generateHuffmanTable(DcLuminanceNumCodes, DcLuminanceValues, dcLuminanceTable);
    generateHuffmanTable(AcLuminanceNumCodes, AcLuminanceValues, acLuminanceTable);
    if (isRGB) {
        generateHuffmanTable(DcPixChrominanceNumCodes, DcPixChrominanceValues, dcChrominanceTable);
        generateHuffmanTable(AcChrominanceNumCodes, AcChrominanceValues, acChrominanceTable);
    }

    // SOI
    output(0xFF); output(0xD8);

    // APP0
    output(0xFF); output(0xE0);
    output(0x00); output(0x10); // Length = 16
    output('J'); output('F'); output('I'); output('F'); output(0x00);
    output(0x01); output(0x01); // Version 1.1
    output(0x00);               // No density unit
    output(0x00); output(0x01); // X density = 1
    output(0x00); output(0x01); // Y density = 1
    output(0x00); output(0x00); // No thumbnail

    // DQT Luminance
    output(0xFF); output(0xDB);
    output(0x00); output(67);   // Length
    output(0x00);               // Table 0, 8-bit
    for (int i = 0; i < 64; ++i) {
        output(quantLuminance[ZigZagInv[i]]);
    }

    // DQT Chrominance
    if (isRGB) {
        output(0xFF); output(0xDB);
        output(0x00); output(67);
        output(0x01);           // Table 1, 8-bit
        for (int i = 0; i < 64; ++i) {
            output(quantChrominance[ZigZagInv[i]]);
        }
    }

    // SOF0 (Baseline DCT)
    output(0xFF); output(0xC0);
    output(0x00); output(static_cast<uint8_t>(8 + (isRGB ? 3 : 1) * 3));
    output(0x08); // 8-bit precision
    output(static_cast<uint8_t>(height >> 8)); output(static_cast<uint8_t>(height & 0xFF));
    output(static_cast<uint8_t>(width >> 8));  output(static_cast<uint8_t>(width & 0xFF));
    output(isRGB ? 3 : 1); // Components
    // Y
    output(0x01); output(0x11); output(0x00);
    if (isRGB) {
        // Cb
        output(0x02); output(0x11); output(0x01);
        // Cr
        output(0x03); output(0x11); output(0x01);
    }

    // DHT - DC Luminance
    output(0xFF); output(0xC4);
    output(0x00); output(static_cast<uint8_t>(3 + 16 + sizeof(DcLuminanceValues)));
    output(0x00); // DC Table 0
    for (int i = 0; i < 16; ++i) output(DcLuminanceNumCodes[i]);
    for (size_t i = 0; i < sizeof(DcLuminanceValues); ++i) output(DcLuminanceValues[i]);

    // DHT - AC Luminance
    output(0xFF); output(0xC4);
    output(0x00); output(static_cast<uint8_t>(3 + 16 + sizeof(AcLuminanceValues)));
    output(0x10); // AC Table 0
    for (int i = 0; i < 16; ++i) output(AcLuminanceNumCodes[i]);
    for (size_t i = 0; i < sizeof(AcLuminanceValues); ++i) output(AcLuminanceValues[i]);

    if (isRGB) {
        // DHT - DC Chrominance
        output(0xFF); output(0xC4);
        output(0x00); output(static_cast<uint8_t>(3 + 16 + sizeof(DcPixChrominanceValues)));
        output(0x01); // DC Table 1
        for (int i = 0; i < 16; ++i) output(DcPixChrominanceNumCodes[i]);
        for (size_t i = 0; i < sizeof(DcPixChrominanceValues); ++i) output(DcPixChrominanceValues[i]);

        // DHT - AC Chrominance
        output(0xFF); output(0xC4);
        output(0x00); output(static_cast<uint8_t>(3 + 16 + sizeof(AcChrominanceValues)));
        output(0x11); // AC Table 1
        for (int i = 0; i < 16; ++i) output(AcChrominanceNumCodes[i]);
        for (size_t i = 0; i < sizeof(AcChrominanceValues); ++i) output(AcChrominanceValues[i]);
    }

    // SOS
    output(0xFF); output(0xDA);
    output(0x00); output(static_cast<uint8_t>(6 + (isRGB ? 3 : 1) * 2));
    output(isRGB ? 3 : 1);
    output(0x01); output(0x00); // Y: DC table 0, AC table 0
    if (isRGB) {
        output(0x02); output(0x11); // Cb: DC table 1, AC table 1
        output(0x03); output(0x11); // Cr: DC table 1, AC table 1
    }
    output(0x00); output(0x3F); output(0x00); // Spectral selection

    // Encode scan data
    BitWriter bitWriter(output);
    int16_t lastDC_Y = 0, lastDC_Cb = 0, lastDC_Cr = 0;
    const uint8_t* bytePixels = static_cast<const uint8_t*>(pixels);

    for (unsigned short y = 0; y < height; y += 8) {
        for (unsigned short x = 0; x < width; x += 8) {
            float blockY[64], blockCb[64], blockCr[64];

            for (int by = 0; by < 8; ++by) {
                for (int bx = 0; bx < 8; ++bx) {
                    unsigned short px = std::min<unsigned short>(x + bx, width - 1);
                    unsigned short py = std::min<unsigned short>(y + by, height - 1);
                    size_t idx = (py * width + px) * (isRGB ? 3 : 1);

                    if (isRGB) {
                        float r = bytePixels[idx + 0];
                        float g = bytePixels[idx + 1];
                        float b = bytePixels[idx + 2];
                        blockY[by * 8 + bx]  = 0.299f * r + 0.587f * g + 0.114f * b - 128.0f;
                        blockCb[by * 8 + bx] = -0.168736f * r - 0.331264f * g + 0.5f * b;
                        blockCr[by * 8 + bx] = 0.5f * r - 0.418688f * g - 0.081312f * b;
                    } else {
                        blockY[by * 8 + bx] = bytePixels[idx] - 128.0f;
                    }
                }
            }

            encodeBlock(bitWriter, blockY, lastDC_Y, dcLuminanceTable, acLuminanceTable, quantLuminance);
            if (isRGB) {
                encodeBlock(bitWriter, blockCb, lastDC_Cb, dcChrominanceTable, acChrominanceTable, quantChrominance);
                encodeBlock(bitWriter, blockCr, lastDC_Cr, dcChrominanceTable, acChrominanceTable, quantChrominance);
            }
        }
    }

    bitWriter.flush();

    // EOI
    output(0xFF); output(0xD9);
    return true;
}

static thread_local std::vector<uint8_t>* g_outputTarget = nullptr;

static void appendByteToVector(unsigned char oneByte) {
    if (g_outputTarget) {
        g_outputTarget->push_back(oneByte);
    }
}

bool encodeJpeg(const void* pixels, unsigned short width, unsigned short height,
                bool isRGB, unsigned char quality, std::vector<uint8_t>& outputBuffer) {
    outputBuffer.clear();
    outputBuffer.reserve(width * height / (isRGB ? 4 : 8));
    g_outputTarget = &outputBuffer;
    bool ok = writeJpeg(appendByteToVector, pixels, width, height, isRGB, quality);
    g_outputTarget = nullptr;
    return ok;
}

} // namespace TooJpeg
