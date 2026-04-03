#include "adc_sensor.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;

volatile uint16_t adc_values[4];

void ADC_Sensor_Init(void)
{
    /* MX_ADC1_Init() (called from main) already configured:
     *   4-channel scan + continuous mode, DMA circular on DMA1_Ch1
     *   and linked hadc1.DMA_Handle → hdma_adc1.
     * Here we run self-calibration once, then start the DMA conversion. */
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_values, 4);
}

/* Battery voltage IIR filter */
#define VBAT_FILTER_ALPHA  0.5f
static float bat_filter_prev = 0.0f;
static float bat_filter(float x)
{
    float y = VBAT_FILTER_ALPHA * x + (1.0f - VBAT_FILTER_ALPHA) * bat_filter_prev;
    bat_filter_prev = y;
    return y;
}

/**
 * @brief  Convert ADC raw value to battery voltage.
 * @note   Voltage divider: R_top / R_bot ≈ 15 / 5.1 with actual ratio 1005.1/5.1.
 *         The + 0.45 V offset was calibrated on the original hardware — verify yours.
 */
float ADC_GetBatteryVoltage(void)
{
    float adc = bat_filter((float)adc_values[1]);
    float v   = adc * 3.3f / 4096.0f;
    v = v / 15.0f / 5.1f * 1005.1f;
    v += 0.45f;
    return v;
}

/* -----------------------------------------------------------------------
 * NTC thermistor lookup table  (0–120 °C, 1 °C steps)
 * Assumes 220 kΩ series pull-up resistor on PA5.
 * ----------------------------------------------------------------------- */
typedef struct {
    float temp;
    float r_min;
    float r_avg;
    float r_max;
} TempResistance_t;

static const TempResistance_t r_table[] = {
    { 0, 31.908f, 32.613f, 33.331f}, { 1, 30.341f, 30.996f, 31.662f},
    { 2, 28.860f, 29.468f, 30.086f}, { 3, 27.460f, 28.024f, 28.598f},
    { 4, 26.136f, 26.660f, 27.192f}, { 5, 24.883f, 25.370f, 25.863f},
    { 6, 23.698f, 24.149f, 24.607f}, { 7, 22.576f, 22.994f, 23.419f},
    { 8, 21.513f, 21.901f, 22.295f}, { 9, 20.507f, 20.867f, 21.231f},
    {10, 19.554f, 19.888f, 20.225f}, {11, 18.649f, 18.959f, 19.271f},
    {12, 17.792f, 18.079f, 18.368f}, {13, 16.980f, 17.245f, 17.513f},
    {14, 16.209f, 16.454f, 16.702f}, {15, 15.477f, 15.704f, 15.933f},
    {16, 14.783f, 14.993f, 15.204f}, {17, 14.123f, 14.317f, 14.512f},
    {18, 13.497f, 13.676f, 13.856f}, {19, 12.902f, 13.067f, 13.233f},
    {20, 12.337f, 12.489f, 12.642f}, {21, 11.799f, 11.940f, 12.080f},
    {22, 11.288f, 11.417f, 11.547f}, {23, 10.802f, 10.921f, 11.040f},
    {24, 10.340f, 10.449f, 10.558f}, {25,  9.900f, 10.000f, 10.100f},
    {26,  9.472f,  9.572f,  9.672f}, {27,  9.066f,  9.165f,  9.265f},
    {28,  8.679f,  8.778f,  8.878f}, {29,  8.311f,  8.409f,  8.508f},
    {30,  7.960f,  8.058f,  8.156f}, {31,  7.627f,  7.724f,  7.821f},
    {32,  7.309f,  7.405f,  7.501f}, {33,  7.006f,  7.101f,  7.196f},
    {34,  6.717f,  6.811f,  6.905f}, {35,  6.441f,  6.534f,  6.628f},
    {36,  6.179f,  6.271f,  6.363f}, {37,  5.928f,  6.019f,  6.110f},
    {38,  5.690f,  5.779f,  5.868f}, {39,  5.461f,  5.549f,  5.638f},
    {40,  5.244f,  5.330f,  5.417f}, {41,  5.036f,  5.121f,  5.207f},
    {42,  4.837f,  4.921f,  5.006f}, {43,  4.648f,  4.730f,  4.813f},
    {44,  4.467f,  4.547f,  4.629f}, {45,  4.294f,  4.373f,  4.453f},
    {46,  4.128f,  4.206f,  4.285f}, {47,  3.970f,  4.046f,  4.124f},
    {48,  3.818f,  3.893f,  3.970f}, {49,  3.674f,  3.747f,  3.822f},
    {50,  3.534f,  3.607f,  3.680f}, {51,  3.402f,  3.473f,  3.545f},
    {52,  3.275f,  3.345f,  3.416f}, {53,  3.154f,  3.222f,  3.291f},
    {54,  3.037f,  3.104f,  3.172f}, {55,  2.926f,  2.991f,  3.058f},
    {56,  2.819f,  2.883f,  2.948f}, {57,  2.717f,  2.780f,  2.843f},
    {58,  2.619f,  2.680f,  2.743f}, {59,  2.524f,  2.585f,  2.646f},
    {60,  2.434f,  2.493f,  2.553f}, {61,  2.348f,  2.405f,  2.464f},
    {62,  2.265f,  2.321f,  2.379f}, {63,  2.185f,  2.240f,  2.297f},
    {64,  2.109f,  2.163f,  2.218f}, {65,  2.035f,  2.088f,  2.142f},
    {66,  1.965f,  2.017f,  2.070f}, {67,  1.897f,  1.948f,  2.000f},
    {68,  1.832f,  1.882f,  1.932f}, {69,  1.769f,  1.818f,  1.868f},
    {70,  1.709f,  1.757f,  1.806f}, {71,  1.652f,  1.698f,  1.746f},
    {72,  1.596f,  1.642f,  1.688f}, {73,  1.543f,  1.588f,  1.633f},
    {74,  1.492f,  1.535f,  1.580f}, {75,  1.442f,  1.485f,  1.529f},
    {76,  1.395f,  1.437f,  1.479f}, {77,  1.349f,  1.390f,  1.432f},
    {78,  1.305f,  1.345f,  1.386f}, {79,  1.263f,  1.302f,  1.342f},
    {80,  1.222f,  1.261f,  1.300f}, {81,  1.183f,  1.221f,  1.259f},
    {82,  1.145f,  1.182f,  1.220f}, {83,  1.109f,  1.145f,  1.182f},
    {84,  1.074f,  1.109f,  1.145f}, {85,  1.040f,  1.075f,  1.110f},
    {86,  1.008f,  1.041f,  1.076f}, {87,  0.976f,  1.009f,  1.043f},
    {88,  0.946f,  0.978f,  1.011f}, {89,  0.917f,  0.948f,  0.981f},
    {90,  0.889f,  0.920f,  0.951f}, {91,  0.862f,  0.892f,  0.923f},
    {92,  0.836f,  0.865f,  0.895f}, {93,  0.810f,  0.839f,  0.869f},
    {94,  0.786f,  0.814f,  0.843f}, {95,  0.762f,  0.790f,  0.818f},
    {96,  0.740f,  0.767f,  0.794f}, {97,  0.718f,  0.744f,  0.771f},
    {98,  0.697f,  0.722f,  0.749f}, {99,  0.676f,  0.701f,  0.728f},
    {100, 0.656f,  0.681f,  0.706f}, {101, 0.637f,  0.662f,  0.687f},
    {102, 0.619f,  0.643f,  0.667f}, {103, 0.601f,  0.624f,  0.648f},
    {104, 0.584f,  0.607f,  0.630f}, {105, 0.567f,  0.589f,  0.612f},
    {106, 0.551f,  0.573f,  0.595f}, {107, 0.536f,  0.557f,  0.579f},
    {108, 0.520f,  0.541f,  0.563f}, {109, 0.506f,  0.526f,  0.547f},
    {110, 0.492f,  0.512f,  0.532f}, {111, 0.478f,  0.498f,  0.518f},
    {112, 0.465f,  0.484f,  0.504f}, {113, 0.452f,  0.471f,  0.490f},
    {114, 0.440f,  0.458f,  0.477f}, {115, 0.428f,  0.446f,  0.464f},
    {116, 0.416f,  0.434f,  0.452f}, {117, 0.405f,  0.422f,  0.440f},
    {118, 0.394f,  0.411f,  0.429f}, {119, 0.384f,  0.400f,  0.417f},
    {120, 0.373f,  0.390f,  0.406f}
};
#define R_TABLE_SIZE  (sizeof(r_table) / sizeof(r_table[0]))

static float resistance_to_temperature(float kohm)
{
    if (kohm < 0.0f) return -99.0f;
    if (kohm >= r_table[0].r_avg)              return 0.0f;
    if (kohm <= r_table[R_TABLE_SIZE-1].r_avg) return 120.0f;

    for (int i = 0; i < (int)R_TABLE_SIZE - 1; i++) {
        if (kohm <= r_table[i].r_avg && kohm >= r_table[i+1].r_avg) {
            float r1 = r_table[i].r_avg, r2 = r_table[i+1].r_avg;
            float t1 = r_table[i].temp,  t2 = r_table[i+1].temp;
            return t1 + (t2 - t1) * (kohm - r1) / (r2 - r1);
        }
    }
    return -99.0f;
}

float ADC_GetTemperature(void)
{
    float v    = (float)adc_values[3] * 3.3f / 4096.0f;
    /* NTC connected as: Vcc → 220kΩ → NTC → GND; PA5 reads the mid-point */
    float ntc  = 220000.0f / ((3.3f / v) - 1.0f) / 1000.0f;  /* kΩ */
    return resistance_to_temperature(ntc);
}
