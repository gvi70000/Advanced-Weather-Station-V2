/***************************************************************************
 * @file [Wind].h/.c
 * This file contains definitions, data structures, and functions
 * for interfacing with the PGA460 ultrasonic transducer driver.
 *
 * Copyright (c) [2024] Grozea Ion gvi70000
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 ***************************************************************************/

// TO BE USED WITH PGA460 Configuration Tool.html to generate registers values

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "tim.h"
#include "PGA460.h"
#include "Wind.h"
#include "debug.h"
#include "cordic.h"         /* CORDIC peripheral for atan2 and sqrt */

// Array to hold the moment in time when the DECPL pin of each PGA460 goes high filled by TIM2 CH1/2/3 DM
extern volatile DecplTimes_t  ToF_Result[3];  
extern volatile uint8_t Decpl_RDY;

#define R_D									287.05f			// Specific gas constant for dry air (J/kg.K)
#define R_V									461.495f		// Specific gas constant for water vapor (J/kg.K)
#define GAMMA								1.4f				// Adiabatic index for air
#define GAMMA_R_D						(GAMMA * R_D)	// GAMMA * R_D
#define L										0.0065f			// Temperature lapse rate (K/m)
#define T0									288.15f			// Standard temperature at sea level (K)
#define P0									101325.0f		// Standard pressure at sea level (Pa)
#define G										9.80665f		// Gravitational acceleration (m/s2)
#define M										0.0289644f	// Molar mass of air (kg/mol)
#define R										8.3144598f	// Universal gas constant (J/(mol.K))
#define KELVIN_OFFSET				273.15f			// Conversion from Celsius to Kelvin
#define RH_DIVISOR					100.0f			// Converts RH percentage to a fraction
#define WATER_VAPOR_EFFECT	0.6077f			// Effect of water vapor on speed of sound
#define SATURATION_CONSTANT	6.1078f			// Constant for saturation vapor pressure calculation
#define SOUND_SPEED					343.0f

// Geometry 
// 3 transducers on a circle: cone base D = 100 mm, at 120deg.
// They form a cone with the reflector in the tip at 150mm from the base
// The cone semi angle is 17.745deg, the Tx-Reflector-Rx distance is 2x157.493=314.986mm
// if c=496um/us the burst will get to receiver after 635us
// if c=306um/us the burst will get to receiver after 1036us
// 6 pulses at 58kHz are 104us
// Time window of interst is from 600us to 1200us
	
#define CONE_L_MM           158.114f					/* v(502+1502)    */
#define CONE_L_UM           158114						/* L in um        */
#define NOMINAL_PATH_UM     (2.0f * (float)CONE_L_UM)	/* 2 * CONE_L_UM */
/*
 * Effective horizontal path unit vectors  *_ij = (pos_j - pos_i) / L
 *
 * Transducer positions in (x=East, y=North) frame (mm):
 *   T0 = (0,           +50)          - North
 *   T1 = (+43.3013,    -25)          - 120deg East
 *   T2 = (-43.3013,    -25)          - 240deg East
 *
 * Distances |Ti-Tj| = Rv3 = 86.6025 mm for all pairs (equilateral triangle).
 */
#define U01_X    0.27386126465714610976890091958966f   /* (T1x - T0x) / L */
#define U01_Y   (-0.47434129805077349254335479464184f) /* (T1y - T0y) / L */

#define U12_X   (-0.54772252931429221953780183917933f) /* (T2x - T1x) / L */
#define U12_Y    0.00000000000000000000000000000000f   /* (T2y - T1y) / L */

#define U20_X    0.27386126465714610976890091958966f   /* (T0x - T2x) / L */
#define U20_Y    0.47434129805077349254335479464184f   /* (T0y - T2y) / L */

// Least-squares weight K = (A^T A)^{-1} scalar = 1/0.45 = 20/9 (A^T A is diagonal due to 120deg symmetry; both diagonal entries = 0.45)
#define LS_K    2.22222222222222f   /* 20/9 */

/*===========================================================================
 * CORDIC CONSTANTS
 *===========================================================================
 * Maximum expected wind speed (m/s = um/us).  vx and vy are divided by this
 * before being written to the CORDIC (which requires inputs in [-1, 1]).
 * 100 m/s covers any meteorological condition with comfortable headroom.
 */

/* Q1.31 scale factor: maps float [-1,1] to int32_t */
#define Q31_SCALE           2147483647.0f			/* 2^31 - 1                 */
#define Q31_INV             4.656612873e-10f	/* 1 / Q31_SCALE						*/

/* CORDIC output for PHASE: angle/p in Q1.31 multiply by p to get radians	*/
#define INV_Q31_TO_PI       1.46291807927e-9f	/* p / (2^31 - 1)           */

/* CORDIC PHASE output [-1,+1] maps to [-180°,+180°]	*/
#define CORDIC_PHASE_TO_DEG   180.0f   

#define CORDIC_Q31_INV_TO_DEG   (CORDIC_PHASE_TO_DEG * Q31_INV)
/*===========================================================================
 * CALIBRATION CONSTANTS
 *===========================================================================*/
#define N_CAL               16     /* number of sample pairs to average    */
#define CAL_PAIR_DELAY_MS   100    /* inter-pair delay during calibration   */

// If PSC = 16 (10 MHz timer)
#define TICKS_PER_US	10.0f // because it has to be divided by 2, 10/2=5
#define TICKS_HALF		(0.5f * TICKS_PER_US)

// TIM2 is 170MHz on STM32G431 and is setup to have 1us period
#define TOF_MIN_TICKS	4000
#define TOF_MAX_TICKS	20000


/*===========================================================================
 * SLIDING-WINDOW MEDIAN FILTER
 *
 * Each of the 6 one-way ToF values has its own circular buffer of depth
 * N_FILT.  On every call to Wind_MeasureWind() one raw hardware measurement
 * cycle is collected (3 transmitter firings -> 6 ToFs via TIM2 DECPL DMA),
 * inserted into the six buffers, and then the median of each buffer is used
 * for the wind calculation.
 *
 * Why filter at the ToF level:
 *   - Outliers (missed threshold crossing, acoustic ringing, UART glitch) are
 *     most easily identified as a single anomalous ToF on one path.
 *   - The b_ij = D/2 * (1/t_ij - 1/t_ji) formula is nonlinear: a 10 us
 *     outlier causes a ~1.9 m/s wind error.  Rejecting it before the
 *     reciprocal step is far more effective than filtering the output.
 *   - Working in (vx, vy) space means no circular wraparound problem;
 *     direction and speed are computed once from the already-filtered medians.
 *
 * Window size N_FILT = 5 (odd, for an exact median):
 *   - Each full cycle takes ~45 ms (3x broadcast + burst + settle).
 *   - At N=5 the window spans ~225 ms -- fast enough to track gusts while
 *     rejecting individual bad echoes.
 *   - Median of 5 tolerates up to 2 simultaneous outliers per window.
 *   - Storage: 6 x 5 x 4 = 120 bytes + 6 head bytes = 126 bytes total.
 *
 * Buffer layout:
 *   g_tof_buf[path][slot]  where path is one of the 6 one-way paths
 *   g_tof_head[path]       circular-buffer write index (0..N_FILT-1)
 *   g_tof_count[path]      number of valid samples so far (0..N_FILT)
 *
 * Until the buffer is full, the median is computed over g_tof_count samples
 * only (no zero-padding), so the very first call returns a single-sample
 * result and convergence improves over the next N_FILT-1 calls.
 *===========================================================================*/

#define N_FILT          5      /* sliding-window depth (must be odd)         */
#define N_FILT_HALF     2      /* N_FILT / 2, index of median in sorted copy */

static float   g_tof_buf  [TOF_PATH_COUNT][N_FILT];  /* circular buffers     */
static uint8_t g_tof_head [TOF_PATH_COUNT];           /* next write index     */
static uint8_t g_tof_count[TOF_PATH_COUNT];           /* valid sample count   */

/* Calibrated one-way path lengths in um for each transmitting sensor.
 * Bytes 0-3 of USER_DATA in the transmitting sensor = IEEE-754 float32.
 * Sensor 0 stores D01, Sensor 1 stores D12, Sensor 2 stores D20.
 * Index i = transmitter index.  Populated by Wind_CalibrateReflector(). */
static float g_pathLen_um[3] = {NOMINAL_PATH_UM, NOMINAL_PATH_UM, NOMINAL_PATH_UM};

const float height = 58.0f; // Height above mean sea level in meters

Wind_EnvData_t externalData = {
	.Temperature = 23.63f,    // degC
	.RH          = 62.22f,    // %
	.Pressure    = 1027.7f,   // hPa
	.SoundSpeed  = 0.0f       // will be computed
};

static void TIM2_IC_Start_All(void) {
    __HAL_TIM_DISABLE(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_ENABLE(&htim2);
}

static void TIM2_IC_Stop_All(void) {
    __HAL_TIM_DISABLE(&htim2);
}

static inline void TIM2_Reset_DMA(void) {
    for (int i = 0; i < 3; i++) {
        DMA_HandleTypeDef *hdma = htim2.hdma[i];
        __HAL_DMA_DISABLE(hdma);
        hdma->Instance->CNDTR = 2;
        __HAL_DMA_ENABLE(hdma);
    }
}

/**
 * @brief Compute speed of sound in moist air from environmental data.
 * @details Uses virtual temperature method accounting for temperature, atmospheric
 *          pressure, and relative humidity.  If Pressure <= 0, it is estimated from
 *          Height using the ISA troposphere barometric formula.  Result is stored in
 *          env->SoundSpeed and also returned.
 * @param env  Pointer to Wind_EnvData_t (Temperature, RH, Pressure, Height).
 * @return Speed of sound in m/s(um/us), or 343.0f if env is NULL.
 */
float Wind_ComputeSoundSpeed(Wind_EnvData_t *env) {
	if (!env) return SOUND_SPEED;
	const float T_C = env->Temperature;
	const float T_K = T_C + KELVIN_OFFSET;
	// --- Pressure (hPa). If not provided, estimate from height with std. atmosphere (0?11 km) ---
	float P_hPa = env->Pressure;
	if (P_hPa <= 0.0) {
		// Barometric formula (troposphere):
		// P = P0 * (1 - L*h/T0)^(g*M/(R*L))  [Pa]
		const float term = 1.0 - (L * height) / T0;
		const float expo = (G * M) / (R * L);
		float P_Pa = P0 * pow(term, expo);
		if (P_Pa < 1.0) P_Pa = 1.0; // guard
		P_hPa = P_Pa * 0.01;        // convert Pa to hPa
		env->Pressure = (float)P_hPa;
	}
	// --- Relative humidity as fraction ---
	const float RH_frac = env->RH / RH_DIVISOR;
	// --- Saturation vapor pressure over water (Tetens, hPa) ---
	// es = 6.1078 * exp(17.2693882 * T_C / (T_C + 237.3))
	const float es = SATURATION_CONSTANT * expf(17.269f * T_C / (T_C + 237.3f));
	// Actual vapor pressure (hPa)
	const float e = RH_frac * es;
	// Mixing ratio (kg/kg); 0.62198  Rd/Rv ratio factor
	const float w = 0.62198f * e / fmaxf(1e-6f, (P_hPa - e));
	// Specific humidity
	const float q = w / (1.0f + w);
	// Virtual temperature Tv = T * (1 + (Rv/Rd - 1)*q); WATER_VAPOR_EFFECT 0.6077
	const float Tv = T_K * (1.0f + 0.6077f * q);
	// Speed of sound in moist air: c = sqrt(gamma * Rd * Tv)
	float c = sqrtf(GAMMA_R_D * Tv); 
	env->SoundSpeed = c;
	return c;
}


/**
 * @brief Clear all six per-path sliding-window ToF buffers.
 * @details Resets head index and sample count to zero for every path.
 *          Call after Wind_Init() and after any recalibration.
 */
static void ResetFilter(void) {
    for (uint8_t p = 0; p < (uint8_t)TOF_PATH_COUNT; p++) {
        g_tof_head [p] = 0;
        g_tof_count[p] = 0;
        for (uint8_t s = 0; s < N_FILT; s++) {
            g_tof_buf[p][s] = 0.0f;
        }
    }
}

/**
 * @brief Insert one new ToF sample into the circular buffer for the given path.
 * @details Overwrites the oldest sample when the buffer is full (circular behaviour).
 * @param path  Path index (see ToFPath_t).
 * @param tof_us  One-way ToF measurement in us.
 */
static void filter_insert(ToFPath_t path, float tof_us) {
    uint8_t h = g_tof_head[path];
    g_tof_buf[path][h] = tof_us;
    g_tof_head[path]   = (uint8_t)((h + 1) % N_FILT);
    if (g_tof_count[path] < N_FILT) {
        g_tof_count[path]++;
    }
}

/**
 * @brief Return the median of the current samples in the buffer for the given path.
 * @details Copies the live samples into a local array, sorts it by insertion sort
 *          (max N_FILT = 5 elements, max 10 FPU ops), and returns the middle element.
 *          Before the buffer is full returns the lower median of available samples.
 * @param path  Path index (see ToFPath_t).
 * @return Median ToF in us, or 0.0f if no samples are present.
 */
static float filter_median(ToFPath_t path){
    uint8_t n = g_tof_count[path];
    if (n == 0) { return 0.0f; }
    /* Local copy -- do not disturb the circular buffer */
    float tmp[N_FILT];
    for (uint8_t i = 0; i < n; i++) {
        tmp[i] = g_tof_buf[path][i];
    }

    /* Insertion sort (ascending) */
    for (uint8_t i = 1; i < n; i++) {
        float   key = tmp[i];
        uint8_t j   = i;
        while (j > 0 && tmp[j - 1] > key) {
            tmp[j] = tmp[j - 1];
            j--;
        }
        tmp[j] = key;
    }
    return tmp[n / 2];   /* lower-median for even n, exact median for odd n */
}

/*===========================================================================
 * PRIVATE HELPERS
 *===========================================================================*/

/**
 * @brief Compute atan2(y, x) in degrees using the STM32G431 CORDIC peripheral.
 * @details Configures CORDIC for the PHASE function (6 iterations, Q1.31, ~24-bit accuracy)
 *          and writes the normalised (x, y) vector directly to the CORDIC registers.
 *          Input is pre-normalised by the largest component to maximise CORDIC precision.
 *          CORDIC PHASE writes X first then Y; output = atan2(Y, X) / pi in Q1.31.
 *          For wind direction: call as cordic_atan2_deg(vx, vy) where x=East, y=North.
 *          Returns the TO direction; add 180 deg to get the FROM (meteorological) direction.
 * @param y  North wind component (m/s) -- first argument maps to CORDIC Y.
 * @param x  East wind component (m/s) -- second argument maps to CORDIC X.
 * @return Angle in degrees in [-180, +180); 0.0f for near-zero input vectors.
 */
static float cordic_atan2_deg(float y, float x) {
    /* --- Normalise to unit circle --- */
    float abs_x = (x < 0.0f) ? -x : x;
    float abs_y = (y < 0.0f) ? -y : y;
    float norm  = (abs_x > abs_y) ? abs_x : abs_y;
    if (norm < 1e-9f) { return 0.0f; } /* near-zero vector: undefined direction, treat as 0 */
    float xn = x / norm;
    float yn = y / norm;
    /* --- Configure CORDIC: PHASE, 6 iterations (24-bit accuracy), Q1.31 --- */
    CORDIC->CSR = CORDIC_FUNCTION_PHASE
                | CORDIC_PRECISION_6CYCLES   /* NB_ITER = 6 ~24 bit	*/
                | CORDIC_SCALE_0
                | CORDIC_NBWRITE_2           /* write X then Y			*/
                | CORDIC_NBREAD_1            /* read one result			*/
                | CORDIC_INSIZE_32BITS
                | CORDIC_OUTSIZE_32BITS;

    /* Write X first, then Y (CORDIC HW order for PHASE) */
    CORDIC->WDATA = (int32_t)(xn * Q31_SCALE);
    CORDIC->WDATA = (int32_t)(yn * Q31_SCALE);
    /* Poll RRDY flag */
    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}
    int32_t raw = (int32_t)CORDIC->RDATA;  /* angle / p in Q1.31 */

    /* Convert: raw * (p / (2^31-1)) * (180/p) = raw * (180 / (2^31-1)) */
    return (float)raw * CORDIC_Q31_INV_TO_DEG;
}

/**
 * @brief Compute sqrt(x^2 + y^2) using the STM32G431 CORDIC MODULUS function.
 * @details Configures CORDIC for the MODULUS function (6 iterations, Q1.31).
 *          Input is pre-normalised by the larger component so both fit [-1, 1].
 *          Result is de-normalised by multiplying back the normalisation factor.
 * @param x  East wind component (m/s).
 * @param y  North wind component (m/s).
 * @return Euclidean magnitude in the same units as the inputs; 0.0f for near-zero vectors.
 */
static float cordic_magnitude(float x, float y) {
    float abs_x = (x < 0.0f) ? -x : x;
    float abs_y = (y < 0.0f) ? -y : y;
    float norm  = (abs_x > abs_y) ? abs_x : abs_y;
    if (norm < 1e-9f) { return 0.0f; }
    float xn = x / norm;
    float yn = y / norm;
    /* --- Configure CORDIC: MODULUS, 6 iterations, Q1.31 --- */
    CORDIC->CSR = CORDIC_FUNCTION_MODULUS
                | CORDIC_PRECISION_6CYCLES
                | CORDIC_SCALE_0
                | CORDIC_NBWRITE_2
                | CORDIC_NBREAD_1
                | CORDIC_INSIZE_32BITS
                | CORDIC_OUTSIZE_32BITS;

    CORDIC->WDATA = (int32_t)(xn * Q31_SCALE);
    CORDIC->WDATA = (int32_t)(yn * Q31_SCALE);

    while (!(CORDIC->CSR & CORDIC_CSR_RRDY)) {}

    int32_t raw = (int32_t)CORDIC->RDATA;   /* magnitude in Q1.31 */

    /* Re-apply normalisation: result = (raw / Q31_SCALE) * norm */
    return (float)raw * Q31_INV * norm;
}

/**
 * @brief Block until the DECPL DMA bitmask callback sets Decpl_RDY, or until timeout.
 * @details Polls Decpl_RDY in a busy-wait loop.  Decpl_RDY is set by
 *          HAL_TIM_IC_CaptureCallback() in tim.c only when all three TIM2 channels
 *          (CH1, CH2, CH3) have each completed their two-edge DMA capture.
 * @param timeout_ms  Maximum wait time in milliseconds.
 * @return HAL_OK when Decpl_RDY goes high; HAL_TIMEOUT if the deadline expires.
 */
static HAL_StatusTypeDef wait_decpl_ready(uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (!Decpl_RDY) {
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return HAL_TIMEOUT;
        }
    }
    return HAL_OK;
}

/**
 * @brief Fire one transmitter and capture echo timestamps on both receivers via TIM2 DMA.
 * @details Sequence:
 *            1. Reset Decpl_RDY and zero ToF_Result[].
 *            2. Arm TIM2 input-capture DMA on CH1/2/3 for 2 edges each.
 *            3. Send BROADCAST_LISTEN_ONLY_P2 -- all DECPL pins assert immediately(P2 = 0 pulses); DMA captures this as E0 (reference, discarded).
 *            4. Send BURST_AND_LISTEN_PRESET1 on the transmitter -- 6 pulses fired; Tx DECPL asserts at burst end -> E1[tx] = T_burst reference.
 *            5. Wait for all three DMA TCs via wait_decpl_ready() (10 ms timeout).
 *            6. Compute ToF = TICK_TO_US(E1[rx] - E1[tx]) for each receiver.
 *            7. Validate: delta ticks must be in [TOF_MIN_TICKS, TOF_MAX_TICKS].
 *          Unsigned tick subtraction is wrap-safe for deltas up to 12.6 s.
 * @param tx  Transmitter index (0-2).
 * @param tof_rx1_ticks  Output: one-way ToF to first receiver in us.
 * @param tof_rx2_ticks  Output: one-way ToF to second receiver in us.
 * @return HAL_OK, HAL_ERROR (invalid ToF), or HAL_TIMEOUT (no DECPL response).
 */
static HAL_StatusTypeDef run_one_measurement(uint8_t tx, float  *tof_rx1_ticks, float  *tof_rx2_ticks) {
    uint8_t rx1 = (tx == 0) ? 1 : 0;
    uint8_t rx2 = (uint8_t)(3 - tx - rx1);
    /*
     * Reset state BEFORE arming DMA.
     * DECPL_ResetReadyFlag() disables interrupts briefly to prevent a stale
     * TC from an aborted previous measurement from instantly asserting Decpl_RDY.
     * The chan_done bitmask in the callback is cleared by Decpl_RDY going to 0
     * and the callback accumulator resetting on the next full mask match, but
     * the explicit flag reset here is sufficient - the bitmask itself is reset
     * inside the callback when the mask completes.
     */
			// Reset state
			Decpl_RDY = 0;
			memset((void *)ToF_Result, 0, sizeof(ToF_Result));
			// Reset DMA counters
			TIM2_Reset_DMA();
			// Start all channels synchronously
			TIM2_IC_Start_All();
		
    /*
     * Command sequence:
     *
     * 1. Broadcast LISTEN_ONLY P2 to all sensors (noWait=1).
     *    All three PGA460 DECPL pins assert within ~1 UART byte time of each
     *    other.  Each DMA captures this as E0 (reference, discarded later).
     *    The transmitter is now also armed in listen mode for P2 - this is
     *    intentional: its DECPL will fire again when the P1 burst ends (E1).
     *
     * 2. BURST_AND_LISTEN P1 on the transmitter (noWait=1).
     *    Tx fires 6 pulses.  At the end of the Tx decouple period, its DECPL
     *    asserts again ? captured as ToF_Result[tx].E1 (= T_burst reference).
     *    The two receivers listen; when each detects its echo threshold
     *    crossing, its DECPL asserts ? captured as ToF_Result[rx].E1.
     */
    PGA460_UltrasonicCmd(rx1, PGA460_CMD_BROADCAST_LISTEN_ONLY_P2, 1); // sensorID irrelevant for broadcast
    PGA460_UltrasonicCmd(tx,  PGA460_CMD_BURST_AND_LISTEN_PRESET1, 1);
    /* --- Wait for all three DECPL TC interrupts via bitmask callback --- */
    if (wait_decpl_ready(10) != HAL_OK) {
        // Stop timer, all captures stop instantly
				TIM2_IC_Stop_All();
        DEBUG("Tx%u: DECPL timeout - check DECPL wiring and P1/P2 config\n", tx);
        return HAL_TIMEOUT;
    }
    // Stop timer, all captures stop instantly
    TIM2_IC_Stop_All();
    /*
     * Extract timestamps:
     *   E1[tx]  = end of Tx burst decouple period  ? acoustic start
     *   E1[rx]  = echo threshold crossing           ? acoustic arrival
     *
     * Unsigned subtraction is wrap-safe for any delta < 2^31 ticks (~12.6 s).
     * The measurement window (< 2 ms) is orders of magnitude shorter.
     */
    uint32_t t_burst  = ToF_Result[tx].E1;
    uint32_t t_echo1  = ToF_Result[rx1].E1;
    uint32_t t_echo2  = ToF_Result[rx2].E1;
    uint32_t delta1 = t_echo1 - t_burst;   /* unsigned wrap-safe subtraction */
    uint32_t delta2 = t_echo2 - t_burst;
    if (delta1 < TOF_MIN_TICKS || delta1 > TOF_MAX_TICKS || delta2 < TOF_MIN_TICKS || delta2 > TOF_MAX_TICKS) {
        DEBUG("Tx%u: ToF ticks out of range (d1=%d, d2=%d) - expected %d..%d\n", tx, delta1, delta2, TOF_MIN_TICKS, TOF_MAX_TICKS);
        return HAL_ERROR;
    }
		*tof_rx1_ticks = (float)delta1;   /* ticks, 100 ns resolution */
		*tof_rx2_ticks = (float)delta2;   /* ticks, 100 ns resolution */
    return HAL_OK;
}

/**
 * @brief Write a float32 calibrated path length to USER_DATA bytes 0-3 of a sensor.
 * @details Splits the IEEE-754 float into 4 bytes using memcpy and writes each byte
 *          to REG_USER_DATA1 + offset (0-3) via Wind_RegisterWrite().
 * @param sensorID  Sensor UART address (0-7).
 * @param path_um  Calibrated one-way acoustic path length in um.
 * @return HAL status.
 */
static HAL_StatusTypeDef store_path_to_eeprom(uint8_t sensorID, float path_um) {
    EEImage_t *ee = PGA460_GetEEData(sensorID);
    if (ee == NULL) return HAL_ERROR;

    memcpy(&ee->UserData[0], &path_um, sizeof(float));

    if (PGA460_EEPROMBulkWrite(sensorID) != HAL_OK) {
        DEBUG("Sensor %u: EEPROM bulk write failed during calibration store\n", sensorID);
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief Read a float32 calibrated path length from USER_DATA bytes 0-3 of a sensor.
 * @details Reads 4 bytes from REG_USER_DATA1 + offset (0-3) and reassembles them into
 *          a float32 using memcpy.  Validates the result against a +/-10% window around
 *          NOMINAL_PATH_UM; returns NOMINAL_PATH_UM if the stored value is out of range.
 * @param sensorID  Sensor UART address (0-7).
 * @return Calibrated path length in um, or NOMINAL_PATH_UM if not stored or invalid.
 */
static float load_path_from_eeprom(uint8_t sensorID) {
    EEImage_t *ee = PGA460_GetEEData(sensorID);
    if (ee == NULL) return NOMINAL_PATH_UM;

    float result;
    memcpy(&result, &ee->UserData[0], sizeof(float));

    if (result < NOMINAL_PATH_UM * 0.90f || result > NOMINAL_PATH_UM * 1.10f) {
        DEBUG("Sensor %u: EEPROM path %.1f um out of range, using nominal\n", sensorID, result);
        return NOMINAL_PATH_UM;
    }
    return result;
}

/**
 * @brief Wind_LoadCalibration function implementation.
 * @details Calls load_path_from_eeprom() for each of the three sensors and stores the
 *          result in g_pathLen_um[].  Falls back to NOMINAL_PATH_UM on failure.
 *          Call once at boot after Wind_Init() so prior calibrations survive power-off.
 */
void Wind_LoadCalibration(void) {
    for (uint8_t i = 0; i < 3; i++) {
        g_pathLen_um[i] = load_path_from_eeprom(i);
        DEBUG("Sensor %u: loaded D = %.2f um\n", i, g_pathLen_um[i]);
    }
}

void Wind_Init(void) {
	ResetFilter();
	Wind_LoadCalibration();
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)&ToF_Result[0].E0, 2);
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t *)&ToF_Result[1].E0, 2);
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_3, (uint32_t *)&ToF_Result[2].E0, 2);
	__HAL_TIM_DISABLE(&htim2);
}

/**
 * @brief Wind_CalibrateReflector function implementation.
 * @details Performs bidirectional still-air calibration for all three transducer pairs.
 *
 *          Repeats N_CAL full measurement cycles (3 transmitter firings each).  For each
 *          bidirectional pair the averaged path length is:
 *            D_ij = c * (t_ij + t_ji) / 2
 *          Wind cancels in the sum, leaving only path geometry.
 *
 *          Pair-to-sensor storage mapping:
 *            D_01 -> sensor 0 USER_DATA (tof[0][0], tof[1][0])
 *            D_12 -> sensor 1 USER_DATA (tof[1][1], tof[2][1])
 *            D_20 -> sensor 2 USER_DATA (tof[2][0], tof[0][1])
 *
 *          Returns HAL_ERROR if fewer than N_CAL/2 cycles produce valid measurements.
 * @param soundSpeed_ms  Speed of sound in m/s from Wind_UpdateEnvironment() (1 m/s = 1 um/us).
 * @param burnEEPROM  1 = commit calibration to PGA460 EEPROM flash; 0 = RAM only.
 * @return HAL status.
 */
HAL_StatusTypeDef Wind_CalibrateReflector(float soundSpeed_ms, uint8_t burnEEPROM) {
    const float c = soundSpeed_ms;   /* 1 m/s = 1 um/us exactly */
    DEBUG("Calibration start: c = %.4f m/s, N_CAL = %u\n", c, N_CAL);
    /*
     * Accumulators for the six one-way ToF sums (us).
     * Index mapping mirrors the tof[tx][rx_slot] layout in Wind_Measure.
     *   sum[0][0] = S t_01,  sum[0][1] = S t_02
     *   sum[1][0] = S t_10,  sum[1][1] = S t_12
     *   sum[2][0] = S t_20,  sum[2][1] = S t_21
     */
    float   sum[3][2] = {{0.0f}};
    uint8_t valid = 0;
    for (uint8_t n = 0; n < N_CAL; n++) {
        float tof[3][2] = {{0.0f}};
        uint8_t all_ok = 1;
        /* Fire each transmitter and capture both receiver ToFs */
        for (uint8_t tx = 0; tx < 3; tx++) {
            if (run_one_measurement(tx, &tof[tx][0], &tof[tx][1]) != HAL_OK) {
                all_ok = 0u;
                break;
            }
            /* Validate: expected range for 316 mm path at 306-496 m/s */
            if (tof[tx][0] < TOF_MIN_TICKS || tof[tx][0] > TOF_MAX_TICKS || tof[tx][1] < TOF_MIN_TICKS || tof[tx][1] > TOF_MAX_TICKS) {
                DEBUG("Cal n=%u Tx%u: ToF out of range (%.1f, %.1f us)\n",
                      n, tx, tof[tx][0], tof[tx][1]);
                all_ok = 0;
                break;
            }
        }
        if (all_ok) {
            for (uint8_t tx = 0; tx < 3; tx++) {
                sum[tx][0] += tof[tx][0];
                sum[tx][1] += tof[tx][1];
            }
            valid++;
        }
        HAL_Delay(CAL_PAIR_DELAY_MS);
    }
    if (valid < (N_CAL / 2)) {
        DEBUG("Calibration failed: only %u/%u valid cycles\n", valid, N_CAL);
        return HAL_ERROR;
    }
    const float inv_valid = 1.0f / (float)valid;
    /*
     * Compute and store the three calibrated path lengths.
     *
     * Bidirectional pair ? transmitter index that owns its USER_DATA:
     *   D_01  stored in sensor 0  (t_01 = sum[0][0], t_10 = sum[1][0])
     *   D_12  stored in sensor 1  (t_12 = sum[1][1], t_21 = sum[2][1])
     *   D_20  stored in sensor 2  (t_20 = sum[2][0], t_02 = sum[0][1])
     */
    /* Average ToF per pair (us) */
		float t_avg_01 = (sum[0][0] + sum[1][0]) * 0.5f * inv_valid;
		float t_avg_12 = (sum[1][1] + sum[2][1]) * 0.5f * inv_valid;
		float t_avg_20 = (sum[2][0] + sum[0][1]) * 0.5f * inv_valid;
    float d[3];
    d[0] = c * t_avg_01 / TICKS_PER_US;   /* D_01 um, owned by sensor 0 */
    d[1] = c * t_avg_12 / TICKS_PER_US;   /* D_12 um, owned by sensor 1 */
    d[2] = c * t_avg_20 / TICKS_PER_US;   /* D_20 um, owned by sensor 2 */
    for (uint8_t i = 0; i < 3; i++) {
        g_pathLen_um[i] = d[i];
        DEBUG("D_%u%u = %.2f um  (%.4f mm, nominal %.2f um, delta %+.1f um, %u samples)\n", i, (i + 1) % 3, d[i], d[i] * 0.001f, NOMINAL_PATH_UM, d[i] - NOMINAL_PATH_UM, valid);
        if (store_path_to_eeprom(i, d[i]) != HAL_OK) {
            return HAL_ERROR;
        }
        if (burnEEPROM) {
            if (PGA460_BurnEEPROM(i) != HAL_OK) {
                DEBUG("Sensor %u: EEPROM burn failed\n", i);
                return HAL_ERROR;
            }
            HAL_Delay(50);   /* PGA460 internal EEPROM write cycle */
        }
    }
    DEBUG("Calibration complete (%u/%u cycles used)\n", valid, N_CAL);
    return HAL_OK;
}

/**
 * @brief Wind_Measure function implementation.
 * @details Executes a full 3-transmitter measurement cycle and returns the filtered wind vector.
 *
 *          Step 1 - Hardware cycle: fires each transmitter, captures 6 raw one-way ToF
 *          values via TIM2 DECPL input-capture DMA (run_one_measurement).
 *
 *          Step 2 - Filter insert: inserts valid raw ToFs into six per-path sliding-window
 *          circular buffers (N = 5).  On hardware fault keeps prior buffer contents.
 *
 *          Step 3 - Median: insertion sort on <= 5 floats per path.  Returns HAL_ERROR
 *          if any median is outside the physical ToF range (buffer not yet warm).
 *
 *          Step 4 - Wind reconstruction:
 *            b_ij = (D_i/2) * (1/t_ij - 1/t_ji)  [um/us = m/s]
 *            vx = K * (u01_x*b01 + u12_x*b12 + u20_x*b20)  [East m/s]
 *            vy = K * (u01_y*b01 + u12_y*b12 + u20_y*b20)  [North m/s]
 *            K = 20/9 (exact closed-form least-squares for 120-deg symmetric layout)
 *
 *          Step 5 - CORDIC: speed via MODULUS, direction via PHASE + 180 deg (FROM convention).
 * @param out  Output: Speed (m/s) and Direction (deg FROM: 0/360=N, 90=E, 180=S, 270=W).
 * @return HAL_OK if all 3 transmitters succeeded; HAL_ERROR otherwise (filtered output still written).
 */
HAL_StatusTypeDef Wind_Measure(Wind_t *out) {
    if (out == NULL) { return HAL_ERROR; }
    /*=========================================================================
     * Step 1 -- Hardware measurement cycle
     *
     * Fire each transmitter in sequence.  Each firing captures two one-way ToF
     * values via TIM2 DECPL input-capture DMA (see run_one_measurement).
     * Three firings -> six raw ToF values, one per directed path.
     *
     * tof_raw[tx][0] = t_{Tx -> Rx1},  tof_raw[tx][1] = t_{Tx -> Rx2}
     * Receiver assignments:
     *   tx=0: rx1=1, rx2=2    tx=1: rx1=0, rx2=2    tx=2: rx1=0, rx2=1
     *=========================================================================*/
    float tof_raw[3][2] = {{0.0f}};
    HAL_StatusTypeDef hw_status = HAL_OK;
    for (uint8_t tx = 0; tx < 3; tx++) {
        if (run_one_measurement(tx, &tof_raw[tx][0], &tof_raw[tx][1]) != HAL_OK) {
            DEBUG("MeasureWind: Tx%u hardware fault\n", tx);
            hw_status = HAL_ERROR;
            break;
        }
    }
    /*=========================================================================
     * Step 2 -- Insert into per-path sliding-window buffers
     *
     * Even if one transmitter fails, we insert the six values we have (keeping
     * failed paths at their previous buffer state so prior good samples still
     * contribute to the median).  A failed cycle is flagged and the caller
     * receives HAL_ERROR but stale-median output is still written to *out,
     * which is preferable to a hard stop in a meteo application.
     *
     * Path-to-raw-slot mapping:
     *   TOF_01: tof_raw[0][0]   TOF_10: tof_raw[1][0]
     *   TOF_12: tof_raw[1][1]   TOF_21: tof_raw[2][1]
     *   TOF_20: tof_raw[2][0]   TOF_02: tof_raw[0][1]
     *=========================================================================*/
    if (hw_status == HAL_OK) {
        filter_insert(TOF_01, tof_raw[0][0]);
        filter_insert(TOF_10, tof_raw[1][0]);
        filter_insert(TOF_12, tof_raw[1][1]);
        filter_insert(TOF_21, tof_raw[2][1]);
        filter_insert(TOF_20, tof_raw[2][0]);
        filter_insert(TOF_02, tof_raw[0][1]);

        DEBUG("raw t01=%.2f t10=%.2f t12=%.2f t21=%.2f t20=%.2f t02=%.2f us\n",
              tof_raw[0][0], tof_raw[1][0], tof_raw[1][1],
              tof_raw[2][1], tof_raw[2][0], tof_raw[0][1]);
    }

    /*=========================================================================
     * Step 3 -- Compute median ToF for each path
     *
     * filter_median() performs an insertion sort on a local copy of the
     * circular buffer (<=5 elements, <=10 FPU compare-and-swap ops) and
     * returns the middle element.  With a full window of N_FILT=5 samples,
     * up to 2 outliers per path are rejected per measurement call.
     *
     * For the first N_FILT-1 calls the buffer is not yet full; filter_median()
     * returns the median of however many valid samples are present (1 to 4),
     * which degrades gracefully to a single-sample pass-through on the first
     * call.
     *=========================================================================*/
    float t01 = filter_median(TOF_01);
    float t10 = filter_median(TOF_10);
    float t12 = filter_median(TOF_12);
    float t21 = filter_median(TOF_21);
    float t20 = filter_median(TOF_20);
    float t02 = filter_median(TOF_02);

    DEBUG("med t01=%.2f t10=%.2f t12=%.2f t21=%.2f t20=%.2f t02=%.2f us\n", t01, t10, t12, t21, t20, t02);

    /* Guard: if any median is still zero the buffer was never populated */

    if (t01 < TOF_MIN_TICKS || t01 > TOF_MAX_TICKS ||
        t10 < TOF_MIN_TICKS || t10 > TOF_MAX_TICKS ||
        t12 < TOF_MIN_TICKS || t12 > TOF_MAX_TICKS ||
        t21 < TOF_MIN_TICKS || t21 > TOF_MAX_TICKS ||
        t20 < TOF_MIN_TICKS || t20 > TOF_MAX_TICKS ||
        t02 < TOF_MIN_TICKS || t02 > TOF_MAX_TICKS)
    {
        DEBUG("MeasureWind: median ToF out of physical range -- buffer not yet warm\n");
        return HAL_ERROR;
    }
    /*=========================================================================
     * Step 4 -- Wind vector reconstruction
     *
     * Wind projection onto each physical path [um/us = m/s]:
     *   b_ij = (D_i / 2) * (1/t_ij - 1/t_ji)
     *
     * b_ij is the component of the wind velocity along the unit vector
     * u_ij = (pos_j - pos_i) / L.  The units work out to m/s because
     * D_i is in um and t_ij is in us, and 1 um/us = 1 m/s exactly.
     *=========================================================================*/
		float b01 = (g_pathLen_um[0] * TICKS_HALF) * (1.0f/t01 - 1.0f/t10);
		float b12 = (g_pathLen_um[1] * TICKS_HALF) * (1.0f/t12 - 1.0f/t21);
		float b20 = (g_pathLen_um[2] * TICKS_HALF) * (1.0f/t20 - 1.0f/t02);
    DEBUG("b01=%.4f b12=%.4f b20=%.4f m/s\n", b01, b12, b20);
    /*
     * Closed-form least-squares reconstruction (K = 20/9, exact for 120 deg layout):
     *   vx = K * (u01_x*b01 + u12_x*b12 + u20_x*b20)    [East component, m/s]
     *   vy = K * (u01_y*b01 + u12_y*b12 + u20_y*b20)    [North component, m/s]
     */
    float vx = LS_K * (U01_X * b01 + U12_X * b12 + U20_X * b20);
    float vy = LS_K * (U01_Y * b01 + U12_Y * b12 + U20_Y * b20);
    DEBUG("vx=%.4f vy=%.4f m/s\n", vx, vy);
    /*=========================================================================
     * Step 5 -- CORDIC: speed and direction
     *
     * MODULUS: speed = sqrt(vx^2 + vy^2)   [m/s]
     * PHASE:   direction via atan2(vx, vy) + 180 deg (FROM convention)
     *
     * Inputs are scaled to [-1,1] by WIND_CORDIC_INV before the CORDIC write.
     * No circular-mean issue: direction is computed from the (vx,vy) vector
     * which was reconstructed from median ToFs -- no 0/360 wraparound problem.
     *=========================================================================*/
    float speed = cordic_magnitude(vx, vy);
    float angle_to = cordic_atan2_deg(vy, vx);   // North=y, East=x
    float dir_from = angle_to + 180.0f;
    if (dir_from >= 360.0f) { dir_from -= 360.0f; }
    if (dir_from <    0.0f) { dir_from += 360.0f; }
    out->Speed     = speed;
    out->Direction = dir_from;
    return hw_status;   /* HAL_OK if all 3 transmitters succeeded this cycle */
}