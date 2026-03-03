#include "AdcScheduler.h"

#include <avr/io.h>
#include <avr/interrupt.h>

// Ring buffer per ADC channel (0..15)
static constexpr uint8_t ADC_CH_MAX = 16;
static constexpr uint8_t RING_N = 4; // small queue; current sensors tolerate small loss

static volatile uint16_t g_ring[ADC_CH_MAX][RING_N];
static volatile uint8_t  g_head[ADC_CH_MAX];
static volatile uint8_t  g_tail[ADC_CH_MAX];
static volatile uint8_t  g_count[ADC_CH_MAX];
static volatile uint16_t g_dropped[ADC_CH_MAX];
static volatile uint8_t  g_queueEn[ADC_CH_MAX];

static const uint8_t* g_sched = nullptr;
static uint8_t g_schedLen = 0;
static volatile uint8_t g_schedIdx = 0;

static volatile uint32_t g_ticks = 0;
static volatile uint32_t g_samples = 0;
static volatile uint8_t  g_currentCh = 0;

static AdcIsrSink g_sink = nullptr;

void adcSchedSetIsrSink(AdcIsrSink sink)
{
    cli();
    g_sink = sink;
    sei();
}

static inline uint8_t analogPinToChannel(uint8_t analogPin)
{
    // Arduino Mega2560: A0..A15 are contiguous.
    if (analogPin >= A0 && analogPin <= A15) return (uint8_t)(analogPin - A0);
    // Fallback: if someone passes raw channel.
    return (uint8_t)(analogPin & 0x0F);
}

static inline void adcSelectChannel(uint8_t ch)
{
    // MUX5 selects channels 8..15
    if (ch & 0x08) ADCSRB |= _BV(MUX5);
    else           ADCSRB &= (uint8_t)~_BV(MUX5);

    // Preserve REFSx / ADLAR, set MUX[2:0]
    ADMUX = (uint8_t)((ADMUX & 0xF0) | (ch & 0x07));
}

void adcSchedBegin(const uint8_t* schedulePins, uint8_t scheduleLen)
{
    g_sched = schedulePins;
    g_schedLen = scheduleLen;
    g_schedIdx = 0;
    g_ticks = 0;
    g_samples = 0;

    for (uint8_t i = 0; i < ADC_CH_MAX; i++) {
        g_head[i] = g_tail[i] = g_count[i] = 0;
        g_dropped[i] = 0;
        g_queueEn[i] = 1;
    }

    cli();

    // --- ADC setup ---
    // AVcc reference, right adjusted.
    ADMUX = _BV(REFS0);
    // Enable ADC + interrupt, prescaler 128 (16MHz/128=125kHz)
    ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    // --- Timer1 setup ---
    // We aim for ~2800 Hz total conversions:
    // 16MHz / 64 = 250kHz. 250kHz / 2809 ≈ 89 -> OCR1A ~ 88.
    // Actual rate: 250kHz/(OCR1A+1)
    TCCR1A = 0;
    TCCR1B = 0;
    TCCR1B |= _BV(WGM12);          // CTC
    OCR1A = 88;                    // ~2809 Hz
    TCCR1B |= _BV(CS11) | _BV(CS10); // prescaler 64
    TIMSK1 |= _BV(OCIE1A);         // enable compare match A

    // Prime first channel
    if (g_sched && g_schedLen) {
        const uint8_t pin = g_sched[0];
        g_currentCh = analogPinToChannel(pin);
        adcSelectChannel(g_currentCh);
    }

    // Start first conversion
    ADCSRA |= _BV(ADSC);

    sei();
}

void adcSchedSetQueueEnabled(uint8_t analogPin, bool enabled)
{
    const uint8_t ch = analogPinToChannel(analogPin);
    cli();
    g_queueEn[ch] = enabled ? 1u : 0u;
    // Reset queue when disabling to keep stats readable.
    if (!enabled) {
        g_head[ch] = g_tail[ch] = g_count[ch] = 0;
    }
    sei();
}

ISR(TIMER1_COMPA_vect)
{
    g_ticks++;
    if (!g_sched || g_schedLen == 0) return;

    // Advance schedule
    uint8_t idx = g_schedIdx + 1;
    if (idx >= g_schedLen) idx = 0;
    g_schedIdx = idx;

    const uint8_t pin = g_sched[idx];
    g_currentCh = analogPinToChannel(pin);
    adcSelectChannel(g_currentCh);

    // Start conversion
    ADCSRA |= _BV(ADSC);
}

ISR(ADC_vect)
{
    const uint16_t v = ADC;
    g_samples++;

    const uint8_t ch = g_currentCh;
    if (g_queueEn[ch]) {
        uint8_t c = g_count[ch];
        if (c < RING_N) {
            const uint8_t h = g_head[ch];
            g_ring[ch][h] = v;
            g_head[ch] = (uint8_t)((h + 1) & (RING_N - 1));
            g_count[ch] = (uint8_t)(c + 1);
        } else {
            // Drop newest when full (bounded memory). Count drops.
            g_dropped[ch]++;
        }
    }

    if (g_sink) {
        g_sink(ch, v);
    }
}

bool adcSchedPop(uint8_t analogPin, uint16_t& out)
{
    const uint8_t ch = analogPinToChannel(analogPin);

    bool ok = false;
    cli();
    const uint8_t c = g_count[ch];
    if (c > 0) {
        const uint8_t t = g_tail[ch];
        out = g_ring[ch][t];
        g_tail[ch] = (uint8_t)((t + 1) & (RING_N - 1));
        g_count[ch] = (uint8_t)(c - 1);
        ok = true;
    }
    sei();
    return ok;
}

uint32_t adcSchedTicks()   { uint32_t x; cli(); x = g_ticks;   sei(); return x; }
uint32_t adcSchedSamples() { uint32_t x; cli(); x = g_samples; sei(); return x; }

uint16_t adcSchedDropped(uint8_t analogPin)
{
    const uint8_t ch = analogPinToChannel(analogPin);
    uint16_t x;
    cli();
    x = g_dropped[ch];
    sei();
    return x;
}