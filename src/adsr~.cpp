// adsr~.cpp — Pure Data external for a nonlinear ADSR envelope with clamping and validation

#include "m_pd.h"
#include <cmath>
#include <algorithm>

static t_class *adsr_tilde_class;

// === ADSR envelope phase enumeration ===
enum class t_adsr_phase
{
    Idle,
    Startup,
    Attack,
    Decay,
    Sustain,
    Release
};

// typedef struct enter_phase enter_phase;
typedef struct t_adsr_tilde t_adsr_tilde;
typedef void (*adsr_phase_ptr)(t_adsr_tilde *);

// === Main object structure ===
struct t_adsr_tilde
{
    t_object x_obj;
    t_outlet *x_out;

    t_adsr_phase phase;
    adsr_phase_ptr phaseFunc;
    double samplerate, sampleratems;
    double attackTime, decayTime, sustainLevel, releaseTime;
    double attackShape, releaseShape;
    double currentEnv, phaseStartEnv;
    bool startAtCurrentEnv;
    int attackPhaseSamples, decayPhaseSamples, releasePhaseSamples, startupPhaseSamples;
    int currentSample;
    double gain;
};

// Startup time defines the time to move the env to zero in the first step
const int startupTime = 3;

// Preliminary definition enter_phase
void enter_phase(t_adsr_tilde *x, t_adsr_phase newPhase);

// Helper: shaped progress with exponential curvature (linear interpolation)
double power_lerp(double start, double end, double p, double shape)
{
    if (shape == 1.0)
        return start + (end - start) * p;

    double curved = (end < start)
        ? 1.0 - std::pow(1.0 - p, shape)
        : std::pow(p, shape);

    return start + (end - start) * curved;
}

void startupPhase(t_adsr_tilde *x)
{
    double p = static_cast<double>(x->currentSample) / (x->startupPhaseSamples);
    x->currentEnv = power_lerp(x->phaseStartEnv, 0.0, p, 1.0);

    if (++x->currentSample >= x->startupPhaseSamples)
    {
        x->phaseStartEnv = 0.0;
        enter_phase(x, t_adsr_phase::Attack);
    }
}

void attackPhase(t_adsr_tilde *x)
{
    double p = static_cast<double>(x->currentSample) / (x->attackPhaseSamples);
    x->currentEnv = power_lerp(x->phaseStartEnv, 1.0, p, x->attackShape);

    if (++x->currentSample >= x->attackPhaseSamples)
        enter_phase(x, t_adsr_phase::Decay);
}

void decayPhase(t_adsr_tilde *x)
{
    double p = static_cast<double>(x->currentSample) / x->decayPhaseSamples;
    x->currentEnv = (1.0 - p) * (1.0 - x->sustainLevel) + x->sustainLevel;

    if (++x->currentSample >= x->decayPhaseSamples)
        enter_phase(x, t_adsr_phase::Sustain);
}

void sustainPhase(t_adsr_tilde *x)
{
    x->currentEnv = x->sustainLevel;
}

void releasePhase(t_adsr_tilde *x)
{
    double p = static_cast<double>(x->currentSample) / (x->releasePhaseSamples - 1);
    x->currentEnv = power_lerp(x->phaseStartEnv, 0.0, p, x->releaseShape);

    if (++x->currentSample >= x->releasePhaseSamples)
        enter_phase(x, t_adsr_phase::Idle);
}

void idlePhase(t_adsr_tilde *x)
{
    x->currentEnv = 0.0;
}

// === Enter a new phase and prepare sample counters ===
void enter_phase(t_adsr_tilde *x, t_adsr_phase newPhase)
{
    x->phase = newPhase;

    switch (newPhase)
    {
    case t_adsr_phase::Startup:
        x->phaseFunc = startupPhase;
        break;

    case t_adsr_phase::Attack:
        x->phaseFunc = attackPhase;
        break;

    case t_adsr_phase::Decay:
        x->phaseFunc = decayPhase;
        break;

    case t_adsr_phase::Sustain:
        x->phaseFunc = sustainPhase;
        break;

    case t_adsr_phase::Release:
        x->phaseFunc = releasePhase;
        break;

    case t_adsr_phase::Idle:
        x->phaseFunc = idlePhase;
        break;

    default:
        x->phaseFunc = idlePhase;
        break;
    }

    x->currentSample = 0;
}

// === Signal processing function ===
t_int *adsr_perform(t_int *w)
{
    t_adsr_tilde *x = (t_adsr_tilde *)(w[1]);
    t_sample *out = (t_sample *)(w[2]);
    int n = (int)(w[3]);

    while (n--)
    {
        x->phaseFunc(x);
        *out++ = static_cast<t_sample>(x->currentEnv * x->gain);
    }
    return (w + 4);
}

// === Trigger methods ===
void adsr_trigger_start(t_adsr_tilde *x)
{
    if (!x->startAtCurrentEnv)
    {
        x->phaseStartEnv = x->currentEnv;
        enter_phase(x, t_adsr_phase::Startup);
    }
    else
    {
        x->phaseStartEnv = x->currentEnv;
        enter_phase(x, t_adsr_phase::Attack);
    }
}

void adsr_trigger_stop(t_adsr_tilde *x)
{
    if (x->phase != t_adsr_phase::Idle && x->phase != t_adsr_phase::Release)
    {
        x->phaseStartEnv = x->currentEnv;
        enter_phase(x, t_adsr_phase::Release);
    }
}

// === Parameter setters with clamping ===
void adsr_attack(t_adsr_tilde *x, t_floatarg f)
{
    x->attackTime = std::clamp(static_cast<double>(f), 0.0, 10000.0);                       // time in milliseconds
    x->attackPhaseSamples = std::max(1, static_cast<int>(x->attackTime * x->sampleratems)); // Samples to process
}

void adsr_decay(t_adsr_tilde *x, t_floatarg f)
{
    x->decayTime = std::clamp(static_cast<double>(f), 0.0, 10000.0);                      // time in milliseconds
    x->decayPhaseSamples = std::max(1, static_cast<int>(x->decayTime * x->sampleratems)); // Samples to process
}

void adsr_release(t_adsr_tilde *x, t_floatarg f)
{
    double t = (!x->startAtCurrentEnv) ? startupTime : 0;
    x->releaseTime = std::clamp(static_cast<double>(f - t), 0.0, 10000.0);                    // time in milliseconds
    x->releasePhaseSamples = std::max(1, static_cast<int>(x->releaseTime * x->sampleratems)); // Samples to process
}

void adsr_sustain(t_adsr_tilde *x, t_floatarg f)
{
    x->sustainLevel = std::clamp(static_cast<double>(f), 0.0, 1.0); // level in [0..1]
}

double map_shape_to_exponent(double f)
{
    double shape = std::clamp(f, -1.0, 1.0);
    return (shape < 0.0) ? 1.0 + shape * 0.9 : 1.0 + shape * 9.0;
}

void adsr_attackshape(t_adsr_tilde *x, t_floatarg f)
{
    x->attackShape = map_shape_to_exponent(f);
}

void adsr_releaseshape(t_adsr_tilde *x, t_floatarg f)
{
    x->releaseShape = map_shape_to_exponent(f);
}

void adsr_g(t_adsr_tilde *x, t_floatarg f)
{
    x->gain = std::clamp(static_cast<double>(f), 0.0, 1.0);
}

void adsr_dsp(t_adsr_tilde *x, t_signal **sp)
{
    x->samplerate = sp[0]->s_sr;
    x->sampleratems = x->samplerate / 1000;
    x->startupPhaseSamples = std::max(1, static_cast<int>(startupTime * x->sampleratems));
    dsp_add(adsr_perform, 3, x, sp[0]->s_vec, sp[0]->s_n);
}

// === Object constructor ===
void *adsr_new(t_floatarg f)
{
    t_adsr_tilde *x = (t_adsr_tilde *)pd_new(adsr_tilde_class);
    x->x_out = outlet_new(&x->x_obj, &s_signal);

    x->startAtCurrentEnv = f == 1.0;
    x->samplerate = 44100.0;
    x->sampleratems = 44.1;
    x->attackTime = 0.01;
    x->decayTime = 0.1;
    x->sustainLevel = 0.7;
    x->releaseTime = 0.2;
    x->attackShape = 2.0;
    x->releaseShape = 1.0;
    x->currentEnv = 0.0;
    x->gain = 1.0;
    x->phaseFunc = idlePhase;
    x->phase = t_adsr_phase::Idle;

    return (void *)x;
}

// === Setup function ===
extern "C"
{
    void adsr_tilde_setup(void)
    {
        adsr_tilde_class = class_new(gensym("adsr~"),
                                     (t_newmethod)adsr_new,
                                     0, sizeof(t_adsr_tilde),
                                     CLASS_DEFAULT, A_DEFFLOAT, 0);

        class_addmethod(adsr_tilde_class, (t_method)adsr_dsp, gensym("dsp"), A_CANT, 0);
        CLASS_MAINSIGNALIN(adsr_tilde_class, t_adsr_tilde, currentEnv);

        class_addmethod(adsr_tilde_class, (t_method)adsr_trigger_start, gensym("start"), A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_trigger_stop, gensym("stop"), A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_attack, gensym("attack"), A_DEFFLOAT, A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_decay, gensym("decay"), A_DEFFLOAT, A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_sustain, gensym("sustain"), A_DEFFLOAT, A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_release, gensym("release"), A_DEFFLOAT, A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_attackshape, gensym("attackshape"), A_DEFFLOAT, A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_releaseshape, gensym("releaseshape"), A_DEFFLOAT, A_NULL);
        class_addmethod(adsr_tilde_class, (t_method)adsr_g, gensym("g"), A_DEFFLOAT, A_NULL);
    }
}
