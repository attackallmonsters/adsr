// adsr~.cpp — Pure Data external for a nonlinear ADSR envelope with clamping and validation

#include "m_pd.h"
#include <cmath>
#include <algorithm>

static t_class *adsr_tilde_class;

// === ADSR envelope phase enumeration ===
enum class t_adsr_phase
{
    Idle,
    Attack,
    Decay,
    Sustain,
    Release
};

// === Main object structure ===
struct t_adsr_tilde
{
    t_object x_obj;
    t_outlet *x_out;

    t_adsr_phase phase;
    double samplerate;
    double attackTime, decayTime, sustainLevel, releaseTime;
    double attackShape, releaseShape;
    double currentEnv, releaseStartEnv;
    int attackPhaseSamples, decayPhaseSamples, releasePhaseSamples;
    int currentSample;
};

// === Helper: shaped progress with exponential curvature ===
double shaped_progress(double p, double shape)
{
    if (shape == 1.0)
        return p;
    return pow(p, shape);
}

// === Enter a new phase and prepare sample counters ===
void enter_phase(t_adsr_tilde *x, t_adsr_phase newPhase)
{
    x->phase = newPhase;
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
        double p, env;

        switch (x->phase)
        {
        case t_adsr_phase::Attack:
            p = static_cast<double>(x->currentSample) / x->attackPhaseSamples;
            env = shaped_progress(p, x->attackShape);
            if (++x->currentSample >= x->attackPhaseSamples)
                enter_phase(x, t_adsr_phase::Decay);
            break;
        case t_adsr_phase::Decay:
            p = static_cast<double>(x->currentSample) / x->decayPhaseSamples;
            env = (1.0 - p) * (1.0 - x->sustainLevel) + x->sustainLevel;
            if (++x->currentSample >= x->decayPhaseSamples)
                enter_phase(x, t_adsr_phase::Sustain);
            break;
        case t_adsr_phase::Sustain:
            env = x->sustainLevel;
            break;
        case t_adsr_phase::Release:
            p = static_cast<double>(x->currentSample) / x->releasePhaseSamples;
            env = shaped_progress(1.0 - p, x->releaseShape) * x->releaseStartEnv;
            if (++x->currentSample >= x->releasePhaseSamples)
                enter_phase(x, t_adsr_phase::Idle);
            break;
        default:
            env = 0.0;
        }

        x->currentEnv = env;
        *out++ = static_cast<t_sample>(env);
    }
    return (w + 4);
}

void adsr_dsp(t_adsr_tilde *x, t_signal **sp)
{
    x->samplerate = sp[0]->s_sr;
    dsp_add(adsr_perform, 3, x, sp[0]->s_vec, sp[0]->s_n);
}

// === Trigger methods ===
void adsr_trigger_start(t_adsr_tilde *x)
{
    x->currentSample = 0;
    x->currentEnv =0;
    enter_phase(x, t_adsr_phase::Attack);
}

void adsr_trigger_stop(t_adsr_tilde *x)
{
    if (x->phase != t_adsr_phase::Idle )
    {
        x->releaseStartEnv = x->currentEnv;
        enter_phase(x, t_adsr_phase::Release);
    }
}

// === Parameter setters with clamping ===
void adsr_attack(t_adsr_tilde *x, t_floatarg f)
{
    x->attackTime = std::clamp(static_cast<double>(f), 0.0, 10000.0);                              // time in milliseconds
    x->attackPhaseSamples = std::max(1, static_cast<int>((x->attackTime / 1000) * x->samplerate)); // Samples to process
}

void adsr_decay(t_adsr_tilde *x, t_floatarg f)
{
    x->decayTime = std::clamp(static_cast<double>(f), 0.0, 10000.0);                             // time in milliseconds
    x->decayPhaseSamples = std::max(1, static_cast<int>((x->decayTime / 1000) * x->samplerate)); // Samples to process
}

void adsr_release(t_adsr_tilde *x, t_floatarg f)
{
    x->releaseTime = std::clamp(static_cast<double>(f), 0.0, 10000.0);                               // time in milliseconds
    x->releasePhaseSamples = std::max(1, static_cast<int>((x->releaseTime / 1000) * x->samplerate)); // Samples to process
}

void adsr_sustain(t_adsr_tilde *x, t_floatarg f)
{
    x->sustainLevel = std::clamp(static_cast<double>(f), 0.0, 1.0); // level in [0..1]
}

void adsr_attackshape(t_adsr_tilde *x, t_floatarg f)
{
    x->attackShape = std::clamp(static_cast<double>(f), 0.01, 10.0); // 1.0 = linear
}

void adsr_releaseshape(t_adsr_tilde *x, t_floatarg f)
{
    x->releaseShape = std::clamp(static_cast<double>(f), 0.01, 10.0); // 1.0 = linear
}

// === Object constructor ===
void *adsr_new()
{
    t_adsr_tilde *x = (t_adsr_tilde *)pd_new(adsr_tilde_class);
    x->x_out = outlet_new(&x->x_obj, &s_signal);

    x->samplerate = 44100.0; // default until dsp is called
    x->attackTime = 0.01;
    x->decayTime = 0.1;
    x->sustainLevel = 0.7;
    x->releaseTime = 0.2;
    x->attackShape = 1.0;
    x->releaseShape = 1.0;
    x->currentEnv = 0.0;
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
                                     CLASS_DEFAULT, A_NULL);

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
    }
}
