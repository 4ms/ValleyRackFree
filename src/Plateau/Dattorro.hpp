//
// This plate reverb is based upon Jon Dattorro's 1997 reverb algorithm.
//

#pragma once
#include <array>
#include <cassert>
#include <algorithm>
#include "dsp/delays/AllpassFilter.hpp"
#include "dsp/filters/OnePoleFilters.hpp"
#include "dsp/modulation/LFO.hpp"

#include "../../../../src/medium/debug_raw.h"


template<typename T = float>
class Dattorro1997Tank {
public:

	Dattorro1997Tank(const T initSampleRate,
									   const T initMaxLfoDepth,
									   const T initMaxTimeScale) :
		maxTimeScale(initMaxTimeScale) 
	{
		timePadding = initMaxLfoDepth;
		setSampleRate(initSampleRate);

		leftOutDCBlock.setCutoffFreq(20.0);
		rightOutDCBlock.setCutoffFreq(20.0);

		lfo1.setFrequency(lfo1Freq);
		lfo2.setFrequency(lfo2Freq);
		lfo3.setFrequency(lfo3Freq);
		lfo4.setFrequency(lfo4Freq);

		lfo2.phase = 0.25;
		lfo3.phase = 0.5;
		lfo4.phase = 0.75;

		lfo1.setRevPoint(0.5);
		lfo2.setRevPoint(0.5);
		lfo3.setRevPoint(0.5);
		lfo4.setRevPoint(0.5);
	}

	void process(const T leftIn, const T rightIn,
								   T* leftOut, T* rightOut) {
		DebugPin3High();//1us
		tickApfModulation();
		DebugPin3Low();

		decay = frozen ? 1.0 : decayParam;

		leftSum += leftIn;
		rightSum += rightIn;

		DebugPin3High();
		leftApf1.input = leftSum;
		leftDelay1.input = leftApf1.process();
		leftDelay1.process();
		leftHighCutFilter.input = leftDelay1.output;
		leftLowCutFilter.input = leftHighCutFilter.process();
		leftApf2.input = (leftDelay1.output * (1.0 - fade) + leftLowCutFilter.process() * fade) * decay;
		leftDelay2.input = leftApf2.process();
		leftDelay2.process();
		DebugPin3Low();

		rightApf1.input = rightSum;
		rightDelay1.input = rightApf1.process();
		rightDelay1.process();
		rightHighCutFilter.input = rightDelay1.output;
		rightLowCutFilter.input =  rightHighCutFilter.process();
		rightApf2.input = (rightDelay1.output * (1.0 - fade) + rightLowCutFilter.process() * fade) * decay;
		rightDelay2.input = rightApf2.process();
		rightDelay2.process();

		rightSum = leftDelay2.output * decay;
		leftSum = rightDelay2.output * decay;

		DebugPin3High();
		leftOutDCBlock.input = leftApf1.output;
		leftOutDCBlock.input += leftDelay1.tap(scaledOutputTaps[L_DELAY_1_L_TAP_1]);
		leftOutDCBlock.input += leftDelay1.tap(scaledOutputTaps[L_DELAY_1_L_TAP_2]);
		leftOutDCBlock.input -= leftApf2.delay.tap(scaledOutputTaps[L_APF_2_L_TAP]);
		leftOutDCBlock.input += leftDelay2.tap(scaledOutputTaps[L_DELAY_2_L_TAP]);
		leftOutDCBlock.input -= rightDelay1.tap(scaledOutputTaps[R_DELAY_1_L_TAP]);
		leftOutDCBlock.input -= rightApf2.delay.tap(scaledOutputTaps[R_APF_2_L_TAP]);
		leftOutDCBlock.input -= rightDelay2.tap(scaledOutputTaps[R_DELAY_2_L_TAP]);

		DebugPin3Low();
		rightOutDCBlock.input = rightApf1.output;
		rightOutDCBlock.input += rightDelay1.tap(scaledOutputTaps[R_DELAY_1_R_TAP_1]);
		rightOutDCBlock.input += rightDelay1.tap(scaledOutputTaps[R_DELAY_1_R_TAP_2]);
		rightOutDCBlock.input -= rightApf2.delay.tap(scaledOutputTaps[R_APF_2_R_TAP]);
		rightOutDCBlock.input += rightDelay2.tap(scaledOutputTaps[R_DELAY_2_R_TAP]);
		rightOutDCBlock.input -= leftDelay1.tap(scaledOutputTaps[L_DELAY_1_R_TAP]);
		rightOutDCBlock.input -= leftApf2.delay.tap(scaledOutputTaps[L_APF_2_R_TAP]);
		rightOutDCBlock.input -= leftDelay2.tap(scaledOutputTaps[L_DELAY_2_R_TAP]);

		DebugPin3High();
		*leftOut = leftOutDCBlock.process() * 0.5;
		*rightOut = rightOutDCBlock.process() * 0.5;

		fade += fadeStep * fadeDir;
		fade = (fade < 0.0) ? 0.0 : ((fade > 1.0) ? 1.0 : fade);

		assert(fade >= 0.0);
		assert(fade <= 1.0);
		DebugPin3Low();
	}

	void freeze(bool freezeFlag) {
		frozen = freezeFlag;
		if (frozen) {
			fadeDir = -1.0;
			decay = 1.0;
		}
		else {
			fadeDir = 1.0;
			decay = decayParam;
		}
	}

	void setSampleRate(const T newSampleRate) {
		sampleRate = newSampleRate;
		sampleRate = sampleRate > maxSampleRate ? maxSampleRate : sampleRate;
		sampleRate = sampleRate < 1.f ? 1.f : sampleRate;
		sampleRateScale = sampleRate / dattorroSampleRate;

		fadeStep = 1.0 / sampleRate;

		leftOutDCBlock.setSampleRate(sampleRate);
		rightOutDCBlock.setSampleRate(sampleRate);

		rescaleTapTimes();
		setTimeScale(timeScale);
		initialiseDelaysAndApfs();
		clear();
	}

	void setTimeScale(const T newTimeScale) {
		timeScale = newTimeScale;
		timeScale = timeScale < minTimeScale ? minTimeScale : timeScale;

		rescaleApfAndDelayTimes();
	}

	void setDecay(const T newDecay) {
		decayParam = (T)(newDecay > 1.0 ? 1.0 :
							 (newDecay < 0.0 ? 0.0 : newDecay));
	}

	void setModSpeed(const T newModSpeed) {
		lfo1.setFrequency(lfo1Freq * newModSpeed);
		lfo2.setFrequency(lfo2Freq * newModSpeed);
		lfo3.setFrequency(lfo3Freq * newModSpeed);
		lfo4.setFrequency(lfo4Freq * newModSpeed);
	}

	void setModDepth(const T newModDepth) {
		modDepth = newModDepth;
		lfoExcursion = newModDepth * lfoMaxExcursion * sampleRateScale;
	}

	void setModShape(const T shape) {
		lfo1.setRevPoint(shape);
		lfo2.setRevPoint(shape);
		lfo3.setRevPoint(shape);
		lfo4.setRevPoint(shape);
	}

	void setHighCutFrequency(const T frequency) {
		leftHighCutFilter.setCutoffFreq(frequency);
		rightHighCutFilter.setCutoffFreq(frequency);
	}

	void setLowCutFrequency(const T frequency) {
		leftLowCutFilter.setCutoffFreq(frequency);
		rightLowCutFilter.setCutoffFreq(frequency);
	}

	void setDiffusion(const T diffusion) {
		assert(diffusion >= 0.0 && diffusion <= 10.0);

		T diffusion1 = scale(diffusion, T(0.0), T(10.0), T(0.0), maxDiffusion1);
		T diffusion2 = scale(diffusion, T(0.0), T(10.0), T(0.0), maxDiffusion2);

		leftApf1.setGain(-diffusion1);
		leftApf2.setGain(diffusion2);
		rightApf1.setGain(-diffusion1);
		rightApf2.setGain(diffusion2);
	}

	void clear() {
		leftApf1.clear();
		leftDelay1.clear();
		leftHighCutFilter.clear();
		leftLowCutFilter.clear();
		leftApf2.clear();
		leftDelay2.clear();

		rightApf1.clear();
		rightDelay1.clear();
		rightHighCutFilter.clear();;
		rightLowCutFilter.clear();
		rightApf2.clear();
		rightDelay2.clear();

		leftOutDCBlock.clear();
		rightOutDCBlock.clear();

		leftSum = 0.0;
		rightSum = 0.0;
	}

private:
    static constexpr T dattorroSampleRate = 29761.0;
    static constexpr T dattorroSampleTime = 1.0 / dattorroSampleRate;

    static constexpr T leftApf1Time = 672.0;
    static constexpr T leftDelay1Time = 4453.0;
    static constexpr T leftApf2Time = 1800.0;
    static constexpr T leftDelay2Time = 3720.0;

    static constexpr T rightApf1Time = 908.0;
    static constexpr T rightDelay1Time = 4217.0;
    static constexpr T rightApf2Time = 2656.0;
    static constexpr T rightDelay2Time = 3163.0;

    static constexpr size_t leftDelay1RightTap1 = 266;
    static constexpr size_t leftDelay1RightTap2 = 2974;
    static constexpr size_t leftApf2RightTap1 = 1913;
    static constexpr size_t leftDelay2RightTap = 1996;
    static constexpr size_t rightDelay1RightTap = 1990;
    static constexpr size_t rightApf2RightTap = 187;
    static constexpr size_t rightDelay2RightTap = 1066;

    enum LeftOutTaps {
        L_DELAY_1_L_TAP_1,
        L_DELAY_1_L_TAP_2,
        L_APF_2_L_TAP,
        L_DELAY_2_L_TAP,
        R_DELAY_1_L_TAP,
        R_APF_2_L_TAP,
        R_DELAY_2_L_TAP
    };

    enum RightOutTaps {
        R_DELAY_1_R_TAP_1,
        R_DELAY_1_R_TAP_2,
        R_APF_2_R_TAP,
        R_DELAY_2_R_TAP,
        L_DELAY_1_R_TAP,
        L_APF_2_R_TAP,
        L_DELAY_2_R_TAP
    };

    const long kOutputTaps[7] = {266, 2974, 1913, 1996, 1990, 187, 1066};

    static constexpr T maxDiffusion1 = 0.7;
    static constexpr T maxDiffusion2 = 0.7;

    static constexpr T lfoMaxExcursion = 16.0;
    static constexpr T lfo1Freq = 0.10;
    static constexpr T lfo2Freq = 0.150;
    static constexpr T lfo3Freq = 0.120;
    static constexpr T lfo4Freq = 0.180;

    static constexpr T minTimeScale = 0.0001;

    T timePadding = 0.0;

    T scaledLeftApf1Time = leftApf1Time;
    T scaledLeftDelay1Time = leftDelay1Time;
    T scaledLeftApf2Time = leftApf2Time;
    T scaledLeftDelay2Time = leftDelay2Time;

    T scaledRightApf1Time = rightApf1Time;
    T scaledRightDelay1Time = rightDelay1Time;
    T scaledRightApf2Time = rightApf2Time;
    T scaledRightDelay2Time = rightDelay2Time;

    std::array<long, 7> scaledOutputTaps;

    T maxSampleRate = 44100.;
    T sampleRate = maxSampleRate;
    T sampleRateScale = sampleRate / dattorroSampleRate;

    T maxTimeScale = 1.0;
    T timeScale = 1.0;

    T modDepth = 0.0;
    T decayParam = 0.0;
    T decay = 0.0;

    T lfoExcursion = 0.0;

    // Freeze Cross fade
    bool frozen = false;
    T fade = 1.0;
    T fadeTime = 0.002;
    T fadeStep = 1.0 / (fadeTime * sampleRate);
    T fadeDir = 1.0;

    TriSawLFO<T> lfo1;
    TriSawLFO<T> lfo2;
    TriSawLFO<T> lfo3;
    TriSawLFO<T> lfo4;

    T leftSum = 0.0;
    T rightSum = 0.0;

    AllpassFilter<T> leftApf1;
    InterpDelay<T> leftDelay1;
    OnePoleLPFilter leftHighCutFilter;
    OnePoleHPFilter leftLowCutFilter;
    AllpassFilter<T> leftApf2;
    InterpDelay<T> leftDelay2;

    AllpassFilter<T> rightApf1;
    InterpDelay<T> rightDelay1;
    OnePoleLPFilter rightHighCutFilter;;
    OnePoleHPFilter rightLowCutFilter;
    AllpassFilter<T> rightApf2;
    InterpDelay<T> rightDelay2;

    OnePoleHPFilter leftOutDCBlock;
    OnePoleHPFilter rightOutDCBlock;

	void initialiseDelaysAndApfs() {
		auto maxScaledOutputTap = *std::max_element(scaledOutputTaps.begin(),
													scaledOutputTaps.end());
		auto calcMaxTime = [&](T delayTime) -> long {
			return (long)(sampleRateScale * (delayTime * maxTimeScale + 
											 maxScaledOutputTap + timePadding));
		};

		const long kLeftApf1MaxTime = calcMaxTime(leftApf1Time);
		const long kLeftDelay1MaxTime = calcMaxTime(leftDelay1Time);
		const long kLeftApf2MaxTime = calcMaxTime(leftApf2Time);
		const long kLeftDelay2MaxTime = calcMaxTime(leftDelay2Time);
		const long kRightApf1MaxTime = calcMaxTime(rightApf1Time);
		const long kRightDelay1MaxTime = calcMaxTime(rightDelay1Time);
		const long kRightApf2MaxTime = calcMaxTime(rightApf2Time);
		const long kRightDelay2MaxTime = calcMaxTime(rightDelay2Time);

		leftApf1 = AllpassFilter<T>(kLeftApf1MaxTime);
		leftDelay1 = InterpDelay<T>(kLeftDelay1MaxTime);
		leftApf2 = AllpassFilter<T>(kLeftApf2MaxTime);
		leftDelay2 = InterpDelay<T>(kLeftDelay2MaxTime);
		rightApf1 = AllpassFilter<T>(kRightApf1MaxTime);
		rightDelay1 = InterpDelay<T>(kRightDelay1MaxTime);
		rightApf2 = AllpassFilter<T>(kRightApf2MaxTime);
		rightDelay2 = InterpDelay<T>(kRightDelay2MaxTime);
	}

	void tickApfModulation() {
		leftApf1.delay.setDelayTime(lfo1.process() * lfoExcursion + scaledLeftApf1Time);
		leftApf2.delay.setDelayTime(lfo2.process() * lfoExcursion + scaledLeftApf2Time);
		rightApf1.delay.setDelayTime(lfo3.process() * lfoExcursion + scaledRightApf1Time);
		rightApf2.delay.setDelayTime(lfo4.process() * lfoExcursion + scaledRightApf2Time);
	}

	void rescaleApfAndDelayTimes() {
		T scaleFactor = timeScale * sampleRateScale;

		scaledLeftApf1Time = leftApf1Time * scaleFactor;
		scaledLeftDelay1Time = leftDelay1Time * scaleFactor;
		scaledLeftApf2Time = leftApf2Time * scaleFactor;
		scaledLeftDelay2Time = leftDelay2Time * scaleFactor;

		scaledRightApf1Time = rightApf1Time * scaleFactor;
		scaledRightDelay1Time = rightDelay1Time * scaleFactor;
		scaledRightApf2Time = rightApf2Time * scaleFactor;
		scaledRightDelay2Time = rightDelay2Time * scaleFactor;

		leftDelay1.setDelayTime(scaledLeftDelay1Time);
		leftDelay2.setDelayTime(scaledLeftDelay2Time);
		rightDelay1.setDelayTime(scaledRightDelay1Time);
		rightDelay2.setDelayTime(scaledRightDelay2Time);
	}

	void rescaleTapTimes() {
		for (size_t i = 0; i < scaledOutputTaps.size(); ++i) {
			scaledOutputTaps[i] = (long)((T)kOutputTaps[i] * sampleRateScale);
		}
	}

};

template<typename T = float>
class Dattorro {
public:

	Dattorro(const T initMaxSampleRate,
					   const T initMaxLfoDepth,
					   const T initMaxTimeScale)
		: tank(initMaxSampleRate, initMaxLfoDepth, initMaxTimeScale)
	{
		sampleRate = initMaxSampleRate;
		dattorroScaleFactor = sampleRate / dattorroSampleRate;

		preDelay = InterpDelay<T>(192010, 0);

		inputLpf = OnePoleLPFilter(22000.0);
		inputHpf = OnePoleHPFilter(0.0);

		inApf1 = AllpassFilter<T>(dattorroScale(8 * kInApf1Time), dattorroScale(kInApf1Time), inputDiffusion1);
		inApf2 = AllpassFilter<T>(dattorroScale(8 * kInApf2Time), dattorroScale(kInApf2Time), inputDiffusion1);
		inApf3 = AllpassFilter<T>(dattorroScale(8 * kInApf3Time), dattorroScale(kInApf3Time), inputDiffusion2);
		inApf4 = AllpassFilter<T>(dattorroScale(8 * kInApf4Time), dattorroScale(kInApf4Time), inputDiffusion2);

		leftInputDCBlock.setCutoffFreq(20.0);
		rightInputDCBlock.setCutoffFreq(20.0);
	}

	void process(T leftInput, T rightInput) {
		leftInputDCBlock.input = leftInput;
		rightInputDCBlock.input = rightInput;
		inputLpf.setCutoffFreq(inputHighCut);
		inputHpf.setCutoffFreq(inputLowCut);
		inputLpf.input = leftInputDCBlock.process() + rightInputDCBlock.process();
		inputHpf.input = inputLpf.process();
		inputHpf.process();
		preDelay.input = inputHpf.output;
		preDelay.process();
		inApf1.input = preDelay.output;
		inApf2.input = inApf1.process();
		inApf3.input = inApf2.process();
		inApf4.input = inApf3.process();
		tankFeed = preDelay.output * (1.0 - diffuseInput) + inApf4.process() * diffuseInput;

		DebugPin2High();
		tank.process(tankFeed, tankFeed, &leftOut, &rightOut);
		DebugPin2Low();
	}

	void clear() {
		leftInputDCBlock.clear();
		rightInputDCBlock.clear();

		inputLpf.clear();
		inputHpf.clear();
		preDelay.clear();
		inApf1.clear();
		inApf2.clear();
		inApf3.clear();
		inApf4.clear();

		tank.clear();
	}

	void setTimeScale(T timeScale) {
		constexpr T minTimeScale = 0.0001;
		timeScale = timeScale < minTimeScale ? minTimeScale : timeScale;
		tank.setTimeScale(timeScale);
	}

	void setPreDelay(T t) {
		preDelayTime = t;
		preDelay.setDelayTime(preDelayTime * sampleRate);
	}

	void setSampleRate(T newSampleRate) {
		assert(newSampleRate > 0.);

		sampleRate = newSampleRate;
		tank.setSampleRate(sampleRate);
		dattorroScaleFactor = sampleRate / dattorroSampleRate;
		setPreDelay(preDelayTime);
		inApf1.delay.setDelayTime(dattorroScale(kInApf1Time));
		inApf2.delay.setDelayTime(dattorroScale(kInApf2Time));
		inApf3.delay.setDelayTime(dattorroScale(kInApf3Time));
		inApf4.delay.setDelayTime(dattorroScale(kInApf4Time));

		leftInputDCBlock.setSampleRate(sampleRate);
		rightInputDCBlock.setSampleRate(sampleRate);
		inputLpf.setSampleRate(sampleRate);
		inputHpf.setSampleRate(sampleRate);

		clear();
	}

	void freeze(bool freezeFlag) {
		tank.freeze(freezeFlag);
	}

	void setInputFilterLowCutoffPitch(T pitch) {
		inputLowCut = 440.0 * std::pow(2.0, pitch - 5.0);
	}

	void setInputFilterHighCutoffPitch(T pitch) {
		inputHighCut = 440.0 * std::pow(2.0, pitch - 5.0);
	}

	void enableInputDiffusion(bool enable) {
		diffuseInput = enable ? 1.0 : 0.0;
	}

	void setDecay(T newDecay) {
		decay = newDecay;
		assert(decay <= 1.0);
		tank.setDecay(decay);
	}

	void setTankDiffusion(const T diffusion) {
		tank.setDiffusion(diffusion);
	}

	void setTankFilterHighCutFrequency(const T pitch) {
		auto frequency = 440.0 * std::pow(2.0, pitch - 5.0);
		tank.setHighCutFrequency(frequency);
	}

	void setTankFilterLowCutFrequency(const T pitch) {
		auto frequency = 440.0 * std::pow(2.0, pitch - 5.0);
		tank.setLowCutFrequency(frequency);
	}

	void setTankModSpeed(const T modSpeed) {
		tank.setModSpeed(modSpeed);
	}

	void setTankModDepth(const T modDepth) {
		tank.setModDepth(modDepth);
	}

	void setTankModShape(const T modShape) {
		tank.setModShape(modShape);
	}

	T getLeftOutput() const {
		return leftOut;
	}

	T getRightOutput() const {
		return rightOut;
	}

private:
    T preDelayTime = 0.0;
    static constexpr long kInApf1Time = 141;
    static constexpr long kInApf2Time = 107;
    static constexpr long kInApf3Time = 379;
    static constexpr long kInApf4Time = 277;

    static constexpr T dattorroSampleRate = 29761.0;
    T sampleRate = 44100.0;
    T dattorroScaleFactor = sampleRate / dattorroSampleRate;
    T leftSum = 0.0;
    T rightSum = 0.0;

    T rightOut = 0.0;
    T leftOut = 0.0;
    T inputLowCut = 0.0;
    T inputHighCut = 10000.0;
    T reverbHighCut = 10000.0;
    T reverbLowCut = 0.0;
    T modDepth = 1.0;
    T inputDiffusion1 = 0.75;
    T inputDiffusion2 = 0.625;
    T decay = 0.9999;
    T modSpeed = 1.0;
    T diffuseInput = 0.0;

    OnePoleHPFilter leftInputDCBlock;
    OnePoleHPFilter rightInputDCBlock;
    OnePoleLPFilter inputLpf;
    OnePoleHPFilter inputHpf;

    InterpDelay<T> preDelay;

    AllpassFilter<T> inApf1;
    AllpassFilter<T> inApf2;
    AllpassFilter<T> inApf3;
    AllpassFilter<T> inApf4;

    Dattorro1997Tank<T> tank;

    T tankFeed = 0.0;


	T dattorroScale(T delayTime) {
		return delayTime * dattorroScaleFactor;
	}

};

