#ifndef PID_HPP
#define PID_HPP

#include <fixed_point/scaled_int.hpp>

#include <stdint.h>

namespace fixed_point {
namespace control {
namespace detail {

typedef int8_t Saturation;

template<typename TValue>
inline Saturation saturate(TValue& value, const TValue& valueMin, const TValue& valueMax)
{
	if (value <= valueMin) {
		value = valueMin;
		return -1;
	}
	else if (value >= valueMax) {
		value = valueMax;
		return 1;
	}
	return 0;
}

template<typename TValue>
inline void limit(TValue& value, const TValue& valueMin, const TValue& valueMax)
{
	if (value <= valueMin) {
		value = valueMin;
	}
	else if (value >= valueMax) {
		value = valueMax;
	}
}

template<typename TInput, typename TOutput, typename TKp>
class P
{
public:
	typedef TInput	input_type;
	typedef TOutput	output_type;
	typedef TKp		p_gain_type;

	typedef fixed_point::scaled_int<p_gain_type::M + input_type::M + 1, p_gain_type::N + input_type::N>	p_gain_intermediate_type;

	P(const p_gain_type& kp)
		: mKp(kp)
	{}

	void setKp(const p_gain_type& kp)
	{
		mKp = kp;
	}

	output_type calculateOutput(const input_type& error, const output_type&, const output_type&) const
	{
		const p_gain_intermediate_type proportionalTerm = mKp * error;
		return proportionalTerm.template convert<output_type::M, output_type::N>();
	}

private:
	p_gain_type mKp;
};

template<typename TInput, typename TOutput, typename TKi>
class I
{
public:
	typedef TInput	input_type;
	typedef TOutput	output_type;
	typedef TKi		i_gain_type;

	typedef fixed_point::scaled_int<output_type::M, output_type::N + output_type::BIT_SIZE> i_term_type;

	I(const i_gain_type& ki)
		: mKi(ki)
		, mSaturation(0)
		, mIntegralTerm(0)
	{}

	void setKi(const i_gain_type& ki)
	{
		mKi = ki;
	}

	output_type calculateOutput(const input_type& error, const output_type& outputMin, const output_type& outputMax)
	{
		if (!((mSaturation < 0 && error.getValue() < 0) || (mSaturation > 0 && error.getValue() > 0))) {
			mIntegralTerm += (mKi * error).template convert<i_term_type::M, i_term_type::N>();
		}
		mSaturation = saturate(
			mIntegralTerm,
			outputMin.template convert<i_term_type::M, i_term_type::N>(),
			outputMax.template convert<i_term_type::M, i_term_type::N>());
		return mIntegralTerm.template convert<output_type::M, output_type::N>();
	}

private:
	i_gain_type mKi;
	Saturation	mSaturation;
	i_term_type	mIntegralTerm;
};

template<typename TInput, typename TOutput, typename TKd>
class D
{
public:
	typedef TInput	input_type;
	typedef TOutput	output_type;
	typedef TKd		d_gain_type;

	typedef fixed_point::scaled_int<d_gain_type::M + input_type::M + 1, d_gain_type::N + input_type::N>	d_gain_intermediate_type;

	D(const d_gain_type& kd)
		: mKd(kd)
		, mPreviousError(0)
	{}

	void setKd(const d_gain_type& kd)
	{
		mKd = kd;
	}

	output_type calculateOutput(const input_type& error, const output_type&, const output_type&)
	{
		const d_gain_intermediate_type derivativeTerm = mKd * (error - mPreviousError);
		mPreviousError = error;
		return derivativeTerm.template convert<output_type::M, output_type::N>();
	}

private:
	d_gain_type	mKd;
	input_type	mPreviousError;
};

template<int8_t TTag>
struct Non
{
	Non()
	{}

	template<typename TInput, typename TOutput>
	TOutput calculateOutput(const TInput&, const TOutput&, const TOutput&)
	{
		return TOutput(0);
	}
};

template<typename TInput, typename TOutput, typename TP, typename TI, typename TD>
class Controller : public TP, public TI, public TD
{
public:
	typedef TInput	input_type;
	typedef TOutput	output_type;

	Controller(const TP& p, const TI& i, const TD& d, const output_type& outputMin, const output_type& outputMax)
		: TP(p)
		, TI(i)
		, TD(d)
		, mOutputMin(outputMin)
		, mOutputMax(outputMax)
	{}

	output_type calculateOutput(const input_type& reference, const input_type& measured)
	{
		const input_type error = reference - measured;

		const output_type p = TP::calculateOutput(error, mOutputMin, mOutputMax);
		const output_type i = TI::calculateOutput(error, mOutputMin, mOutputMax);
		const output_type d = TD::calculateOutput(error, mOutputMin, mOutputMax);
		output_type output = p + i + d;
		limit(output, mOutputMin, mOutputMax);
		return output;
	}

private:
	const output_type mOutputMin;
	const output_type mOutputMax;
};

} // namespace detail

template<typename TInput, typename TOutput, typename TKp>
struct PController : detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::Non<0>, detail::Non<1> >
{
	PController(const TKp& kp, const TOutput& minOutput, const  TOutput& maxOutput)
		: detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::Non<0>, detail::Non<1> >(kp, detail::Non<0>(), detail::Non<1>(), minOutput, maxOutput)
	{}
};

template<typename TInput, typename TOutput, typename TKp, typename TKi>
struct PIController : detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::I<TInput, TOutput, TKi>, detail::Non<0> >
{
	PIController(const TKp& kp, const TKi& ki, const TOutput& minOutput, const  TOutput& maxOutput)
		: detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::I<TInput, TOutput, TKi>, detail::Non<0> >(kp, ki, detail::Non<0>(), minOutput, maxOutput)
	{}
};

template<typename TInput, typename TOutput, typename TKp, typename TKd>
struct PDController : detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::Non<0>, detail::D<TInput, TOutput, TKd> >
{
	PDController(const TKp& kp, const TKd& kd, const TOutput& minOutput, const  TOutput& maxOutput)
		: detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::Non<0>, detail::D<TInput, TOutput, TKd> >(kp, detail::Non<0>(), kd, minOutput, maxOutput)
	{}
};

template<typename TInput, typename TOutput, typename TKp, typename TKi, typename TKd>
struct PIDController : detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::I<TInput, TOutput, TKi>, detail::D<TInput, TOutput, TKd> >
{
	PIDController(const TKp& kp, const TKi& ki, const TKd& kd, const TOutput& minOutput, const  TOutput& maxOutput)
		: detail::Controller<TInput, TOutput, detail::P<TInput, TOutput, TKp>, detail::I<TInput, TOutput, TKi>, detail::D<TInput, TOutput, TKd> >(kp, ki, kd, minOutput, maxOutput)
	{}
};

} // namespace control
} // fixed_point

#endif // !PID_HPP