#include "offboard_control/pid_controller_base.h"

PidControllerBase::PidControllerBase()
	: kp_(0.0),
		ki_(0.0),
		kd_(0.0),
		ui_old_(0.0),
		error_old_(0.0),
		u_max_(std::numeric_limits<double>::infinity()),
		u_min_(-std::numeric_limits<double>::infinity())
{

}

PidControllerBase::PidControllerBase(double kp, double ki, double kd)
	: kp_(kp),
		ki_(ki),
		kd_(kd),
		ui_old_(0.0),
		error_old_(0.0),
		u_max_(std::numeric_limits<double>::infinity()),
		u_min_(-std::numeric_limits<double>::infinity())
{
	// kp_ = kp;
	// ki_ = ki;
	// kd_ = kd;
}

PidControllerBase::PidControllerBase(double kp, double ki, double kd,
									bool proportional_on_measurement, bool differetial_on_measurement)
	: kp_(kp),
		ki_(ki),
		kd_(kd),
		ui_old_(0.0),
		error_old_(0.0),
		proportional_on_measurement_(true),
		differetial_on_measurement_(true),
		u_max_(std::numeric_limits<double>::infinity()),
		u_min_(-std::numeric_limits<double>::infinity())
{
	// kp_ = kp;
	// ki_ = ki;
	// kd_ = kd;
}


PidControllerBase::~PidControllerBase()
{

};

double PidControllerBase::getKp(void)
{
	return kp_;
}

void PidControllerBase::SetModeP(int mode)
{
	proportional_on_measurement_ = mode;
}

void PidControllerBase::SetModeD(int mode)
{
	differetial_on_measurement_ = mode;
}


double PidControllerBase::getKi(void)
{
	return ki_;
}

double PidControllerBase::getKd(void)
{
	return kd_;
}

double PidControllerBase::getUMax(void)
{
	return u_max_;
}

double PidControllerBase::getUMin(void)
{
	return u_min_;
}

double PidControllerBase::getPTerm(void)
{
	return pTerm;
}

double PidControllerBase::getITerm(void)
{
	return iTerm;
}

double PidControllerBase::getDTerm(void)
{
	return dTerm;
}

void PidControllerBase::setKp(double kp)
{
	kp_ = kp;
}

void PidControllerBase::setKi(double ki)
{
	ki_ = ki;
}

void PidControllerBase::setKd(double kd)
{
	kd_ = kd;
}

void PidControllerBase::setUMax(double u_max)
{
	u_max_ = u_max;
}

void PidControllerBase::setUMin(double u_min)
{
	u_min_ = u_min;
}

double PidControllerBase:: compute(double setpoint, double measurement)
{
	double dt;
	double u;

	ros::Time now = ros::Time::now();

	if (!lastTime.isZero()) {

		dt = (now - lastTime).toSec();

		if (0 == dt) {

			return last_u;
		}

	} else {

		ROS_INFO("lastTime is 0, doing nothing");
		lastTime = ros::Time::now();

		return last_u;
	}

	if (sample_time > dt) {

		return last_u;
	}

	/*
	* Error signal
	*/
	double error = setpoint - measurement;

	/*
	* Proportional
	*/
	pTerm = kp_ * error;

	/*
	* Integral
	*/
	iTerm = iTerm + 0.5 * ki_ * dt * (error + prevError);

	/* Anti-wind-up via integrator clamping */
	if (iTerm > limMaxInt) {

		iTerm = limMaxInt;

	} else if (iTerm < limMinInt) {

		iTerm = limMinInt;
	}

	/*
	* Derivative (band-limited differentiator)
	*/

	dTerm = -(2.0 * kd_ * (measurement - prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
						+ (2.0 * tau - dt) * dTerm)
						/ (2.0 * tau + dt);

	/*
	* Compute output and apply limits
	*/
	u = pTerm + iTerm + dTerm;

	u = clamp(u, u_max_, u_min_);

	/* Store error and measurement for later use */
	prevError       = error;
	prevMeasurement = measurement;

	last_u = u;
	lastTime = now;

	return u;
}

double PidControllerBase::clamp(double value, double u_max, double u_min){

	if (value > u_max)
	{
		return u_max;
	} else if (value < u_min) {

		return u_min;
	}

	return value;
}
