#include "PID.h"
#include <cmath>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : Kp(0.0), Ki(0.0), Kd(0.0) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	dp[0] = 0.05 * Kp;
	dp[1] = 0.05 * Ki;
	dp[2] = 0.05 * Kd;
	paramIndex = 0;
	e = first;

	p_error = 0;
	i_error = 0;
	d_error = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	return (- Kp * p_error - Ki * i_error - Kd * d_error);
}

void PID::twiddleUpdate(double curr_cte)
{
	double p[3]; p[0] = Kp; p[1] = Ki; p[2] = Kd;
	switch (e)
	{
		case first:
		{
			p[paramIndex] += dp[paramIndex];
			Kp = p[0]; Ki = p[1]; Kd = p[2];
			e = second;
			break;
		}

		case second:
		{
			if (curr_cte < best_error)
			{
				best_error = curr_cte;
				dp[paramIndex] *= 1.1;
				e = first;
				paramIndex = ++paramIndex % 3;
			}
			else
			{
				p[paramIndex] -= 2*dp[paramIndex];
				Kp = p[0]; Ki = p[1]; Kd = p[2];
				e = third;
			}
			break;
		}

		case third:
		{
			if (curr_cte >= best_error)
			{
				p[paramIndex] += dp[paramIndex];
				Kp = p[0]; Ki = p[1]; Kd = p[2];
				dp[paramIndex] *= 0.9;
			}
			else
				best_error = curr_cte;
			paramIndex = ++paramIndex % 3;
			p[paramIndex] += dp[paramIndex];
			Kp = p[0]; Ki = p[1]; Kd = p[2];
			e = second;
			break;
		}
	}
}


