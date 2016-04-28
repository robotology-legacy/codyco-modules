
#ifndef SPLINEINTERPOLATOR_H
#define SPLINEINTERPOLATOR_H

#include <cassert>
#include <cstdlib>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <limits>

#include "string_utils.h"

template <typename VectorType>
struct SplineInterpolator {
	SplineInterpolator() :
		initialized(false) {}
    
    /** Time values found in the input file
     */
	std::vector<double> t_values;
    /** 3D Coordinates found in the input file
     */
	std::vector<VectorType> p_values;
    /** Has the same size as p_values. Computes
     *  derivatives of every three points.
     */
	std::vector<VectorType> m_values;

    /** Puts time values from FILENAME into t_values and
     *  the corresponding 3D coordinates into p_values.
     */
	bool generateFromCSV (const char* filename);
	void addPoints (double t, const VectorType &values);
    /** Computes derivatives of the input data storing 
     *  them in m_values
     */
	void initialize ();

    /** Calls initialize() and getInterpolants() 
     */
	VectorType getValues (double t);
	VectorType getDerivatives (double t);
	VectorType getSecondDerivatives (double t);

	double getStartTime();
	double getEndTime();

	bool initialized;

	private:
	double h00 (double t) { return 2. * t * t * t - 3. * t * t + 1.; };
	double h10 (double t) { return t * t * t - 2. * t * t + t; };
	double h01 (double t) { return - 2. * t * t * t + 3. * t * t; };
	double h11 (double t) { return t * t * t - t * t; };

	double h00_dot (double t) { return 6. * t * t - 6. * t; };
	double h10_dot (double t) { return 3. * t * t - 4. * t + 1.; };
	double h01_dot (double t) { return - 6. * t * t + 6. * t; };
	double h11_dot (double t) { return 3. * t * t - 2. * t; };

	double h00_ddot (double t) { return 12. * t - 6.; };
	double h10_ddot (double t) { return 6. * t - 4.; };
	double h01_ddot (double t) { return - 12. * t + 6.; };
	double h11_ddot (double t) { return 6. * t - 2.; };
    /** For a desired time t, it returns the two consecutive times and coordinates
     *  as read from the input file that will contain it.
     */
	void getInterpolants (double t, double &t0, double &t1, VectorType &p0, VectorType &m0, VectorType &p1, VectorType &m1);
};

template <typename VectorType>
inline bool SplineInterpolator<VectorType>::generateFromCSV (const char* filename) {
	std::ifstream infile (filename);
	if (!infile) {
		std::cerr << "Error spline input file from '" << filename << "'" << std::endl;
		abort();
		return false;
	}

	t_values.clear();
	p_values.clear();

	int line_index = 0;
	std::string line;
	std::string previous_line;

	while (!infile.eof()) {
		previous_line = line;
		getline (infile, line);
    
		if (infile.eof()) {
			break;
		} else {
			line_index++;
		}

		std::vector<std::string> tokens = tokenize_csv_strip_whitespaces (line);
		if (tokens.size() == 0)
			continue;

		VectorType state = VectorType::Zero (tokens.size() - 1);
		for (size_t i = 0; i < tokens.size(); i++) {
			double value;
			std::stringstream value_stream (tokens[i]);
			if (!(value_stream >> value)) {
				std::cerr << "Error: could not convert std::string '" << tokens[i] << "' to number in " << filename << ", line " << line_index << ", column " << i << std::endl;
				abort();
			}

			if (i == 0)
				t_values.push_back(value);
			else {
				state[i - 1] = value;
			}
		}

		p_values.push_back(state);
		//std::cout << "[SplineInterpolator::generateFromCSV] At t = " << t_values[t_values.size() - 1] << " added " << p_values[p_values.size() - 1].transpose() << std::endl;
	}
    //std::cout << std::endl;

	infile.close();

	initialized = false;

	return true;
}

template <typename VectorType>
void SplineInterpolator<VectorType>::addPoints (double t, const VectorType &values) {
	initialized = false;

	t_values.push_back (t);
	p_values.push_back (values);
}

template <typename VectorType>
inline void SplineInterpolator<VectorType>::initialize() {
	assert (t_values.size() == p_values.size());

	if (initialized) 
		return;

	if (t_values.size() < 2) {
		std::cerr << "Error: too few points for interpolation!" << std::endl;
		abort();
	}

	m_values = std::vector<VectorType>(p_values.size(), VectorType::Zero(p_values[0].size()));

	m_values[0] = (p_values[1] - p_values[0]) / (t_values[1] - t_values[0]);

	size_t last_index = t_values.size() - 1;
	m_values[last_index] = (p_values[last_index] - p_values[last_index - 1]) / (t_values[last_index] - t_values[last_index - 1]);

	for (size_t i = 1; i < t_values.size() - 1; i ++) {
		VectorType m_value = VectorType::Zero(p_values[0].size());

		// finite difference
		// m_value = (p_values[i + 1] - p_values[i]) / (2. * (t_values[i + 1] - t_values[i])) + (p_values[i] - p_values[i - 1]) / (2. * (t_values[i] - t_values[i - 1]));

		// central difference quotient
		//m_value = (p_values[i + 1] - p_values[i - 1]) / (2. * (t_values[i + 1] - t_values[i - 1]));

		// Catmull-Rom Spline
		m_value = (p_values[i + 1] - p_values[i - 1]) / (t_values[i + 1] - t_values[i - 1]);

		m_values[i] = m_value;
//		std::cerr << "Added derivatives: " << m_value.transpose() << std::endl;
	}

	initialized = true;
}

template <typename VectorType>
inline void SplineInterpolator<VectorType>::getInterpolants (double t, double &t0, double &t1, VectorType &p0, VectorType &m0, VectorType &p1, VectorType &m1) {
	for (size_t i = 1; i < t_values.size(); i ++) {
		if (t >= t_values[i - 1] && (t <= t_values[i] || t - t_values[i] < std::numeric_limits<double>::epsilon())) {
			t0 = t_values[i - 1];
			t1 = t_values[i];
			p0 = p_values[i - 1];
			m0 = m_values[i - 1];
			p1 = p_values[i];
			m1 = m_values[i];
			
			return;
		}
	}

	std::cerr.precision(16);
	std::cerr << "Could not find interpolants at time " << std::scientific << t << ". Range is [" << getStartTime() << ", " << getEndTime() << "]!" << std::endl;
	abort();
}

template <typename VectorType>
inline VectorType SplineInterpolator<VectorType>::getValues(double t) {
	initialize();

	double t0;
	double t1;
	VectorType p0 (VectorType::Zero(p_values[0].size()));
	VectorType m0 (VectorType::Zero(p_values[0].size()));
	VectorType p1 (VectorType::Zero(p_values[0].size()));
	VectorType m1 (VectorType::Zero(p_values[0].size()));

	getInterpolants (t, t0, t1, p0, m0, p1, m1);	

//	std::cout << "Using interpolants: t0 = " << t0 << " t1 = " << t1 << std::endl 
//		<< " p0 = " << p0.transpose() << std::endl
//		<< " m0 = " << m0.transpose() << std::endl
//		<< " p1 = " << p1.transpose() << std::endl
//		<< " m1 = " << m1.transpose() << std::endl;

	double tau = (t - t0) / (t1 - t0);

	return h00(tau) * p0 + h10(tau) * (t1 - t0) * m0 + h01(tau) * p1 + h11(tau) * (t1 - t0) * m1;
}

template <typename VectorType>
inline VectorType SplineInterpolator<VectorType>::getDerivatives(double t) {
	initialize();

	double t0;
	double t1;
	VectorType p0 (VectorType::Zero(p_values[0].size()));
	VectorType m0 (VectorType::Zero(p_values[0].size()));
	VectorType p1 (VectorType::Zero(p_values[0].size()));
	VectorType m1 (VectorType::Zero(p_values[0].size()));

	getInterpolants (t, t0, t1, p0, m0, p1, m1);	

//	std::cout << "Using interpolants: t0 = " << t0 << " t1 = " << t1 << std::endl 
//		<< " p0 = " << p0.transpose() << std::endl
//		<< " m0 = " << m0.transpose() << std::endl
//		<< " p1 = " << p1.transpose() << std::endl
//		<< " m1 = " << m1.transpose() << std::endl;

	double tau = (t - t0) / (t1 - t0);
	double taudot = (1.) / ((t1 - t0));

	return h00_dot(tau) * taudot * p0 + h10_dot(tau) * taudot * (t1 - t0) * m0 + h01_dot(tau) * taudot * p1 + h11_dot(tau) * taudot * (t1 - t0) * m1;
}

template <typename VectorType>
inline VectorType SplineInterpolator<VectorType>::getSecondDerivatives(double t) {
	initialize();

	double t0;
	double t1;
	VectorType p0 (VectorType::Zero(p_values[0].size()));
	VectorType m0 (VectorType::Zero(p_values[0].size()));
	VectorType p1 (VectorType::Zero(p_values[0].size()));
	VectorType m1 (VectorType::Zero(p_values[0].size()));

	getInterpolants (t, t0, t1, p0, m0, p1, m1);	

//	std::cout << "Using interpolants: t0 = " << t0 << " t1 = " << t1 << std::endl 
//		<< " p0 = " << p0.transpose() << std::endl
//		<< " m0 = " << m0.transpose() << std::endl
//		<< " p1 = " << p1.transpose() << std::endl
//		<< " m1 = " << m1.transpose() << std::endl;

	double tau = (t - t0) / (t1 - t0);
	double taudot_2 = (1.) / ((t1 - t0) * (t1 - t0));

	return h00_ddot(tau) * taudot_2 * p0 + h10_ddot(tau) * taudot_2 * (t1 - t0) * m0 + h01_ddot(tau) * taudot_2 * p1 + h11_ddot(tau) * taudot_2 * (t1 - t0) * m1;
}

template <typename VectorType>
inline double SplineInterpolator<VectorType>::getStartTime () {
	if (t_values.size() == 0) {
		return 0.;
	}

	return t_values[0];
}

template <typename VectorType>
inline double SplineInterpolator<VectorType>::getEndTime () {
	if (t_values.size() == 0) {
		return 0.;
	}

	return t_values[t_values.size() - 1];
}

/* SPLINEINTERPOLATOR_H */
#endif
