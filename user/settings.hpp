 /* Copyright (C) 2017 Emmanouil Tzorakoleftherakis
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SETTINGS_HPP
#define SETTINGS_HPP

/* Define Constants */
#define PI (3.14159)

namespace sac {

	/*********************************************/
	/* The type of container used to hold the state vector */
	typedef std::vector< double > state_type;


	/*********************************************/
	/* State and control vector length*/
	const size_t xlen = 12, ulen = 4;

	/*********************************************/
	/* Function prototypes */
	inline void initialize_cost_weights();
	inline void u_nominal( const double , state_type & );
	inline void get_DesTraj( const double , Eigen::Matrix< double, xlen, 1 > &  );


	/*********************************************/
	/* Parameters of dynamics*/
	const double Ixx = 1.2;
	const double Iyy = 1.2;
	const double Izz = 2.3;
	const double k_quad = 1;
	const double l = 0.25;
	const double m = 2;
	const double b = 0.2;
	const double g = 9.81;


	/*********************************************
	/* Initial condition */
	const double x_init[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


	/*********************************************
	/* Indexes of states to be angle wrapped - leave brace blank if no states are to be wrapped */
	const int x_wrap[] = {};

	/*********************************************/
	/* Time Parameters */
	const double t_init = 0.0; // initial time
	const double t_final = 25.0; // final time

	const double T = 2.0; // prediction horizon
	const double ts = 0.05; //sampling time
	//const double calc_tm = 0; // I don't use that


	/*********************************************/
	/* Control saturation */
	const double usat[ulen][2] = { {12, 0}, {12, 0}, {12, 0}, {12, 0} };

	
	/*********************************************/
	/* Aggressiveness */
	const double lam = -50000; 


	/*********************************************/
	/*Search for application time? */
	const bool u2Search = true;
	
	/*********************************************/
	/*Write actions to file? */
	const bool save_actions_to_file = true;


	/*********************************************/
	/* Backtracking parameters */
	const double maxdt = .2;//initial control duration
	const double factor = 0.5;//how much the duration decreases at each iteration
	const int max_iters = 5;//number of backtracking iterations (# of times the duration is decreased)


	/*********************************************/
	/* Backtracking parameters for physical hardware
	Basically we are only allowing that are multiples of 2*ctr_com_freq*/
	/*const double ctr_com_freq = 50.0; //how fast the control can be updated in the physical hardware (Hz)
	const double max_iters = 4;//number of backtracking iterations (# of times the duration is decreased)

	//Do not change
	const double min_duration = 2.0/ctr_com_freq;//minimum possible duration of action
	const double maxdt = min_duration + max_iters*min_duration;//initial control duration
	//! No "factor" here; must change backtracking such that*/


	/*********************************************/
	/* Numerical tolerance */
	const double epsilon = 1e-5;


	/*********************************************/
	/*Initialize cost weight matrices (has to be within a function due to <<)*/
	Eigen::Matrix< double, xlen, xlen > Q = Eigen::Matrix< double, xlen, xlen >::Zero();
	Eigen::Matrix< double, ulen, ulen > R = Eigen::Matrix< double, ulen, ulen >::Identity(ulen, ulen);
	Eigen::Matrix< double, xlen, xlen > P = Eigen::Matrix< double, xlen, xlen >::Zero();

	inline void initialize_cost_weights() {
		//Q matrix
		Q(0,0) = 35;
		Q(1,1) = 35;    
		Q(2,2) = 35;
		Q(3,3) = 0.5;
		Q(4,4) = 0.5;
		Q(5,5) = 15;

		//P matrix
		/*P(0,0) = 50;
		P(2,2) = 100;*/

		//R matrix         
		R = 0.1*R;    		
	}


	/*********************************************/
	/* Define unominal */
	inline void u_nominal( const double t, const state_type & x, state_type & u_nom_t ) {
		for ( size_t i=0; i<ulen; ++i ) { u_nom_t[i] = m*g/(4.0*k_quad); }//Calculate unominal at t
	}


	/*********************************************/
	/* Desired trajectory */
	inline void get_DesTraj( const double t, Eigen::Matrix< double, xlen, 1 > &m_mxdes ) {
		m_mxdes << 6.0*sin(t/3.0), -6.0*sin(t/3.0)*cos(t/3.0), 6.0*cos(t/3.0), 0, 0, 0, 0, 0, 0, 0, 0, 0;
		//m_mxdes << 1.0, 1.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
	}

}

#endif  // SETTINGS_HPP
