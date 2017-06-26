#ifndef INC_COST_HPP
#define INC_COST_HPP

namespace sac {
  //! \warning Class MUST BE MODIFIED BY USER to accomodate NON-QUADRATIC running cost
  /*
    General incremental trajectory tracking cost, \f$l(x)\f$, for integration
    \f$J_1 = \int_{t_0}^{t_f} l(x) \, dt + m(x(t_f))\f$.  Keeps references to 
    state interpolator so that changes state trajectory are automatically 
    accounted for.
  */
  class inc_cost {
    Eigen::Matrix< double, xlen, 1 > mx_;
    size_t indx_;
    const int wrap_size_;//number of states to be wrapped

  
  public:
    state_intp & m_x_intp; // store current state
    state_type m_x;
    Eigen::Matrix< double, xlen, 1 > m_mxdes;
  
    //! \todo: make inputs const ref type
    /*!
      Initializes references to user maintained trajectory object and a pointer
      to the desired trajectory.
      \param[in] x_intp User maintained state interpolation object
    */
    inc_cost( state_intp & x_intp): 
		 mx_(Eigen::Matrix< double,xlen,1 >::Zero(xlen,1)), 
 		 m_x_intp( x_intp ), m_x( xlen ),
		 m_mxdes(Eigen::Matrix< double,xlen,1 >::Zero(xlen,1)),
		 wrap_size_(sizeof(x_wrap)/sizeof(x_wrap[0])) { }
  
    /*!
      Computes the value of incremental trajectory tracking cost, \f$l(x)\f$.
      \param[in] J The current cost
      \param[out] dJdt The previous incremental cost
      \param[in] t The current time
    */
    void operator() (const state_type &J, state_type &dJdt, const double t)
    {
      m_x_intp(t, m_x); // store the current state in x
	for (indx_ = 0; indx_ < wrap_size_; ++indx_ ) {AngleWrap( m_x[x_wrap[indx_]] );} // Angle wrapping (if any)
      State2Mat( m_x, mx_ ); // convert state to matrix form
      //
      get_DesTraj( t, m_mxdes ); // Store desired trajectory point in m_mxdes
      //
      dJdt[0] = l_of_x(mx_, m_mxdes);
    }


    /*!Alternative implementation of the () operator. This one does not
	interpolate the state (adaptive step is still faster though in general).
      calculates a vector of l_of_x values using the time and state vector from the integration.
      \param[in] l_of_x_vec - Vector to be populated with incremental cost values
      \param[in] t_vec - Pointer to time vector from state integration
    */
   /* void operator() (state_type & l_of_x_vec, const state_type * t_vec)
    {
		std::vector< state_type > * st_vec = m_x_intp.state_vec( );//get reference to integrated state vector
		size_t index, i;
		size_t length = t_vec->size();
		for(index=0; index<length-1; ++index)
		{
			for(i=0; i<xlen; ++i){
				m_x[i] = (*st_vec)[index][i];//do not change the integrated state - just copy whatever u need
			}
			for (indx_ = 0; indx_ < wrap_size_; ++indx_ ) {AngleWrap( m_x[x_wrap[indx_]] );} // Angle wrapping (if any)
			State2Mat( m_x, mx_ ); // convert state to matrix form
			//
			get_DesTraj( (*t_vec)[index], m_mxdes ); // Store desired trajectory point in m_mxdes
			//
			l_of_x_vec[index] = l_of_x(mx_, m_mxdes);
		}
    }*/

    /*!
      Incremental cost formula
      \param[in] J The current state and current desired state
      \param[out] The incremental cost
    */
    inline double l_of_x(const Eigen::Matrix< double, xlen, 1 > & x_curr, const Eigen::Matrix< double, xlen, 1 > & xd_curr) { 
	return( ( (x_curr-xd_curr).transpose() * Q * (x_curr-xd_curr) ) / 2.0 )[0];
    }


    /*!
      Computes the value of the derivative of the incremental cost, 
      \f$D_x l(x)\f$.
      \param[in] t The current time
      \param[in] mx The current state
      \param[out] dldx The derivative \f$D_x l(x)\f$.
    */
    inline void dx( const double t, const Eigen::Matrix< double, xlen, 1 > &mx,
		    Eigen::Matrix< double, 1, xlen > &dldx ) { 
      get_DesTraj( t, m_mxdes ); // Store desired trajectory point in m_mxdes
      //
      dldx = (mx-m_mxdes).transpose()*Q;
    }

    /*!
      Returns the initial time for integration, \f$t_0\f$
      \return Initial integration time \f$t_0\f$
    */
    double begin( ) { return m_x_intp.begin( ); }

    /*!
      Returns the final time for integration, \f$t_f\f$
      \return Final integration time \f$t_f\f$
    */
    double end( ) { return m_x_intp.end( ); }
  };

}

#endif  // INC_COST_HPP
