#ifndef SYS_DYNAM_HPP
#define SYS_DYNAM_HPP

namespace sac {

  //[ The rhs of x' = f(x) defined as a class
  // USER SPECIFIED:
  class sys_dynam {
    b_control & u_;
    state_type u_curr_;
  
  public:
    sys_dynam( b_control & u ) : u_(u) , u_curr_(ulen) { }
  
    void operator() (const state_type &x, state_type &dxdt, const double t)
    {
      u_(t, x, u_curr_);


	  
	/*u1 = u_curr_[0];
	u2 = u_curr_[1];
	u3 = u_curr_[2];
	u4 = u_curr_[3];;
	x = x[0];
	y = x[1]; 
	z = x[2];
	phi = x[3];
	theta = x[4]; 
	psi = x[5];
	xdot = x[6];
	ydot = x[7]; 
	zdot = x[8]; 
	phidot = x[9]; 
	thetadot = x[10]; 
	psidot = x[11];

	k = k_quad;*/

	// System dynamics
	dxdt[0] = x[6];
	dxdt[1] = x[7];
	dxdt[2] = x[8];
	dxdt[3] = x[9];
	dxdt[4] = x[10];
	dxdt[5] = x[11];
	dxdt[6] = (k_quad*(sin(x[3])*sin(x[5])+cos(x[3])*cos(x[5])*sin(x[4]))*(u_curr_[0]+u_curr_[1]+u_curr_[2]+u_curr_[3]))/m;
	dxdt[7] = -(k_quad*(cos(x[5])*sin(x[3])-cos(x[3])*sin(x[4])*sin(x[5]))*(u_curr_[0]+u_curr_[1]+u_curr_[2]+u_curr_[3]))/m;
	dxdt[8] = -g+(k_quad*cos(x[3])*cos(x[4])*(u_curr_[0]+u_curr_[1]+u_curr_[2]+u_curr_[3]))/m;
	dxdt[9] = -(sin(x[4])*(Iyy-Iyy*pow(sin(x[3]),2.0)+Izz*pow(sin(x[3]),2.0))*(-b*u_curr_[0]+b*u_curr_[1]-b*u_curr_[2]+b*u_curr_[3]-Ixx*sin(x[4]*2.0)*x[10]*x[11]*(1.0/2.0)+Iyy*sin(x[4]*2.0)*x[10]*x[11]*(1.0/2.0)+Ixx*cos(x[4])*x[9]*x[10]*(1.0/2.0)+Iyy*cos(x[4])*x[9]*x[10]*(1.0/2.0)-Izz*cos(x[4])*x[9]*x[10]*(1.0/2.0)-Iyy*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[10]+Izz*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[10]+Iyy*cos(x[3])*sin(x[3])*sin(x[4])*(x[10]*x[10])*(1.0/2.0)-Izz*cos(x[3])*sin(x[3])*sin(x[4])*(x[10]*x[10])*(1.0/2.0)-Iyy*cos(x[3])*pow(cos(x[4]),2.0)*sin(x[3])*x[9]*x[11]-Iyy*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]+Izz*cos(x[3])*pow(cos(x[4]),2.0)*sin(x[3])*x[9]*x[11]+Izz*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]))/(Iyy*Izz*(pow(sin(x[4]),2.0)-1.0))-((-k_quad*l*u_curr_[1]+k_quad*l*u_curr_[3]+Ixx*cos(x[4])*x[10]*x[11]*(1.0/2.0))*(Iyy*Izz+Ixx*Iyy*pow(sin(x[4]),2.0)-Iyy*Izz*pow(sin(x[4]),2.0)-Ixx*Iyy*pow(sin(x[3]),2.0)*pow(sin(x[4]),2.0)+Ixx*Izz*pow(sin(x[3]),2.0)*pow(sin(x[4]),2.0)))/(Ixx*Iyy*Izz*(pow(sin(x[4]),2.0)-1.0))+(sin(x[4])*sin(x[3]*2.0)*(Iyy*(1.0/2.0)-Izz*(1.0/2.0))*(k_quad*l*u_curr_[0]-k_quad*l*u_curr_[2]-Iyy*sin(x[3]*2.0)*x[9]*x[10]*(1.0/2.0)+Izz*sin(x[3]*2.0)*x[9]*x[10]*(1.0/2.0)-Iyy*cos(x[4])*x[9]*x[11]*(1.0/2.0)+Izz*cos(x[4])*x[9]*x[11]*(1.0/2.0)+Iyy*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[11]-Izz*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[11]-Iyy*cos(x[3])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)+Izz*cos(x[3])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)))/(Iyy*Izz*cos(x[4]));
	dxdt[10] = ((Iyy*Iyy)*pow(cos(x[3]),2.0)*sin(x[4])*(x[10]*x[10])*(-1.0/2.0)+(Iyy*Iyy)*pow(cos(x[3]),4.0)*sin(x[4])*(x[10]*x[10])*(1.0/2.0)-(Izz*Izz)*pow(cos(x[3]),2.0)*sin(x[4])*(x[10]*x[10])*(1.0/2.0)+(Izz*Izz)*pow(cos(x[3]),4.0)*sin(x[4])*(x[10]*x[10])*(1.0/2.0)+(Iyy*Iyy)*pow(cos(x[4]),2.0)*x[9]*x[11]*(1.0/2.0)+Iyy*b*sin(x[3]*2.0)*u_curr_[0]*(1.0/2.0)-Iyy*b*sin(x[3]*2.0)*u_curr_[1]*(1.0/2.0)+Iyy*b*sin(x[3]*2.0)*u_curr_[2]*(1.0/2.0)-Iyy*b*sin(x[3]*2.0)*u_curr_[3]*(1.0/2.0)-Izz*b*sin(x[3]*2.0)*u_curr_[0]*(1.0/2.0)+Izz*b*sin(x[3]*2.0)*u_curr_[1]*(1.0/2.0)-Izz*b*sin(x[3]*2.0)*u_curr_[2]*(1.0/2.0)+Izz*b*sin(x[3]*2.0)*u_curr_[3]*(1.0/2.0)+Iyy*Izz*pow(cos(x[3]),2.0)*sin(x[4])*(x[10]*x[10])-Iyy*Izz*pow(cos(x[3]),4.0)*sin(x[4])*(x[10]*x[10])-Iyy*Izz*pow(cos(x[4]),2.0)*x[9]*x[11]*(1.0/2.0)-Iyy*k_quad*l*cos(x[4])*u_curr_[0]+Iyy*k_quad*l*cos(x[4])*u_curr_[2]-(Iyy*Iyy)*pow(cos(x[3]),2.0)*pow(cos(x[4]),2.0)*x[9]*x[11]*(1.0/2.0)+(Izz*Izz)*pow(cos(x[3]),2.0)*pow(cos(x[4]),2.0)*x[9]*x[11]*(1.0/2.0)+Iyy*k_quad*l*pow(cos(x[3]),2.0)*cos(x[4])*u_curr_[0]-Iyy*k_quad*l*pow(cos(x[3]),2.0)*cos(x[4])*u_curr_[2]-Izz*k_quad*l*pow(cos(x[3]),2.0)*cos(x[4])*u_curr_[0]+Izz*k_quad*l*pow(cos(x[3]),2.0)*cos(x[4])*u_curr_[2]+(Iyy*Iyy)*cos(x[3])*cos(x[4])*sin(x[3])*x[9]*x[10]*(1.0/2.0)-(Izz*Izz)*cos(x[3])*cos(x[4])*sin(x[3])*x[9]*x[10]*(1.0/2.0)+(Iyy*Iyy)*pow(cos(x[3]),3.0)*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)+(Izz*Izz)*pow(cos(x[3]),3.0)*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)+Iyy*k_quad*l*cos(x[3])*sin(x[3])*sin(x[4])*u_curr_[1]-Iyy*k_quad*l*cos(x[3])*sin(x[3])*sin(x[4])*u_curr_[3]-Izz*k_quad*l*cos(x[3])*sin(x[3])*sin(x[4])*u_curr_[1]+Izz*k_quad*l*cos(x[3])*sin(x[3])*sin(x[4])*u_curr_[3]-Ixx*Iyy*cos(x[3])*cos(x[4])*sin(x[3])*x[9]*x[10]*(1.0/2.0)+Ixx*Izz*cos(x[3])*cos(x[4])*sin(x[3])*x[9]*x[10]*(1.0/2.0)-(Iyy*Iyy)*cos(x[3])*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)-Iyy*Izz*pow(cos(x[3]),3.0)*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]+Ixx*Iyy*cos(x[3])*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)-Ixx*Izz*cos(x[3])*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0)+Iyy*Izz*cos(x[3])*cos(x[4])*sin(x[3])*sin(x[4])*x[10]*x[11]*(1.0/2.0))/(Iyy*Izz*cos(x[4]));
	dxdt[11] = (1.0/pow(cos(x[4]),2.0)*(Izz*b*u_curr_[0]*-2.0+Izz*b*u_curr_[1]*2.0-Izz*b*u_curr_[2]*2.0+Izz*b*u_curr_[3]*2.0-Iyy*b*pow(cos(x[3]),2.0)*u_curr_[0]*2.0+Iyy*b*pow(cos(x[3]),2.0)*u_curr_[1]*2.0-Iyy*b*pow(cos(x[3]),2.0)*u_curr_[2]*2.0+Iyy*b*pow(cos(x[3]),2.0)*u_curr_[3]*2.0+Izz*b*pow(cos(x[3]),2.0)*u_curr_[0]*2.0-Izz*b*pow(cos(x[3]),2.0)*u_curr_[1]*2.0+Izz*b*pow(cos(x[3]),2.0)*u_curr_[2]*2.0-Izz*b*pow(cos(x[3]),2.0)*u_curr_[3]*2.0-(Izz*Izz)*cos(x[4])*x[9]*x[10]-(Izz*Izz)*cos(x[3])*sin(x[3])*sin(x[4])*(x[10]*x[10])-Ixx*Izz*sin(x[4]*2.0)*x[10]*x[11]*(1.0/2.0)+Iyy*Izz*sin(x[4]*2.0)*x[10]*x[11]+(Iyy*Iyy)*pow(cos(x[3]),3.0)*sin(x[3])*sin(x[4])*(x[10]*x[10])+(Izz*Izz)*pow(cos(x[3]),3.0)*sin(x[3])*sin(x[4])*(x[10]*x[10])-(Iyy*Iyy)*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[10]+(Izz*Izz)*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[10]-Izz*k_quad*l*sin(x[4])*u_curr_[1]*2.0+Izz*k_quad*l*sin(x[4])*u_curr_[3]*2.0+Ixx*Izz*cos(x[4])*x[9]*x[10]+Iyy*Izz*cos(x[4])*x[9]*x[10]-Iyy*k_quad*l*pow(cos(x[3]),2.0)*sin(x[4])*u_curr_[1]*2.0+Iyy*k_quad*l*pow(cos(x[3]),2.0)*sin(x[4])*u_curr_[3]*2.0+Izz*k_quad*l*pow(cos(x[3]),2.0)*sin(x[4])*u_curr_[1]*2.0-Izz*k_quad*l*pow(cos(x[3]),2.0)*sin(x[4])*u_curr_[3]*2.0-Iyy*Izz*pow(cos(x[3]),3.0)*sin(x[3])*sin(x[4])*(x[10]*x[10])*2.0+Ixx*Iyy*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[10]-Ixx*Izz*pow(cos(x[3]),2.0)*cos(x[4])*x[9]*x[10]-(Iyy*Iyy)*cos(x[3])*pow(cos(x[4]),2.0)*sin(x[3])*x[9]*x[11]+(Iyy*Iyy)*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]-(Iyy*Iyy)*pow(cos(x[3]),4.0)*cos(x[4])*sin(x[4])*x[10]*x[11]+(Izz*Izz)*cos(x[3])*pow(cos(x[4]),2.0)*sin(x[3])*x[9]*x[11]+(Izz*Izz)*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]-(Izz*Izz)*pow(cos(x[3]),4.0)*cos(x[4])*sin(x[4])*x[10]*x[11]+Iyy*Izz*cos(x[3])*sin(x[3])*sin(x[4])*(x[10]*x[10])+Iyy*k_quad*l*cos(x[3])*cos(x[4])*sin(x[3])*u_curr_[0]*2.0-Iyy*k_quad*l*cos(x[3])*cos(x[4])*sin(x[3])*u_curr_[2]*2.0-Izz*k_quad*l*cos(x[3])*cos(x[4])*sin(x[3])*u_curr_[0]*2.0+Izz*k_quad*l*cos(x[3])*cos(x[4])*sin(x[3])*u_curr_[2]*2.0-Ixx*Iyy*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]+Ixx*Izz*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]-Iyy*Izz*pow(cos(x[3]),2.0)*cos(x[4])*sin(x[4])*x[10]*x[11]*2.0+Iyy*Izz*pow(cos(x[3]),4.0)*cos(x[4])*sin(x[4])*x[10]*x[11]*2.0)*(1.0/2.0))/(Iyy*Izz);

   }
  };
  //]

}

#endif  // SYS_DYNAM_HPP
