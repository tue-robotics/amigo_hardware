/**************************************************************************
 *                                                                        *
 *   B. Willems                                                           *
 *   Eindhoven University of Technology                                   *
 *   2011                                                                 *
 *                                                                        *
 **************************************************************************/

#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include "PeraControllers_Backup.hpp"

#define PI 3.1415926535897932384626433
#define d3 0.32
#define d5 0.28
#define a7 0.2
#define m3 2.9
#define m5 0.8
#define m7 0.2
#define c3y 0.213
#define c5y 0.140
#define c7x 0.150

using namespace RTT;
using namespace PERA;

	PERACONTROLLERS::PERACONTROLLERS(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addEventPort("in", errPort);
			
			/// Outports
			addPort("out",tmpCmdEffortPort);
			
			// Loading properties
						/// Controller
			addProperty( "q1Controller", q1ConVal );
			addProperty( "q2Controller", q2ConVal );
			addProperty( "q3Controller", q3ConVal );
			addProperty( "q4Controller", q4ConVal );
			addProperty( "q5Controller", q5ConVal );
			addProperty( "q6Controller", q6ConVal );
			addProperty( "q7Controller", q7ConVal );
			addProperty( "q8Controller", q8ConVal );
			
			/// Coulomb friction feedforward
			addProperty( "kfcFeedForward", kfcValues );
			
			/// Gravity compensation torque factors
			addProperty( "gravCompTorqueFactors", gCTF );
			
			/// Controlloop sampletime
			addProperty( "sampletime", Ts );
			
			// Resize controller related vectors
			nrControllers = 8;
			mControllers.resize(nrControllers);

			// Gravity Compensation vectors/matrices
			a.setZero(1,7);
			d.setZero(1,7);
			alpha.setZero(1,7);
			mlist.setZero(1,7);
			grav.setZero(3,1);
			q.setZero(7,1);
			Istore.setZero(3,21);
			gravComp.setZero(7,1);
			coglist.setZero(3,7);

	  }


	PERACONTROLLERS::~PERACONTROLLERS(){}

	bool PERACONTROLLERS::configureHook(){

		// Initialize previous loop time
		prevTime = 0;

		// Gravity compensation
		grav << -9.81,0,0;
		a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, a7;
		d << 0,0,-d3,0,-d5, 0, 0;
		alpha << -PI/2, -PI/2, PI/2, -PI/2, PI/2, -PI/2, 0.0;

		// Centers Of Gravity of the 3 links (upper and lowerarm and hand)
		coglist(1,2) = c3y;
		coglist(1,4) = c5y;
		coglist(0,6) = -c7x; // minussign

		// Masses of the 3 links (upper and lowerarm and hand)
		mlist(0,2) = m3;
		mlist(0,4) = m5;
		mlist(0,6) = m7;

		// Load parameters controller parameters
		rv = mControllers[0].SetParameters( q1ConVal[0], q1ConVal[1], q1ConVal[2], q1ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q1 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q1 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q1 P="<<q1ConVal[0]<<", I="<<q1ConVal[1]<<", D="<<q1ConVal[2]<<endlog();
		
		rv = mControllers[1].SetParameters( q2ConVal[0], q2ConVal[1], q2ConVal[2], q2ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q2 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q2 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q2 P="<<q2ConVal[0]<<", I="<<q2ConVal[1]<<", D="<<q2ConVal[2]<<endlog();
		
		rv = mControllers[2].SetParameters( q3ConVal[0], q3ConVal[1], q3ConVal[2], q3ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q3 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q3 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q3 P="<<q3ConVal[0]<<", I="<<q3ConVal[1]<<", D="<<q3ConVal[2]<<endlog();

		rv = mControllers[3].SetParameters( q4ConVal[0], q4ConVal[1], q4ConVal[2], q4ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q4 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q4 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q4 P="<<q4ConVal[0]<<", I="<<q4ConVal[1]<<", D="<<q4ConVal[2]<<endlog();
		
		rv = mControllers[4].SetParameters( q5ConVal[0], q5ConVal[1], q5ConVal[2], q5ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q5 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q5 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q5 P="<<q5ConVal[0]<<", I="<<q5ConVal[1]<<", D="<<q5ConVal[2]<<endlog();
		
		rv = mControllers[5].SetParameters( q6ConVal[0], q6ConVal[1], q6ConVal[2], q6ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q6 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q6 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q6 P="<<q6ConVal[0]<<", I="<<q6ConVal[1]<<", D="<<q6ConVal[2]<<endlog();
		
		rv = mControllers[6].SetParameters( q7ConVal[0], q7ConVal[1], q7ConVal[2], q7ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q7 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q7 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q7 P="<<q7ConVal[0]<<", I="<<q7ConVal[1]<<", D="<<q7ConVal[2]<<endlog();
		
		rv = mControllers[7].SetParameters( q8ConVal[0], q8ConVal[1], q8ConVal[2], q8ConVal[3], Ts);
		if(rv==4){
			log(Warning)<<"Controller q8 set to zero"<<endlog();
		}
		else if(rv==-1){
			log(Error)<<"Controller q8 sample time mismatch"<<endlog();
			return false;
		}
		log(Info)<<"Controller q8 P="<<q8ConVal[0]<<", I="<<q8ConVal[1]<<", D="<<q8ConVal[2]<<endlog();
		
		// Actuator factor (provided by Philips)
		Kact=1000;

		return true;

	}

	bool PERACONTROLLERS::startHook(){

		log(Info)<<"StartHook complete "<<endlog();

		initTime = os::TimeService::Instance()->getNSecs();

		return true;

	  }

	void PERACONTROLLERS::updateHook(){
		
		/*timeNow = (os::TimeService::Instance()->getNSecs())-initTime;
		deltaTime = timeNow - prevTime;
		prevTime = timeNow;
		dt = std::min<double>(fabs(deltaTime/1E9),Ts);*/
		
		doubles err;
		doubles tmpCmdEffort;
		doubles jointAngles;
		jointAngles.resize(nrControllers);
		err.resize(nrControllers);
		tmpCmdEffort.resize(nrControllers);
		
		errPort.read( err );	
		
		for(unsigned int i=0;i<nrControllers;i++){

			// Compute controller i effort
			tmpCmdEffort[i]=mControllers[i].Evaluate(err[i])*Kact;
			
		}
		
		//log(Debug)<<"error q1 = "<<err[0]<<endlog();
		//log(Debug)<<"error q2 = "<<err[1]<<endlog();
		
		//log(Debug)<<"tmpCmdEffort q1<< = "<<tmpCmdEffort[0]<<endlog();
		//log(Debug)<<"tmpCmdEffort q2<< = "<<tmpCmdEffort[1]<<endlog();
		

		/*/Feedforward part

		///Coulomb friction Fc
		if (ref_q1_right.vel > 0.0){
			tmpCmdEffort[0]=tmpCmdEffort[0]+kfcValues[0];
		}
		else if (ref_q1_right.vel < 0.0){
			tmpCmdEffort[0]=tmpCmdEffort[0]-kfcValues[0];
		}
		if (ref_q2_right.vel > 0.0){
			tmpCmdEffort[1]=tmpCmdEffort[1]-kfcValues[1];
		}
		else if (ref_q2_right.vel < 0.0){
			tmpCmdEffort[1]=tmpCmdEffort[1]+kfcValues[1];
		}
		if (ref_q4_right.vel > 0.0){
			tmpCmdEffort[3]=tmpCmdEffort[3]+kfcValues[3];
		}
		else if (ref_q4_right.vel < 0.0){
			tmpCmdEffort[3]=tmpCmdEffort[3]-kfcValues[3];
		}
		if (ref_q8_right.vel > 0.0){
			tmpCmdEffort[7]=tmpCmdEffort[7]+kfcValues[7];
		}
		else if (ref_q8_right.vel < 0.0){
			tmpCmdEffort[7]=tmpCmdEffort[7]-kfcValues[7];
		}*/
		
		/// Gravity compensation
		q << -jointAngles[1]*(PI/180), -jointAngles[0]*(PI/180)-(PI/2), jointAngles[6]*(PI/180), jointAngles[2]*(PI/180), -jointAngles[3]*(PI/180), jointAngles[4]*(PI/180)-(PI/2), jointAngles[5]*(PI/180);

		gravComp = ComputeGravity(a,d,alpha,coglist,mlist,Istore,q,grav);

		tmpCmdEffort[0]=tmpCmdEffort[0]+gravComp(0,0)*gCTF[0];
		tmpCmdEffort[1]=tmpCmdEffort[1]-gravComp(1,0)*gCTF[1];
		tmpCmdEffort[2]=tmpCmdEffort[2]+gravComp(2,0)*gCTF[2];
		tmpCmdEffort[3]=tmpCmdEffort[3]-gravComp(3,0)*gCTF[3];
		tmpCmdEffort[4]=tmpCmdEffort[4]-gravComp(4,0)*gCTF[4];
		tmpCmdEffort[5]=tmpCmdEffort[5]-gravComp(5,0)*gCTF[5];
		tmpCmdEffort[6]=tmpCmdEffort[6]-gravComp(6,0)*gCTF[6];//*/
		
		tmpCmdEffortPort.write(tmpCmdEffort);

	}

	void PERACONTROLLERS::stopHook(){


	}
	
	// Matrix computing the rotation matrix from link i-1 to link i, using the DH-parameters
	Eigen::Matrix3d ComputeRotationMatrix(double d,double alpha,double q){
		Eigen::Matrix3d R;
		R = Eigen::Matrix3d::Zero(3,3);
		R(0,0) = cos(q);
		R(0,1) = -sin(q)*cos(alpha);
		R(0,2) = sin(q)*sin(alpha);
		R(1,0) = sin(q);
		R(1,1) = cos(q)*cos(alpha);
		R(1,2) = -cos(q)*sin(alpha);
		R(2,0) = 0;
		R(2,1) = sin(alpha);
		R(2,2) = cos(alpha);
		return R;	
	}

	// Matrix computing the rotation matrix from Euler angles
	Eigen::Matrix3d eul2rot(double phi, double theta, double psi){
		Eigen::Matrix3d R;
		R = Eigen::Matrix3d::Zero(3,3);
		R(0,0) = cos(phi)*cos(theta)*cos(psi) - sin(phi)*sin(psi);
		R(0,1) = -cos(phi)*cos(theta)*sin(psi) - sin(phi)*cos(psi);
		R(0,2) = cos(phi)*sin(theta);
		R(1,0) = sin(phi)*cos(theta)*cos(psi) + cos(phi)*sin(psi);
		R(1,1) = -sin(phi)*cos(theta)*sin(psi) + cos(phi)*cos(psi);
		R(1,2) = sin(phi)*sin(theta);
		R(2,0) = -sin(theta)*cos(psi);
		R(2,1) = sin(theta)*sin(psi);
		R(2,2) = cos(theta);
		return R;	
	}

	// Compute Recursive Newton Euler (only for revolute joints!!)
	Eigen::MatrixXd rne(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd Q,Eigen::MatrixXd Qd,Eigen::MatrixXd Qdd,Eigen::MatrixXd grav, Eigen::MatrixXd Fext){
		// Declare matrices, vectors and scalars
		Eigen::MatrixXd R(3,3);
		Eigen::Vector3d pstar;
		Eigen::Vector3d r;
		Eigen::Vector3d wd;
		Eigen::Vector3d w;
		Eigen::Vector3d Iiw;
		Eigen::Vector3d v;
		Eigen::Vector3d vd;
		Eigen::Vector3d vhat;
		Eigen::Vector3d z0;
		Eigen::Vector3d f;
		Eigen::Vector3d nn;
		Eigen::Vector3d Rtranpos_pstar;
		Eigen::Vector3d F;
		Eigen::Vector3d N;  
		Eigen::MatrixXd Ii(3,3);
		Eigen::Vector3d Fmi;   
		int n,np;
			  
		// Rotation axis
		z0 << 0,0,1;    
		// Determine number of links
		n = mlist.cols();      
		// Determine if matrix or vector has to be computed
		np = Q.rows();
		// Define size of tau
		Eigen::MatrixXd tau(np,n);
		tau = Eigen::MatrixXd::Zero(np,n);
		
		// Declare dimension matrices
		Eigen::MatrixXd Fm(3,n);
		Eigen::MatrixXd Nm(3,n);
		Eigen::MatrixXd Rs(3,3*n);
		Eigen::MatrixXd pstarm(3,n);
		Eigen::MatrixXd q(n,1);
		Eigen::MatrixXd qd(n,1);
		Eigen::MatrixXd qdd(n,1);
		Eigen::MatrixXd Qtransposed(Q.cols(),Q.rows());
		Eigen::MatrixXd Qdtransposed(Qd.cols(),Qd.rows());
		Eigen::MatrixXd Qddtransposed(Qdd.cols(),Qdd.rows());
		f = Eigen::Vector3d::Zero();
		nn = Eigen::Vector3d::Zero();
		Fmi = Eigen::Vector3d::Zero();
		vhat = Eigen::Vector3d::Zero();
	   
		Qtransposed = Q.transpose();
		Qdtransposed = Qd.transpose();
		Qddtransposed = Qdd.transpose();
	   
		int i;
		for(i=1; i< np+1; i++){
		   // Collect q, qd, qdd
		   q = Qtransposed.block(0,i-1,n,1);
		   qd = Qdtransposed.block(0,i-1,n,1);
		   qdd = Qddtransposed.block(0,i-1,n,1);  
	  
		   w = Eigen::Vector3d::Zero();
		   Iiw = Eigen::Vector3d::Zero();
		   wd = Eigen::Vector3d::Zero();
		   v = Eigen::Vector3d::Zero();
		   vd = grav;
		   Fm = Eigen::MatrixXd::Zero(3,n);
		   Nm = Eigen::MatrixXd::Zero(3,n);
		   pstarm = Eigen::MatrixXd::Zero(3,n);
		   Rs = Eigen::MatrixXd::Zero(3,3*n);
		   
		   // Compute link rotation matrices
		   int j;
		   for(j=1; j<n+1; j++){
			 Rs.block(0,3*j-3,3,3) = ComputeRotationMatrix(d(j-1),alpha(j-1),q(j-1));  
			 pstarm.block(0,j-1,3,1) << a(j-1), d(j-1)*sin(alpha(j-1)), d(j-1)*cos(alpha(j-1));
		   }

		   //The forward recursion
		   int jj;
		   for(jj=1; jj<n+1; jj++){
			   R = Rs.block(0,3*jj-3,3,3).transpose();
			   pstar = pstarm.block(0,jj-1,3,1);
			   r = coglist.block(0,jj-1,3,1);
			   wd = R*(wd + z0*qdd(jj-1) + w.cross(z0*qd(jj-1)));
			   w = R*(w + z0*qd(jj-1));
			   vd = wd.cross(pstar) + w.cross(w.cross(pstar)) + R*vd;
			   vhat = wd.cross(r) + w.cross(w.cross(r)) + vd;
			   F = mlist(0,jj-1)*vhat;
			   Ii = Istore.block(0,3*jj-3,3,3);
			   Iiw = Ii*w;
			   N = Ii*wd + w.cross(Iiw);
			   Fm.block(0,jj-1,3,1) = F;
			   Nm.block(0,jj-1,3,1) = N;
		   }
	   
		   //The backward recursion
		   f = Fext.block(0,0,3,1);
		   nn = Fext.block(3,0,3,1);       
		   int k;
		   for(k=n; k>0; k--){
			   pstar = pstarm.block(0,k-1,3,1);
			   if(k==n){
				   R = Eigen::MatrixXd::Identity(3,3);
			   }
			   else{
				   R   = Rs.block(0,3*k,3,3);
			   }
			   r = coglist.block(0,k-1,3,1);
			   Rtranpos_pstar = R.transpose()*pstar;
			   Fmi = Fm.block(0,k-1,3,1);
			   nn = R*(nn + Rtranpos_pstar.cross(f)) + (pstar+r).cross(Fmi) + Nm.block(0,k-1,3,1);
			   f = R*f + Fm.block(0,k-1,3,1);
			   R = Rs.block(0,3*k-3,3,3);
			   tau(i-1,k-1) = nn.dot(R.transpose()*z0);
		   }
		 }
			
		 return tau;	
	}

	Eigen::MatrixXd ComputeGravity(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd grav){
		// Declare matrices and vectors 
		Eigen::MatrixXd Gravity(1,mlist.cols());
		Eigen::MatrixXd no_Fext(6,1);
		
		// No external force to compute gravitational forces
		no_Fext = Eigen::MatrixXd::Zero(6,1);
		
		// Compute gravitational forces  
		Gravity = rne(a,d,alpha,coglist,mlist,Istore,q.transpose(),0*q.transpose(),0*q.transpose(),grav,no_Fext);
		
		return Gravity.transpose();	
	}
	
	Controller::Controller()
      : xample(0.004)
      , mKp(0)
      , mKi(0)
      , mKd(0)
      , mILim(0.0)
      , mErr(0.0)
      , mSumI(0.0)
   {
   }

   int Controller::SetParameters( double kp, double ki, double kd, double ilim, double tS )
   {
	   
	  int rv =0;
	  
	  if(mKp==kp){
		  rv++;
	  }
	  else{
		  mKp = kp;
	  }
	  if(mKi==ki){
		  rv++;
	  }
	  else{
		  mKi = ki;
	  }
	  if(mKd==kd){
		  rv++;
	  }
	  else{
		  mKd = kd;
	  }
	  if(mILim==ilim){
		  rv++;
	  }
	  else{
		  mILim = ilim;
	  }
	  if(xample!=tS){
		  rv=-1;
	  }
	  else{
		  xample = tS;
	  }
	  
	  return rv;
   }

   double Controller::Evaluate( double err )
   {
      double mOutP = mKp * err;
      double mOutD = mKp * mKd * (err-mErr)/xample;
	  mSumI += mKi * mKp * xample * err;
      
      if ( mSumI > mILim ) 
      {
         mSumI = mILim;
      }
      else if ( mSumI < -mILim )
      {
         mSumI = -mILim;
      }
      
      double mOut = mOutP + mSumI + mOutD;

      mErr = err;
      
      return mOut;
   }

ORO_CREATE_COMPONENT(PERACONTROLLERS)

