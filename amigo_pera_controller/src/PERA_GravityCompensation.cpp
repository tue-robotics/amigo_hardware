/*! 
 * \author Bas Willems
 * \date May, 2011
 * \version 1.0 
 */

//TODO: make general component -> outputs for given DH-parameters

#include <rtt/TaskContext.hpp>
#include <ocl/Component.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Port.hpp>
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <Eigen/Eigen>
#include "PERA_GravityCompensation.hpp"

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

	GravityCompensation::GravityCompensation(const std::string& name)
        : TaskContext(name, PreOperational)

		{
			// Creating the ports
			
			/// Inports
			addEventPort("in", jointAnglesPort);
			
			/// Outports
			addPort("out",gravCompPort);
			
			// Resize controller related vectors
			nrJoints = 8;
			nrCompensatedJoints = 7;
			
			// Gravity Compensation vectors/matrices
			a.setZero(1,7); // DH parameter a
			d.setZero(1,7); // DH parameter d
			alpha.setZero(1,7); // DH parameter alpha
			mlist.setZero(1,7); // link masses
			grav.setZero(3,1); // gravity vector
			q.setZero(7,1); // q vector
			Istore.setZero(3,21); // inertia matrix
			gravComp.setZero(7,1); // gravity compensation outcome
			coglist.setZero(3,7); // centers of gravity vector

	  }


	GravityCompensation::~GravityCompensation(){}

	bool GravityCompensation::configureHook(){

		// Gravity compensation
		grav << -9.81,0,0;
		a << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, a7;
		d << 0,0,-d3,0,-d5, 0, 0;
		alpha << -PI/2, -PI/2, PI/2, -PI/2, PI/2, -PI/2, 0.0;

		// Centers Of Gravity of the 3 links (upper and lowerarm and hand)
		coglist(1,2) = c3y;
		coglist(1,4) = c5y;
		coglist(0,6) = c7x; // minussign

		// Masses of the 3 links (upper and lowerarm and hand)
		mlist(0,2) = m3;
		mlist(0,4) = m5;
		mlist(0,6) = m7;

		return true;

	}

	bool GravityCompensation::startHook(){

		log(Info)<<"StartHook complete "<<endlog();
		
		// Check validity of Ports:
		if ( !jointAnglesPort.connected() ){
			log(Error)<<"One or more inputports not connected!"<<endlog();
			return false;
		}
		if ( !gravCompPort.connected() ){
			log(Warning)<<"Outputport not connected!"<<endlog();
		}

		return true;

	  }

	void GravityCompensation::updateHook(){
		
		doubles additionalTorques; // gravity compensation outcome
		doubles jointAngles; // measured actual joint angles
		
		jointAngles.resize(nrJoints);
		additionalTorques.resize(nrCompensatedJoints);
		
		jointAnglesPort.read(jointAngles);
		
		/// Gravity compensation
		q << -jointAngles[0], jointAngles[1]-(PI/2), jointAngles[2], jointAngles[3], jointAngles[4], jointAngles[5]-(PI/2), jointAngles[6];

		gravComp = ComputeGravity(a,d,alpha,coglist,mlist,Istore,q,grav);

		additionalTorques[0]=gravComp(0,0);
		additionalTorques[1]=gravComp(1,0);
		additionalTorques[2]=gravComp(2,0);
		additionalTorques[3]=gravComp(3,0);
		additionalTorques[4]=gravComp(4,0);
		additionalTorques[5]=gravComp(5,0);
		additionalTorques[6]=gravComp(6,0);
		
		gravCompPort.write(additionalTorques);

	}

	void GravityCompensation::stopHook(){


	}
	
	// Matrix computing the rotation matrix from link i-1 to link i, using the DH-parameters
	Eigen::Matrix3d GravityCompensation::ComputeRotationMatrix(double d,double alpha,double q){
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
	Eigen::Matrix3d GravityCompensation::eul2rot(double phi, double theta, double psi){
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
	Eigen::MatrixXd GravityCompensation::rne(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd Q,Eigen::MatrixXd Qd,Eigen::MatrixXd Qdd,Eigen::MatrixXd grav, Eigen::MatrixXd Fext){
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
		   Rs = Eigen::MatrixXd::Zero(3,3*n); // Rotation matrices
		   
		   // Compute link rotation matrices
		   int j;
		   for(j=1; j<n+1; j++){
			 Rs.block(0,3*j-3,3,3) = ComputeRotationMatrix(d(j-1),alpha(j-1),q(j-1)); // Fill Rs with the computed rotation matrics
			 pstarm.block(0,j-1,3,1) << a(j-1), d(j-1)*sin(alpha(j-1)), d(j-1)*cos(alpha(j-1));
		   }

		   //The forward recursion
		   int jj;
		   for(jj=1; jj<n+1; jj++){
			   R = Rs.block(0,3*jj-3,3,3).transpose();
			   pstar = pstarm.block(0,jj-1,3,1);
			   r = coglist.block(0,jj-1,3,1); 
			   // compute omegadot_i wrt base frame (eq 7.152)
			   wd = R*(wd + z0*qdd(jj-1) + w.cross(z0*qd(jj-1)));
			   // compute omega_i (eq 7.149 + eq 7.150 but in GC axis of rot. is constantly z0 because of DH)
			   w = R*(w + z0*qd(jj-1));
			   // note that the computation of alpha_i (eq 7.153) is missing because it only relies on qd and qdd.
			   // compute acceleration of link i (eq 7.159)
			   vd = wd.cross(pstar) + w.cross(w.cross(pstar)) + R*vd;
			   // compute the acceleration of the end of link i (eq 7.158)
			   vhat = wd.cross(r) + w.cross(w.cross(r)) + vd;
			   
			   F = mlist(0,jj-1)*vhat;
			   Ii = Istore.block(0,3*jj-3,3,3);
			   Iiw = Ii*w;
			   // (eq 7.136)
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

	Eigen::MatrixXd GravityCompensation::ComputeGravity(Eigen::MatrixXd a,Eigen::MatrixXd d,Eigen::MatrixXd alpha,Eigen::MatrixXd coglist,Eigen::MatrixXd mlist,Eigen::MatrixXd Istore,Eigen::MatrixXd q,Eigen::MatrixXd grav){
		// Declare matrices and vectors 
		Eigen::MatrixXd Gravity(1,mlist.cols());
		Eigen::MatrixXd no_Fext(6,1);
		
		// No external force to compute gravitational forces
		no_Fext = Eigen::MatrixXd::Zero(6,1);
		
		// Compute gravitational forces  
		Gravity = rne(a,d,alpha,coglist,mlist,Istore,q.transpose(),0*q.transpose(),0*q.transpose(),grav,no_Fext);
		
		return Gravity.transpose();	
	}

ORO_CREATE_COMPONENT(PERA::GravityCompensation)

