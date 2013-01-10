/*
 * SpindleSupervisor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 * Wrecked by Tim 
 */
//#include <ros/package.h>
#include <rtt/Component.hpp>
#include "SpindleSupervisor.hpp"

using namespace std;
using namespace RTT;
using namespace AMIGO;


//ORO_LIST_COMPONENT_TYPE( BASE::SpindleSupervisor )

SpindleSupervisor::SpindleSupervisor(const string& name) :
    TaskContext(name, PreOperational)
{
    addOperation("AddSpindleSupervisoredPeer", &SpindleSupervisor::addSpindleSupervisoredPeer, this, OwnThread)
            .doc("Add a peer to the SpindleSupervisored list, all these components are restarted if a reset command is received!")
            .arg("peerName","Name of the peer to add to the list");

    addOperation("DisplaySpindleSupervisoredPeers", &SpindleSupervisor::displaySpindleSupervisoredPeers, this, ClientThread)
               .doc("Display the list of peers");
  addEventPort( "rosemergency", rosemergencyport );
  addEventPort( "rosstandby", rosstandbyport );

}

SpindleSupervisor::~SpindleSupervisor()
{
}

//------------------------------------------------------------------------------------------------------------------

bool SpindleSupervisor::configureHook()
{
    return true;
}

bool SpindleSupervisor::startHook()
{
	started = true;
    return true;
}

void SpindleSupervisor::updateHook()
{
    std_msgs::Bool rosemergencymsg;
    std_msgs::Bool rosstandbymsg;
    rosemergencyport.read( rosemergencymsg );
    rosstandbyport.read( rosstandbymsg );
    bool emergency = rosemergencymsg.data;
    bool standby = rosstandbymsg.data;
    
    if ( ( emergency || standby ) && started )
    {
      log(Warning) << "Stopping components" << endlog();
      vector<TaskContext*>::iterator i;
      for ( i = m_SpindleSupervisoredList.begin() ; i != m_SpindleSupervisoredList.end() ; i++ )
      {
          TaskContext* tc = (*i);

          if( tc == NULL )
          {
              log(Error) << "m_SpindleSupervisoredList should not contain null values ! (update)" << endlog();
              error();
          }
          else
          {
			  log(Warning) << "SpindleSupervisor: Stopping: " << tc->getName() << endlog();
              tc->stop();
          }
      }
      started = false;
    }  
    else if ( !( emergency || standby ) && !started )
    {
      log(Warning) << "Starting components" << endlog();
      vector<TaskContext*>::iterator i;
      for ( i = m_SpindleSupervisoredList.begin() ; i != m_SpindleSupervisoredList.end() ; i++ )
      {
          TaskContext* tc = (*i);

          if( tc == NULL )
          {
              log(Error) << "m_SpindleSupervisoredList should not contain null values ! (update)" << endlog();
              error();
          }
          else
          {
			  log(Warning) << "SpindleSupervisor: Starting: " << tc->getName() << endlog();
              tc->start();
          }
      }
      started = true;
    }
      
}

void SpindleSupervisor::stopHook()
{
  
}

void SpindleSupervisor::cleanupHook()
{

}


//-----------------------------------------------------

bool SpindleSupervisor::addSpindleSupervisoredPeer(std::string peerName )
{
    bool res = true;

    if( ! hasPeer(peerName) )
    {
        log(Error) << "You can't SpindleSupervisor a component that is not your peer !" << endlog();
        res = false;
    }
    /*else if( getTaskState() !=  getPeer(peerName)->getTaskState() )
    {
        log(Error) << "You can't add a new SpindleSupervisored component that is not in your current state !" << endlog();
        res = false;
    }*/
    else
    {
        vector<TaskContext*>::iterator i;
        for ( i = m_SpindleSupervisoredList.begin() ; i != m_SpindleSupervisoredList.end() ; i++ )
        {
            TaskContext* tc = (*i);
            if( tc == NULL )
            {
              log(Error) << "m_SpindleSupervisoredList should not contain null values ! (addSpindleSupervisoredPeer)" << endlog();
              res = false;
            }
            else
            {
              if( tc->getName() == peerName )
              {
                  log(Error) << tc->getName() << " is already in the list !" << endlog();
                  res = false;
                  break;
              }
            }
        }

        if( res == true )
        {
            log(Info) << "New peer to SpindleSupervisor : " << peerName << endlog();
            m_SpindleSupervisoredList.push_back (getPeer(peerName));
        }
    }

    return res;
}


void SpindleSupervisor::displaySpindleSupervisoredPeers()
{
    cout << endl;
    cout << "List of SpindleSupervisored peers : " << endl;
    cout << endl;

    vector<TaskContext*>::iterator i;
    for ( i = m_SpindleSupervisoredList.begin() ; i != m_SpindleSupervisoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "m_SpindleSupervisoredList should not contain null values ! (displaySpindleSupervisoredPeers)" << endl;
        }
        else
        {
            cout << tc->getName() << endl;
        }
    }

    cout << "------------------------" << endl;
    cout << endl;
}

ORO_CREATE_COMPONENT(AMIGO::SpindleSupervisor)
