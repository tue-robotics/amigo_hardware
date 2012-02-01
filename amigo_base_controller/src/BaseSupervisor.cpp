/*
 * BaseSupervisor.cpp
 *
 *  Created on: 6 janv. 2012
 *      Author: ard, wla
 * Wrecked by Tim 
 */
//#include <ros/package.h>
#include <rtt/Component.hpp>
#include "BaseSupervisor.hpp"

using namespace RTT;
using namespace AMIGO;


//ORO_LIST_COMPONENT_TYPE( BASE::BaseSupervisor )

BaseSupervisor::BaseSupervisor(const string& name) :
    TaskContext(name, PreOperational)
{
    addOperation("AddBaseSupervisoredPeer", &BaseSupervisor::addBaseSupervisoredPeer, this, OwnThread)
            .doc("Add a peer to the BaseSupervisored list, all these components are restarted if a reset command is received!")
            .arg("peerName","Name of the peer to add to the list");

    addOperation("DisplayBaseSupervisoredPeers", &BaseSupervisor::displayBaseSupervisoredPeers, this, ClientThread)
               .doc("Display the list of peers");
  addEventPort( "rosreset", rosresetport );

}

BaseSupervisor::~BaseSupervisor()
{
}

//------------------------------------------------------------------------------------------------------------------

bool BaseSupervisor::configureHook()
{
    return true;
}

bool BaseSupervisor::startHook()
{
    return true;
}

void BaseSupervisor::updateHook()
{
    std_msgs::Bool rosboolmsg;
    if (rosresetport.read( rosboolmsg ) == NewData )
    {
      log(Warning) << "Restarting components" << endlog();
      vector<TaskContext*>::iterator i;
      for ( i = m_BaseSupervisoredList.begin() ; i != m_BaseSupervisoredList.end() ; i++ )
      {
          TaskContext* tc = (*i);

          if( tc == NULL )
          {
              log(Error) << "m_BaseSupervisoredList should not contain null values ! (update)" << endlog();
              error();
          }
          else
          {
              tc->stop();
              if( !tc->isConfigured() )
              {
                  tc->configure();
              }
              tc->start();
          }
      }
    }
}

void BaseSupervisor::stopHook()
{
  
}

void BaseSupervisor::cleanupHook()
{

}


//-----------------------------------------------------

bool BaseSupervisor::addBaseSupervisoredPeer(std::string peerName )
{
    bool res = true;

    if( ! hasPeer(peerName) )
    {
        log(Error) << "You can't BaseSupervisor a component that is not your peer !" << endlog();
        res = false;
    }
    /*else if( getTaskState() !=  getPeer(peerName)->getTaskState() )
    {
        log(Error) << "You can't add a new BaseSupervisored component that is not in your current state !" << endlog();
        res = false;
    }*/
    else
    {
        vector<TaskContext*>::iterator i;
        for ( i = m_BaseSupervisoredList.begin() ; i != m_BaseSupervisoredList.end() ; i++ )
        {
            TaskContext* tc = (*i);
            if( tc == NULL )
            {
              log(Error) << "m_BaseSupervisoredList should not contain null values ! (addBaseSupervisoredPeer)" << endlog();
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
            log(Info) << "New peer to BaseSupervisor : " << peerName << endlog();
            m_BaseSupervisoredList.push_back (getPeer(peerName));
        }
    }

    return res;
}


void BaseSupervisor::displayBaseSupervisoredPeers()
{
    cout << endl;
    cout << "List of BaseSupervisored peers : " << endl;
    cout << endl;

    vector<TaskContext*>::iterator i;
    for ( i = m_BaseSupervisoredList.begin() ; i != m_BaseSupervisoredList.end() ; i++ )
    {
        TaskContext* tc = (*i);

        if( tc == NULL )
        {
            cout << "m_BaseSupervisoredList should not contain null values ! (displayBaseSupervisoredPeers)" << endl;
        }
        else
        {
            cout << tc->getName() << endl;
        }
    }

    cout << "------------------------" << endl;
    cout << endl;
}

ORO_CREATE_COMPONENT(AMIGO::BaseSupervisor)