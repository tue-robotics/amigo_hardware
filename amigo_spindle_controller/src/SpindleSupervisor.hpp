/*
 * SpindleSupervisor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 * Wrecked by Tim 
 */

#ifndef SpindleSupervisor_HPP_
#define SpindleSupervisor_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO
{
    /** \ingroup ARP-arp_core
     *
     * \class SpindleSupervisor
     *
     * Cette classe permet de gerer plusieurs composants. Il y en a au moins une par projet
     * qui permet de faire l'interface de pilotage pour les projets supérieurs.
     */
    class SpindleSupervisor
    : public RTT::TaskContext
      {
      public:

        InputPort<std_msgs::Bool> rosemergencyport;
        InputPort<std_msgs::Bool> rosstandbyport;
        bool started; //Keep track if components are on or off

        /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
        SpindleSupervisor(const std::string& name);
        /** Destructeur par défaut */
        virtual ~SpindleSupervisor();

        /**
         * Configure all peers previously registered by the addSpindleSupervisoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool configureHook();

        /**
         * Start all peers previously registered by the addSpindleSupervisoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool startHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Stop all peers previously registered by the addSpindleSupervisoredPeer command
         * The configuration is done in the reverse order in which elements where inserted
         */
        virtual void stopHook();

        /**
         * Cleanup all peers previously registered by the addSpindleSupervisoredPeer command
         * The configuration is done in the reversed order in which elements where inserted
         */
        virtual void cleanupHook();

        /**
         * @param peerName : the name of the Orocos component that needs to be SpindleSupervisored
         * @return true if success, false if the peer to SpindleSupervisor is not in the peer list.
         */
        virtual bool addSpindleSupervisoredPeer(std::string peerName );

        /**
         * display the list of SpindleSupervisored peers
         */
        virtual void displaySpindleSupervisoredPeers();

    protected:
        /** List of peers to SpindleSupervisor */
        vector<TaskContext*> m_SpindleSupervisoredList;



      };
}

#endif /* SpindleSupervisor_HPP_ */
