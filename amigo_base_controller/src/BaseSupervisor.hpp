/*
 * BaseSupervisor.hpp
 *
 *  Created on: 6 janv 2012
 *      Author: wla
 * Wrecked by Tim 
 */

#ifndef BASESUPERVISOR_HPP_
#define BASESUPERVISOR_HPP_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Bool.h>

using namespace std;
using namespace RTT;

namespace AMIGO
{
    /** \ingroup ARP-arp_core
     *
     * \class BaseSupervisor
     *
     * Cette classe permet de gerer plusieurs composants. Il y en a au moins une par projet
     * qui permet de faire l'interface de pilotage pour les projets supérieurs.
     */
    class BaseSupervisor
    : public RTT::TaskContext
      {
      public:

        InputPort<std_msgs::Bool> rosresetport;

        /** Constructeur pour définir le chemin vers le projet. Utile pour ROS*/
        BaseSupervisor(const std::string& name);
        /** Destructeur par défaut */
        virtual ~BaseSupervisor();

        /**
         * Configure all peers previously registered by the addBaseSupervisoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool configureHook();

        /**
         * Start all peers previously registered by the addBaseSupervisoredPeer command
         * The configuration is done in the order in which elements where inserted
         */
        virtual bool startHook();

        /**
         * Checks if all components are still running
         */
        virtual void updateHook();

        /**
         * Stop all peers previously registered by the addBaseSupervisoredPeer command
         * The configuration is done in the reverse order in which elements where inserted
         */
        virtual void stopHook();

        /**
         * Cleanup all peers previously registered by the addBaseSupervisoredPeer command
         * The configuration is done in the reversed order in which elements where inserted
         */
        virtual void cleanupHook();

        /**
         * @param peerName : the name of the Orocos component that needs to be BaseSupervisored
         * @return true if success, false if the peer to BaseSupervisor is not in the peer list.
         */
        virtual bool addBaseSupervisoredPeer(std::string peerName );

        /**
         * display the list of BaseSupervisored peers
         */
        virtual void displayBaseSupervisoredPeers();

    protected:
        /** List of peers to BaseSupervisor */
        vector<TaskContext*> m_BaseSupervisoredList;



      };
}

#endif /* BaseSupervisor_HPP_ */
