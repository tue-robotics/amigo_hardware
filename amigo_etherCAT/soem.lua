require("rttlib")
rttlib.color=true

tc = rtt.getTC()
depl = tc:getPeer("Deployer")
cp=rtt.Variable("ConnPolicy")

-- #Import all the dependencies from the amigo_etherCAT package. 
-- #This enables you to import components from all these packages.
-- depl:import("amigo_etherCAT")
depl:import("amigo_etherCAT")

-- Set variables
Ts = 0.001
HighestPriority = 99
LowestPriority = 0

-- ### LOAD SOEM COMPONENT FOR ETHERCAT COMMUNICATION ###
-- #This component enables communication with EtherCAT through the SoemMaster component.
depl:loadComponent("Soem","soem_master::SoemMasterComponent")
Soem = depl:getPeer("Soem")
-- #Configure the component. This looks for connected EtherCAT slaves and creates ports for all of the slaves.
Soem:configure()
-- #Set a realtime priority to this component and run it every 1ms.
-- Soem:setActivity("Soem",Ts,HighestPriority,ORO_SCHED_RT)
depl:setActivity("Soem",Ts,HighestPriority,rtt.globals.ORO_SCHED_RT)

-- #TODO What happens if you set the priority to 0.0?
-- 

-- #### LOAD COMPONENT TO ENABLE ANALOG OUTS ###
-- #This component aggregates all the signals from the controllers and sends them to the EtherCAT stack at once.7
depl:loadComponent("AnalogOuts","SOEM::AnalogOuts")
AnalogOuts = depl:getPeer("AnalogOuts")
AnalogOuts:configure()
depl:setActivity("AnalogOuts",0.0,HighestPriority,rtt.globals.ORO_SCHED_RT)
-- AnalogOuts.max_volt = array ( 5.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0 )
depl:connect ("AnalogOuts.Analog_out","Soem.Slave_100d.values", cp )

-- #### LOAD COMPONENT TO ENABLE ANALOG INS ###
depl:loadComponent("AnalogIns","SOEM::AnalogIns")
AnalogIns = depl:getPeer("AnalogIns")
AnalogIns:configure()
depl:setActivity("AnalogIns",1.0,LowestPriority,rtt.globals.ORO_SCHED_OTHER)
depl:connect ("AnalogIns.in","Soem.Slave_100e.values", cp );
depl:stream("AnalogIns.out1", rtt.provides("ros"):topic("/battery_value"))


-- #### LOAD COMPONENT TO ENABLE DIGITAL OUTS ###
-- loadComponent("DigitalOuts","SOEM::DigitalOuts")
-- connect ("DigitalOuts.digital_out","Soem.Slave_100b.bits", ConnPolicy() )
-- DigitalOuts.configure
-- setActivity("DigitalOuts",0.0,HighestPriority/2,ORO_SCHED_RT)

-- #### LOAD COMPONENT TO ENABLE DIGITAL INS (Slave_1009) ###
-- loadComponent("DigitalIns","SOEM::DigitalIns")
-- DigitalIns.setPeriod(0.1)
-- connect ("Soem.Slave_1009.bits", "DigitalIns.in", ConnPolicy() )
-- stream("DigitalIns.out1", ros.topic("/fuse1"))
-- stream("DigitalIns.out2", ros.topic("/fuse2"))
-- stream("DigitalIns.out3", ros.topic("/fuse3"))
-- stream("DigitalIns.out4", ros.topic("/fuse4"))
-- DigitalIns.flip_out5 = 1
-- stream("DigitalIns.out5", ros.topic("/runstop"))
-- DigitalIns.flip_out6 = 1
-- stream("DigitalIns.out6", ros.topic("/emergency_switch"))
-- DigitalIns.configure
-- setActivity("DigitalIns",0.1,LowestPriority,ORO_SCHED_OTHER)

-- #### LOAD COMPONENT TO ENABLE DIGITAL INS (Slave_100a) ###
-- loadComponent("DigitalIns2","SOEM::DigitalIns")
-- DigitalIns2.setPeriod(0.1)
-- connect ("Soem.Slave_100a.bits", "DigitalIns2.in", ConnPolicy() )
-- DigitalIns2.configure
-- setActivity("DigitalIns2",0.1,LowestPriority,ORO_SCHED_OTHER)
-- # Temp
-- stream("DigitalIns2.out2", ros.topic("/spindle_endstop") )

AnalogOuts:start()
-- DigitalOuts.start
Soem:start()
AnalogIns:start()
-- DigitalIns.start
-- DigitalIns2.start
