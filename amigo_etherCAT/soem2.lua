-- #### LOAD COMPONENT TO ENABLE ANALOG INS ###
depl:loadComponent("AnalogIns","SOEM::AnalogIns")
AnalogIns = depl:getPeer("AnalogIns")
AnalogIns:configure()
depl:setActivity("AnalogIns",1.0,LowestPriority,rtt.globals.ORO_SCHED_OTHER)
depl:connect ("AnalogIns.in","Soem.Slave_100e.values", cp );
depl:stream("AnalogIns.out1", rtt.provides("ros"):topic("/battery_value"))
rtt.logl('Info', "AnalogIns Deployment complete!")
AnalogIns:start()
