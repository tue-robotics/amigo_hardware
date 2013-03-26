require("rttlib")
rttlib.color=true

tc = rtt.getTC()
depl = tc:getPeer("Deployer")
depl:import("amigo_etherCAT")
cp=rtt.Variable("ConnPolicy")


tc:exec_file("soem1.lua")
tc:exec_file("soem2.lua")
