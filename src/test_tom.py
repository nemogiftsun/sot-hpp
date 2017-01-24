from dynamic_graph.sot.hpp import PathSampler
ps = PathSampler ('ps')
ps.loadRobotModel ('tom_description', 'anchor', 'tom_hpp')
q_init = 12*[0]
ps.addWaypoint (tuple (q_init))
q_init [0] = 1
q_init [1] = 1
q_init [2] = 
q_end = q_init [::]
#q_end [34] = 2
ps.addWaypoint (tuple (q_end))
ps.position.value = 34*[0]
ps.configuration.recompute (0)
ps.setTimeStep (0.1)
ps.start ()
ps.configuration.value
ps.createJointReference('l_wrist_roll_joint')
ps.l_wrist_roll_joint.recompute(1)
ps.l_wrist_roll_joint.value
for i in range (11):
    ps.configuration.recompute (i)
    print (ps.configuration.value [0])

ps.resetPath ()

for i in range (111, 121):
    ps.configuration.recompute (i)
    #print (ps.configuration.value [0])

ps.addWaypoint (tuple (q_init))

for i in range (121, 131):
    ps.configuration.recompute (i)
    #print (ps.configuration.value [0])

ps.start ()

for i in range (131, 240):
    ps.configuration.recompute (i)
    #print (ps.configuration.value [0])
