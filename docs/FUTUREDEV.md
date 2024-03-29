## Future TODOs ###
Here are my notes for what should be developed on in the future and what I think the best path forward is for CASA. -JH

#### Remedies ####
1. **Bridging**: This node does not currently work. The bridging node creates two seperate nodes, one for the incoming topics and one for the outgoing topic. The two nodes created are given the same name which is what I think is causing the problem. It seems using domain-bridge at runtime will be more challenging than I thought. The best path forward may be to fork the source code and modify it for our needs there.

2. **Task Allocation**: Task allocation still needs to be integrated with the the behavior ROS interface, I should be able to knock this out before things get too crazy for me.


#### Feature Completion ####
1. **Behavior Launching**: If you were to launch a behavior as it stands right now the behavior would have to be launch on the actual vehicle. We need to be able to launch from a groundstation and pass the behavior parameters from the GS to the vehicle and trigger the agent to start the node.

#### Future Development ####
1. **Collision Avoidance**: Pretty simple, the best way to integrate this may be to bring in Nav2, it would also make Obstance Avoidance easier if that route was ever taken.

2. **Trajectory Planner**: Right now the "trajcetory planner" is really just passing on the goto commands. Once a collision avoidance algorithm is implemented this will need further development as the CA should intervene here. My trajectory planner does have a clever way to swtich between velocity and waypoint navigation (for PX4). Right now this is platform specific, the TP for the pixhawk is different for the turtlebot so this will probably need to be redeveloped. 

3. **GS GUI**: A groundstation GUI should be development so that the swarm can easily be operated. I would try to leverage WebTools, but I'm not a web developer so I'm not sure how realistic that is, it just leaves a lot of options open, like streaming swarm data on the web to anyone who wants to watch. I wrote just about all of my code without this in mind, i.e. everything was developed to be run directly on the vehicle as an easier first step. You guys can work on how to transmit parameters set on the GS and how to activate nodes. Domain Bridge only allows bridging for topics and services so I'm not sure how to transmit parameters across domains.

4. **Data Logger**: We need something to log essential data in a proper database. This will make debugging things easier. Its also necessary to make CASA more robust. If one agent loses contact with another then it should be able to fill in the the data it missed when they were not talking. 

5. **Network Monitoring**: Right now when CASA starts I scan the network and see what other agents are on there. And then pass those `DOMAIN_ID`'s to the nodes that need them. The way my launch file works, the last octet of the IP address becomes the `ROS_DOMAIN_ID`. The agents need to constantly be monitoring the network so they know who they are and are not talking to and when new agents come on the network. When new agents are detected, we'll need to domain-bridge between the agents. 

#### Nice To Have But Not Necessary ####
1. **High Level Planner**: Right now the task allocation node is the high level planner and that is okay. This can be expanded upon and I think this would be a good research thrust for the RRC. How do you make the task allocation more robust to communcation dropouts? What do you do if you misplanned something because two agents weren't talking? How do you know if you misplanned? How do you plan for continues domains? (A: semi-discrete optimal transport) Those are all good research questions that go hand-in-hand with the development of CASA. It would also be nice to make this so that a user can write planners like a behavior.
